"""Publish ``sensor_msgs/Imu`` from an MPU6050 (or a mock signal).

A complementary filter fuses the accelerometer and gyroscope into a roll/pitch
estimate, published as the orientation quaternion. This is the *same*
``sensor_msgs/Imu`` message the Gazebo IMU produces, so the balance controller
consumes one interface in both simulation and hardware.

Run ``mock:=true`` (or let it fall back automatically) to publish an upright
signal on machines without an MPU6050.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool


def quaternion_from_euler(roll, pitch, yaw):
    """Convert roll/pitch/yaw (rad) to a (x, y, z, w) quaternion."""
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,  # x
        cr * sp * cy + sr * cp * sy,  # y
        cr * cp * sy - sr * sp * cy,  # z
        cr * cp * cy + sr * sp * sy,  # w
    )


class ImuNode(Node):

    def __init__(self):
        super().__init__('imu_node')
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('i2c_address', 0x68)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('publish_rate', 100.0)
        self.declare_parameter('complementary_alpha', 0.98)
        self.declare_parameter('mock', False)
        self.declare_parameter('max_read_failures', 10)
        self.declare_parameter('calibration_samples', 200)

        self._frame_id = self.get_parameter('frame_id').value
        self._alpha = self.get_parameter('complementary_alpha').value
        self._rate = self.get_parameter('publish_rate').value
        if self._rate <= 0:
            raise ValueError(f'publish_rate must be > 0 (got {self._rate})')
        self._mock = self.get_parameter('mock').value
        self._max_read_failures = self.get_parameter('max_read_failures').value

        self._roll = 0.0
        self._pitch = 0.0
        self._last_tick = None
        self._sensor = None
        # Degraded-mode tracking. _fallback latches when the sensor could not
        # be opened and we auto-switched to mock (a hardware fault the operator
        # must see -- the mock signal is "perfectly upright" and must never be
        # used for control). _degraded covers runtime read-failure streaks and
        # clears if reads recover. Explicit mock:=true is NOT degraded.
        self._fallback = False
        self._degraded = False
        self._read_failures = 0
        if not self._mock:
            try:
                from sbr_drivers.mpu6050 import Mpu6050
                self._sensor = Mpu6050(
                    bus=self.get_parameter('i2c_bus').value,
                    address=self.get_parameter('i2c_address').value,
                )
                n = self.get_parameter('calibration_samples').value
                if n > 0:
                    self.get_logger().info(
                        f'Calibrating gyro bias ({n} samples) - keep the robot still...')
                    bias = self._sensor.calibrate_gyro(samples=n)
                    self.get_logger().info(
                        f'Gyro bias [rad/s]: {bias[0]:+.4f} {bias[1]:+.4f} {bias[2]:+.4f}')
            except Exception as exc:  # pragma: no cover - hardware path
                self.get_logger().error(
                    f'Could not open MPU6050 ({exc}); falling back to mock mode.')
                self._mock = True
                self._fallback = True

        self._pub = self.create_publisher(Imu, 'imu/data', qos_profile_sensor_data)
        self._status_pub = self.create_publisher(Bool, 'imu/hardware_ok', 10)
        self._timer = self.create_timer(1.0 / self._rate, self._tick)
        self._status_timer = self.create_timer(1.0, self._publish_status)
        self.get_logger().info(
            f"imu_node started ({'mock' if self._mock else 'hardware'}) "
            f'at {self._rate:.0f} Hz')

    def _publish_status(self):
        ok = self._sensor is not None and not self._degraded and not self._fallback
        self._status_pub.publish(Bool(data=ok))

    def _tick(self):
        if self._mock:
            ax, ay, az = 0.0, 0.0, 9.80665
            gx, gy, gz = 0.0, 0.0, 0.0
        else:
            try:
                ax, ay, az, gx, gy, gz = self._sensor.read()
                self._read_failures = 0
                if self._degraded:
                    self.get_logger().info('IMU reads recovered; leaving degraded mode.')
                    self._degraded = False
            except Exception as exc:  # pragma: no cover - hardware path
                self._read_failures += 1
                if self._read_failures > self._max_read_failures:
                    self._degraded = True
                    self.get_logger().error(
                        f'IMU DEGRADED: {self._read_failures} consecutive read '
                        f'failures ({exc}); controller will cut the motors.',
                        throttle_duration_sec=5.0)
                else:
                    self.get_logger().warn(
                        f'IMU read failed: {exc}', throttle_duration_sec=2.0)
                return

        if self._fallback:
            self.get_logger().error(
                'IMU DEGRADED: hardware missing, publishing invalid orientation; '
                'motors will be cut by the controller watchdog.',
                throttle_duration_sec=10.0)

        # Measured dt (clamped): timer jitter or a slow I2C read would
        # otherwise skew the gyro integration, which assumes the period.
        nominal_dt = 1.0 / self._rate
        tick_now = self.get_clock().now()
        if self._last_tick is None:
            dt = nominal_dt
        else:
            dt = (tick_now - self._last_tick).nanoseconds * 1e-9
            dt = max(0.25 * nominal_dt, min(dt, 4.0 * nominal_dt))
        self._last_tick = tick_now
        # Tilt estimate from gravity vector.
        acc_roll = math.atan2(ay, az)
        acc_pitch = math.atan2(-ax, math.sqrt(ay * ay + az * az))
        # Complementary filter: trust the gyro short-term, the accel long-term.
        a = self._alpha
        self._roll = a * (self._roll + gx * dt) + (1.0 - a) * acc_roll
        self._pitch = a * (self._pitch + gy * dt) + (1.0 - a) * acc_pitch

        qx, qy, qz, qw = quaternion_from_euler(self._roll, self._pitch, 0.0)
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz
        msg.orientation.w = qw
        msg.angular_velocity.x = gx
        msg.angular_velocity.y = gy
        msg.angular_velocity.z = gz
        msg.linear_acceleration.x = ax
        msg.linear_acceleration.y = ay
        msg.linear_acceleration.z = az
        # Degraded convention: negative orientation covariance marks the sample
        # unusable for control; the balance controller drops flagged samples and
        # its imu_timeout watchdog then zeroes the motors. Explicit mock:=true
        # (dev machines) stays valid so sim/bench flows keep working.
        if self._fallback or self._degraded:
            msg.orientation_covariance[0] = -1.0
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ImuNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            if getattr(node, '_sensor', None) is not None:
                node._sensor.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
