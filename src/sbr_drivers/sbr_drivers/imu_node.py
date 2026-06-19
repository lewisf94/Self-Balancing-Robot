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

        self._frame_id = self.get_parameter('frame_id').value
        self._alpha = self.get_parameter('complementary_alpha').value
        self._rate = self.get_parameter('publish_rate').value
        self._mock = self.get_parameter('mock').value

        self._roll = 0.0
        self._pitch = 0.0
        self._sensor = None
        if not self._mock:
            try:
                from sbr_drivers.mpu6050 import Mpu6050
                self._sensor = Mpu6050(
                    bus=self.get_parameter('i2c_bus').value,
                    address=self.get_parameter('i2c_address').value,
                )
            except Exception as exc:  # pragma: no cover - hardware path
                self.get_logger().error(
                    f'Could not open MPU6050 ({exc}); falling back to mock mode.')
                self._mock = True

        self._pub = self.create_publisher(Imu, 'imu/data', qos_profile_sensor_data)
        self._timer = self.create_timer(1.0 / self._rate, self._tick)
        self.get_logger().info(
            f"imu_node started ({'mock' if self._mock else 'hardware'}) "
            f'at {self._rate:.0f} Hz')

    def _tick(self):
        if self._mock:
            ax, ay, az = 0.0, 0.0, 9.80665
            gx, gy, gz = 0.0, 0.0, 0.0
        else:
            try:
                ax, ay, az, gx, gy, gz = self._sensor.read()
            except Exception as exc:  # pragma: no cover - hardware path
                self.get_logger().warn(f'IMU read failed: {exc}', throttle_duration_sec=2.0)
                return

        dt = 1.0 / self._rate
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
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node._sensor is not None:
            node._sensor.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
