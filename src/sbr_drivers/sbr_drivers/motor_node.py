"""Drive two DC motors (no encoders) from ``/wheel_cmd``.

Subscribes to ``std_msgs/Float64MultiArray`` carrying ``[left, right]`` effort
in [-1, 1] and converts each to a direction + PWM duty cycle for a dual
H-bridge (e.g. TB6612FNG or L298N). The GPIO dependency (``lgpio``) is imported
lazily, so the node runs in ``mock`` mode on any machine. A watchdog stops the
motors if no command arrives within ``cmd_timeout`` seconds.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class _MockBackend:
    """Logs commands instead of touching GPIO."""

    def __init__(self, logger):
        self._logger = logger

    def set(self, left, right):
        self._logger.info(f'motor mock: L={left:+.2f} R={right:+.2f}',
                          throttle_duration_sec=1.0)

    def stop(self):
        pass

    def close(self):
        pass


class _LgpioBackend:
    """Drives a dual H-bridge via lgpio (works on Raspberry Pi / Ubuntu)."""

    def __init__(self, pins, pwm_freq, max_duty, deadband, invert):
        import lgpio
        self._lg = lgpio
        self._h = lgpio.gpiochip_open(0)
        self._pins = pins
        self._pwm_freq = pwm_freq
        self._max_duty = max_duty
        self._deadband = deadband
        self._invert = invert
        for name in ('left_in1', 'left_in2', 'right_in1', 'right_in2'):
            lgpio.gpio_claim_output(self._h, pins[name], 0)
        # TB6612FNG STBY: must be HIGH to enable the H-bridge.
        lgpio.gpio_claim_output(self._h, pins['stby'], 1)

    def _drive(self, in1, in2, pwm_pin, value):
        if value >= 0.0:
            self._lg.gpio_write(self._h, in1, 1)
            self._lg.gpio_write(self._h, in2, 0)
        else:
            self._lg.gpio_write(self._h, in1, 0)
            self._lg.gpio_write(self._h, in2, 1)
        duty = min(abs(value), 1.0) * self._max_duty * 100.0
        if abs(value) < self._deadband:
            duty = 0.0
        self._lg.tx_pwm(self._h, pwm_pin, self._pwm_freq, duty)

    def set(self, left, right):
        if self._invert['left']:
            left = -left
        if self._invert['right']:
            right = -right
        self._drive(self._pins['left_in1'], self._pins['left_in2'],
                    self._pins['left_pwm'], left)
        self._drive(self._pins['right_in1'], self._pins['right_in2'],
                    self._pins['right_pwm'], right)

    def stop(self):
        for pwm_pin in (self._pins['left_pwm'], self._pins['right_pwm']):
            self._lg.tx_pwm(self._h, pwm_pin, self._pwm_freq, 0.0)

    def close(self):
        try:
            self.stop()
            self._lg.gpio_write(self._h, self._pins['stby'], 0)  # disable bridge
            self._lg.gpiochip_close(self._h)
        except Exception:
            pass


class MotorNode(Node):

    def __init__(self):
        super().__init__('motor_node')
        self.declare_parameter('mock', False)
        # 20 kHz: above hearing -- 1 kHz makes the motors whine (docs/hardware.md).
        self.declare_parameter('pwm_frequency', 20000)
        self.declare_parameter('max_duty', 1.0)
        self.declare_parameter('deadband', 0.02)
        self.declare_parameter('cmd_timeout', 0.5)
        self.declare_parameter('invert_left', False)
        self.declare_parameter('invert_right', False)
        # TB6612 / L298N control pins (BCM numbering).
        self.declare_parameter('left_in1', 17)
        self.declare_parameter('left_in2', 27)
        self.declare_parameter('left_pwm', 18)
        self.declare_parameter('right_in1', 22)
        self.declare_parameter('right_in2', 23)
        self.declare_parameter('right_pwm', 13)
        self.declare_parameter('stby', 24)   # TB6612FNG standby/enable

        self._timeout = self.get_parameter('cmd_timeout').value
        mock = self.get_parameter('mock').value

        if mock:
            self._backend = _MockBackend(self.get_logger())
        else:
            pins = {k: self.get_parameter(k).value for k in (
                'left_in1', 'left_in2', 'left_pwm',
                'right_in1', 'right_in2', 'right_pwm', 'stby')}
            invert = {'left': self.get_parameter('invert_left').value,
                      'right': self.get_parameter('invert_right').value}
            try:
                self._backend = _LgpioBackend(
                    pins,
                    self.get_parameter('pwm_frequency').value,
                    self.get_parameter('max_duty').value,
                    self.get_parameter('deadband').value,
                    invert)
            except Exception as exc:  # pragma: no cover - hardware path
                self.get_logger().error(
                    f'Could not open GPIO ({exc}); falling back to mock mode.')
                self._backend = _MockBackend(self.get_logger())

        self._last_cmd = self.get_clock().now()
        self._sub = self.create_subscription(
            Float64MultiArray, 'wheel_cmd', self._on_cmd, 10)
        self._watchdog = self.create_timer(0.1, self._check_watchdog)
        self.get_logger().info(
            f"motor_node started ({'mock' if mock else 'hardware'})")

    def _on_cmd(self, msg):
        if len(msg.data) < 2:
            return
        self._last_cmd = self.get_clock().now()
        self._backend.set(float(msg.data[0]), float(msg.data[1]))

    def _check_watchdog(self):
        age = (self.get_clock().now() - self._last_cmd).nanoseconds * 1e-9
        if age > self._timeout:
            self._backend.stop()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MotorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            if getattr(node, '_backend', None) is not None:
                node._backend.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
