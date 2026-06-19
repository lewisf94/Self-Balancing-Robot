"""Minimal MPU6050 I2C driver (accelerometer + gyroscope).

Only the registers needed for balancing are configured. The hardware
dependency (``smbus2``) is imported lazily inside ``__init__`` so this module
can be imported, built and unit-tested on machines without I2C hardware.
"""
import struct

# --- MPU6050 registers -------------------------------------------------------
_PWR_MGMT_1 = 0x6B
_SMPLRT_DIV = 0x19
_CONFIG = 0x1A
_GYRO_CONFIG = 0x1B
_ACCEL_CONFIG = 0x1C
_ACCEL_XOUT_H = 0x3B

# --- Full-scale ranges (default config below) --------------------------------
_ACCEL_SCALE_2G = 16384.0   # LSB / g     at +-2g
_GYRO_SCALE_250 = 131.0     # LSB / (deg/s) at +-250 deg/s
_GRAVITY = 9.80665          # m / s^2
_DEG2RAD = 0.017453292519943295


class Mpu6050:
    """Reads calibrated accelerometer (m/s^2) and gyroscope (rad/s) data."""

    def __init__(self, bus=1, address=0x68):
        from smbus2 import SMBus  # lazy import - hardware only
        self._address = address
        self._bus = SMBus(bus)
        # Wake the device (clear sleep bit).
        self._bus.write_byte_data(self._address, _PWR_MGMT_1, 0x00)
        # 1 kHz sample rate, ~44 Hz DLPF, +-2g, +-250 deg/s.
        self._bus.write_byte_data(self._address, _SMPLRT_DIV, 0x00)
        self._bus.write_byte_data(self._address, _CONFIG, 0x03)
        self._bus.write_byte_data(self._address, _GYRO_CONFIG, 0x00)
        self._bus.write_byte_data(self._address, _ACCEL_CONFIG, 0x00)

    def read(self):
        """Return (ax, ay, az [m/s^2], gx, gy, gz [rad/s])."""
        raw = self._bus.read_i2c_block_data(self._address, _ACCEL_XOUT_H, 14)
        ax, ay, az, _temp, gx, gy, gz = struct.unpack('>hhhhhhh', bytes(raw))
        return (
            ax / _ACCEL_SCALE_2G * _GRAVITY,
            ay / _ACCEL_SCALE_2G * _GRAVITY,
            az / _ACCEL_SCALE_2G * _GRAVITY,
            gx / _GYRO_SCALE_250 * _DEG2RAD,
            gy / _GYRO_SCALE_250 * _DEG2RAD,
            gz / _GYRO_SCALE_250 * _DEG2RAD,
        )

    def close(self):
        try:
            self._bus.close()
        except Exception:
            pass
