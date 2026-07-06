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
_WHO_AM_I = 0x75

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
        self._gyro_bias = (0.0, 0.0, 0.0)
        # Identity check: catches a missing/mis-wired device (or a different
        # chip squatting on 0x68) before we trust its data.
        whoami = self._bus.read_byte_data(self._address, _WHO_AM_I)
        if whoami != 0x68:
            raise RuntimeError(
                f'MPU6050 WHO_AM_I mismatch: got 0x{whoami:02x}, expected 0x68 '
                '(check wiring/AD0; some clones report other values)')
        # Wake the device (clear sleep bit).
        self._bus.write_byte_data(self._address, _PWR_MGMT_1, 0x00)
        # 1 kHz sample rate, ~44 Hz DLPF, +-2g, +-250 deg/s.
        self._bus.write_byte_data(self._address, _SMPLRT_DIV, 0x00)
        self._bus.write_byte_data(self._address, _CONFIG, 0x03)
        self._bus.write_byte_data(self._address, _GYRO_CONFIG, 0x00)
        self._bus.write_byte_data(self._address, _ACCEL_CONFIG, 0x00)

    def _read_raw(self):
        """Return uncorrected (ax, ay, az [m/s^2], gx, gy, gz [rad/s])."""
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

    def read(self):
        """Return (ax, ay, az [m/s^2], gx, gy, gz [rad/s]), bias-corrected."""
        ax, ay, az, gx, gy, gz = self._read_raw()
        bx, by, bz = self._gyro_bias
        return (ax, ay, az, gx - bx, gy - by, gz - bz)

    def calibrate_gyro(self, samples=200, delay_s=0.002):
        """Average the gyro output at rest. The robot MUST be stationary.

        Raw gyros have a constant bias that the complementary filter
        integrates into a steady pitch drift; subtracting a startup average
        removes it.
        """
        import time
        sx = sy = sz = 0.0
        for _ in range(samples):
            _, _, _, gx, gy, gz = self._read_raw()
            sx += gx
            sy += gy
            sz += gz
            time.sleep(delay_s)
        self._gyro_bias = (sx / samples, sy / samples, sz / samples)
        return self._gyro_bias

    def close(self):
        try:
            self._bus.close()
        except Exception:
            pass
