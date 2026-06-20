"""Smoke tests for sbr_drivers that run without I2C/GPIO hardware."""
from sbr_drivers.mpu6050 import _ACCEL_SCALE_2G, _GRAVITY, Mpu6050


def test_constants_are_sane():
    assert _GRAVITY > 9.7
    assert _ACCEL_SCALE_2G == 16384.0


def test_class_imports_without_smbus2():
    # smbus2 is imported lazily inside __init__, so the class itself imports
    # fine on a machine with no I2C bus.
    assert hasattr(Mpu6050, 'read')
    assert hasattr(Mpu6050, 'close')
