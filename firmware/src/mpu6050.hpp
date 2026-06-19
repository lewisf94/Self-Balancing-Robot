#pragma once
//
// Minimal MPU-6050 reader + complementary-filter pitch estimate.
//
// Registers, full-scale ranges and the fusion math match
// src/sbr_drivers/sbr_drivers/mpu6050.py and imu_node.py, so the firmware, the
// Raspberry-Pi driver and Gazebo all agree on the pitch convention:
//   acc_pitch = atan2(-ax, hypot(ay, az));  pitch = a*(pitch + gy*dt) + (1-a)*acc_pitch
// pitch_rate is the gyro Y rate (rad/s). Positive pitch = leaning forward; flip
// the whole sign downstream with sbr_cfg::kInvertPitch if your IMU is mounted
// the other way.
//
#include <Arduino.h>
#include <Wire.h>
#include <cmath>

class Mpu6050
{
public:
  bool begin(TwoWire & wire, uint8_t address, double alpha)
  {
    wire_ = &wire;
    addr_ = address;
    alpha_ = alpha;
    pitch_ = 0.0;
    gy_ = 0.0;
    // Wake (clear sleep) + configure: 1 kHz sample, ~44 Hz DLPF, +-2g, +-250 dps.
    if (!writeReg(kPwrMgmt1, 0x00)) {
      return false;            // device didn't ACK -> not present / miswired
    }
    writeReg(kSmplrtDiv, 0x00);
    writeReg(kConfig, 0x03);
    writeReg(kGyroConfig, 0x00);
    writeReg(kAccelConfig, 0x00);
    // Seed the filter from gravity so we don't spend the first second chasing
    // a phantom angle (docs/hardware.md: don't boot from a bad offset).
    double ax, ay, az, gx, gy, gz;
    if (readAll(ax, ay, az, gx, gy, gz)) {
      pitch_ = std::atan2(-ax, std::sqrt(ay * ay + az * az));
    }
    return true;
  }

  // Read one sample and advance the fused pitch. Returns false on a bus error
  // (the caller keeps the previous estimate / stops the motors after a streak).
  bool update(double dt)
  {
    double ax, ay, az, gx, gy, gz;
    if (!readAll(ax, ay, az, gx, gy, gz)) {
      return false;
    }
    const double acc_pitch = std::atan2(-ax, std::sqrt(ay * ay + az * az));
    pitch_ = alpha_ * (pitch_ + gy * dt) + (1.0 - alpha_) * acc_pitch;
    gy_ = gy;
    return true;
  }

  double pitch() const {return pitch_;}        // fused tilt [rad]
  double pitch_rate() const {return gy_;}      // gyro Y [rad/s]

private:
  static constexpr uint8_t kPwrMgmt1 = 0x6B;
  static constexpr uint8_t kSmplrtDiv = 0x19;
  static constexpr uint8_t kConfig = 0x1A;
  static constexpr uint8_t kGyroConfig = 0x1B;
  static constexpr uint8_t kAccelConfig = 0x1C;
  static constexpr uint8_t kAccelXoutH = 0x3B;
  static constexpr double kAccelScale2G = 16384.0;  // LSB / g
  static constexpr double kGyroScale250 = 131.0;    // LSB / (deg/s)
  static constexpr double kGravity = 9.80665;
  static constexpr double kDeg2Rad = 0.017453292519943295;

  TwoWire * wire_ = nullptr;
  uint8_t addr_ = 0x68;
  double alpha_ = 0.98;
  double pitch_ = 0.0;
  double gy_ = 0.0;

  bool writeReg(uint8_t reg, uint8_t val)
  {
    wire_->beginTransmission(addr_);
    wire_->write(reg);
    wire_->write(val);
    return wire_->endTransmission() == 0;
  }

  bool readAll(
    double & ax, double & ay, double & az,
    double & gx, double & gy, double & gz)
  {
    wire_->beginTransmission(addr_);
    wire_->write(kAccelXoutH);
    if (wire_->endTransmission(false) != 0) {     // repeated start, keep bus
      return false;
    }
    if (wire_->requestFrom(addr_, static_cast<uint8_t>(14)) != 14) {
      return false;
    }
    int16_t r[7];
    for (int i = 0; i < 7; ++i) {
      const uint8_t hi = wire_->read();           // read in order: '|' is not
      const uint8_t lo = wire_->read();           // a sequence point
      r[i] = static_cast<int16_t>((hi << 8) | lo);
    }
    ax = r[0] / kAccelScale2G * kGravity;
    ay = r[1] / kAccelScale2G * kGravity;
    az = r[2] / kAccelScale2G * kGravity;
    // r[3] = temperature (ignored)
    gx = r[4] / kGyroScale250 * kDeg2Rad;
    gy = r[5] / kGyroScale250 * kDeg2Rad;
    gz = r[6] / kGyroScale250 * kDeg2Rad;
    return true;
  }
};
