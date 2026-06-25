#pragma once
//
// Central configuration for the ESP32-S3 balancing firmware.
//
// Pin map comes from docs/hardware.md (the "ESP32-S3 (primary — firmware)"
// table). Control gains mirror sbr_bringup/config/sbr_params.yaml so the
// firmware and the ROS 2 / Gazebo node behave identically. The IMU fusion and
// motor semantics mirror src/sbr_drivers (imu_node.py / motor_node.py).
//
// Re-confirm pins against YOUR exact S3 board's strap / USB pins before
// flashing — putting PWM on a strap pin can stop the board booting.
//
#include <cstdint>

#include "sbr_control/balance_controller.hpp"  // reused via firmware/lib/sbr_control

namespace sbr_cfg
{

// ---- Control loop -----------------------------------------------------------
constexpr double kLoopRateHz  = 200.0;   // balance_controller_node: loop_rate
constexpr double kCmdTimeoutS = 0.5;     // drop stale /cmd_vel (motor watchdog)
constexpr bool   kInvertPitch = false;   // flip if the robot drives into its fall
constexpr int    kTelemetryDivider = 4;  // publish balance_state at loop/4 (~50 Hz)
constexpr uint32_t kTelemetryPeriodMs =  // derived: ~50 Hz telemetry on core 1
  static_cast<uint32_t>(1000.0 / (kLoopRateHz / kTelemetryDivider));

// ---- IMU: MPU-6050 on its own I2C bus ---------------------------------------
constexpr int      kImuSdaPin = 32;
constexpr int      kImuSclPin = 33;
constexpr uint8_t  kImuAddress = 0x68;
constexpr uint32_t kI2cHz = 400000;            // 400 kHz (docs/hardware.md)
constexpr double   kComplementaryAlpha = 0.98; // imu_node: complementary_alpha

// ---- Motors: TB6612FNG (Motor A = LEFT wheel, Motor B = RIGHT wheel) ---------
constexpr int kPinAIN1 = 18;
constexpr int kPinAIN2 = 17;
constexpr int kPinPWMA = 16;
constexpr int kPinBIN1 = 21;
constexpr int kPinBIN2 = 22;
constexpr int kPinPWMB = 23;
constexpr int kPinSTBY = 19;               // HIGH = bridge enabled
constexpr int kPinLBO  = 34;               // low-battery, input-only, ext. pull-up

constexpr int    kPwmFreqHz  = 20000;      // 20 kHz: above hearing (docs/hardware.md)
constexpr int    kPwmResBits = 10;         // LEDC duty resolution -> 0..1023
constexpr double kMaxDuty    = 1.0;        // motor_node: max_duty
constexpr double kDeadband   = 0.02;       // motor_node: deadband
constexpr bool   kInvertLeft  = false;     // swap a wheel's direction in software
constexpr bool   kInvertRight = false;
constexpr bool   kHasLowBatteryPin = true; // set false if PowerBoost LBO isn't wired

// ---- Balance gains (mirror sbr_params.yaml: balance_controller_node) ---------
inline sbr_control::BalanceController::Params make_params()
{
  sbr_control::BalanceController::Params p;
  p.pitch_gains.kp = 6.0;
  p.pitch_gains.ki = 0.0;
  p.pitch_gains.kd = 0.4;
  p.pitch_gains.integral_limit = 0.5;
  p.pitch_gains.output_limit = 1.0;
  p.pitch_offset = 0.0;        // trim: the chassis' balanced tilt [rad]
  p.output_scale = 1.0;        // hardware: [-1, 1] maps straight to motor duty
  p.fall_threshold = 0.78;     // ~45 deg tip-kill
  p.lean_per_velocity = 0.08;
  p.steer_gain = 0.4;
  return p;
}

}  // namespace sbr_cfg
