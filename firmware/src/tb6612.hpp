#pragma once
//
// TB6612FNG dual H-bridge driver using the ESP32 LEDC PWM peripheral.
//
// Effort semantics match src/sbr_drivers/sbr_drivers/motor_node.py:
//   value >= 0 -> IN1 HIGH, IN2 LOW ;  value < 0 -> IN1 LOW, IN2 HIGH
//   duty = clamp(|value|, 0, 1) * max_duty ;  |value| < deadband -> 0
// Motor A drives the LEFT wheel, Motor B the RIGHT (docs/hardware.md wiring).
//
// NOTE: uses the arduino-esp32 v2.x LEDC API (ledcSetup / ledcAttachPin /
// ledcWrite(channel, duty)). On arduino-esp32 v3.x switch to
// ledcAttach(pin, freq, res) and ledcWrite(pin, duty). platformio.ini pins the
// platform to the v2.x line.
//
#include <Arduino.h>
#include <cmath>

#include "sbr_config.hpp"

class Tb6612
{
public:
  void begin()
  {
    pinMode(sbr_cfg::kPinAIN1, OUTPUT);
    pinMode(sbr_cfg::kPinAIN2, OUTPUT);
    pinMode(sbr_cfg::kPinBIN1, OUTPUT);
    pinMode(sbr_cfg::kPinBIN2, OUTPUT);
    pinMode(sbr_cfg::kPinSTBY, OUTPUT);
    ledcSetup(kChA, sbr_cfg::kPwmFreqHz, sbr_cfg::kPwmResBits);
    ledcSetup(kChB, sbr_cfg::kPwmFreqHz, sbr_cfg::kPwmResBits);
    ledcAttachPin(sbr_cfg::kPinPWMA, kChA);
    ledcAttachPin(sbr_cfg::kPinPWMB, kChB);
    stop();
  }

  // left / right in [-1, 1] (already scaled by output_scale upstream).
  void set(double left, double right)
  {
    digitalWrite(sbr_cfg::kPinSTBY, HIGH);   // (re)enable the bridge
    drive(sbr_cfg::kPinAIN1, sbr_cfg::kPinAIN2, kChA, left, sbr_cfg::kInvertLeft);
    drive(sbr_cfg::kPinBIN1, sbr_cfg::kPinBIN2, kChB, right, sbr_cfg::kInvertRight);
  }

  // Coast: zero PWM but leave the bridge enabled (used when fallen / stale cmd).
  void stop()
  {
    digitalWrite(sbr_cfg::kPinSTBY, HIGH);
    ledcWrite(kChA, 0);
    ledcWrite(kChB, 0);
  }

  // Hard cut: zero PWM and pull STBY LOW (used on low battery).
  void disable()
  {
    ledcWrite(kChA, 0);
    ledcWrite(kChB, 0);
    digitalWrite(sbr_cfg::kPinSTBY, LOW);
  }

private:
  static constexpr int kChA = 0;            // LEDC channel for Motor A (left)
  static constexpr int kChB = 1;            // LEDC channel for Motor B (right)
  static constexpr int kDutyMax = (1 << sbr_cfg::kPwmResBits) - 1;

  void drive(int in1, int in2, int channel, double value, bool invert)
  {
    if (invert) {
      value = -value;
    }
    if (value >= 0.0) {
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
    } else {
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
    }
    double mag = std::fabs(value);
    if (mag < sbr_cfg::kDeadband) {
      mag = 0.0;
    }
    if (mag > 1.0) {
      mag = 1.0;
    }
    const uint32_t duty = static_cast<uint32_t>(mag * sbr_cfg::kMaxDuty * kDutyMax);
    ledcWrite(channel, duty);
  }
};
