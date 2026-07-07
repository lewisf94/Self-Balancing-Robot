#include "sbr_control/pid.hpp"

#include <algorithm>

namespace sbr_control
{

namespace
{
double clamp(double v, double lo, double hi)
{
  return std::max(lo, std::min(v, hi));
}
}  // namespace

Pid::Pid(const Gains & gains)
: gains_(gains) {}

void Pid::set_gains(const Gains & gains) {gains_ = gains;}

void Pid::reset() {integral_ = 0.0;}

double Pid::update(double setpoint, double measurement, double derivative, double dt)
{
  const double error = setpoint - measurement;

  // Derivative-on-measurement: d(error)/dt = -d(measurement)/dt for a
  // (locally) constant setpoint.
  const double d_term = gains_.kd * (-derivative);

  // Tentative integral with clamping. Non-positive dt (clock glitch, first
  // sample) must not corrupt the accumulator; P and D are dt-independent.
  double integral = integral_;
  if (dt > 0.0) {
    integral += error * dt;
  }
  integral = clamp(integral, -gains_.integral_limit, gains_.integral_limit);

  const double output = gains_.kp * error + gains_.ki * integral + d_term;
  const double saturated = clamp(output, -gains_.output_limit, gains_.output_limit);

  // Conditional-integration anti-windup: only commit the integral when we are
  // not saturating further in the same direction as the error.
  const bool winding_up = (output != saturated) && ((output - saturated) * error > 0.0);
  if (!winding_up) {
    integral_ = integral;
  }

  return saturated;
}

}  // namespace sbr_control
