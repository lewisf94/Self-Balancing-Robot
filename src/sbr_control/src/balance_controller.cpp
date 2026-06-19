#include "sbr_control/balance_controller.hpp"

#include <algorithm>
#include <cmath>

namespace sbr_control
{

namespace
{
double clamp(double v, double lo, double hi)
{
  return std::max(lo, std::min(v, hi));
}
}  // namespace

BalanceController::BalanceController(const Params & params)
: params_(params)
{
  pitch_pid_.set_gains(params_.pitch_gains);
}

void BalanceController::set_params(const Params & params)
{
  params_ = params;
  pitch_pid_.set_gains(params_.pitch_gains);
}

void BalanceController::reset() {pitch_pid_.reset();}

BalanceController::Command BalanceController::update(
  double pitch, double pitch_rate, double linear_cmd, double angular_cmd, double dt)
{
  Command cmd;
  cmd.pitch_setpoint = params_.pitch_offset;

  // Safety: if the robot has fallen, cut the motors and reset the integrator.
  if (std::fabs(pitch) > params_.fall_threshold) {
    pitch_pid_.reset();
    return cmd;
  }

  // Outer (open-loop) drive: lean the setpoint to accelerate forward / back.
  const double setpoint = params_.pitch_offset + linear_cmd * params_.lean_per_velocity;
  cmd.pitch_setpoint = setpoint;

  // Inner balance loop. Output is normalized to roughly [-1, 1].
  const double effort = pitch_pid_.update(setpoint, pitch, pitch_rate, dt);

  // Steering: differential effort between the wheels. A positive yaw request
  // (CCW / turn left) drives the right wheel harder than the left.
  const double steer = angular_cmd * params_.steer_gain;

  cmd.left_effort = clamp(effort - steer, -1.0, 1.0) * params_.output_scale;
  cmd.right_effort = clamp(effort + steer, -1.0, 1.0) * params_.output_scale;
  cmd.balancing = true;
  return cmd;
}

}  // namespace sbr_control
