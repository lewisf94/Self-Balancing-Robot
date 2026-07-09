#ifndef SBR_CONTROL__BALANCE_CONTROLLER_HPP_
#define SBR_CONTROL__BALANCE_CONTROLLER_HPP_

#include "sbr_control/pid.hpp"

namespace sbr_control
{

/// Pure (ROS-free) balancing logic for a two-wheeled inverted-pendulum robot.
///
/// The chassis is kept upright by a PID loop on the pitch angle that uses the
/// gyro pitch rate for its derivative term. Forward / turn requests bias the
/// pitch setpoint and add a differential term between the wheels.
///
/// This robot has no wheel encoders, so there is no closed-loop velocity or
/// position control: motion commands are open-loop and the robot will drift in
/// position. See docs/control_tuning.md. Keeping this class free of ROS makes
/// it unit-testable and portable to a microcontroller (e.g. via micro-ROS).
class BalanceController
{
public:
  struct Params
  {
    Pid::Gains pitch_gains{};
    double pitch_offset{0.0};        ///< trim added to the setpoint [rad]
    double output_scale{1.0};        ///< maps normalized effort -> output units
    double fall_threshold{0.78};     ///< |pitch| beyond this cuts the motors [rad] (~45 deg)
    double recover_threshold{0.4};   ///< |pitch| must drop below this to re-arm after a fall [rad]
    double lean_per_velocity{0.08};  ///< setpoint lean per (m/s) forward request
    double steer_gain{0.4};          ///< differential effort per (rad/s) yaw request
  };

  struct Command
  {
    double left_effort{0.0};     ///< normalized [-1, 1] x output_scale
    double right_effort{0.0};
    double pitch_setpoint{0.0};  ///< setpoint used this cycle [rad]
    bool balancing{false};       ///< false when fallen (motors cut)
  };

  BalanceController() = default;
  explicit BalanceController(const Params & params);

  void set_params(const Params & params);
  const Params & params() const {return params_;}

  /// Compute wheel efforts for one control step.
  /// \param pitch       estimated tilt angle [rad] (0 = upright)
  /// \param pitch_rate  tilt angular velocity [rad/s]
  /// \param linear_cmd  forward velocity request [m/s]
  /// \param angular_cmd yaw rate request [rad/s]
  /// \param dt          timestep [s]
  Command update(
    double pitch, double pitch_rate,
    double linear_cmd, double angular_cmd, double dt);

  void reset();

private:
  Params params_{};
  Pid pitch_pid_{};
  bool fallen_{false};   ///< tip-kill latch; clears below recover_threshold
};

}  // namespace sbr_control

#endif  // SBR_CONTROL__BALANCE_CONTROLLER_HPP_
