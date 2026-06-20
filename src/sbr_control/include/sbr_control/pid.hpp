#ifndef SBR_CONTROL__PID_HPP_
#define SBR_CONTROL__PID_HPP_

namespace sbr_control
{

/// Minimal PID controller with integral clamping and output saturation.
///
/// The derivative term is supplied externally (e.g. a gyro rate) rather than
/// computed by numerically differentiating the measurement. For a balancing
/// robot the gyro gives a clean rate signal, which avoids amplifying sensor
/// noise.
class Pid
{
public:
  struct Gains
  {
    double kp{0.0};
    double ki{0.0};
    double kd{0.0};
    double integral_limit{1.0};  ///< |integral| is clamped to this
    double output_limit{1.0};    ///< |output| is clamped to this
  };

  Pid() = default;
  explicit Pid(const Gains & gains);

  void set_gains(const Gains & gains);
  const Gains & gains() const {return gains_;}

  /// Run one PID step.
  /// \param setpoint    desired value
  /// \param measurement current value
  /// \param derivative  d(measurement)/dt (e.g. gyro rate)
  /// \param dt          timestep in seconds (must be > 0)
  /// \return saturated control output
  double update(double setpoint, double measurement, double derivative, double dt);

  void reset();

private:
  Gains gains_{};
  double integral_{0.0};
};

}  // namespace sbr_control

#endif  // SBR_CONTROL__PID_HPP_
