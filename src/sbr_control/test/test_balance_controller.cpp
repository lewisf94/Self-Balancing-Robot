#include <gtest/gtest.h>

#include <cmath>

#include "sbr_control/balance_controller.hpp"

using sbr_control::BalanceController;

namespace
{
BalanceController make_controller()
{
  BalanceController::Params p;
  p.pitch_gains.kp = 6.0;
  p.pitch_gains.kd = 0.4;
  p.pitch_gains.output_limit = 1.0;
  p.output_scale = 1.0;
  p.fall_threshold = 0.6;
  p.recover_threshold = 0.3;
  return BalanceController(p);
}
}  // namespace

TEST(BalanceController, UprightProducesNearZeroEffort)
{
  auto c = make_controller();
  auto cmd = c.update(0.0, 0.0, 0.0, 0.0, 0.005);
  EXPECT_TRUE(cmd.balancing);
  EXPECT_NEAR(cmd.left_effort, 0.0, 1e-9);
  EXPECT_NEAR(cmd.right_effort, 0.0, 1e-9);
}

TEST(BalanceController, TiltForwardDrivesWheelsForward)
{
  auto c = make_controller();
  // Positive pitch (leaning forward) should command positive (forward) effort
  // to drive the wheels under the centre of mass.
  auto cmd = c.update(0.1, 0.0, 0.0, 0.0, 0.005);
  EXPECT_GT(cmd.left_effort, 0.0);
  EXPECT_GT(cmd.right_effort, 0.0);
}

TEST(BalanceController, TiltSignSymmetry)
{
  auto c = make_controller();
  auto fwd = c.update(0.1, 0.0, 0.0, 0.0, 0.005);
  c.reset();
  auto back = c.update(-0.1, 0.0, 0.0, 0.0, 0.005);
  EXPECT_NEAR(fwd.left_effort, -back.left_effort, 1e-9);
}

TEST(BalanceController, FallenCutsMotors)
{
  auto c = make_controller();
  auto cmd = c.update(1.0, 0.0, 0.0, 0.0, 0.005);  // well past fall_threshold
  EXPECT_FALSE(cmd.balancing);
  EXPECT_DOUBLE_EQ(cmd.left_effort, 0.0);
  EXPECT_DOUBLE_EQ(cmd.right_effort, 0.0);
}

TEST(BalanceController, SteeringIsDifferential)
{
  auto c = make_controller();
  // Pure yaw request while upright: wheels should be equal and opposite.
  auto cmd = c.update(0.0, 0.0, 0.0, 1.0, 0.005);
  EXPECT_GT(cmd.right_effort, cmd.left_effort);
  EXPECT_NEAR(cmd.left_effort, -cmd.right_effort, 1e-9);
}

TEST(BalanceController, OutputScaleApplied)
{
  BalanceController::Params p;
  p.pitch_gains.kp = 100.0;       // force saturation
  p.pitch_gains.output_limit = 1.0;
  p.output_scale = 3.0;
  p.fall_threshold = 0.6;
  BalanceController c(p);
  auto cmd = c.update(0.5, 0.0, 0.0, 0.0, 0.005);
  EXPECT_NEAR(cmd.left_effort, 3.0, 1e-9);   // 1.0 (saturated) * scale
  EXPECT_NEAR(cmd.right_effort, 3.0, 1e-9);
}

TEST(BalanceController, FallLatchesUntilRecoverThreshold)
{
  auto c = make_controller();  // fall 0.6, recover 0.3
  EXPECT_FALSE(c.update(0.7, 0.0, 0.0, 0.0, 0.005).balancing);   // falls
  // Back below fall_threshold but above recover_threshold: still latched.
  EXPECT_FALSE(c.update(0.5, 0.0, 0.0, 0.0, 0.005).balancing);
  // Within recover_threshold: re-arms.
  EXPECT_TRUE(c.update(0.2, 0.0, 0.0, 0.0, 0.005).balancing);
}

TEST(BalanceController, ResetClearsFallLatch)
{
  auto c = make_controller();
  EXPECT_FALSE(c.update(0.7, 0.0, 0.0, 0.0, 0.005).balancing);
  c.reset();
  // 0.5 < fall_threshold and the latch is cleared: balancing again.
  EXPECT_TRUE(c.update(0.5, 0.0, 0.0, 0.0, 0.005).balancing);
}

TEST(BalanceController, LeanPerVelocityShiftsSetpoint)
{
  auto c = make_controller();  // lean_per_velocity default 0.08
  auto cmd = c.update(0.0, 0.0, 1.0, 0.0, 0.005);
  EXPECT_NEAR(cmd.pitch_setpoint, 0.08, 1e-9);
}

TEST(BalanceController, PitchOffsetIsSetpoint)
{
  BalanceController::Params p;
  p.pitch_gains.kp = 6.0;
  p.pitch_gains.output_limit = 1.0;
  p.output_scale = 1.0;
  p.fall_threshold = 0.6;
  p.recover_threshold = 0.3;
  p.pitch_offset = 0.05;
  BalanceController c(p);
  // Sitting exactly at the trimmed setpoint: no corrective effort.
  auto cmd = c.update(0.05, 0.0, 0.0, 0.0, 0.005);
  EXPECT_NEAR(cmd.pitch_setpoint, 0.05, 1e-9);
  EXPECT_NEAR(cmd.left_effort, 0.0, 1e-9);
}

TEST(BalanceController, KdOpposesPitchRate)
{
  auto c = make_controller();
  // Upright but pitching forward fast: drive forward to get under the fall.
  auto cmd = c.update(0.0, 1.0, 0.0, 0.0, 0.005);
  EXPECT_GT(cmd.left_effort, 0.0);
}

TEST(BalanceController, KiIntegratesSmallError)
{
  BalanceController::Params p;
  p.pitch_gains.kp = 0.1;
  p.pitch_gains.ki = 0.5;
  p.pitch_gains.integral_limit = 1.0;
  p.pitch_gains.output_limit = 1.0;
  p.output_scale = 1.0;
  p.fall_threshold = 0.6;
  p.recover_threshold = 0.3;
  BalanceController c(p);
  const double first = c.update(0.1, 0.0, 0.0, 0.0, 0.01).left_effort;
  double last = first;
  for (int i = 0; i < 49; ++i) {
    last = c.update(0.1, 0.0, 0.0, 0.0, 0.01).left_effort;
  }
  EXPECT_GT(std::fabs(last), std::fabs(first));  // integral builds
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
