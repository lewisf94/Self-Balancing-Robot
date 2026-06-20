#include <gtest/gtest.h>

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

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
