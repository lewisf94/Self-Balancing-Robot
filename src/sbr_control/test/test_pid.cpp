#include <gtest/gtest.h>

#include <cmath>

#include "sbr_control/pid.hpp"

using sbr_control::Pid;

namespace
{
Pid::Gains gains(double kp, double ki, double kd,
                 double integral_limit = 1.0, double output_limit = 1.0)
{
  Pid::Gains g;
  g.kp = kp;
  g.ki = ki;
  g.kd = kd;
  g.integral_limit = integral_limit;
  g.output_limit = output_limit;
  return g;
}
}  // namespace

TEST(Pid, ProportionalOnly)
{
  Pid pid(gains(2.0, 0.0, 0.0, 1.0, 10.0));
  EXPECT_NEAR(pid.update(1.0, 0.0, 0.0, 0.01), 2.0, 1e-9);
}

TEST(Pid, IntegralAccumulates)
{
  Pid pid(gains(0.0, 1.0, 0.0, 10.0, 10.0));
  EXPECT_NEAR(pid.update(1.0, 0.0, 0.0, 0.5), 0.5, 1e-9);
  EXPECT_NEAR(pid.update(1.0, 0.0, 0.0, 0.5), 1.0, 1e-9);
}

TEST(Pid, IntegralClamped)
{
  Pid pid(gains(0.0, 1.0, 0.0, 0.1, 10.0));
  for (int i = 0; i < 100; ++i) {
    pid.update(1.0, 0.0, 0.0, 0.1);
  }
  // integral clamped at 0.1 -> output <= ki * 0.1
  EXPECT_LE(pid.update(1.0, 0.0, 0.0, 0.1), 0.1 + 1e-9);
}

TEST(Pid, AntiWindupConditionalIntegration)
{
  // Saturate hard with kp while ki tries to wind up; the conditional
  // integration must keep the integral from accumulating, so when the error
  // flips sign the output must leave saturation immediately.
  Pid pid(gains(100.0, 1.0, 0.0, 10.0, 1.0));
  for (int i = 0; i < 200; ++i) {
    EXPECT_NEAR(pid.update(1.0, 0.0, 0.0, 0.01), 1.0, 1e-9);  // pinned at limit
  }
  const double after_flip = pid.update(-1.0, 0.0, 0.0, 0.01);
  Pid fresh(gains(100.0, 1.0, 0.0, 10.0, 1.0));
  const double fresh_flip = fresh.update(-1.0, 0.0, 0.0, 0.01);
  EXPECT_NEAR(after_flip, fresh_flip, 1e-6);  // no wound-up integral lag
  EXPECT_NEAR(after_flip, -1.0, 1e-9);        // straight to the other rail
}

TEST(Pid, DerivativeOnMeasurementSign)
{
  // d(error)/dt = -d(measurement)/dt: a rising measurement must push the
  // output down.
  Pid pid(gains(0.0, 0.0, 0.4, 1.0, 10.0));
  EXPECT_NEAR(pid.update(0.0, 0.0, 1.0, 0.01), -0.4, 1e-9);
}

TEST(Pid, OutputSaturatesAtLimit)
{
  Pid pid(gains(100.0, 0.0, 0.0, 1.0, 1.0));
  EXPECT_NEAR(pid.update(1.0, 0.0, 0.0, 0.01), 1.0, 1e-9);
  EXPECT_NEAR(pid.update(-1.0, 0.0, 0.0, 0.01), -1.0, 1e-9);
}

TEST(Pid, ResetClearsIntegral)
{
  Pid pid(gains(0.0, 1.0, 0.0, 10.0, 10.0));
  pid.update(1.0, 0.0, 0.0, 1.0);              // integral = 1.0
  pid.reset();
  EXPECT_NEAR(pid.update(1.0, 0.0, 0.0, 0.5), 0.5, 1e-9);  // starts from zero
}

TEST(Pid, NonPositiveDtSafe)
{
  Pid pid(gains(2.0, 1.0, 0.0, 10.0, 10.0));
  const double at_zero = pid.update(1.0, 0.0, 0.0, 0.0);
  EXPECT_TRUE(std::isfinite(at_zero));
  EXPECT_NEAR(at_zero, 2.0, 1e-9);             // pure P; integral untouched
  const double at_negative = pid.update(1.0, 0.0, 0.0, -0.01);
  EXPECT_NEAR(at_negative, 2.0, 1e-9);
  // Integral must still be zero: a normal step now behaves like the first.
  EXPECT_NEAR(pid.update(1.0, 0.0, 0.0, 0.5), 2.0 + 0.5, 1e-9);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
