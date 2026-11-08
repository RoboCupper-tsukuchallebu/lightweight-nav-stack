#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "lightnav_safety_gate/safety_gate_logic.hpp"

using namespace lightnav_safety_gate;  // NOLINT

namespace
{

constexpr double kPeriodSeconds = 0.05;
const double kNaN = std::numeric_limits<double>::quiet_NaN();
const double kInfinity = std::numeric_limits<double>::infinity();

SafetyGateLimits defaultLimits()
{
  return {0.8, 0.6, 0.5, 1.0, 1.0, 2.0};
}

EmergencyStopTimeouts defaultTimeouts()
{
  return {0.3, 0.5};
}

EmergencyStopInputs healthyInputs()
{
  EmergencyStopInputs inputs;
  inputs.is_active = true;
  inputs.has_obstacle_status = true;
  inputs.obstacle_status_age_s = 0.1;
  inputs.has_command = true;
  inputs.command_age_s = 0.1;
  return inputs;
}

}  // namespace

TEST(EmergencyStop, healthyInputsDoNotStop)
{
  EXPECT_FALSE(shouldEmergencyStop(healthyInputs(), defaultTimeouts()));
}

// Each priority-0 condition alone must stop; table of single-fault cases.
TEST(EmergencyStop, everySingleFaultStops)
{
  struct Case
  {
    const char * name;
    void (* apply)(EmergencyStopInputs &);
  };
  const Case cases[] = {
    {"not active", [](EmergencyStopInputs & inputs) {inputs.is_active = false;}},
    {"no obstacle status yet", [](EmergencyStopInputs & inputs) {inputs.has_obstacle_status = false;}},
    {"obstacle status timed out", [](EmergencyStopInputs & inputs) {inputs.obstacle_status_age_s = 0.6;}},
    {"obstacle status age NaN", [](EmergencyStopInputs & inputs) {inputs.obstacle_status_age_s = kNaN;}},
    {"obstacle status age negative", [](EmergencyStopInputs & inputs) {inputs.obstacle_status_age_s = -1.0;}},
    {"stop zone occupied", [](EmergencyStopInputs & inputs) {inputs.is_stop_zone_occupied = true;}},
    {"input stale", [](EmergencyStopInputs & inputs) {inputs.is_input_stale = true;}},
    {"no command yet", [](EmergencyStopInputs & inputs) {inputs.has_command = false;}},
    {"command timed out", [](EmergencyStopInputs & inputs) {inputs.command_age_s = 0.4;}},
    {"command age NaN", [](EmergencyStopInputs & inputs) {inputs.command_age_s = kNaN;}},
    {"command age infinite", [](EmergencyStopInputs & inputs) {inputs.command_age_s = kInfinity;}},
    {"hard stop", [](EmergencyStopInputs & inputs) {inputs.is_hard_stop = true;}},
  };
  for (const Case & test_case : cases) {
    EmergencyStopInputs inputs = healthyInputs();
    test_case.apply(inputs);
    EXPECT_TRUE(shouldEmergencyStop(inputs, defaultTimeouts())) << test_case.name;
  }
}

// All 2^8 combinations of stop conditions: stop unless every one is benign.
// The age bits exceed the default timeouts (obstacle 0.5 s, command 0.3 s).
TEST(EmergencyStop, allConditionCombinationsStopUnlessAllBenign)
{
  constexpr int kNotActiveBit = 1 << 0;
  constexpr int kNoObstacleStatusBit = 1 << 1;
  constexpr int kObstacleStatusTimedOutBit = 1 << 2;
  constexpr int kStopZoneOccupiedBit = 1 << 3;
  constexpr int kInputStaleBit = 1 << 4;
  constexpr int kNoCommandBit = 1 << 5;
  constexpr int kCommandTimedOutBit = 1 << 6;
  constexpr int kHardStopBit = 1 << 7;
  constexpr int kCombinationCount = 1 << 8;
  constexpr double kFreshAgeSeconds = 0.1;
  constexpr double kStaleObstacleAgeSeconds = 0.6;
  constexpr double kStaleCommandAgeSeconds = 0.4;

  for (int bits = 0; bits < kCombinationCount; ++bits) {
    EmergencyStopInputs inputs = healthyInputs();
    if ((bits & kNotActiveBit) != 0) {
      inputs.is_active = false;
    }
    if ((bits & kNoObstacleStatusBit) != 0) {
      inputs.has_obstacle_status = false;
    }
    inputs.obstacle_status_age_s =
      ((bits & kObstacleStatusTimedOutBit) != 0) ? kStaleObstacleAgeSeconds : kFreshAgeSeconds;
    if ((bits & kStopZoneOccupiedBit) != 0) {
      inputs.is_stop_zone_occupied = true;
    }
    if ((bits & kInputStaleBit) != 0) {
      inputs.is_input_stale = true;
    }
    if ((bits & kNoCommandBit) != 0) {
      inputs.has_command = false;
    }
    inputs.command_age_s =
      ((bits & kCommandTimedOutBit) != 0) ? kStaleCommandAgeSeconds : kFreshAgeSeconds;
    if ((bits & kHardStopBit) != 0) {
      inputs.is_hard_stop = true;
    }
    EXPECT_EQ(shouldEmergencyStop(inputs, defaultTimeouts()), bits != 0) << "bits=" << bits;
  }
}

TEST(EmergencyStop, ageExactlyAtTimeoutIsStillFresh)
{
  EmergencyStopInputs inputs = healthyInputs();
  inputs.command_age_s = 0.3;
  inputs.obstacle_status_age_s = 0.5;
  EXPECT_FALSE(shouldEmergencyStop(inputs, defaultTimeouts()));
}

TEST(Finite, rejectsNonFiniteComponents)
{
  EXPECT_TRUE(isFinite({0.1, -0.2}));
  EXPECT_FALSE(isFinite({kNaN, 0.0}));
  EXPECT_FALSE(isFinite({0.0, kNaN}));
  EXPECT_FALSE(isFinite({kInfinity, 0.0}));
  EXPECT_FALSE(isFinite({0.0, -kInfinity}));
}

TEST(Saturate, clampsBothSigns)
{
  const PlanarVelocity result = saturate({5.0, -5.0}, defaultLimits());
  EXPECT_DOUBLE_EQ(result.linear_mps, 0.8);
  EXPECT_DOUBLE_EQ(result.angular_rps, -0.6);
  const PlanarVelocity inside = saturate({0.3, 0.2}, defaultLimits());
  EXPECT_DOUBLE_EQ(inside.linear_mps, 0.3);
  EXPECT_DOUBLE_EQ(inside.angular_rps, 0.2);
}

TEST(LimitRate, zeroTargetBypassesRateLimit)
{
  const PlanarVelocity result = limitRate({0.8, 0.6}, {0.0, 0.0}, defaultLimits(), kPeriodSeconds);
  EXPECT_EQ(result.linear_mps, 0.0);
  EXPECT_EQ(result.angular_rps, 0.0);
}

TEST(LimitRate, accelerationIsLimitedPerPeriod)
{
  const PlanarVelocity result = limitRate({0.0, 0.0}, {0.6, 0.4}, defaultLimits(), kPeriodSeconds);
  EXPECT_NEAR(result.linear_mps, 0.5 * kPeriodSeconds, 1e-12);
  EXPECT_NEAR(result.angular_rps, 1.0 * kPeriodSeconds, 1e-12);
}

TEST(LimitRate, decelerationUsesDecelerationLimit)
{
  const PlanarVelocity result = limitRate({0.6, 0.4}, {0.2, 0.1}, defaultLimits(), kPeriodSeconds);
  EXPECT_NEAR(result.linear_mps, 0.6 - 1.0 * kPeriodSeconds, 1e-12);
  EXPECT_NEAR(result.angular_rps, 0.4 - 2.0 * kPeriodSeconds, 1e-12);
}

TEST(LimitRate, smallStepReachesTargetWithoutOvershoot)
{
  const PlanarVelocity result =
    limitRate({0.10, 0.0}, {0.11, 0.0}, defaultLimits(), kPeriodSeconds);
  EXPECT_DOUBLE_EQ(result.linear_mps, 0.11);
}

TEST(LimitRate, negativeDirectionAcceleratesSymmetrically)
{
  const PlanarVelocity result = limitRate({0.0, 0.0}, {-0.6, -0.4}, defaultLimits(), kPeriodSeconds);
  EXPECT_NEAR(result.linear_mps, -0.5 * kPeriodSeconds, 1e-12);
  EXPECT_NEAR(result.angular_rps, -1.0 * kPeriodSeconds, 1e-12);
}

TEST(LimitRate, signReversalDeceleratesTowardZeroFirst)
{
  const PlanarVelocity result = limitRate({0.3, 0.3}, {-0.5, -0.5}, defaultLimits(), kPeriodSeconds);
  EXPECT_NEAR(result.linear_mps, 0.3 - 1.0 * kPeriodSeconds, 1e-12);
  EXPECT_NEAR(result.angular_rps, 0.3 - 2.0 * kPeriodSeconds, 1e-12);
}

TEST(LimitRate, signReversalNearZeroDoesNotCrossZeroInOneStep)
{
  const PlanarVelocity result = limitRate({0.01, 0.0}, {-0.5, 0.0}, defaultLimits(), kPeriodSeconds);
  EXPECT_DOUBLE_EQ(result.linear_mps, 0.0);
}

TEST(LimitRate, rampReachesTargetInExpectedSteps)
{
  PlanarVelocity output{0.0, 0.0};
  for (int step = 0; step < 24; ++step) {
    output = limitRate(output, {0.6, 0.0}, defaultLimits(), kPeriodSeconds);
  }
  EXPECT_NEAR(output.linear_mps, 0.6, 1e-9);  // 0.6 / (0.5 * 0.05) = 24 steps
}

TEST(Limits, validationRejectsNonPositiveAndNonFinite)
{
  EXPECT_TRUE(areLimitsValid(defaultLimits()));
  SafetyGateLimits limits = defaultLimits();
  limits.linear_max_mps = 0.0;
  EXPECT_FALSE(areLimitsValid(limits));
  limits = defaultLimits();
  limits.angular_deceleration_max_rps2 = kNaN;
  EXPECT_FALSE(areLimitsValid(limits));
  limits = defaultLimits();
  limits.acceleration_max_mps2 = -1.0;
  EXPECT_FALSE(areLimitsValid(limits));
  limits = defaultLimits();
  limits.angular_max_rps = kInfinity;
  EXPECT_FALSE(areLimitsValid(limits));
}

TEST(LimitRate, oneZeroAxisIsStillRateLimited)
{
  // Transition to rotate in place: linear target 0, angular target non-zero.
  const PlanarVelocity result = limitRate({0.6, 0.0}, {0.0, 0.3}, defaultLimits(), kPeriodSeconds);
  EXPECT_NEAR(result.linear_mps, 0.6 - 1.0 * kPeriodSeconds, 1e-12);
  EXPECT_NEAR(result.angular_rps, 1.0 * kPeriodSeconds, 1e-12);
  const PlanarVelocity opposite = limitRate({0.0, 0.4}, {0.5, 0.0}, defaultLimits(), kPeriodSeconds);
  EXPECT_NEAR(opposite.angular_rps, 0.4 - 2.0 * kPeriodSeconds, 1e-12);
}

TEST(LimitRate, rampDoesNotReachTargetOneStepEarly)
{
  PlanarVelocity output{0.0, 0.0};
  for (int step = 0; step < 23; ++step) {
    output = limitRate(output, {0.6, 0.0}, defaultLimits(), kPeriodSeconds);
  }
  EXPECT_LT(output.linear_mps, 0.6 - 1e-6);
}

TEST(EmergencyStop, negativeAgesStop)
{
  EmergencyStopInputs inputs = healthyInputs();
  inputs.command_age_s = -0.001;
  EXPECT_TRUE(shouldEmergencyStop(inputs, defaultTimeouts()));
}
