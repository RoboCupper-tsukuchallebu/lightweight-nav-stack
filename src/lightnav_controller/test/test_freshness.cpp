#include <gtest/gtest.h>

#include <limits>

#include "lightnav_controller/freshness.hpp"

using namespace lightnav_controller;

namespace {

struct Case {
  const char* name;
  std::optional<double> pose_age_s;
  std::optional<double> obstacle_age_s;
  bool is_input_stale;
  bool expected_pose_fresh;
  bool expected_obstacle_fresh;
};

}  // namespace

TEST(Freshness, Table) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const Case cases[] = {
      {"both fresh", 0.1, 0.1, false, true, true},
      {"pose at timeout is fresh", 0.5, 0.1, false, true, true},
      {"pose just over timeout", 0.5001, 0.1, false, false, true},
      {"obstacle at timeout is fresh", 0.1, 0.5, false, true, true},
      {"obstacle just over timeout", 0.1, 0.5001, false, true, false},
      {"pose never received", std::nullopt, 0.1, false, false, true},
      {"obstacle never received", 0.1, std::nullopt, false, true, false},
      {"is_input_stale overrides fresh age", 0.1, 0.1, true, true, false},
      {"negative pose age (clock rollback)", -0.1, 0.1, false, false, true},
      {"negative obstacle age", 0.1, -0.1, false, true, false},
      {"nan ages", nan, nan, false, false, false},
      {"zero age", 0.0, 0.0, false, true, true},
  };
  for (const Case& test_case : cases) {
    SCOPED_TRACE(test_case.name);
    InputAges ages;
    ages.pose_age_s = test_case.pose_age_s;
    ages.obstacle_status_age_s = test_case.obstacle_age_s;
    ages.is_obstacle_status_input_stale = test_case.is_input_stale;
    const Freshness freshness = evaluateFreshness(ages, FreshnessTimeouts{0.5, 0.5});
    EXPECT_EQ(freshness.is_pose_fresh, test_case.expected_pose_fresh);
    EXPECT_EQ(freshness.is_obstacle_status_fresh, test_case.expected_obstacle_fresh);
  }
}

TEST(Freshness, TimeoutsAreIndependent) {
  InputAges ages;
  ages.pose_age_s = 0.4;
  ages.obstacle_status_age_s = 0.4;
  const Freshness freshness = evaluateFreshness(ages, FreshnessTimeouts{0.3, 1.0});
  EXPECT_FALSE(freshness.is_pose_fresh);
  EXPECT_TRUE(freshness.is_obstacle_status_fresh);
}
