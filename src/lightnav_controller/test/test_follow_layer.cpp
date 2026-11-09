#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <vector>

#include "lightnav_controller/arbitrate.hpp"
#include "lightnav_controller/avoid_layer.hpp"
#include "lightnav_controller/follow_layer.hpp"

using namespace lightnav_controller;

namespace {

std::vector<Point2D> straightPath(double length_m, double step_m = 0.1) {
  std::vector<Point2D> points;
  for (double x_m = 0.0; x_m <= length_m + 1e-9; x_m += step_m) {
    points.push_back(Point2D{x_m, 0.0});
  }
  return points;
}

FollowInputs freshInputs() {
  FollowInputs inputs;
  inputs.is_obstacle_status_fresh = true;
  return inputs;
}

}  // namespace

TEST(PathAcceptance, SameSessionOnlyAcceptsNonDecreasingPathId) {
  FollowLayer layer{FollowParameters{}};
  EXPECT_TRUE(layer.tryAcceptPath(7, 2, straightPath(5.0)));
  EXPECT_FALSE(layer.tryAcceptPath(7, 1, straightPath(5.0)));
  EXPECT_TRUE(layer.tryAcceptPath(7, 2, straightPath(5.0)));
  EXPECT_TRUE(layer.tryAcceptPath(7, 3, straightPath(5.0)));
  EXPECT_EQ(layer.pathId(), 3u);
}

TEST(PathAcceptance, DifferentSessionAlwaysReplaces) {
  FollowLayer layer{FollowParameters{}};
  layer.tryAcceptPath(7, 9, straightPath(5.0));
  EXPECT_TRUE(layer.tryAcceptPath(8, 1, straightPath(5.0)));
  EXPECT_EQ(layer.sessionId(), 8u);
  EXPECT_EQ(layer.pathId(), 1u);
}

TEST(PathAcceptance, ZeroSessionIsRejected) {
  FollowLayer layer{FollowParameters{}};
  EXPECT_FALSE(layer.tryAcceptPath(0, 1, straightPath(5.0)));
}

TEST(FollowLayer, NoPathStops) {
  FollowLayer layer{FollowParameters{}};
  const FollowResult result = layer.compute(Pose2D{}, freshInputs());
  EXPECT_FALSE(result.output.is_asserted);
  EXPECT_EQ(result.output.reason, StopReason::kNoPath);
}

TEST(FollowLayer, EmptyPathMeansStop) {
  FollowLayer layer{FollowParameters{}};
  layer.tryAcceptPath(1, 1, {});
  EXPECT_EQ(layer.compute(Pose2D{}, freshInputs()).output.reason, StopReason::kNoPath);
}

TEST(FollowLayer, StraightPathDrivesAtMaxSpeedWithoutTurning) {
  FollowLayer layer{FollowParameters{}};
  layer.tryAcceptPath(1, 1, straightPath(20.0));
  const FollowResult result = layer.compute(Pose2D{0.0, 0.0, 0.0}, freshInputs());
  ASSERT_TRUE(result.output.is_asserted);
  EXPECT_NEAR(result.output.twist.linear_mps, 0.6, 1e-9);
  EXPECT_NEAR(result.output.twist.angular_rps, 0.0, 1e-9);
}

TEST(FollowLayer, LateralOffsetTurnsTowardPath) {
  FollowLayer layer{FollowParameters{}};
  layer.tryAcceptPath(1, 1, straightPath(20.0));
  // Robot is left of the path (y>0), facing +x: must turn right (negative).
  const FollowResult result = layer.compute(Pose2D{0.0, 0.5, 0.0}, freshInputs());
  ASSERT_TRUE(result.output.is_asserted);
  EXPECT_LT(result.output.twist.angular_rps, 0.0);
  EXPECT_LE(std::fabs(result.output.twist.angular_rps), 0.4 + 1e-9);
}

TEST(FollowLayer, LargeHeadingErrorRotatesInPlace) {
  FollowLayer layer{FollowParameters{}};
  layer.tryAcceptPath(1, 1, straightPath(20.0));
  const FollowResult result = layer.compute(Pose2D{0.0, 0.0, 3.0}, freshInputs());
  ASSERT_TRUE(result.output.is_asserted);
  EXPECT_DOUBLE_EQ(result.output.twist.linear_mps, 0.0);
  EXPECT_DOUBLE_EQ(std::fabs(result.output.twist.angular_rps), 0.3);
}

TEST(FollowLayer, SlowZoneCapsSpeed) {
  FollowLayer layer{FollowParameters{}};
  layer.tryAcceptPath(1, 1, straightPath(20.0));
  FollowInputs inputs = freshInputs();
  inputs.is_slow_zone_occupied = true;
  EXPECT_DOUBLE_EQ(layer.compute(Pose2D{}, inputs).output.twist.linear_mps, 0.25);
}

TEST(FollowLayer, StopZoneAndStaleObstacleStatusStop) {
  FollowLayer layer{FollowParameters{}};
  layer.tryAcceptPath(1, 1, straightPath(20.0));
  FollowInputs blocked = freshInputs();
  blocked.is_stop_zone_occupied = true;
  EXPECT_EQ(layer.compute(Pose2D{}, blocked).output.reason, StopReason::kBlocked);
  EXPECT_EQ(layer.compute(Pose2D{}, FollowInputs{}).output.reason, StopReason::kObstacleStale);
}

TEST(FollowLayer, SlowsNearEndAndStopsWithinGoalTolerance) {
  FollowLayer layer{FollowParameters{}};
  layer.tryAcceptPath(1, 1, straightPath(10.0));
  layer.compute(Pose2D{5.0, 0.0, 0.0}, freshInputs());  // advance within search_window_m
  const FollowResult near_end = layer.compute(Pose2D{9.0, 0.0, 0.0}, freshInputs());
  ASSERT_TRUE(near_end.output.is_asserted);
  EXPECT_NEAR(near_end.output.twist.linear_mps, 0.5 * near_end.remaining_distance_m, 1e-9);
  const FollowResult at_end = layer.compute(Pose2D{9.9, 0.0, 0.0}, freshInputs());
  EXPECT_FALSE(at_end.output.is_asserted);
  EXPECT_EQ(at_end.output.reason, StopReason::kAtPathEnd);
}

TEST(FollowLayer, NearestSearchOnlyMovesForwardWithinWindow) {
  FollowLayer layer{FollowParameters{}};
  layer.tryAcceptPath(1, 1, straightPath(20.0));
  EXPECT_EQ(layer.compute(Pose2D{3.0, 0.0, 0.0}, freshInputs()).closest_index, 30u);
  // Jumping back does not move the index backwards.
  EXPECT_EQ(layer.compute(Pose2D{1.0, 0.0, 0.0}, freshInputs()).closest_index, 30u);
  // A pose beyond search_window_m (5 m) ahead is clamped to the window.
  const std::size_t clamped_index =
      layer.compute(Pose2D{15.0, 0.0, 0.0}, freshInputs()).closest_index;
  EXPECT_GE(clamped_index, 79u);
  EXPECT_LE(clamped_index, 80u);
}

// Straight path along +x (0.25 m steps: exact in binary so the lookahead point is x=1.0), defaults: lookahead 1.0, v_max 0.6, w_max 0.4, w_rotate 0.3.
TEST(PurePursuit, RightOfPathWithPositiveYawTurnsLeftWithExactCurvature) {
  FollowLayer layer{FollowParameters{}};
  layer.tryAcceptPath(1, 1, straightPath(20.0, 0.25));
  // Target (1,0). Body frame: forward=cos(0.3), lateral=-sin(0.3), L^2=1.
  const double expected_curvature = 2.0 * -std::sin(0.3);
  const FollowResult result = layer.compute(Pose2D{0.0, 0.0, 0.3}, freshInputs());
  ASSERT_TRUE(result.output.is_asserted);
  EXPECT_NEAR(result.output.twist.linear_mps, 0.6, 1e-9);
  EXPECT_NEAR(result.output.twist.angular_rps, 0.6 * expected_curvature, 1e-9);
}

TEST(PurePursuit, NegativeYawMirrorsTheSign) {
  FollowLayer layer{FollowParameters{}};
  layer.tryAcceptPath(1, 1, straightPath(20.0, 0.25));
  const FollowResult result = layer.compute(Pose2D{0.0, 0.0, -0.3}, freshInputs());
  ASSERT_TRUE(result.output.is_asserted);
  EXPECT_NEAR(result.output.twist.angular_rps, 0.6 * 2.0 * std::sin(0.3), 1e-9);
}

TEST(PurePursuit, RotateInPlaceTurnsTowardTheTargetSide) {
  FollowLayer layer{FollowParameters{}};
  layer.tryAcceptPath(1, 1, straightPath(20.0, 0.25));
  // Facing +y with the target at +x: target is on the right -> negative.
  const FollowResult facing_left = layer.compute(Pose2D{0.0, 0.0, M_PI / 2.0}, freshInputs());
  EXPECT_DOUBLE_EQ(facing_left.output.twist.linear_mps, 0.0);
  EXPECT_DOUBLE_EQ(facing_left.output.twist.angular_rps, -0.3);
  // Facing -y: target is on the left -> positive.
  const FollowResult facing_right = layer.compute(Pose2D{0.0, 0.0, -M_PI / 2.0}, freshInputs());
  EXPECT_DOUBLE_EQ(facing_right.output.twist.angular_rps, 0.3);
}

TEST(PurePursuit, CurvatureLimitsSpeedAndMatchesFormula) {
  FollowLayer layer{FollowParameters{}};
  layer.tryAcceptPath(1, 1, straightPath(20.0, 0.25));
  // 0.5 m left of the path: target (1,0) -> lateral=-0.5, L^2=1.25, kappa=-0.8.
  const FollowResult result = layer.compute(Pose2D{0.0, 0.5, 0.0}, freshInputs());
  EXPECT_NEAR(result.output.twist.linear_mps, 0.4 / 0.8, 1e-9);  // w_max/|kappa|
  EXPECT_NEAR(result.output.twist.angular_rps, -0.4, 1e-9);
}

TEST(PurePursuit, MinimumSpeedFloorAppliesAndAngularRateIsClamped) {
  FollowParameters parameters;
  parameters.v_min_mps = 0.55;  // above the curvature limit 0.5 for kappa=-0.8
  FollowLayer layer{parameters};
  layer.tryAcceptPath(1, 1, straightPath(20.0, 0.25));
  const FollowResult result = layer.compute(Pose2D{0.0, 0.5, 0.0}, freshInputs());
  EXPECT_NEAR(result.output.twist.linear_mps, 0.55, 1e-9);
  EXPECT_NEAR(result.output.twist.angular_rps, -0.4, 1e-9);  // 0.55*-0.8 = -0.44 clamped
}

TEST(NullAvoidLayer, NeverAsserts) {
  NullAvoidLayer layer;
  const std::vector<Point2D> points = straightPath(5.0);
  EXPECT_FALSE(layer.compute(AvoidContext{Pose2D{}, points, 0, true}).is_asserted);
}

TEST(FollowLayer, OneControlCycleIsFast) {
  FollowLayer layer{FollowParameters{}};
  layer.tryAcceptPath(1, 1, straightPath(200.0));
  NullAvoidLayer avoid_layer;
  constexpr int kIterations = 1000;
  const auto start = std::chrono::steady_clock::now();
  for (int iteration = 0; iteration < kIterations; ++iteration) {
    const FollowResult follow = layer.compute(Pose2D{1.0, 0.2, 0.0}, freshInputs());
    ArbitrationInput input;
    input.follow = follow.output;
    input.avoid = avoid_layer.compute(
        AvoidContext{Pose2D{}, layer.pathPoints(), follow.closest_index, true});
    input.is_pose_fresh = true;
    arbitrate(input);
  }
  const double per_cycle_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count() /
      kIterations;
  std::printf("per cycle: %.4f ms\n", per_cycle_ms);
  EXPECT_LT(per_cycle_ms, 10.0);  // CI threshold; spec target is 2 ms incl. avoid layer (PR4)
}

TEST(FollowLayer, HeadingAlignmentHasHysteresisAndStaleStops) {
  FollowLayer layer{FollowParameters{}};
  layer.tryAcceptPath(1, 1, straightPath(20.0));
  EXPECT_DOUBLE_EQ(layer.compute(Pose2D{0, 0, 1.3}, freshInputs()).output.twist.linear_mps, 0.0);
  EXPECT_DOUBLE_EQ(layer.compute(Pose2D{0, 0, 1.1}, freshInputs()).output.twist.linear_mps, 0.0);
  EXPECT_GT(layer.compute(Pose2D{0, 0, 0.9}, freshInputs()).output.twist.linear_mps, 0.0);
  const auto stale = layer.compute(Pose2D{0, 0, 3.0}, FollowInputs{});
  EXPECT_EQ(stale.output.reason, StopReason::kObstacleStale);
  EXPECT_DOUBLE_EQ(stale.output.twist.angular_rps, 0.0);
}

TEST(FollowLayer, EmptyPausePathStopsAnActiveRotation) {
  FollowLayer layer{FollowParameters{}};
  layer.tryAcceptPath(1, 1, straightPath(20.0));
  ASSERT_NE(layer.compute(Pose2D{0, 0, 3.0}, freshInputs()).output.twist.angular_rps, 0.0);
  ASSERT_TRUE(layer.tryAcceptPath(1, 2, {}));
  const auto stopped = layer.compute(Pose2D{0, 0, 3.0}, freshInputs());
  EXPECT_EQ(stopped.output.reason, StopReason::kNoPath);
  EXPECT_DOUBLE_EQ(stopped.output.twist.angular_rps, 0.0);
}
