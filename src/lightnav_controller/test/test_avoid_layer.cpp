#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <iostream>
#include <string>
#include <cmath>

#include "lightnav_controller/arbitrate.hpp"
#include "lightnav_controller/avoid_layer.hpp"
#include "lightnav_controller/follow_layer.hpp"

using namespace lightnav_controller;

namespace {

// 80 x 80 cells of 0.1 m, origin x: -2..6, y: -4..4 like the obstacle monitor.
LocalGrid makeEmptyGrid() {
  LocalGrid grid;
  grid.width_cells = 80;
  grid.height_cells = 80;
  grid.resolution_m = 0.1;
  grid.origin_x_m = -2.0;
  grid.origin_y_m = -4.0;
  grid.cells.assign(80 * 80, 0);
  return grid;
}

void fillRectangle(LocalGrid& grid, double min_x_m, double max_x_m, double min_y_m, double max_y_m) {
  for (double x_m = min_x_m; x_m <= max_x_m; x_m += 0.05) {
    for (double y_m = min_y_m; y_m <= max_y_m; y_m += 0.05) {
      const auto column = static_cast<std::size_t>(std::floor((x_m - grid.origin_x_m) / grid.resolution_m));
      const auto row = static_cast<std::size_t>(std::floor((y_m - grid.origin_y_m) / grid.resolution_m));
      grid.cells[row * grid.width_cells + column] = 100;
    }
  }
}

std::vector<Point2D> makeStraightPath() {
  std::vector<Point2D> path;
  for (int index = 0; index <= 200; ++index) {
    path.push_back(Point2D{0.1 * index, 0.0});
  }
  return path;
}

AvoidParameters makeAvoidParameters() {
  AvoidParameters parameters;
  parameters.max_path_deviation_m = 2.5;
  return parameters;
}

const Pose2D kOrigin{0.0, 0.0, 0.0};

AvoidContext makeContext(const std::vector<Point2D>& path, const LocalGrid* grid, double now_s,
                         const Pose2D& pose = kOrigin, bool is_grid_fresh = true) {
  return AvoidContext{pose, path, 0, is_grid_fresh, grid, now_s};
}

}  // namespace

TEST(EvaluateArcCandidates, FreeGridMakesStraightArcTheBest) {
  const auto path = makeStraightPath();
  const auto candidates = evaluateArcCandidates(makeEmptyGrid(), kOrigin, path, 0, 0.0, makeAvoidParameters());
  ASSERT_EQ(candidates.size(), 15u);
  const auto best = selectBestCandidate(candidates);
  ASSERT_TRUE(best.has_value());
  EXPECT_NEAR(best->curvature_per_m, 0.0, 1e-9);
}

TEST(EvaluateArcCandidates, CurvatureRangeMatchesWMaxOverVAvoid) {
  const auto candidates =
      evaluateArcCandidates(makeEmptyGrid(), kOrigin, makeStraightPath(), 0, 0.0, makeAvoidParameters());
  EXPECT_NEAR(candidates.front().curvature_per_m, -1.6, 1e-9);
  EXPECT_NEAR(candidates.back().curvature_per_m, 1.6, 1e-9);
}

TEST(EvaluateArcCandidates, OccupiedCellOnArcRejectsIt) {
  LocalGrid grid = makeEmptyGrid();
  fillRectangle(grid, 1.0, 1.2, -0.1, 0.1);  // straight ahead
  const auto candidates = evaluateArcCandidates(grid, kOrigin, makeStraightPath(), 0, 0.0, makeAvoidParameters());
  EXPECT_FALSE(candidates[7].is_feasible);
  const auto best = selectBestCandidate(candidates);
  ASSERT_TRUE(best.has_value());
  EXPECT_NE(candidates[7].curvature_per_m, best->curvature_per_m);
}

TEST(EvaluateArcCandidates, ArcsEndingBeyondMaxPathDeviationAreRejected) {
  AvoidParameters parameters = makeAvoidParameters();
  parameters.max_path_deviation_m = 0.2;
  const auto candidates = evaluateArcCandidates(makeEmptyGrid(), kOrigin, makeStraightPath(), 0, 0.0, parameters);
  for (const ArcCandidate& candidate : candidates) {
    if (std::fabs(candidate.curvature_per_m) > 0.5) {
      EXPECT_FALSE(candidate.is_feasible) << candidate.curvature_per_m;
    }
  }
  EXPECT_TRUE(candidates[7].is_feasible);
}

TEST(EvaluateArcCandidates, AllBlockedSelectsNothing) {
  LocalGrid grid = makeEmptyGrid();
  fillRectangle(grid, 0.3, 0.4, -3.9, 3.9);  // wall across every arc
  const auto candidates = evaluateArcCandidates(grid, kOrigin, makeStraightPath(), 0, 0.0, makeAvoidParameters());
  EXPECT_FALSE(selectBestCandidate(candidates).has_value());
}

TEST(EvaluateArcCandidates, SmoothnessTermPrefersPreviousCurvatureOnTie) {
  // Symmetric free grid: left and right arcs score the same except for w_smooth.
  const auto candidates =
      evaluateArcCandidates(makeEmptyGrid(), kOrigin, makeStraightPath(), 0, 0.4, makeAvoidParameters());
  EXPECT_LT(candidates[8].score, candidates[6].score);  // +0.2286 vs -0.2286
}

TEST(EvaluateArcCandidates, LocalMinimumInConcavePocketHasNoFeasibleArc) {
  LocalGrid grid = makeEmptyGrid();
  fillRectangle(grid, 0.6, 0.7, -1.5, 1.5);   // front
  fillRectangle(grid, 0.3, 0.7, 1.4, 1.5);    // left wall
  fillRectangle(grid, 0.3, 0.7, -1.5, -1.4);  // right wall
  AvoidParameters parameters = makeAvoidParameters();
  parameters.arc_length_m = 2.0;
  const auto candidates = evaluateArcCandidates(grid, kOrigin, makeStraightPath(), 0, 0.0, parameters);
  EXPECT_FALSE(selectBestCandidate(candidates).has_value());
}

TEST(ArcAvoidLayer, DoesNotAssertWhenPathAheadIsFree) {
  ArcAvoidLayer layer{makeAvoidParameters()};
  const auto path = makeStraightPath();
  const LocalGrid grid = makeEmptyGrid();
  EXPECT_FALSE(layer.compute(makeContext(path, &grid, 0.0)).is_asserted);
  EXPECT_FALSE(layer.isAvoiding());
}

TEST(ArcAvoidLayer, TriggersAndSteersAroundObstacleOnPath) {
  ArcAvoidLayer layer{makeAvoidParameters()};
  const auto path = makeStraightPath();
  LocalGrid grid = makeEmptyGrid();
  fillRectangle(grid, 1.0, 1.4, -0.3, 0.3);
  const LayerOutput output = layer.compute(makeContext(path, &grid, 0.0));
  EXPECT_TRUE(output.is_asserted);
  EXPECT_EQ(output.reason, StopReason::kOk);
  EXPECT_DOUBLE_EQ(output.twist.linear_mps, 0.25);
  EXPECT_NE(output.twist.angular_rps, 0.0);
  EXPECT_LE(std::fabs(output.twist.angular_rps), 0.4 + 1e-9);
}

TEST(ArcAvoidLayer, GridLossReleasesImmediatelyAndDoesNotAssert) {
  ArcAvoidLayer layer{makeAvoidParameters()};
  const auto path = makeStraightPath();
  LocalGrid grid = makeEmptyGrid();
  fillRectangle(grid, 1.0, 1.4, -0.3, 0.3);
  ASSERT_TRUE(layer.compute(makeContext(path, &grid, 0.0)).is_asserted);
  EXPECT_FALSE(layer.compute(makeContext(path, &grid, 0.05, kOrigin, false)).is_asserted);
  EXPECT_FALSE(layer.compute(makeContext(path, nullptr, 0.1)).is_asserted);
  EXPECT_FALSE(layer.isAvoiding());
}

TEST(ArcAvoidLayer, BlockedWaitsThenReportsStuckAndRecovers) {
  ArcAvoidLayer layer{makeAvoidParameters()};
  const auto path = makeStraightPath();
  LocalGrid wall = makeEmptyGrid();
  fillRectangle(wall, 0.5, 0.7, -3.9, 3.9);
  LayerOutput output = layer.compute(makeContext(path, &wall, 0.0));
  EXPECT_TRUE(output.is_asserted);
  EXPECT_EQ(output.reason, StopReason::kBlocked);
  EXPECT_DOUBLE_EQ(output.twist.linear_mps, 0.0);
  EXPECT_EQ(layer.compute(makeContext(path, &wall, 9.9)).reason, StopReason::kBlocked);
  EXPECT_EQ(layer.compute(makeContext(path, &wall, 10.0)).reason, StopReason::kStuck);

  const LocalGrid free_grid = makeEmptyGrid();
  output = layer.compute(makeContext(path, &free_grid, 10.1));  // wall removed: resumes
  EXPECT_TRUE(output.is_asserted);
  EXPECT_EQ(output.reason, StopReason::kOk);
}

TEST(ArcAvoidLayer, ReleasesToFollowAfterHoldWhenClearAndOnPath) {
  ArcAvoidLayer layer{makeAvoidParameters()};
  const auto path = makeStraightPath();
  LocalGrid blocked = makeEmptyGrid();
  fillRectangle(blocked, 1.0, 1.4, -0.3, 0.3);
  const LocalGrid free_grid = makeEmptyGrid();
  ASSERT_TRUE(layer.compute(makeContext(path, &blocked, 0.0)).is_asserted);
  EXPECT_TRUE(layer.compute(makeContext(path, &free_grid, 1.0)).is_asserted);   // hold starts
  EXPECT_TRUE(layer.compute(makeContext(path, &free_grid, 1.9)).is_asserted);   // hold 0.9 s
  EXPECT_FALSE(layer.compute(makeContext(path, &free_grid, 2.0)).is_asserted);  // hold 1.0 s
}

TEST(ArcAvoidLayer, DoesNotReleaseWhileFarFromPath) {
  ArcAvoidLayer layer{makeAvoidParameters()};
  const auto path = makeStraightPath();
  LocalGrid blocked = makeEmptyGrid();
  fillRectangle(blocked, 1.0, 1.4, -0.3, 0.3);
  const LocalGrid free_grid = makeEmptyGrid();
  ASSERT_TRUE(layer.compute(makeContext(path, &blocked, 0.0)).is_asserted);
  const Pose2D offset_pose{0.0, 0.8, 0.0};
  EXPECT_TRUE(layer.compute(makeContext(path, &free_grid, 1.0, offset_pose)).is_asserted);
  EXPECT_TRUE(layer.compute(makeContext(path, &free_grid, 5.0, offset_pose)).is_asserted);
}

TEST(ArcAvoidLayer, FlickeringObstacleDoesNotChatterBetweenAvoidAndFollow) {
  ArcAvoidLayer layer{makeAvoidParameters()};
  const auto path = makeStraightPath();
  LocalGrid blocked = makeEmptyGrid();
  fillRectangle(blocked, 1.0, 1.4, -0.3, 0.3);
  const LocalGrid free_grid = makeEmptyGrid();
  int toggle_count = 0;
  bool was_asserted = false;
  for (int step = 0; step < 40; ++step) {  // 20 Hz, grid flips every cycle for 2 s
    const LocalGrid& grid = (step % 2 == 0) ? blocked : free_grid;
    const bool is_asserted = layer.compute(makeContext(path, &grid, 0.05 * step)).is_asserted;
    if (step > 0 && is_asserted != was_asserted) {
      ++toggle_count;
    }
    was_asserted = is_asserted;
  }
  EXPECT_EQ(toggle_count, 0);
  EXPECT_TRUE(was_asserted);
}

namespace {

struct ClosedLoopResult {
  bool did_avoid{false};
  bool did_enter_stop_zone{false};
  bool did_report_stuck{false};
  double max_deviation_m{0.0};
  Pose2D final_pose;
};

// Unicycle at 20 Hz with the real follow layer and arbitrate. The grid is
// built like the obstacle monitor: the disc obstacle is inflated by 0.5 m.
// The gate is modelled as speed 0 while >= 3 raw cells lie in the stop zone
// (x 0..1.0, |y| <= 0.4).
ClosedLoopResult runClosedLoop(const AvoidParameters& parameters, const Point2D& obstacle_center) {
  const double obstacle_radius_m = 0.5;
  const double inflation_radius_m = 0.5;
  const double time_step_s = 0.05;
  const auto path = makeStraightPath();
  FollowLayer follow_layer{FollowParameters{}};
  follow_layer.tryAcceptPath(1, 1, path);
  ArcAvoidLayer avoid_layer{parameters};
  ClosedLoopResult result;
  Pose2D pose = kOrigin;
  for (int step = 0; step < 1600 && pose.x_m < 19.0; ++step) {
    LocalGrid grid = makeEmptyGrid();
    int stop_zone_cells = 0;
    for (std::size_t row = 0; row < grid.height_cells; ++row) {
      for (std::size_t column = 0; column < grid.width_cells; ++column) {
        const double body_x_m = grid.origin_x_m + (static_cast<double>(column) + 0.5) * grid.resolution_m;
        const double body_y_m = grid.origin_y_m + (static_cast<double>(row) + 0.5) * grid.resolution_m;
        const double world_x_m = pose.x_m + std::cos(pose.yaw_rad) * body_x_m - std::sin(pose.yaw_rad) * body_y_m;
        const double world_y_m = pose.y_m + std::sin(pose.yaw_rad) * body_x_m + std::cos(pose.yaw_rad) * body_y_m;
        const double distance_m = std::hypot(world_x_m - obstacle_center.x_m, world_y_m - obstacle_center.y_m);
        if (distance_m <= obstacle_radius_m + inflation_radius_m) {
          grid.cells[row * grid.width_cells + column] = 100;
        }
        if (distance_m <= obstacle_radius_m && body_x_m >= 0.0 && body_x_m <= 1.0 && std::fabs(body_y_m) <= 0.4) {
          ++stop_zone_cells;
        }
      }
    }
    const bool is_stop_zone_occupied = stop_zone_cells >= 3;
    FollowInputs follow_inputs;
    follow_inputs.is_obstacle_status_fresh = true;
    follow_inputs.is_stop_zone_occupied = is_stop_zone_occupied;
    const FollowResult follow_result = follow_layer.compute(pose, follow_inputs);
    const double now_s = time_step_s * step;
    ArbitrationInput arbitration;
    arbitration.follow = follow_result.output;
    arbitration.avoid = avoid_layer.compute(AvoidContext{
        pose, follow_layer.pathPoints(), follow_result.closest_index, true, &grid, now_s,
        follow_result.remaining_distance_m, is_stop_zone_occupied, true});
    arbitration.is_pose_fresh = true;
    arbitration.is_grid_fresh = true;
    const ArbitrationResult command = arbitrate(arbitration);
    result.did_avoid = result.did_avoid || command.source_layer == Layer::kAvoid;
    result.did_enter_stop_zone = result.did_enter_stop_zone || is_stop_zone_occupied;
    result.did_report_stuck = result.did_report_stuck || command.reason == StopReason::kStuck;
    if (command.reason == StopReason::kAtPathEnd) {
      break;
    }
    const Twist2D twist = is_stop_zone_occupied ? Twist2D{} : command.twist;
    pose.yaw_rad += twist.angular_rps * time_step_s;
    pose.x_m += std::cos(pose.yaw_rad) * twist.linear_mps * time_step_s;
    pose.y_m += std::sin(pose.yaw_rad) * twist.linear_mps * time_step_s;
    result.max_deviation_m = std::max(result.max_deviation_m, std::fabs(pose.y_m));
  }
  result.final_pose = pose;
  return result;
}

}  // namespace

// Acceptance H with arc length 2.0 m, inflation 0.5 m and the gate model.
class ArcAvoidClosedLoop : public ::testing::TestWithParam<double> {};

TEST_P(ArcAvoidClosedLoop, DetoursOffsetObstacleWithoutEnteringStopZoneAndRejoins) {
  AvoidParameters parameters = makeAvoidParameters();
  parameters.arc_length_m = 2.0;
  const ClosedLoopResult result = runClosedLoop(parameters, Point2D{6.0, GetParam()});
  EXPECT_TRUE(result.did_avoid);
  EXPECT_FALSE(result.did_enter_stop_zone);
  EXPECT_FALSE(result.did_report_stuck);
  EXPECT_LE(result.max_deviation_m, parameters.max_path_deviation_m);
  EXPECT_GE(result.final_pose.x_m, 18.0);
  EXPECT_LT(std::fabs(result.final_pose.y_m), 0.3);
}

INSTANTIATE_TEST_SUITE_P(ObstacleOffsets, ArcAvoidClosedLoop, ::testing::Values(0.0, 0.1, 0.2, 0.3));

TEST(ArcAvoidLayer, NarrowCourseStopsBeforeCenteredObstacle) {
  AvoidParameters parameters = makeAvoidParameters();
  parameters.arc_length_m = 2.0;
  parameters.max_path_deviation_m = 1.5;
  const ClosedLoopResult result = runClosedLoop(parameters, Point2D{6.0, 0.0});
  EXPECT_FALSE(result.did_enter_stop_zone);
  EXPECT_TRUE(result.did_report_stuck);
  EXPECT_LE(result.max_deviation_m, parameters.max_path_deviation_m);
  EXPECT_LT(result.final_pose.x_m, 6.0);
}

TEST(AvoidParameters, DefaultArcLengthIsTwoMeters) {
  EXPECT_DOUBLE_EQ(AvoidParameters{}.arc_length_m, 2.0);
}

TEST(EvaluateArcCandidates, InvalidArcLengthRejectsAllCandidates) {
  for (const double invalid_value : {std::numeric_limits<double>::quiet_NaN(),
                                     std::numeric_limits<double>::infinity(), 0.0, -1.0}) {
    SCOPED_TRACE(invalid_value);
    AvoidParameters parameters = makeAvoidParameters();
    parameters.arc_length_m = invalid_value;
    EXPECT_TRUE(evaluateArcCandidates(makeEmptyGrid(), kOrigin, makeStraightPath(), 0, 0.0,
                                      parameters).empty());
  }
}

TEST(EvaluateArcCandidates, InvalidArcStepRejectsAllCandidates) {
  for (const double invalid_value : {std::numeric_limits<double>::quiet_NaN(),
                                     std::numeric_limits<double>::infinity(), 0.0, -1.0}) {
    SCOPED_TRACE(invalid_value);
    AvoidParameters parameters = makeAvoidParameters();
    parameters.arc_step_m = invalid_value;
    EXPECT_TRUE(evaluateArcCandidates(makeEmptyGrid(), kOrigin, makeStraightPath(), 0, 0.0,
                                      parameters).empty());
  }
}

TEST(EvaluateArcCandidates, MissingOrInvalidDeviationRejectsAllCandidates) {
  for (const double deviation_m : {std::numeric_limits<double>::quiet_NaN(),
                                   std::numeric_limits<double>::infinity(), 0.0, -1.0}) {
    AvoidParameters parameters;
    parameters.max_path_deviation_m = deviation_m;
    EXPECT_TRUE(evaluateArcCandidates(makeEmptyGrid(), kOrigin, makeStraightPath(), 0, 0.0,
                                      parameters).empty());
  }
}

TEST(ArcAvoidLayer, StopZoneOccupiedMakesLayerWaitAndEventuallyReportStuck) {
  ArcAvoidLayer layer{makeAvoidParameters()};
  const auto path = makeStraightPath();
  LocalGrid grid = makeEmptyGrid();
  fillRectangle(grid, 1.0, 1.4, -0.3, 0.3);
  AvoidContext context = makeContext(path, &grid, 0.0);
  context.is_stop_zone_occupied = true;
  LayerOutput output = layer.compute(context);
  EXPECT_TRUE(output.is_asserted);
  EXPECT_EQ(output.reason, StopReason::kBlocked);
  EXPECT_DOUBLE_EQ(output.twist.linear_mps, 0.0);
  context.now_s = 10.0;
  EXPECT_EQ(layer.compute(context).reason, StopReason::kStuck);
}

TEST(ArcAvoidLayer, StaleObstacleStatusWaitsWithObstacleStaleReason) {
  ArcAvoidLayer layer{makeAvoidParameters()};
  const auto path = makeStraightPath();
  LocalGrid grid = makeEmptyGrid();
  fillRectangle(grid, 1.0, 1.4, -0.3, 0.3);
  AvoidContext context = makeContext(path, &grid, 0.0);
  context.is_obstacle_status_fresh = false;
  const LayerOutput output = layer.compute(context);
  EXPECT_TRUE(output.is_asserted);
  EXPECT_EQ(output.reason, StopReason::kObstacleStale);
  EXPECT_DOUBLE_EQ(output.twist.linear_mps, 0.0);
}

TEST(ArcAvoidLayer, AtPathEndHandsOverUnlessEndIsOccupied) {
  ArcAvoidLayer layer{makeAvoidParameters()};
  std::vector<Point2D> path;
  for (int index = 0; index <= 10; ++index) {
    path.push_back(Point2D{0.1 * index, 0.0});
  }
  LocalGrid grid = makeEmptyGrid();
  fillRectangle(grid, 0.6, 0.8, -0.2, 0.2);
  AvoidContext context = makeContext(path, &grid, 0.0, Pose2D{0.0, 0.0, 0.0});
  context.remaining_distance_m = 1.0;
  ASSERT_TRUE(layer.compute(context).is_asserted);  // avoiding
  context.remaining_distance_m = 0.1;
  context.grid = &grid;
  fillRectangle(grid, 0.95, 1.05, -0.05, 0.05);  // end point (1.0, 0) occupied
  LayerOutput output = layer.compute(context);
  EXPECT_EQ(output.reason, StopReason::kBlocked);
  EXPECT_DOUBLE_EQ(output.twist.linear_mps, 0.0);
  const LocalGrid free_grid = makeEmptyGrid();
  context.grid = &free_grid;
  EXPECT_FALSE(layer.compute(context).is_asserted);
}

TEST(EvaluateArcCandidates, DeviationIsCheckedAlongTheArcNotOnlyAtTheEnd) {
  // 1.4 m off the path heading away: the end point is within 1.5 m but a middle sample is not.
  AvoidParameters parameters = makeAvoidParameters();
  parameters.max_path_deviation_m = 1.5;
  parameters.arc_length_m = 2.0;
  const Pose2D pose{0.0, 1.4, 1.5708};
  const auto candidates = evaluateArcCandidates(makeEmptyGrid(), pose, makeStraightPath(), 0, 0.0, parameters);
  for (const ArcCandidate& candidate : candidates) {
    if (candidate.curvature_per_m < -1.5) {
      EXPECT_FALSE(candidate.is_feasible);
    }
  }
}

TEST(ArcAvoidLayer, CycleWithFifteenArcsOnEightyByEightyGridIsFast) {
  AvoidParameters parameters = makeAvoidParameters();
  const auto path = makeStraightPath();
  LocalGrid grid = makeEmptyGrid();
  fillRectangle(grid, 1.0, 1.4, -0.3, 0.3);
  const int iterations = 1000;
  const auto start = std::chrono::steady_clock::now();
  double sink = 0.0;
  for (int iteration = 0; iteration < iterations; ++iteration) {
    ArcAvoidLayer layer{parameters};
    sink += layer.compute(makeContext(path, &grid, 0.0)).twist.angular_rps;
  }
  const double mean_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count() / iterations;
  RecordProperty("mean_ms", std::to_string(mean_ms));
  std::cout << "avoid cycle mean_ms=" << mean_ms << " sink=" << sink << std::endl;
  EXPECT_LT(mean_ms, 10.0);
}
