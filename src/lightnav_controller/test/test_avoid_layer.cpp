#include <gtest/gtest.h>

#include <chrono>
#include <cmath>

#include "lightnav_controller/avoid_layer.hpp"

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

const Pose2D kOrigin{0.0, 0.0, 0.0};

AvoidContext makeContext(const std::vector<Point2D>& path, const LocalGrid* grid, double now_s,
                         const Pose2D& pose = kOrigin, bool is_grid_fresh = true) {
  return AvoidContext{pose, path, 0, is_grid_fresh, grid, now_s};
}

}  // namespace

TEST(EvaluateArcCandidates, FreeGridMakesStraightArcTheBest) {
  const auto path = makeStraightPath();
  const auto candidates = evaluateArcCandidates(makeEmptyGrid(), kOrigin, path, 0, 0.0, AvoidParameters{});
  ASSERT_EQ(candidates.size(), 15u);
  const auto best = selectBestCandidate(candidates);
  ASSERT_TRUE(best.has_value());
  EXPECT_NEAR(best->curvature_per_m, 0.0, 1e-9);
}

TEST(EvaluateArcCandidates, CurvatureRangeMatchesWMaxOverVAvoid) {
  const auto candidates =
      evaluateArcCandidates(makeEmptyGrid(), kOrigin, makeStraightPath(), 0, 0.0, AvoidParameters{});
  EXPECT_NEAR(candidates.front().curvature_per_m, -1.6, 1e-9);
  EXPECT_NEAR(candidates.back().curvature_per_m, 1.6, 1e-9);
}

TEST(EvaluateArcCandidates, OccupiedCellOnArcRejectsIt) {
  LocalGrid grid = makeEmptyGrid();
  fillRectangle(grid, 1.0, 1.2, -0.1, 0.1);  // straight ahead
  const auto candidates = evaluateArcCandidates(grid, kOrigin, makeStraightPath(), 0, 0.0, AvoidParameters{});
  EXPECT_FALSE(candidates[7].is_feasible);
  const auto best = selectBestCandidate(candidates);
  ASSERT_TRUE(best.has_value());
  EXPECT_NE(candidates[7].curvature_per_m, best->curvature_per_m);
}

TEST(EvaluateArcCandidates, ArcsEndingBeyondMaxPathDeviationAreRejected) {
  AvoidParameters parameters;
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
  const auto candidates = evaluateArcCandidates(grid, kOrigin, makeStraightPath(), 0, 0.0, AvoidParameters{});
  EXPECT_FALSE(selectBestCandidate(candidates).has_value());
}

TEST(EvaluateArcCandidates, SmoothnessTermPrefersPreviousCurvatureOnTie) {
  // Symmetric free grid: left and right arcs score the same except for w_smooth.
  const auto candidates =
      evaluateArcCandidates(makeEmptyGrid(), kOrigin, makeStraightPath(), 0, 0.4, AvoidParameters{});
  EXPECT_LT(candidates[8].score, candidates[6].score);  // +0.2286 vs -0.2286
}

TEST(EvaluateArcCandidates, LocalMinimumInConcavePocketHasNoFeasibleArc) {
  LocalGrid grid = makeEmptyGrid();
  fillRectangle(grid, 0.6, 0.7, -1.5, 1.5);   // front
  fillRectangle(grid, 0.3, 0.7, 1.4, 1.5);    // left wall
  fillRectangle(grid, 0.3, 0.7, -1.5, -1.4);  // right wall
  AvoidParameters parameters;
  parameters.arc_length_m = 2.0;
  const auto candidates = evaluateArcCandidates(grid, kOrigin, makeStraightPath(), 0, 0.0, parameters);
  EXPECT_FALSE(selectBestCandidate(candidates).has_value());
}

TEST(ArcAvoidLayer, DoesNotAssertWhenPathAheadIsFree) {
  ArcAvoidLayer layer{AvoidParameters{}};
  const auto path = makeStraightPath();
  const LocalGrid grid = makeEmptyGrid();
  EXPECT_FALSE(layer.compute(makeContext(path, &grid, 0.0)).is_asserted);
  EXPECT_FALSE(layer.isAvoiding());
}

TEST(ArcAvoidLayer, TriggersAndSteersAroundObstacleOnPath) {
  ArcAvoidLayer layer{AvoidParameters{}};
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
  ArcAvoidLayer layer{AvoidParameters{}};
  const auto path = makeStraightPath();
  LocalGrid grid = makeEmptyGrid();
  fillRectangle(grid, 1.0, 1.4, -0.3, 0.3);
  ASSERT_TRUE(layer.compute(makeContext(path, &grid, 0.0)).is_asserted);
  EXPECT_FALSE(layer.compute(makeContext(path, &grid, 0.05, kOrigin, false)).is_asserted);
  EXPECT_FALSE(layer.compute(makeContext(path, nullptr, 0.1)).is_asserted);
  EXPECT_FALSE(layer.isAvoiding());
}

TEST(ArcAvoidLayer, BlockedWaitsThenReportsStuckAndRecovers) {
  ArcAvoidLayer layer{AvoidParameters{}};
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
  ArcAvoidLayer layer{AvoidParameters{}};
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
  ArcAvoidLayer layer{AvoidParameters{}};
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
  ArcAvoidLayer layer{AvoidParameters{}};
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

TEST(ArcAvoidLayer, RejoinsPathAfterPassingObstacle) {
  // Closed loop on a unicycle: obstacle 0.3 m off the path, radius 0.5 m.
  AvoidParameters parameters;
  ArcAvoidLayer layer{parameters};
  const auto path = makeStraightPath();
  Pose2D pose = kOrigin;
  const Point2D obstacle{3.0, 0.3};
  double max_deviation_m = 0.0;
  bool did_avoid = false;
  const double time_step_s = 0.05;
  for (int step = 0; step < 1200 && pose.x_m < 8.0; ++step) {
    LocalGrid grid = makeEmptyGrid();
    const double relative_x_m = obstacle.x_m - pose.x_m;
    const double relative_y_m = obstacle.y_m - pose.y_m;
    const double body_x_m = std::cos(pose.yaw_rad) * relative_x_m + std::sin(pose.yaw_rad) * relative_y_m;
    const double body_y_m = -std::sin(pose.yaw_rad) * relative_x_m + std::cos(pose.yaw_rad) * relative_y_m;
    fillRectangle(grid, body_x_m - 0.5, body_x_m + 0.5, body_y_m - 0.5, body_y_m + 0.5);
    LayerOutput output = layer.compute(AvoidContext{pose, path, static_cast<std::size_t>(std::max(0.0, pose.x_m * 10.0)),
                                                    true, &grid, time_step_s * step});
    if (!output.is_asserted) {
      output.twist = Twist2D{0.5, -2.0 * pose.y_m - 1.0 * std::sin(pose.yaw_rad)};  // crude follow
    } else {
      did_avoid = true;
    }
    pose.yaw_rad += output.twist.angular_rps * time_step_s;
    pose.x_m += std::cos(pose.yaw_rad) * output.twist.linear_mps * time_step_s;
    pose.y_m += std::sin(pose.yaw_rad) * output.twist.linear_mps * time_step_s;
    max_deviation_m = std::max(max_deviation_m, std::fabs(pose.y_m));
  }
  EXPECT_TRUE(did_avoid);
  EXPECT_LE(max_deviation_m, parameters.max_path_deviation_m);
  EXPECT_GE(pose.x_m, 8.0);
  EXPECT_LT(std::fabs(pose.y_m), 0.3);
}

TEST(ArcAvoidLayer, CycleWithFifteenArcsOnEightyByEightyGridIsFast) {
  AvoidParameters parameters;
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
