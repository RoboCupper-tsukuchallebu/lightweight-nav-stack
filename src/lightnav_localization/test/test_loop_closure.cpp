#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <string>
#include <stdexcept>
#include <vector>

#include "lightnav_localization/loop_closure.hpp"
#include "synthetic_world.hpp"

namespace lightnav_localization {
namespace {

using lightnav_localization::testing::SyntheticWorld;

constexpr double kBaseHeightBelowSensorM = 0.5;
constexpr double kKeyframeSpacingM = 1.0;
constexpr double kDegreesToRadians = M_PI / 180.0;
// 350 rays per turn: a quarter turn of the vehicle is not a whole number of rays, as on a real sensor.
constexpr int kAzimuthSteps = 350;
constexpr int kRingCount = 8;

// An L-shaped corridor (16 m, then 12 m to the left) with boxes at irregular places: every stretch looks
// different from the others, and nothing repeats along a leg.
SyntheticWorld make_corridor() {
  SyntheticWorld world({{0.0, 0.0}, {16.0, 0.0}, {16.0, 12.0}});
  world.add_box({2.0, 0.5}, {2.8, 1.5});
  world.add_box({5.0, -1.7}, {5.5, -0.4});
  world.add_box({8.5, 0.9}, {10.0, 1.6});
  world.add_box({12.0, -1.8}, {12.4, -1.0});
  world.add_box({16.6, 3.4}, {17.4, 4.2});
  world.add_box({14.4, 6.5}, {15.2, 8.0});
  world.add_box({16.9, 9.6}, {17.7, 10.0});
  return world;
}

Eigen::Isometry3d planar_pose(double x, double y, double yaw_rad) {
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() = Eigen::Vector3d(x, y, 0.0);
  pose.linear() = Eigen::AngleAxisd(yaw_rad, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  return pose;
}

// The scan a vehicle standing at `base_pose` sees, in its base frame (the sensor is 0.5 m above the base).
PointList scan_in_base(const SyntheticWorld& world, const Eigen::Isometry3d& base_pose) {
  Eigen::Isometry3d sensor_pose = base_pose;
  sensor_pose.translation().z() += kBaseHeightBelowSensorM;
  const auto timed_points =
      world.synthesize_scan([&](double) { return sensor_pose; }, 0.0, 0.1, true, kAzimuthSteps, kRingCount);
  PointList points;
  points.reserve(timed_points.size());
  for (const auto& point : timed_points) {
    points.push_back(point.position + Eigen::Vector3f(0.0F, 0.0F, static_cast<float>(kBaseHeightBelowSensorM)));
  }
  return points;
}

struct Drive {
  std::vector<KeyframeCloud> keyframes;
  std::vector<Eigen::Isometry3d> true_poses;
  std::size_t turnaround_index = 0;  // the last keyframe of the way out
};

// One pose every kKeyframeSpacingM along the legs between `waypoints`, heading along the leg.
std::vector<Eigen::Isometry3d> poses_along(const std::vector<Eigen::Vector2d>& waypoints) {
  std::vector<Eigen::Isometry3d> poses;
  for (std::size_t leg = 0; leg + 1 < waypoints.size(); ++leg) {
    const Eigen::Vector2d displacement = waypoints[leg + 1] - waypoints[leg];
    const double yaw = std::atan2(displacement.y(), displacement.x());
    const int step_count = static_cast<int>(std::lround(displacement.norm() / kKeyframeSpacingM));
    // The first pose of a leg is the last of the one before it (except for the very first leg).
    for (int step = leg == 0 ? 0 : 1; step <= step_count; ++step) {
      const Eigen::Vector2d position = waypoints[leg] + displacement * (static_cast<double>(step) / step_count);
      poses.push_back(planar_pose(position.x(), position.y(), yaw));
    }
  }
  return poses;
}

// Drives the corridor from its start to the end of the second leg; with `returns_to_start` it turns around
// there and drives the same way back.
Drive make_drive(const SyntheticWorld& world, bool returns_to_start) {
  const std::vector<Eigen::Vector2d> way_out{{2.0, 0.0}, {16.0, 0.0}, {16.0, 10.0}};
  Drive drive;
  drive.true_poses = poses_along(way_out);
  drive.turnaround_index = drive.true_poses.size() - 1;
  if (returns_to_start) {
    const std::vector<Eigen::Isometry3d> way_back = poses_along({way_out[2], way_out[1], way_out[0]});
    drive.true_poses.insert(drive.true_poses.end(), way_back.begin() + 1, way_back.end());
  }
  for (std::size_t index = 0; index < drive.true_poses.size(); ++index) {
    KeyframeCloud keyframe;
    keyframe.node_index = static_cast<int>(index);
    keyframe.stamp_sec = static_cast<double>(index);
    keyframe.points_in_base = scan_in_base(world, drive.true_poses[index]);
    drive.keyframes.push_back(std::move(keyframe));
  }
  return drive;
}

// The estimate of an odometry whose heading jumped by `heading_error_rad` at the turnaround: the way back
// is rotated about that point, so the end of the drive lands away from its start.
std::vector<Eigen::Isometry3d> estimate_with_heading_error_on_the_way_back(const Drive& drive,
                                                                           double heading_error_rad) {
  const Eigen::Isometry3d turnaround = drive.true_poses[drive.turnaround_index];
  const Eigen::Isometry3d rotation_about_turnaround =
      turnaround * planar_pose(0.0, 0.0, heading_error_rad) * turnaround.inverse();
  std::vector<Eigen::Isometry3d> estimate = drive.true_poses;
  for (std::size_t index = drive.turnaround_index + 1; index < estimate.size(); ++index) {
    estimate[index] = rotation_about_turnaround * drive.true_poses[index];
  }
  return estimate;
}

LoopClosureParameters test_parameters() {
  LoopClosureParameters parameters;
  parameters.enabled = true;
  parameters.minimum_path_gap_m = 10.0;
  parameters.query_spacing_m = 4.0;
  parameters.search_radius_m = 3.0;
  parameters.search_radius_per_m = 0.1;
  parameters.maximum_search_radius_m = 6.0;
  parameters.descriptor_search_radius_m = 5.0;
  parameters.descriptor_target_shift_m = 3.0;
  parameters.search.maximum_range_m = 15.0;
  // Same-place pairs of this bare corridor score 0.50 to 0.63 (the best wrong pair 0.38): lower than on
  // the real drive, because few surfaces are visible both ways. The thresholds follow these scores.
  parameters.search.peaks.score_min = 0.45;
  parameters.search.peaks.margin_min = 0.1;
  return parameters;
}

double yaw_of(const Eigen::Isometry3d& pose) { return std::atan2(pose.linear()(1, 0), pose.linear()(0, 0)); }

void expect_closures_match_the_truth(const std::vector<LoopClosure>& closures,
                                     const std::vector<Eigen::Isometry3d>& true_poses) {
  for (const LoopClosure& closure : closures) {
    SCOPED_TRACE("target " + std::to_string(closure.target_node_index) + " query " +
                 std::to_string(closure.query_node_index));
    const Eigen::Isometry3d truth = true_poses[static_cast<std::size_t>(closure.target_node_index)].inverse() *
                                    true_poses[static_cast<std::size_t>(closure.query_node_index)];
    EXPECT_LT((closure.target_to_query.translation() - truth.translation()).head<2>().norm(), 0.2);
    EXPECT_LT(std::abs(std::remainder(yaw_of(closure.target_to_query) - yaw_of(truth), 2.0 * M_PI)),
              2.0 * kDegreesToRadians);
  }
}

TEST(PlaceDescriptor, SamePlaceIsCloseWhateverTheHeadingAndAnotherPlaceIsFar) {
  const SyntheticWorld world = make_corridor();
  const PlaceDescriptorParameters parameters;
  const PlaceDescriptor here = make_place_descriptor(scan_in_base(world, planar_pose(6.0, 0.0, 0.0)), parameters);
  const PlaceDescriptor here_turned =
      make_place_descriptor(scan_in_base(world, planar_pose(6.0, 0.0, M_PI / 2.0)), parameters);
  const PlaceDescriptor elsewhere =
      make_place_descriptor(scan_in_base(world, planar_pose(16.0, 8.0, 0.0)), parameters);
  std::printf("[ PLACE_DESCRIPTOR ] same place turned %.3f, other place %.3f\n",
              place_descriptor_distance(here, here_turned), place_descriptor_distance(here, elsewhere));

  const double same_place_distance = place_descriptor_distance(here, here_turned);
  const double other_place_distance = place_descriptor_distance(here, elsewhere);

  EXPECT_LT(same_place_distance, 0.1);
  EXPECT_GT(other_place_distance, same_place_distance + 0.1);
  EXPECT_NEAR(place_descriptor_distance(here, here), 0.0, 1e-6);
}

TEST(PlaceDescriptor, RejectsDescriptorsOfDifferentShape) {
  const SyntheticWorld world = make_corridor();
  PlaceDescriptorParameters coarse;
  coarse.sector_count = 30;
  const PointList scan = scan_in_base(world, planar_pose(6.0, 0.0, 0.0));
  EXPECT_THROW(place_descriptor_distance(make_place_descriptor(scan, PlaceDescriptorParameters{}),
                                         make_place_descriptor(scan, coarse)),
               std::invalid_argument);
}

TEST(LoopClosure, FindsTheWayBackAlongTheSameCorridorDespiteAHeadingError) {
  const SyntheticWorld world = make_corridor();
  const Drive drive = make_drive(world, true);
  // 6 degrees over the 24 m way back: the end of the drive is 1.9 m from where it really is.
  const std::vector<Eigen::Isometry3d> estimate =
      estimate_with_heading_error_on_the_way_back(drive, 6.0 * kDegreesToRadians);

  LoopClosureSearchSummary summary;
  const std::vector<LoopClosure> closures =
      find_loop_closures(drive.keyframes, estimate, test_parameters(), {}, &summary);

  for (const LoopClosureCandidateReport& report : summary.candidates) {
    std::printf("[ LOOP_CANDIDATE ] target %d query %d apart %.2f radius %.1f best %.3f second %.3f %s %s\n",
                report.target_node_index, report.query_node_index, report.estimate_distance_m,
                report.search_radius_m, report.best_score, report.second_score,
                report.is_descriptor_candidate ? "appearance" : "position",
                report.accepted ? "ACCEPTED" : report.rejection.c_str());
  }
  EXPECT_GE(closures.size(), 2U) << "queries " << summary.query_count << ", candidates "
                                 << summary.candidate_count << ", rejected: " << summary.rejections;
  EXPECT_EQ(summary.accepted_count, closures.size());
  expect_closures_match_the_truth(closures, drive.true_poses);
  for (const LoopClosure& closure : closures) {
    EXPECT_LT(closure.target_node_index, closure.query_node_index);
  }
}

TEST(LoopClosure, AppearanceFindsTheStartWhenTheEstimateIsTooFarOffForThePositionSearch) {
  const SyntheticWorld world = make_corridor();
  const Drive drive = make_drive(world, true);
  // 20 degrees: the end of the drive is 8 m from its start on the estimate.
  const std::vector<Eigen::Isometry3d> estimate =
      estimate_with_heading_error_on_the_way_back(drive, 20.0 * kDegreesToRadians);
  LoopClosureParameters parameters = test_parameters();
  parameters.search_radius_m = 1.0;
  parameters.search_radius_per_m = 0.0;
  parameters.maximum_search_radius_m = 1.0;

  LoopClosureSearchSummary summary;
  const std::vector<LoopClosure> closures = find_loop_closures(drive.keyframes, estimate, parameters, {}, &summary);

  ASSERT_FALSE(closures.empty()) << "queries " << summary.query_count << ", candidates "
                                 << summary.candidate_count << ", rejected: " << summary.rejections;
  expect_closures_match_the_truth(closures, drive.true_poses);
  double largest_correction_m = 0.0;
  for (const LoopClosure& closure : closures) largest_correction_m = std::max(largest_correction_m, closure.correction_m);
  EXPECT_GT(largest_correction_m, 2.0 * parameters.maximum_search_radius_m)
      << "a closure should bridge an error that the position search cannot reach";

  parameters.descriptor_candidates = 0;
  EXPECT_TRUE(find_loop_closures(drive.keyframes, estimate, parameters, {}).empty())
      << "without appearance candidates nothing is within the 1 m position search";
}

// A second corridor that looks the same must not close a loop: one wrong closure ruins a good map.
// Appearance candidates that would move the estimate by more than a plausible share of the path are skipped.
TEST(LoopClosure, AnAppearanceMatchThatWouldMoveTheEstimateImplausiblyFarIsNotTried) {
  const SyntheticWorld world = make_corridor();
  const Drive drive = make_drive(world, true);
  // 90 degrees: the end of the drive is 34 m from its start on the estimate, after only 48 m of path.
  const std::vector<Eigen::Isometry3d> estimate =
      estimate_with_heading_error_on_the_way_back(drive, 90.0 * kDegreesToRadians);
  LoopClosureParameters parameters = test_parameters();
  parameters.search_radius_m = 1.0;
  parameters.search_radius_per_m = 0.0;
  parameters.maximum_search_radius_m = 1.0;

  LoopClosureSearchSummary summary;
  const std::vector<LoopClosure> closures = find_loop_closures(drive.keyframes, estimate, parameters, {}, &summary);

  for (const LoopClosure& closure : closures) {
    EXPECT_LT(closure.correction_m, 15.0) << "target " << closure.target_node_index << " query "
                                          << closure.query_node_index;
  }
  EXPECT_NE(summary.rejections.find("implausible"), std::string::npos) << summary.rejections;

  // The same candidates are matched (and found) once the limit allows the correction.
  parameters.descriptor_max_correction_per_m = 1.0;
  double largest_correction_m = 0.0;
  for (const LoopClosure& closure : find_loop_closures(drive.keyframes, estimate, parameters, {})) {
    largest_correction_m = std::max(largest_correction_m, closure.correction_m);
  }
  EXPECT_GT(largest_correction_m, 15.0);
}

TEST(LoopClosure, ADriveThatNeverComesBackHasNoClosure) {
  const SyntheticWorld world = make_corridor();
  const Drive drive = make_drive(world, false);

  LoopClosureSearchSummary summary;
  const std::vector<LoopClosure> closures =
      find_loop_closures(drive.keyframes, drive.true_poses, test_parameters(), {}, &summary);

  EXPECT_TRUE(closures.empty()) << closures.size() << " closures on a one-way drive";
  EXPECT_GT(summary.query_count, 0U);
}

TEST(LoopClosure, QueriesNextToAnExistingClosureAreNotSearchedAgain) {
  const SyntheticWorld world = make_corridor();
  const Drive drive = make_drive(world, true);
  const std::vector<Eigen::Isometry3d> estimate =
      estimate_with_heading_error_on_the_way_back(drive, 6.0 * kDegreesToRadians);
  const LoopClosureParameters parameters = test_parameters();
  const std::vector<LoopClosure> first_round = find_loop_closures(drive.keyframes, estimate, parameters, {});
  ASSERT_FALSE(first_round.empty());

  const std::vector<LoopClosure> second_round =
      find_loop_closures(drive.keyframes, estimate, parameters, first_round);

  for (const LoopClosure& repeated : second_round) {
    for (const LoopClosure& existing : first_round) {
      EXPECT_NE(repeated.query_node_index, existing.query_node_index);
    }
  }
}

TEST(LoopClosure, RejectsMismatchedInputAndUnusableParameters) {
  const SyntheticWorld world = make_corridor();
  const Drive drive = make_drive(world, false);
  std::vector<Eigen::Isometry3d> too_few_poses = drive.true_poses;
  too_few_poses.pop_back();
  EXPECT_THROW(find_loop_closures(drive.keyframes, too_few_poses, test_parameters(), {}), std::invalid_argument);

  LoopClosureParameters negative_shift = test_parameters();
  negative_shift.descriptor_target_shift_m = -1.0;
  EXPECT_THROW(find_loop_closures(drive.keyframes, drive.true_poses, negative_shift, {}), std::invalid_argument);

  LoopClosureParameters negative_correction_share = test_parameters();
  negative_correction_share.descriptor_max_correction_per_m = -0.1;
  EXPECT_THROW(find_loop_closures(drive.keyframes, drive.true_poses, negative_correction_share, {}),
               std::invalid_argument);

  LoopClosureParameters zero_spacing = test_parameters();
  zero_spacing.query_spacing_m = 0.0;
  EXPECT_THROW(find_loop_closures(drive.keyframes, drive.true_poses, zero_spacing, {}), std::invalid_argument);
}

}  // namespace
}  // namespace lightnav_localization
