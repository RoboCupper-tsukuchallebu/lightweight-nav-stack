#include <gtest/gtest.h>

#include <cmath>

#include "lightnav_localization/deskew.hpp"
#include "lightnav_localization/downsample.hpp"
#include "lightnav_localization/matcher.hpp"
#include "synthetic_world.hpp"

namespace {

using lightnav_localization::deskew_scan;
using lightnav_localization::downsample_voxel;
using lightnav_localization::match_scan;
using lightnav_localization::MatcherParameters;
using lightnav_localization::MatchResult;
using lightnav_localization::PointList;
using lightnav_localization::PoseBuffer;
using lightnav_localization::PreparedTarget;
using lightnav_localization::TimedPoint;
using lightnav_localization::testing::arc_pose;
using lightnav_localization::testing::SyntheticWorld;

constexpr double kScanDurationSec = 0.1;
constexpr double kTurnRateRps = 0.4;
constexpr double kSpeedMps = 0.6;
constexpr double kScanVoxelM = 0.5;
constexpr double kMapSpacingM = 0.2;
constexpr double kCorrectedMaxYawErrorRad = 0.002;
constexpr double kUncorrectedMinYawErrorRad = 0.01;

MatcherParameters make_parameters() {
  MatcherParameters parameters;
  parameters.max_correspondence_distance_m = 0.3;
  parameters.max_iterations = 20;
  return parameters;
}

Eigen::Isometry3d pose_at(double time) { return arc_pose(time, {24.0, 0.0}, 0.0, kSpeedMps, kTurnRateRps); }

struct Scenario {
  SyntheticWorld world{{{0.0, 0.0}, {30.0, 0.0}, {30.0, 30.0}}};
  PreparedTarget target{world.sample_map(kMapSpacingM)};
  PoseBuffer odometry;
  Scenario() {
    for (double time = -0.2; time <= 0.3; time += 0.02) odometry.add(time, pose_at(time));
  }
};

PointList raw_positions(const std::vector<TimedPoint>& scan) {
  PointList points;
  for (const TimedPoint& point : scan) points.push_back(point.position);
  return points;
}

Eigen::Isometry3d offset_pose(const Eigen::Isometry3d& pose, double dx, double dy, double dyaw) {
  Eigen::Isometry3d shifted = pose;
  shifted.translation() += Eigen::Vector3d(dx, dy, 0.0);
  shifted.linear() = Eigen::AngleAxisd(dyaw, Eigen::Vector3d::UnitZ()).toRotationMatrix() * pose.linear();
  return shifted;
}

double yaw_error_rad(const Eigen::Isometry3d& estimate, const Eigen::Isometry3d& truth) {
  return Eigen::AngleAxisd(truth.rotation().transpose() * estimate.rotation()).angle();
}

PointList corrected_scan(const Scenario& scenario, bool stamp_at_scan_end) {
  const double stamp = stamp_at_scan_end ? kScanDurationSec : 0.0;
  const auto scan = scenario.world.synthesize_scan(pose_at, 0.0, kScanDurationSec, stamp_at_scan_end);
  return downsample_voxel(*deskew_scan(scan, stamp, scenario.odometry, Eigen::Isometry3d::Identity()), kScanVoxelM);
}

}  // namespace

// Acceptance B (revised, pending the user's confirmation): the effect of deskewing is measured as
// the pose error after matching from the true pose.
TEST(Matcher, deskew_reduces_yaw_error_after_matching_for_both_time_signs) {
  const Scenario scenario;
  for (const bool stamp_at_scan_end : {true, false}) {
    const double stamp = stamp_at_scan_end ? kScanDurationSec : 0.0;
    const auto scan = scenario.world.synthesize_scan(pose_at, 0.0, kScanDurationSec, stamp_at_scan_end);

    const MatchResult with_correction = match_scan(scenario.target, corrected_scan(scenario, stamp_at_scan_end),
                                                   pose_at(stamp), make_parameters());
    const MatchResult without_correction = match_scan(
        scenario.target, downsample_voxel(raw_positions(scan), kScanVoxelM), pose_at(stamp), make_parameters());

    EXPECT_LE(yaw_error_rad(with_correction.target_to_source, pose_at(stamp)), kCorrectedMaxYawErrorRad)
        << "stamp_at_scan_end=" << stamp_at_scan_end;
    EXPECT_GE(yaw_error_rad(without_correction.target_to_source, pose_at(stamp)), kUncorrectedMinYawErrorRad)
        << "stamp_at_scan_end=" << stamp_at_scan_end;
  }
}

TEST(Matcher, recovers_the_true_pose_from_a_shifted_prediction_and_accepts) {
  const Scenario scenario;
  const PointList scan = corrected_scan(scenario, true);
  const Eigen::Isometry3d truth = pose_at(kScanDurationSec);

  const MatchResult result = match_scan(scenario.target, scan, offset_pose(truth, 0.1, -0.1, 0.02), make_parameters());

  EXPECT_TRUE(result.is_accepted);
  EXPECT_LT((result.target_to_source.translation() - truth.translation()).norm(), 0.05);
  EXPECT_LT(yaw_error_rad(result.target_to_source, truth), 0.005);
}

TEST(Matcher, rejects_only_by_inlier_ratio) {
  const Scenario scenario;
  MatcherParameters parameters = make_parameters();
  parameters.inlier_ratio_min = 1.01;  // unreachable

  const MatchResult result =
      match_scan(scenario.target, corrected_scan(scenario, true), pose_at(kScanDurationSec), parameters);

  EXPECT_TRUE(result.converged);
  EXPECT_LE(result.inlier_ratio, 1.0);
  EXPECT_FALSE(result.is_accepted);
}

TEST(Matcher, rejects_only_by_translation_gate) {
  const Scenario scenario;
  MatcherParameters parameters = make_parameters();
  parameters.gate_translation_m = 0.01;

  const MatchResult result = match_scan(scenario.target, corrected_scan(scenario, true),
                                        offset_pose(pose_at(kScanDurationSec), 0.1, -0.1, 0.0), parameters);

  EXPECT_TRUE(result.converged);
  EXPECT_GE(result.inlier_ratio, parameters.inlier_ratio_min);
  EXPECT_FALSE(result.is_accepted);
}

TEST(Matcher, rejects_only_by_rotation_gate) {
  const Scenario scenario;
  MatcherParameters parameters = make_parameters();
  parameters.gate_rotation_rad = 0.001;

  const MatchResult result = match_scan(scenario.target, corrected_scan(scenario, true),
                                        offset_pose(pose_at(kScanDurationSec), 0.0, 0.0, 0.05), parameters);

  EXPECT_TRUE(result.converged);
  EXPECT_GE(result.inlier_ratio, parameters.inlier_ratio_min);
  EXPECT_FALSE(result.is_accepted);
}

TEST(Matcher, too_few_points_is_not_converged) {
  const MatchResult result = match_scan(PreparedTarget(PointList{}), {}, Eigen::Isometry3d::Identity(), MatcherParameters{});
  EXPECT_FALSE(result.converged);
  EXPECT_FALSE(result.is_accepted);
}

TEST(Matcher, vgicp_recovers_shifted_predictions) {
  const Scenario scenario;
  MatcherParameters parameters = make_parameters();
  parameters.method = "vgicp";
  parameters.vgicp_voxel_m = 0.3;
  parameters.max_correspondence_distance_m = 1.0;
  parameters.max_iterations = 60;
  parameters.gate_translation_m = 2.0;
  const PreparedTarget target(scenario.world.sample_map(kMapSpacingM), parameters);
  const auto truth = pose_at(kScanDurationSec);
  for (const double offset : {0.1, 0.5}) {
    const auto result = match_scan(target, corrected_scan(scenario, true),
                                   offset_pose(truth, offset, -offset, 0.02), parameters);
    EXPECT_TRUE(result.converged) << offset;
    EXPECT_LT((result.target_to_source.translation() - truth.translation()).norm(), 0.15) << offset;
  }
}
