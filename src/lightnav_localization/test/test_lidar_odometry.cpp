#include <gtest/gtest.h>

#include "lightnav_localization/downsample.hpp"
#include "lightnav_localization/lidar_odometry.hpp"
#include "synthetic_world.hpp"

using namespace lightnav_localization;
using namespace lightnav_localization::testing;

namespace {
PointList scan_at(const SyntheticWorld& world, const Eigen::Isometry3d& pose) {
  const auto scan = world.synthesize_scan([&pose](double) { return pose; }, 0.0, 0.1, true, 360, 8);
  PointList points;
  for (const TimedPoint& point : scan) points.push_back(point.position);
  return downsample_voxel(points, 0.5);
}

Eigen::Isometry3d pose_at_x(double x) { return arc_pose(0.0, Eigen::Vector2d(x, 0.0), 0.0, 0.0, 0.0); }
}  // namespace

class LidarOdometryTest : public ::testing::Test {
 protected:
  LidarOdometryTest() : world_({{0.0, 0.0}, {40.0, 0.0}}), odometry_(LidarOdometryParameters{}) {
    for (const double x : {10.0, 11.0, 12.0}) odometry_.add_keyframe(pose_at_x(x), scan_at(world_, pose_at_x(x)));
  }
  SyntheticWorld world_;
  LidarOdometry odometry_;
};

TEST_F(LidarOdometryTest, RecoversTrueMotionFromSlightlyWrongPrediction) {
  const Eigen::Isometry3d truth = pose_at_x(12.6);
  Eigen::Isometry3d prediction = truth;
  prediction.translation().x() += 0.15;  // wheel odometry error
  const MatchResult result = odometry_.match(scan_at(world_, truth), prediction, 0.6);
  ASSERT_TRUE(result.is_accepted);
  EXPECT_LT((result.target_to_source.translation() - truth.translation()).norm(), 0.1);
}

// C: a match that ends up far from the prediction must not become a factor. Offset is across the
// corridor: along it the corridor is degenerate (see the PR notes), so only the lateral direction
// is observable and rejectable.
TEST_F(LidarOdometryTest, RejectsMatchFarFromPredictionAcrossCorridor) {
  const Eigen::Isometry3d truth = pose_at_x(12.6);
  Eigen::Isometry3d wrong_prediction = truth;
  wrong_prediction.translation().y() += 1.5;
  const MatchResult result = odometry_.match(scan_at(world_, truth), wrong_prediction, 0.6);
  EXPECT_FALSE(result.is_accepted);
}
