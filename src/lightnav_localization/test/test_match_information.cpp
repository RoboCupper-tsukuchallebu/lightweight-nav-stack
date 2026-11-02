#include <gtest/gtest.h>

#include "lightnav_localization/downsample.hpp"
#include "lightnav_localization/lidar_odometry.hpp"
#include "lightnav_localization/match_information.hpp"
#include "synthetic_world.hpp"

using namespace lightnav_localization;
using namespace lightnav_localization::testing;

TEST(MatchInformation, ClampsDegenerateDirectionToMaxSigma) {
  Eigen::Matrix<double, 6, 6> hessian = Eigen::Matrix<double, 6, 6>::Identity() * 1e6;
  hessian(3, 3) = 1e-6;  // tx unobservable
  const Eigen::Matrix<double, 6, 6> information = information_from_hessian(hessian, InformationLimits{});
  EXPECT_NEAR(information(3, 3), 1.0 / (10.0 * 10.0), 1e-9);
  EXPECT_NEAR(information(4, 4), 1.0 / (0.02 * 0.02), 1e-6);  // floor
  EXPECT_NEAR(information(0, 0), 1.0 / (0.005 * 0.005), 1e-3);
}

// Confirms the small_gicp Hessian order (rotation first, then translation) from a real match: in a
// very long corridor the along-axis (x) translation is the weakest, so H(3,3) << H(4,4) only if
// indices 3..5 are translation.
TEST(MatchInformation, HessianOrderIsRotationThenTranslation) {
  const SyntheticWorld world({{0.0, 0.0}, {400.0, 0.0}});
  const Eigen::Isometry3d pose = arc_pose(0.0, Eigen::Vector2d(200.0, 0.0), 0.0, 0.0, 0.0);
  const auto timed = world.synthesize_scan([&pose](double) { return pose; }, 0.0, 0.1, true, 360, 8);
  PointList points;
  for (const TimedPoint& point : timed) points.push_back(point.position);
  const PointList cloud = downsample_voxel(points, 0.5);
  LidarOdometry odometry(LidarOdometryParameters{});
  odometry.add_keyframe(pose, cloud);
  const MatchResult result = odometry.match(cloud, pose, 0.0);
  EXPECT_LT(result.information(3, 3), 0.05 * result.information(4, 4));
  // Rotation about the corridor axis (roll, index 0) is observed by the walls and floor.
  EXPECT_GT(result.information(0, 0), 0.0);
}
