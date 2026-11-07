#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "lightnav_localization/initial_alignment.hpp"
#include "synthetic_world.hpp"

using namespace lightnav_localization;
using namespace lightnav_localization::testing;

namespace {

PointList make_scan(const SyntheticWorld& world, const Eigen::Vector3d& position,
                    double yaw_rad) {
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() = position;
  pose.linear() =
      Eigen::AngleAxisd(yaw_rad, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const std::vector<TimedPoint> timed_points = world.synthesize_scan(
      [&pose](double /*time_sec*/) { return pose; }, 10.0, 0.1, true, 360, 8);
  PointList points;
  points.reserve(timed_points.size());
  for (const TimedPoint& timed_point : timed_points) points.push_back(timed_point.position);
  return points;
}

}  // namespace

TEST(InitialAlignment, RecoversUnknownYawFromStaticScan) {
  const SyntheticWorld world({Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(30.0, 0.0)}, true);
  const PointList map = world.sample_map(0.2);
  const Eigen::Vector3d position(6.0, 0.0, 1.0);
  const double truth_yaw = 0.7;
  const PointList scan = make_scan(world, position, truth_yaw);

  InitialAlignmentParameters parameters;
  const InitialAlignmentResult result =
      align_initial_yaw(map, scan, position, parameters);
  ASSERT_TRUE(result.accepted) << result.error;
  EXPECT_GE(result.inlier_ratio, parameters.inlier_ratio_min);
  EXPECT_NEAR(result.yaw_rad, truth_yaw, 5.0 * M_PI / 180.0);
}

TEST(InitialAlignment, RemovesTheAntennaLeverArmBeforeMatching) {
  const SyntheticWorld world({Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(30.0, 0.0)}, true);
  const PointList map = world.sample_map(0.2);
  const Eigen::Vector3d base_position(6.0, 0.0, 1.0);
  const double truth_yaw = 0.7;
  const PointList scan = make_scan(world, base_position, truth_yaw);

  Eigen::Isometry3d base_pose = Eigen::Isometry3d::Identity();
  base_pose.translation() = base_position;
  base_pose.linear() =
      Eigen::AngleAxisd(truth_yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const Eigen::Vector3d lever_arm(-0.2, 0.0, 0.7);
  const Eigen::Vector3d antenna_position = base_pose * lever_arm;

  InitialAlignmentParameters parameters;
  const InitialAlignmentResult result =
      align_initial_yaw(map, scan, antenna_position, parameters, lever_arm);
  ASSERT_TRUE(result.accepted) << result.error;
  EXPECT_NEAR(result.yaw_rad, truth_yaw, 5.0 * M_PI / 180.0);
  EXPECT_LT((result.map_to_base.translation() - base_position).norm(), 0.1);
}

TEST(InitialAlignment, RejectsWhenTheMapHasNoPointsNearby) {
  const PointList map;
  const PointList scan{Eigen::Vector3f(1.0F, 0.0F, 0.0F),
                       Eigen::Vector3f(2.0F, 0.0F, 0.0F)};
  InitialAlignmentParameters parameters;
  const InitialAlignmentResult result =
      align_initial_yaw(map, scan, Eigen::Vector3d(0.0, 0.0, 0.0), parameters);
  EXPECT_FALSE(result.accepted);
  EXPECT_FALSE(result.error.empty());
}
