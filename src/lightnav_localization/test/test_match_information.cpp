#include <gtest/gtest.h>

#include "lightnav_localization/downsample.hpp"
#include "lightnav_localization/lidar_odometry.hpp"
#include "lightnav_localization/match_information.hpp"
#include "synthetic_world.hpp"

using namespace lightnav_localization;
using namespace lightnav_localization::testing;

namespace {
// Real GICP Hessian of a scan matched against itself, in a corridor with or without end caps.
Eigen::Matrix<double, 6, 6> corridor_hessian(bool has_end_caps, double max_range_m = 60.0) {
  SyntheticWorld world({{0.0, 0.0}, {400.0, 0.0}}, has_end_caps);
  world.set_max_range(max_range_m);
  const Eigen::Isometry3d pose = arc_pose(0.0, Eigen::Vector2d(200.0, 0.0), 0.0, 0.0, 0.0);
  const auto timed = world.synthesize_scan([&pose](double) { return pose; }, 0.0, 0.1, true, 360, 8);
  PointList points;
  for (const TimedPoint& point : timed) points.push_back(point.position);
  const PointList cloud = downsample_voxel(points, 0.5);
  LidarOdometry odometry(LidarOdometryParameters{});
  odometry.add_keyframe(pose, cloud);
  return odometry.match(cloud, pose, 0.0).information;
}
}  // namespace

TEST(MatchInformation, ClampsDegenerateDirectionToMaxSigma) {
  Eigen::Matrix<double, 6, 6> hessian = Eigen::Matrix<double, 6, 6>::Identity() * 1e6;
  hessian(3, 3) = 1e-6;  // tx unobservable
  const Eigen::Matrix<double, 6, 6> information = information_from_hessian(hessian, InformationLimits{});
  EXPECT_NEAR(information(3, 3), 1.0 / (10.0 * 10.0), 1e-6);
  EXPECT_NEAR(information(4, 4), 1.0 / (0.02 * 0.02), 1e-3);
}

TEST(MatchInformation, HessianOrderIsRotationThenTranslation) {
  const Eigen::Matrix<double, 6, 6> hessian = corridor_hessian(true);
  EXPECT_LT(hessian(3, 3), hessian(4, 4));  // along-corridor x is weaker than across y
}

// Review high 1: with the real (normalized-covariance) Hessian of a corridor whose ends cannot be
// seen, the sliding direction must come out weak (sigma >= 1 m) while the lateral direction keeps
// the floor-level sigma. 
TEST(MatchInformation, RealHessianOfOpenCorridorWeakensSlidingDirectionOnly) {
  const InformationLimits limits;
  const Eigen::Matrix<double, 6, 6> open_information =
      information_from_hessian(corridor_hessian(false), limits);
  // Sigma = sqrt of the covariance (inverse information) diagonal: the information diagonal alone
  // would hide the coupling between translation and rotation.
  const Eigen::Matrix<double, 6, 6> open_covariance = open_information.inverse();
  const double sliding_sigma_m = std::sqrt(open_covariance(3, 3));
  const double lateral_sigma_m = std::sqrt(open_covariance(4, 4));
  std::printf("open corridor: sliding sigma %.3f m, lateral sigma %.3f m\n", sliding_sigma_m, lateral_sigma_m);
  EXPECT_GE(sliding_sigma_m, 1.0);
  EXPECT_LE(lateral_sigma_m, 0.05);

}

// Review medium 2: the verdict must not depend on how far the sensor sees. Outdoors the effective
// range is often 40-50 m; the sliding direction must be weak there too.
TEST(MatchInformation, SlidingDirectionIsWeakAtEveryEffectiveRange) {
  for (const double range_m : {20.0, 30.0, 40.0, 45.0, 50.0, 60.0}) {
    const Eigen::Matrix<double, 6, 6> covariance =
        information_from_hessian(corridor_hessian(false, range_m), InformationLimits{}).inverse();
    std::printf("range %.0f m: sliding sigma %.2f m, lateral sigma %.3f m\n", range_m,
                std::sqrt(covariance(3, 3)), std::sqrt(covariance(4, 4)));
    EXPECT_GE(std::sqrt(covariance(3, 3)), 1.0) << "range " << range_m;
    EXPECT_LE(std::sqrt(covariance(4, 4)), 0.05) << "range " << range_m;
  }
}
