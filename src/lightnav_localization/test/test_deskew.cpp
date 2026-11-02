#include <gtest/gtest.h>

#include "lightnav_localization/deskew.hpp"
#include "synthetic_world.hpp"

using namespace lightnav_localization;
using namespace lightnav_localization::testing;

namespace {

constexpr double kScanDurationSec = 0.1;
constexpr double kSpeedMps = 0.6;
constexpr double kYawRateRps = 0.4;
constexpr double kOdometryPeriodSec = 0.02;

PoseBuffer make_odometry(const Eigen::Vector2d& start_xy, double start_yaw) {
  PoseBuffer buffer;
  for (double time = -0.2; time <= 0.3; time += kOdometryPeriodSec) {
    buffer.add(time, arc_pose(time, start_xy, start_yaw, kSpeedMps, kYawRateRps));
  }
  return buffer;
}

}  // namespace

TEST(Deskew, negative_and_positive_time_offsets_describe_the_same_scan) {
  const SyntheticWorld world({{0.0, 0.0}, {30.0, 0.0}, {30.0, 30.0}});
  const auto pose_at = [](double time) { return arc_pose(time, {10.0, 0.0}, 0.0, kSpeedMps, kYawRateRps); };
  const PoseBuffer odometry = make_odometry({10.0, 0.0}, 0.0);
  const auto scan_end_stamped = world.synthesize_scan(pose_at, 0.0, kScanDurationSec, true);
  const auto scan_start_stamped = world.synthesize_scan(pose_at, 0.0, kScanDurationSec, false);

  const auto from_end = deskew_scan(scan_end_stamped, kScanDurationSec, odometry, Eigen::Isometry3d::Identity());
  const auto from_start = deskew_scan(scan_start_stamped, 0.0, odometry, Eigen::Isometry3d::Identity());

  ASSERT_TRUE(from_end && from_start);
  ASSERT_EQ(from_end->size(), from_start->size());
  ASSERT_FALSE(from_end->empty());
  EXPECT_LT(scan_end_stamped.front().time_offset_sec, 0.0F);
  EXPECT_GE(scan_start_stamped.front().time_offset_sec, 0.0F);
  const Eigen::Isometry3d end_pose = pose_at(kScanDurationSec);
  const Eigen::Isometry3d start_pose = pose_at(0.0);
  for (std::size_t index = 0; index < from_end->size(); ++index) {
    const Eigen::Vector3d in_odom_from_end = end_pose * (*from_end)[index].cast<double>();
    const Eigen::Vector3d in_odom_from_start = start_pose * (*from_start)[index].cast<double>();
    ASSERT_LT((in_odom_from_end - in_odom_from_start).norm(), 1e-3);
  }
}

TEST(Deskew, drops_scan_when_odometry_does_not_cover_the_point_times) {
  PoseBuffer odometry;
  odometry.add(0.0, Eigen::Isometry3d::Identity());
  odometry.add(0.05, Eigen::Isometry3d::Identity());
  const std::vector<TimedPoint> scan = {{Eigen::Vector3f(1, 0, 0), 0.0F}, {Eigen::Vector3f(2, 0, 0), 0.1F}};

  EXPECT_FALSE(deskew_scan(scan, 0.0, odometry, Eigen::Isometry3d::Identity()).has_value());
}
