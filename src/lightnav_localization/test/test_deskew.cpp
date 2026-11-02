#include <gtest/gtest.h>

#include <algorithm>

#include "lightnav_localization/deskew.hpp"
#include "synthetic_world.hpp"

namespace {

using lightnav_localization::deskew_scan;
using lightnav_localization::PointList;
using lightnav_localization::PoseBuffer;
using lightnav_localization::TimedPoint;
using lightnav_localization::testing::arc_pose;
using lightnav_localization::testing::SyntheticWorld;

constexpr double kScanDurationSec = 0.1;
constexpr double kSpeedMps = 0.6;
constexpr double kYawRateRps = 0.4;
constexpr double kOdometryPeriodSec = 0.02;
constexpr double kCorrectedMaxErrorM = 1e-3;
constexpr double kUncorrectedMinP95ErrorM = 0.1;
constexpr double kP95Fraction = 0.95;

Eigen::Isometry3d base_pose_at(double time) {
  return arc_pose(time, {24.0, 0.0}, 0.0, kSpeedMps, kYawRateRps);
}

Eigen::Isometry3d non_identity_extrinsic() {
  Eigen::Isometry3d extrinsic = Eigen::Isometry3d::Identity();
  extrinsic.translation() = Eigen::Vector3d(0.3, -0.1, 0.5);
  extrinsic.linear() = (Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitZ()) *
                        Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitY()))
                           .toRotationMatrix();
  return extrinsic;
}

PoseBuffer make_odometry() {
  PoseBuffer buffer;
  for (double time = -0.2; time <= 0.3; time += kOdometryPeriodSec) buffer.add(time, base_pose_at(time));
  return buffer;
}

double percentile(std::vector<double> values, double fraction) {
  std::sort(values.begin(), values.end());
  return values[static_cast<std::size_t>(fraction * static_cast<double>(values.size() - 1))];
}

}  // namespace

// Ground truth residual, both time_offset signs and both extrinsics: each point's true position is
// pose(stamp + time) * extrinsic * p; the deskewed point, moved by pose(stamp), must agree.
TEST(Deskew, matches_ground_truth_for_both_time_signs_and_extrinsics) {
  const SyntheticWorld world({{0.0, 0.0}, {30.0, 0.0}, {30.0, 30.0}});
  const PoseBuffer odometry = make_odometry();
  for (const Eigen::Isometry3d& lidar_to_base : {Eigen::Isometry3d::Identity(), non_identity_extrinsic()}) {
    for (const bool stamp_at_scan_end : {true, false}) {
      const double stamp = stamp_at_scan_end ? kScanDurationSec : 0.0;
      const auto lidar_pose_at = [&](double time) { return base_pose_at(time) * lidar_to_base; };
      const auto scan = world.synthesize_scan(lidar_pose_at, 0.0, kScanDurationSec, stamp_at_scan_end);
      ASSERT_FALSE(scan.empty());
      ASSERT_EQ(stamp_at_scan_end, scan.front().time_offset_sec < 0.0F);

      const auto corrected = deskew_scan(scan, stamp, odometry, lidar_to_base);

      ASSERT_TRUE(corrected.has_value());
      ASSERT_EQ(scan.size(), corrected->size());
      std::vector<double> corrected_errors;
      std::vector<double> uncorrected_errors;
      for (std::size_t index = 0; index < scan.size(); ++index) {
        const Eigen::Vector3d truth =
            base_pose_at(stamp + scan[index].time_offset_sec) * lidar_to_base * scan[index].position.cast<double>();
        corrected_errors.push_back((base_pose_at(stamp) * (*corrected)[index].cast<double>() - truth).norm());
        uncorrected_errors.push_back(
            (base_pose_at(stamp) * lidar_to_base * scan[index].position.cast<double>() - truth).norm());
      }
      EXPECT_LE(*std::max_element(corrected_errors.begin(), corrected_errors.end()), kCorrectedMaxErrorM)
          << "stamp_at_scan_end=" << stamp_at_scan_end;
      EXPECT_GE(percentile(uncorrected_errors, kP95Fraction), kUncorrectedMinP95ErrorM)
          << "stamp_at_scan_end=" << stamp_at_scan_end;
    }
  }
}

TEST(Deskew, drops_scan_when_odometry_does_not_cover_the_point_times) {
  PoseBuffer odometry;
  odometry.add(0.0, Eigen::Isometry3d::Identity());
  odometry.add(0.05, Eigen::Isometry3d::Identity());
  const std::vector<TimedPoint> scan = {{Eigen::Vector3f(1, 0, 0), 0.0F}, {Eigen::Vector3f(2, 0, 0), 0.1F}};

  EXPECT_FALSE(deskew_scan(scan, 0.0, odometry, Eigen::Isometry3d::Identity()).has_value());
}
