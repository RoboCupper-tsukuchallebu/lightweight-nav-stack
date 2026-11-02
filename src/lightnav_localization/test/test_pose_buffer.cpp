#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "lightnav_localization/pose_buffer.hpp"

using lightnav_localization::PoseBuffer;

namespace {

Eigen::Isometry3d pose_with(double x, double yaw) {
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation().x() = x;
  pose.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  return pose;
}

}  // namespace

TEST(PoseBuffer, interpolates_translation_linearly_and_rotation_by_slerp) {
  PoseBuffer buffer;
  buffer.add(1.0, pose_with(0.0, 0.0));
  buffer.add(2.0, pose_with(4.0, 0.2));

  const auto middle = buffer.interpolate(1.25);

  ASSERT_TRUE(middle.has_value());
  EXPECT_NEAR(1.0, middle->translation().x(), 1e-12);
  EXPECT_NEAR(0.05, Eigen::AngleAxisd(middle->rotation()).angle(), 1e-12);
}

TEST(PoseBuffer, returns_exact_poses_at_the_boundaries) {
  PoseBuffer buffer;
  buffer.add(1.0, pose_with(0.0, 0.0));
  buffer.add(2.0, pose_with(4.0, 0.2));

  EXPECT_NEAR(0.0, buffer.interpolate(1.0)->translation().x(), 1e-12);
  EXPECT_NEAR(4.0, buffer.interpolate(2.0)->translation().x(), 1e-12);
}

TEST(PoseBuffer, rejects_outside_empty_and_nan_stamps) {
  PoseBuffer buffer;
  EXPECT_FALSE(buffer.interpolate(0.0).has_value());
  buffer.add(1.0, pose_with(0.0, 0.0));
  buffer.add(2.0, pose_with(4.0, 0.2));

  EXPECT_FALSE(buffer.interpolate(0.99).has_value());
  EXPECT_FALSE(buffer.interpolate(2.01).has_value());
  EXPECT_FALSE(buffer.interpolate(std::numeric_limits<double>::quiet_NaN()).has_value());
}
