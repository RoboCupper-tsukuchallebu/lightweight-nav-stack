#include <gtest/gtest.h>

#include "lightnav_localization/corrector.hpp"

using namespace lightnav_localization;

namespace {
Eigen::Isometry3d odom_at(double x_m) {
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation().x() = x_m;
  return pose;
}
}  // namespace

TEST(Corrector, LimitsOutputChangeRateAndFlagsLargeStep) {
  Corrector corrector(CorrectorParameters{});
  corrector.reset(Eigen::Isometry3d::Identity());
  Eigen::Isometry3d target = Eigen::Isometry3d::Identity();
  target.translation() = Eigen::Vector3d(1.5, 0.0, 0.0);
  corrector.set_target(target);
  const Eigen::Isometry3d odom = odom_at(0.0);
  EXPECT_TRUE(corrector.is_large_correction(odom));

  constexpr double kStepSec = 0.02;
  Eigen::Vector3d previous = (corrector.applied() * odom).translation();
  for (int step = 0; step < 100; ++step) {
    const Eigen::Vector3d current = (corrector.step(kStepSec, odom) * odom).translation();
    EXPECT_LE((current - previous).norm() / kStepSec, 0.2 + 1e-9);
    previous = current;
  }
  EXPECT_NEAR(previous.x(), 0.4, 1e-6);
  for (int step = 0; step < 500; ++step) corrector.step(0.02, odom);
  EXPECT_NEAR((corrector.applied() * odom).translation().x(), 1.5, 1e-6);
  EXPECT_FALSE(corrector.is_large_correction(odom));
}

TEST(Corrector, LimitsRotationRate) {
  Corrector corrector(CorrectorParameters{});
  corrector.reset(Eigen::Isometry3d::Identity());
  Eigen::Isometry3d target = Eigen::Isometry3d::Identity();
  target.linear() = Eigen::AngleAxisd(0.5, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  corrector.set_target(target);
  const Eigen::Isometry3d odom = odom_at(0.0);
  corrector.step(1.0, odom);
  EXPECT_NEAR(Eigen::AngleAxisd((corrector.applied() * odom).rotation()).angle(), 0.1, 1e-9);
}

// Review high 2: a pure yaw correction of the map->odom transform, 100 m from the odom origin,
// moves the base by up to 100 m x 0.01 rad = 1 m if applied on T_map_odom. At the base the
// correction is a rotation in place: the base position must stay put (<= 0.2 m/s) and no large
// correction may be flagged.
TEST(Corrector, YawCorrectionFarFromOdomOriginDoesNotSwingOutput) {
  Corrector corrector(CorrectorParameters{});
  corrector.reset(Eigen::Isometry3d::Identity());
  Eigen::Isometry3d target = Eigen::Isometry3d::Identity();
  // The corrected map->odom keeps the base at the same map position (rotation about the base).
  const Eigen::Isometry3d odom = odom_at(100.0);
  const Eigen::Isometry3d base_in_map = odom;
  Eigen::Isometry3d rotated_base = base_in_map;
  rotated_base.linear() = Eigen::AngleAxisd(0.01, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  target = rotated_base * odom.inverse();
  corrector.set_target(target);
  EXPECT_FALSE(corrector.is_large_correction(odom));
  EXPECT_NEAR(corrector.pending_correction_m(odom), 0.0, 1e-9);
  constexpr double kStepSec = 0.02;
  Eigen::Vector3d previous = (corrector.applied() * odom).translation();
  for (int step = 0; step < 200; ++step) {
    const Eigen::Vector3d current = (corrector.step(kStepSec, odom) * odom).translation();
    EXPECT_LE((current - previous).norm() / kStepSec, 0.2 + 1e-9);
    previous = current;
  }
  EXPECT_NEAR(previous.x(), 100.0, 1e-6);
}

TEST(Corrector, NegativeElapsedTimeDoesNotMove) {
  Corrector corrector(CorrectorParameters{});
  corrector.reset(Eigen::Isometry3d::Identity());
  Eigen::Isometry3d target = Eigen::Isometry3d::Identity();
  target.translation().x() = 1.0;
  corrector.set_target(target);
  corrector.step(-0.5, odom_at(0.0));
  EXPECT_NEAR(corrector.applied().translation().x(), 0.0, 1e-12);
}

// Decision (tentative): z, roll and pitch are not slewed; only x, y, yaw are.
TEST(Corrector, HeightAndTiltFollowTargetImmediately) {
  Corrector corrector(CorrectorParameters{});
  corrector.reset(Eigen::Isometry3d::Identity());
  Eigen::Isometry3d target = Eigen::Isometry3d::Identity();
  target.translation().z() = 0.8;
  target.linear() = Eigen::AngleAxisd(0.17, Eigen::Vector3d::UnitY()).toRotationMatrix();
  corrector.set_target(target);
  const Eigen::Isometry3d odom = odom_at(0.0);
  const Eigen::Isometry3d output = corrector.step(0.02, odom) * odom;
  EXPECT_NEAR(output.translation().z(), 0.8, 1e-9);
  EXPECT_NEAR(std::asin(-output.rotation()(2, 0)), 0.17, 1e-9);
  EXPECT_FALSE(corrector.is_large_correction(odom));  // pending counts the horizontal part only
}
