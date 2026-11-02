#include <gtest/gtest.h>

#include "lightnav_localization/corrector.hpp"

using namespace lightnav_localization;

TEST(Corrector, LimitsOutputChangeRateAndFlagsLargeStep) {
  CorrectorParameters parameters;  // 0.2 m/s, 0.1 rad/s, threshold 1.0 m
  Corrector corrector(parameters);
  corrector.reset(Eigen::Isometry3d::Identity());
  Eigen::Isometry3d target = Eigen::Isometry3d::Identity();
  target.translation() = Eigen::Vector3d(1.5, 0.0, 0.0);
  corrector.set_target(target);
  EXPECT_TRUE(corrector.is_large_correction());

  constexpr double kStepSec = 0.02;
  Eigen::Vector3d previous = corrector.applied().translation();
  for (int step = 0; step < 100; ++step) {
    const Eigen::Vector3d current = corrector.step(kStepSec).translation();
    EXPECT_LE((current - previous).norm() / kStepSec, 0.2 + 1e-9);
    previous = current;
  }
  EXPECT_NEAR(previous.x(), 0.4, 1e-6);  // 2 s at 0.2 m/s
  for (int step = 0; step < 500; ++step) corrector.step(0.02);
  EXPECT_NEAR(corrector.applied().translation().x(), 1.5, 1e-6);
  EXPECT_FALSE(corrector.is_large_correction());
}

TEST(Corrector, LimitsRotationRate) {
  Corrector corrector(CorrectorParameters{});
  corrector.reset(Eigen::Isometry3d::Identity());
  Eigen::Isometry3d target = Eigen::Isometry3d::Identity();
  target.linear() = Eigen::AngleAxisd(0.5, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  corrector.set_target(target);
  corrector.step(1.0);
  EXPECT_NEAR(Eigen::AngleAxisd(corrector.applied().rotation()).angle(), 0.1, 1e-9);
}
