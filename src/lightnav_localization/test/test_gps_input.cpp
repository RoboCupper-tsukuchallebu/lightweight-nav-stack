#include <gtest/gtest.h>

#include <cmath>

#include "lightnav_localization/gps_input.hpp"

using namespace lightnav_localization;

namespace {
constexpr double kMetersPerDegreeLatitude = 111320.0;
const GpsOrigin kOrigin{36.0, 140.0, 20.0};

GpsFix fix_at_enu(double east_m, double north_m) {
  GpsFix fix;
  fix.latitude_deg = kOrigin.latitude_deg + north_m / kMetersPerDegreeLatitude;
  fix.longitude_deg = kOrigin.longitude_deg +
                      east_m / (kMetersPerDegreeLatitude * std::cos(kOrigin.latitude_deg * M_PI / 180.0));
  fix.altitude_m = kOrigin.altitude_m;
  fix.status = GpsFixStatus::kFix;
  return fix;
}
}  // namespace

TEST(GpsInput, ConvertsToEnuAboutOrigin) {
  const GpsConverter converter(kOrigin, GpsParameters{});
  const Eigen::Vector3d enu = converter.to_enu(fix_at_enu(30.0, 100.0));
  EXPECT_NEAR(enu.x(), 30.0, 0.5);  // spherical approximation in the test helper
  EXPECT_NEAR(enu.y(), 100.0, 0.5);
}

TEST(GpsInput, RejectsTwentyMeterJumpButAcceptsGoodFix) {
  const GpsConverter converter(kOrigin, GpsParameters{});
  Eigen::Isometry3d predicted = Eigen::Isometry3d::Identity();
  predicted.translation() = Eigen::Vector3d(10.0, 10.0, 0.0);
  const Eigen::Vector3d good = converter.to_enu(fix_at_enu(10.5, 9.5));
  EXPECT_TRUE(converter.evaluate(fix_at_enu(10.5, 9.5), predicted, 0.1).measurement.has_value());
  EXPECT_NEAR(good.x(), 10.5, 0.1);
  const GpsDecision jump = converter.evaluate(fix_at_enu(30.0, 10.0), predicted, 0.1);
  EXPECT_EQ(jump.rejection, GpsRejection::kGate);
  EXPECT_FALSE(jump.measurement.has_value());
}

TEST(GpsInput, RejectsNoFixAndHighHdopAndRemovesLeverArm) {
  GpsParameters parameters;
  parameters.antenna_in_base = Eigen::Vector3d(0.5, 0.0, 0.0);
  const GpsConverter converter(kOrigin, parameters);
  Eigen::Isometry3d predicted = Eigen::Isometry3d::Identity();
  predicted.linear() = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ()).toRotationMatrix();

  GpsFix no_fix = fix_at_enu(0, 0);
  no_fix.status = GpsFixStatus::kNoFix;
  EXPECT_EQ(converter.evaluate(no_fix, predicted, 0.1).rejection, GpsRejection::kNoFix);
  GpsFix bad_hdop = fix_at_enu(0, 0);
  bad_hdop.hdop = 3.5;
  EXPECT_EQ(converter.evaluate(bad_hdop, predicted, 0.1).rejection, GpsRejection::kHdop);

  // Base heading north, antenna 0.5 m ahead: antenna at (0, 0.5) means base at (0, 0).
  const GpsDecision decision = converter.evaluate(fix_at_enu(0.0, 0.5), predicted, 0.1);
  ASSERT_TRUE(decision.measurement.has_value());
  EXPECT_NEAR(decision.measurement->base_position.y(), 0.0, 0.05);
  EXPECT_GE(decision.measurement->sigma_m.z(), 1.0);
}

TEST(GpsInput, ConflictFlagAfterConsecutiveRejections) {
  GpsConflictCounter counter(5);
  for (int index = 0; index < 4; ++index) counter.record(true);
  EXPECT_FALSE(counter.is_conflict());
  counter.record(true);
  EXPECT_TRUE(counter.is_conflict());
  counter.record(false);
  EXPECT_FALSE(counter.is_conflict());
}
