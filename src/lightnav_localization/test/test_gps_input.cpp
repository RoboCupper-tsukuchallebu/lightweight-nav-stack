#include <gtest/gtest.h>

#include <cmath>
#include <limits>

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
  GpsConflictCounter counter(5, 1.0);
  for (int index = 0; index < 4; ++index) counter.record(true);
  EXPECT_FALSE(counter.is_conflict());
  counter.record(true);
  EXPECT_TRUE(counter.is_conflict());
  counter.record(false);
  EXPECT_FALSE(counter.is_conflict());
}

TEST(GpsInput, RejectsNanAndFallsBackWhenKnownStdIsZero) {
  const GpsConverter converter(kOrigin, GpsParameters{});
  const Eigen::Isometry3d predicted = Eigen::Isometry3d::Identity();
  GpsFix nan_fix = fix_at_enu(0, 0);
  nan_fix.latitude_deg = std::nan("");
  EXPECT_EQ(converter.evaluate(nan_fix, predicted, 0.1).rejection, GpsRejection::kInvalid);
  GpsFix zero_std = fix_at_enu(0, 0);
  zero_std.has_known_covariance = true;
  zero_std.known_std_m = 0.0;
  const GpsDecision decision = converter.evaluate(zero_std, predicted, 0.1);
  ASSERT_TRUE(decision.measurement.has_value());
  EXPECT_GT(decision.measurement->sigma_m.x(), 0.0);
}

TEST(GpsInput, TrustedKnownCovarianceBypassesOnlyGate) {
  GpsParameters parameters;
  parameters.gps_absolute_gate_m = 200.0;  // isolate the Mahalanobis bypass
  const GpsConverter converter(kOrigin, parameters);
  const Eigen::Isometry3d predicted = Eigen::Isometry3d::Identity();
  GpsFix fix = fix_at_enu(100.0, 0.0);
  fix.has_known_covariance = true;
  fix.known_std_m = 0.3;
  EXPECT_TRUE(converter.evaluate(fix, predicted, 0.1).measurement.has_value());
  fix.known_std_m = 0.31;
  EXPECT_EQ(converter.evaluate(fix, predicted, 0.1).rejection, GpsRejection::kGate);
  for (double invalid_std : {0.0, -0.1, std::nan(""), std::numeric_limits<double>::infinity()}) {
    fix.known_std_m = invalid_std;
    EXPECT_EQ(converter.evaluate(fix, predicted, 0.1).rejection, GpsRejection::kGate);
  }
  fix.known_std_m = 0.1;
  fix.has_known_covariance = false;
  EXPECT_EQ(converter.evaluate(fix, predicted, 0.1).rejection, GpsRejection::kGate);
  fix.has_known_covariance = true;
  fix.hdop = 4.0;
  EXPECT_EQ(converter.evaluate(fix, predicted, 0.1).rejection, GpsRejection::kHdop);
  fix.hdop = 0.0;
  fix.status = GpsFixStatus::kNoFix;
  EXPECT_EQ(converter.evaluate(fix, predicted, 0.1).rejection, GpsRejection::kNoFix);
}

TEST(GpsInput, ConflictUsesNonconsecutiveRejectionsAndEvictsOldObservations) {
  GpsConflictCounter counter(4, 0.5);
  counter.record(true);
  counter.record(false);
  counter.record(true);
  EXPECT_FALSE(counter.is_conflict());
  counter.record(false);
  EXPECT_TRUE(counter.is_conflict());
  counter.record(false);
  EXPECT_FALSE(counter.is_conflict());
  counter.record(true);
  EXPECT_TRUE(counter.is_conflict());
  for (int index = 0; index < 4; ++index) counter.record(false);
  EXPECT_FALSE(counter.is_conflict());
}

TEST(GpsInput, ConflictRejectsInvalidWindowParameters) {
  EXPECT_THROW(GpsConflictCounter(0), std::invalid_argument);
  EXPECT_THROW(GpsConflictCounter(10, 0.0), std::invalid_argument);
  EXPECT_THROW(GpsConflictCounter(10, std::nan("")), std::invalid_argument);
  EXPECT_THROW(GpsConflictCounter(10, 1.1), std::invalid_argument);
}

TEST(GpsInput, ConflictDefaultsToHalfOfTenObservations) {
  GpsConflictCounter counter;
  for (int index = 0; index < 9; ++index) {
    counter.record(index % 2 == 0);
    EXPECT_FALSE(counter.is_conflict());
  }
  counter.record(false);
  EXPECT_TRUE(counter.is_conflict());
  counter.record(false);
  EXPECT_FALSE(counter.is_conflict());
}

TEST(GpsInput, TrustedFixReportsOrdinaryGateAndObeysAbsoluteGate) {
  const GpsConverter converter(kOrigin, GpsParameters{});
  GpsFix fix = fix_at_enu(1.0, 0.0);
  fix.has_known_covariance = true;
  fix.known_std_m = 0.1;
  const auto accepted = converter.evaluate(fix, Eigen::Isometry3d::Identity(), 0.1);
  EXPECT_TRUE(accepted.measurement);
  ASSERT_TRUE(accepted.gate_would_reject);
  EXPECT_TRUE(*accepted.gate_would_reject);
  fix = fix_at_enu(3.0, 0.0);
  fix.has_known_covariance = true;
  fix.known_std_m = 0.1;
  // A confident prediction must still reject a precise fix biased by three metres.
  const auto rejected = converter.evaluate(fix, Eigen::Isometry3d::Identity(), 0.1);
  EXPECT_EQ(rejected.rejection, GpsRejection::kGate);
  ASSERT_TRUE(rejected.gate_would_reject);
  EXPECT_TRUE(*rejected.gate_would_reject);
  fix.status = GpsFixStatus::kNoFix;
  EXPECT_FALSE(converter.evaluate(fix, Eigen::Isometry3d::Identity(), 0.1).gate_would_reject);
}


TEST(GpsInput, TrustedFixRecoversWhenPredictionIsUncertain) {
  const GpsConverter converter(kOrigin, GpsParameters{});
  GpsFix fix = fix_at_enu(0.0, 0.0);
  fix.has_known_covariance = true;
  fix.known_std_m = 0.1;
  Eigen::Isometry3d prediction = Eigen::Isometry3d::Identity();
  prediction.translation().x() = 3.0;
  const auto decision = converter.evaluate(fix, prediction, 1.5);
  ASSERT_TRUE(decision.measurement);
  EXPECT_EQ(decision.rejection, GpsRejection::kNone);
  ASSERT_TRUE(decision.gate_would_reject);
  EXPECT_TRUE(*decision.gate_would_reject);  // recovery does not hide the conflict diagnostic
  EXPECT_EQ(converter.evaluate(fix, prediction, 0.1).rejection, GpsRejection::kGate);
  prediction.translation().x() = 5.0;
  EXPECT_EQ(converter.evaluate(fix, prediction, 1.5).rejection, GpsRejection::kGate);
}

TEST(GpsInput, OrdinaryFixUsesMahalanobisGateBeyondTwoMeters) {
  const GpsConverter converter(kOrigin, GpsParameters{});
  GpsFix fix = fix_at_enu(2.5, 0.0);
  fix.has_known_covariance = true;
  fix.known_std_m = 1.0;
  const auto decision = converter.evaluate(fix, Eigen::Isometry3d::Identity(), 0.1);
  EXPECT_TRUE(decision.measurement);
  EXPECT_EQ(decision.rejection, GpsRejection::kNone);
  ASSERT_TRUE(decision.gate_would_reject);
  EXPECT_TRUE(*decision.gate_would_reject);
}
