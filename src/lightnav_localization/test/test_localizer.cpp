#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <random>

#include "lightnav_localization/localizer.hpp"
#include "synthetic_world.hpp"

using namespace lightnav_localization;
using namespace lightnav_localization::testing;

namespace {

constexpr double kDt = 0.02;
constexpr double kScanPeriod = 0.2;
constexpr double kScanDuration = 0.1;
constexpr double kSpeed = 0.6;
constexpr double kTurnRate = 0.4;
constexpr double kLegLength = 20.0;
constexpr double kMetersPerDegreeLatitude = 111320.0;
const GpsOrigin kOrigin{36.0, 140.0, 0.0};

// L-shaped course: straight leg, in-place 90 degree turn, straight leg.
const double kFirstLegSec = kLegLength / kSpeed;
const double kTurnSec = (M_PI / 2) / kTurnRate;
const double kTotalSec = 2 * kFirstLegSec + kTurnSec;

Eigen::Isometry3d truth_at(double time_sec) {
  if (time_sec <= kFirstLegSec) return arc_pose(time_sec, {0, 0}, 0.0, kSpeed, 0.0);
  if (time_sec <= kFirstLegSec + kTurnSec) {
    return arc_pose(time_sec - kFirstLegSec, {kLegLength, 0}, 0.0, 0.0, kTurnRate);
  }
  return arc_pose(time_sec - kFirstLegSec - kTurnSec, {kLegLength, 0}, M_PI / 2, kSpeed, 0.0);
}

GpsFix fix_at_enu(double east_m, double north_m) {
  GpsFix fix;
  fix.latitude_deg = kOrigin.latitude_deg + north_m / kMetersPerDegreeLatitude;
  fix.longitude_deg = kOrigin.longitude_deg +
                      east_m / (kMetersPerDegreeLatitude * std::cos(kOrigin.latitude_deg * M_PI / 180.0));
  fix.status = GpsFixStatus::kFix;
  return fix;
}

struct Scenario {
  double gps_bias_east_m = 0.0;
  double gps_noise_m = 1.0;  // also reported as the fix's known standard deviation when != 1.0
  bool is_gps_enabled = true;
  bool has_map = true;
  double map_hole_begin_sec = -1.0;  // map removed while the truth is in [begin, end]
  double map_hole_end_sec = -1.0;
  double gps_off_begin_sec = -1.0;
  double gps_off_end_sec = -1.0;
  double odom_only_max_distance_m = 50.0;
  int false_convergence_every = 0;
  double false_shift_m = 0.0;
};

struct Report {
  double max_position_error_m = 0.0;
  double max_yaw_error_deg = 0.0;
  double max_error_in_hole_m = 0.0;
  bool saw_tracking_after_hole = false;
  bool saw_gps_only = false;
  bool saw_odom_only = false;
  bool saw_conflict = false;
  bool saw_lost = false;
  bool published_after_lost = false;
  bool is_tracking_at_end = false;
  double wheel_only_end_error_m = 0.0;
  double scan_ms_max = 0.0;
  double match_ms_max = 0.0;
  double rebuild_ms_max = 0.0;
};

Report run(const Scenario& scenario) {
  const SyntheticWorld world({{0, 0}, {kLegLength, 0}, {kLegLength, kLegLength}});
  const PointList full_map = world.sample_map(0.2);
  LocalizerParameters parameters;
  parameters.state.odom_only_max_distance_m = scenario.odom_only_max_distance_m;
  Localizer* localizer_pointer = nullptr;
  PointList map = scenario.has_map ? full_map : PointList{};
  Localizer localizer(parameters, map, scenario.is_gps_enabled ? std::optional<GpsOrigin>(kOrigin) : std::nullopt);
  localizer_pointer = &localizer;
  (void)localizer_pointer;
  int match_counter = 0;
  double current_time_sec = 0.0;
  if (scenario.false_convergence_every > 0 || scenario.map_hole_end_sec > 0.0) {
    localizer.set_map_match_hook([&](MatchResult& result) {
      // Map hole: the matcher finds nothing to match against.
      if (current_time_sec >= scenario.map_hole_begin_sec && current_time_sec <= scenario.map_hole_end_sec) {
        result.converged = false;
        result.inlier_ratio = 0.0;
        return;
      }
      if (scenario.false_convergence_every == 0) return;
      if (++match_counter % scenario.false_convergence_every != 0) return;
      // Walk the result along the corridor and claim a clean match (inlier ratio 0.9).
      result.target_to_source.translation().x() += scenario.false_shift_m;
      result.inlier_ratio = 0.9;
      result.converged = true;
    });
  }
  localizer.initialize(truth_at(0.0));

  std::mt19937 random_generator(11);
  std::normal_distribution<double> noise(0.0, 1.0);
  Eigen::Isometry3d odom = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d wheel_only = truth_at(0.0);
  Report report;
  const int step_count = static_cast<int>(kTotalSec / kDt);
  for (int step = 0; step <= step_count; ++step) {
    const double time_sec = step * kDt;
    current_time_sec = time_sec;
    const Eigen::Isometry3d truth = truth_at(time_sec);
    if (step > 0) {
      const Eigen::Isometry3d relative = truth_at(time_sec - kDt).inverse() * truth;
      const double distance = relative.translation().norm();
      Eigen::Isometry3d drifted = relative;
      drifted.translation() *= 1.05;                 // 5 % scale error (wheel-only ends ~1.25 m off on this 40 m course)
      drifted.translation().z() = 0.0;
      drifted.linear() = Eigen::AngleAxisd(std::atan2(relative.rotation()(1, 0), relative.rotation()(0, 0)) +
                                               distance * (0.5 * M_PI / 180.0 / 10.0),
                                           Eigen::Vector3d::UnitZ())
                             .toRotationMatrix();     // 0.5 deg / 10 m yaw drift
      odom = odom * drifted;
      wheel_only = wheel_only * drifted;
    }
    // Scans are fed BEFORE the wheel sample of the same stamp so that the queue has to wait.
    if (step > 0 && step % static_cast<int>(kScanPeriod / kDt) == 0 && time_sec >= kScanDuration) {
      localizer.on_scan(time_sec, world.synthesize_scan(truth_at, time_sec - kScanDuration, kScanDuration, true, 360, 8));
    }
    const bool is_gps_time = step % static_cast<int>(1.0 / kDt) == 13;  // not aligned with scans
    const bool is_gps_off = time_sec >= scenario.gps_off_begin_sec && time_sec <= scenario.gps_off_end_sec;
    if (is_gps_time && !is_gps_off) {
      const Eigen::Isometry3d gps_truth = truth_at(time_sec);
      GpsFix fix = fix_at_enu(gps_truth.translation().x() + scenario.gps_noise_m * noise(random_generator) + scenario.gps_bias_east_m,
                              gps_truth.translation().y() + scenario.gps_noise_m * noise(random_generator));
      if (scenario.gps_noise_m != 1.0) {
        fix.has_known_covariance = true;  // GST available: the receiver reports its own sigma
        fix.known_std_m = scenario.gps_noise_m;
      }
      localizer.on_gps(time_sec, fix);
    }
    const auto output = localizer.on_wheel_odom(time_sec, odom);
    report.scan_ms_max = std::max(report.scan_ms_max, localizer.last_scan_time_ms());
    report.match_ms_max = std::max(report.match_ms_max, localizer.last_match_time_ms());
    report.rebuild_ms_max = std::max(report.rebuild_ms_max, localizer.last_submap_rebuild_time_ms());
    const bool is_hole = time_sec >= scenario.map_hole_begin_sec && time_sec <= scenario.map_hole_end_sec;
    if (report.saw_lost && output) report.published_after_lost = true;
    if (!output) {
      if (output.has_value() == false && time_sec > 3.0 && report.saw_odom_only) report.saw_lost = true;
      continue;
    }
    if (time_sec < 5.0) continue;  // warm-up: yaw settles from the 0.1 rad initial prior (2 deg at 3 s)
    const double position_error = (output->map_to_base.translation() - truth.translation()).head<2>().norm();
    const double yaw_error = std::abs(Eigen::AngleAxisd(truth.rotation().transpose() * output->map_to_base.rotation()).angle()) * 180.0 / M_PI;
    report.max_position_error_m = std::max(report.max_position_error_m, position_error);
    report.max_yaw_error_deg = std::max(report.max_yaw_error_deg, yaw_error);
    if (is_hole) report.max_error_in_hole_m = std::max(report.max_error_in_hole_m, position_error);
    report.saw_gps_only |= output->state == LocalizationState::kGpsOnly && is_hole;
    report.saw_odom_only |= output->state == LocalizationState::kOdomOnly;
    report.saw_conflict |= output->is_gps_conflict;
    if (time_sec > scenario.map_hole_end_sec && output->state == LocalizationState::kTracking) report.saw_tracking_after_hole = true;
    report.is_tracking_at_end = output->state == LocalizationState::kTracking;
  }
  report.wheel_only_end_error_m = (wheel_only.translation() - truth_at(kTotalSec).translation()).head<2>().norm();
  return report;
}

}  // namespace

TEST(Localizer, FusionStaysCloseToTruthWhileWheelOdometryDrifts) {
  const Report report = run({});
  std::printf("A: max pos err %.3f m, max yaw err %.2f deg, wheel-only end err %.2f m, scan max %.1f ms, match max %.1f ms, submap rebuild max %.1f ms\n",
              report.max_position_error_m, report.max_yaw_error_deg, report.wheel_only_end_error_m,
              report.scan_ms_max, report.match_ms_max, report.rebuild_ms_max);
  EXPECT_GT(report.wheel_only_end_error_m, 1.0);  // drift exists
  EXPECT_LT(report.max_position_error_m, 0.2);
  EXPECT_LT(report.max_yaw_error_deg, 2.0);
  EXPECT_TRUE(report.is_tracking_at_end);
  EXPECT_LT(report.scan_ms_max, 450.0);  // 150 ms limit x3 in CI
}

TEST(Localizer, GpsBiasDoesNotMoveEstimateWhileMapTracks) {
  Scenario scenario;
  scenario.gps_bias_east_m = 3.0;
  scenario.gps_noise_m = 0.5;
  const Report report = run(scenario);
  std::printf("E: max pos err %.3f m, conflict=%d\n", report.max_position_error_m, report.saw_conflict);
  EXPECT_LT(report.max_position_error_m, 0.2);
  EXPECT_TRUE(report.saw_conflict);
  EXPECT_TRUE(report.is_tracking_at_end);
}

TEST(Localizer, FallsBackToGpsOnlyWithoutMapAndRecovers) {
  Scenario scenario;
  scenario.map_hole_begin_sec = 10.0;
  scenario.map_hole_end_sec = 20.0;
  const Report report = run(scenario);
  std::printf("F1: hole err %.3f m gps_only=%d tracking_after=%d\n", report.max_error_in_hole_m,
              report.saw_gps_only, report.saw_tracking_after_hole);
  EXPECT_LT(report.max_error_in_hole_m, 1.5);
  EXPECT_TRUE(report.saw_gps_only);
  EXPECT_TRUE(report.saw_tracking_after_hole);
}

TEST(Localizer, OdomOnlyBecomesLostAndStopsPublishing) {
  Scenario scenario;
  scenario.has_map = false;
  scenario.is_gps_enabled = false;
  scenario.odom_only_max_distance_m = 8.0;
  const Report report = run(scenario);
  EXPECT_TRUE(report.saw_odom_only);
  EXPECT_TRUE(report.saw_lost);
  EXPECT_FALSE(report.published_after_lost);
}

TEST(Localizer, FalseConvergenceAlongCorridorDoesNotJumpThePose) {
  for (const double shift_m : {5.0, 0.9}) {  // 5 m is gated; 0.9 m passes the 1.0 m gate
    Scenario scenario;
    scenario.false_convergence_every = 10;
    scenario.false_shift_m = shift_m;
    const Report report = run(scenario);
    std::printf("C: shift %.1f m -> max pos err %.3f m\n", shift_m, report.max_position_error_m);
    EXPECT_LT(report.max_position_error_m, 0.3);
  }
}
