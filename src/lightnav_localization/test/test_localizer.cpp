#include <gtest/gtest.h>

#include <cmath>
#include <deque>
#include <filesystem>
#include <fstream>
#include <functional>
#include <map>
#include <sstream>
#include <cstdio>
#include <random>
#include <array>
#include <limits>
#include <vector>
#include <string>

#include "lightnav_localization/localizer.hpp"
#include "golden_output.hpp"
#include "synthetic_world.hpp"

using namespace lightnav_localization;
using namespace lightnav_localization::testing;

namespace {

constexpr double kDt = 0.02;
constexpr double kScanDuration = 0.1;
constexpr double kSpeed = 0.6;
constexpr double kTurnRate = 0.4;
constexpr double kMetersPerDegreeLatitude = 111320.0;
const GpsOrigin kOrigin{36.0, 140.0, 0.0};

// Straight legs joined by in-place 90 degree turns (sign +1 = left, -1 = right).
struct Course {
  std::vector<double> leg_lengths_m;
  std::vector<double> turn_signs;

  double leg_duration_sec(std::size_t leg) const { return leg_lengths_m[leg] / kSpeed; }
  static double turn_duration_sec() { return (M_PI / 2) / kTurnRate; }
  double total_sec() const {
    double total = turn_duration_sec() * turn_signs.size();
    for (std::size_t leg = 0; leg < leg_lengths_m.size(); ++leg) total += leg_duration_sec(leg);
    return total;
  }
  std::vector<Eigen::Vector2d> centerline() const {
    std::vector<Eigen::Vector2d> points{{0, 0}};
    double heading = 0.0;
    for (std::size_t leg = 0; leg < leg_lengths_m.size(); ++leg) {
      points.push_back(points.back() + leg_lengths_m[leg] * Eigen::Vector2d(std::cos(heading), std::sin(heading)));
      if (leg < turn_signs.size()) heading += turn_signs[leg] * M_PI / 2;
    }
    return points;
  }
  Eigen::Isometry3d truth_at(double time_sec) const {
    Eigen::Vector2d start(0, 0);
    double heading = 0.0;
    double remaining = time_sec;
    for (std::size_t leg = 0; leg < leg_lengths_m.size(); ++leg) {
      if (remaining <= leg_duration_sec(leg)) return arc_pose(remaining, start, heading, kSpeed, 0.0);
      remaining -= leg_duration_sec(leg);
      start += leg_lengths_m[leg] * Eigen::Vector2d(std::cos(heading), std::sin(heading));
      if (leg >= turn_signs.size()) break;
      if (remaining <= turn_duration_sec()) return arc_pose(remaining, start, heading, 0.0, turn_signs[leg] * kTurnRate);
      remaining -= turn_duration_sec();
      heading += turn_signs[leg] * M_PI / 2;
    }
    return arc_pose(0.0, start, heading, 0.0, 0.0);
  }
};

const Course kShortCourse{{20.0, 20.0}, {1.0}};
// The Issue's course: 200 m with two 90 degree turns.
const Course kOpenCorridor{{120.0}, {}};
const Course kSpecCourse{{70.0, 60.0, 70.0}, {1.0, -1.0}};

GpsFix fix_at_enu(double east_m, double north_m) {
  GpsFix fix;
  fix.latitude_deg = kOrigin.latitude_deg + north_m / kMetersPerDegreeLatitude;
  fix.longitude_deg = kOrigin.longitude_deg +
                      east_m / (kMetersPerDegreeLatitude * std::cos(kOrigin.latitude_deg * M_PI / 180.0));
  fix.status = GpsFixStatus::kFix;
  return fix;
}

struct Scenario {
  double scan_period_sec = 0.2;
  double wheel_sigma_translation_per_m = PoseGraphParameters{}.wheel_sigma_translation_per_m;
  double gps_sigma_m = 1.0;
  double degeneracy_ratio = InformationLimits{}.degeneracy_ratio;
  bool has_pillars = false;
  double pillar_half_length_m = 0.2;
  double pillar_half_width_m = 0.2;
  double pillar_spacing_m = 10.0;
  double pillar_lateral_offset_m = 1.5;
  bool pillar_both_sides = false;
  double gps_bias_east_m = 0.0;
  double gps_noise_m = 1.0;  // also reported as the fix's known standard deviation when != 1.0
  bool is_gps_enabled = true;
  bool has_map = true;
  double map_hole_begin_sec = -1.0;  // map removed while the truth is in [begin, end]
  double map_hole_end_sec = -1.0;
  double gps_off_begin_sec = -1.0;
  double gps_off_end_sec = -1.0;
  double odom_only_max_distance_m = 50.0;
  bool use_spec_course = false;
  bool has_real_map_hole = false;  // remove map points inside the hole instead of faking a failed match
  double wheel_scale = 1.05;
  double ramp_blend_m = 0.6;  // 0 = pitch steps 10 degrees at once
  double open_corridor_length_m = 120.0;
  bool is_open_corridor = false;  // 120 m straight, no end caps
  double map_rebuild_distance_m = 50.0;
  int rebuild_failure_mode = 0;
  bool has_ramp = false;  // 10 degree climb over 10 m (horizontal) starting at x = 5 m
  double initial_offset_m = 0.0;  // start this far to the left of truth (needs a correction)
  double large_correction_threshold_m = 1.0;
  int gps_outlier_every = 0;  // 20 m east jump on every n-th GPS fix
  double fail_optimization_at_sec = -1.0;  // injects a GTSAM failure once
  bool reinitialize_after_lost = false;
  int false_convergence_every = 0;
  double false_shift_m = 0.0;
  // No wheel, scan or GPS input in (begin, end): the odometry keeps integrating, as it does while
  // a bag has a hole in it (Issue #45).
  double wheel_gap_begin_sec = -1.0;
  double wheel_gap_end_sec = -1.0;
  bool reinitialize_after_gap = false;  // set_initial_pose with the truth when the input resumes
  double correction_slew_rps = CorrectorParameters{}.slew_rps;
  double slip_gate_yaw_rad = LidarOdometryParameters{}.slip_gate_yaw_rad;
  double wheel_yaw_slip_threshold_rad = PoseGraphParameters{}.wheel_yaw_slip_threshold_rad;
  double slip_begin_sec = -1.0;
  double slip_end_sec = -1.0;
  double stop_sec = -1.0;  // ends the run early
  bool gives_gyro_increments = false;  // passes the true yaw change (plus a small bias) as the gyro increment
};

struct Measurement {
  double time_sec;
  double truth_x_m;
  double pitch_error_deg;
  double along_track_error_m;
  double position_std_m;
  double eigenvalue_ratio;
  Eigen::Vector3d weakest_axis;
};

struct Report {
  std::vector<Measurement> samples;
  std::array<int, 3> gps_accepted_by_interval{};  // [0,10), [10,20), [20,infinity)

  int gps_submitted_count = 0;
  std::size_t gps_time_gap_rejection_count = 0;
  double gps_acceptance_ratio = 0.0;
  double max_position_error_m = 0.0;
  double max_yaw_error_deg = 0.0;
  double max_error_in_hole_m = 0.0;
  bool saw_tracking_after_hole = false;
  bool saw_gps_only = false;
  bool saw_odom_only = false;
  bool saw_odom_only_during_slip = false;
  bool saw_conflict = false;
  bool saw_lost = false;
  bool published_after_lost = false;
  bool is_tracking_at_end = false;
  double wheel_only_end_error_m = 0.0;
  bool published_after_reinitialize = false;
  bool saw_large_correction = false;
  double max_pitch_error_deg = 0.0;          // whole run, includes the ramp entry transient
  double max_pitch_error_on_ramp_deg = 0.0;  // truth x in [5, 15] m: the whole climb
  double pitch_step_recovery_sec = std::numeric_limits<double>::quiet_NaN();
  double max_pitch_error_after_step_entry_deg = 0.0;
  double max_pitch_error_after_step_exit_deg = 0.0;
  int pitch_samples_after_step_entry = 0;
  int pitch_samples_after_step_exit = 0;
  double max_output_speed_mps = 0.0;
  double final_position_error_m = 0.0;
  std::string last_error;
  double scan_ms_max = 0.0;
  double match_ms_max = 0.0;
  double rebuild_ms_max = 0.0;
  std::vector<Eigen::Isometry3d> lidar_keyframe_poses;  // at the end of the run
  Eigen::Isometry3d final_truth = Eigen::Isometry3d::Identity();
};

Report run(const Scenario& scenario) {
  const Course open_corridor{{scenario.open_corridor_length_m}, {}};
  const Course& course = scenario.is_open_corridor ? open_corridor : (scenario.use_spec_course ? kSpecCourse : kShortCourse);
  SyntheticWorld world(course.centerline(), !scenario.is_open_corridor);
  if (scenario.has_pillars) {
    world.add_pillars(scenario.pillar_half_length_m, scenario.pillar_half_width_m,
                      scenario.pillar_spacing_m, scenario.pillar_lateral_offset_m,
                      scenario.pillar_both_sides);
  }
  constexpr double kRampAngleRad = 10.0 * M_PI / 180.0;
  if (scenario.has_ramp) world.set_ramp(5.0, 10.0, kRampAngleRad);
  const double ramp_blend_m = scenario.ramp_blend_m;
  const auto truth_at = [&course, &world, ramp_blend_m](double time) {
    Eigen::Isometry3d pose = course.truth_at(time);
    const double x = pose.translation().x();
    const double yaw = std::atan2(pose.rotation()(1, 0), pose.rotation()(0, 0));
    pose.translation().z() += world.floor_height(x);
    if (world.is_on_ramp(x) && std::abs(yaw) < 0.1) {  // climbing along +x: nose up
      // The pitch changes over ~1 s (0.6 m) at both ends of the ramp, as a vehicle does; a step of 10
      // degrees would also exceed the 0.1 rad LiDAR odometry rotation gate by construction.
      const double blend = ramp_blend_m <= 0.0 ? 1.0 : std::min({1.0, (x - 5.0) / ramp_blend_m, (15.0 - x) / ramp_blend_m});
      pose.linear() = pose.linear() * Eigen::AngleAxisd(-world.ramp_angle_rad() * blend, Eigen::Vector3d::UnitY()).toRotationMatrix();
    }
    return pose;
  };
  const PointList full_map = world.sample_map(0.2);
  LocalizerParameters parameters;
  parameters.graph.wheel_sigma_translation_per_m = scenario.wheel_sigma_translation_per_m;
  parameters.lidar_odometry.slip_gate_yaw_rad = scenario.slip_gate_yaw_rad;
  parameters.graph.wheel_yaw_slip_threshold_rad = scenario.wheel_yaw_slip_threshold_rad;
  parameters.map_information.degeneracy_ratio = scenario.degeneracy_ratio;
  parameters.state.odom_only_max_distance_m = scenario.odom_only_max_distance_m;
  parameters.map_submap_rebuild_distance_m = scenario.map_rebuild_distance_m;
  parameters.corrector.large_correction_threshold_m = scenario.large_correction_threshold_m;
  parameters.corrector.slew_rps = scenario.correction_slew_rps;
  Localizer* localizer_pointer = nullptr;
  PointList map = scenario.has_map ? full_map : PointList{};
  if (scenario.has_real_map_hole) {
    std::vector<Eigen::Vector2d> hole_centers;  // the path between the hole times, every 0.3 m
    for (double time = scenario.map_hole_begin_sec; time <= scenario.map_hole_end_sec; time += 0.5) {
      hole_centers.push_back(course.truth_at(time).translation().head<2>());
    }
    PointList kept;
    for (const auto& point : map) {
      bool is_inside = false;
      for (const Eigen::Vector2d& center : hole_centers) {
        if ((point.head<2>().cast<double>() - center).norm() < 3.0) { is_inside = true; break; }
      }
      if (!is_inside) kept.push_back(point);
    }
    map = kept;
  }
  Localizer localizer(parameters, map, scenario.is_gps_enabled ? std::optional<GpsOrigin>(kOrigin) : std::nullopt);
  localizer_pointer = &localizer;
  (void)localizer_pointer;
  int match_counter = 0;
  double current_time_sec = 0.0;
  int gps_counter = 0;
  if (scenario.false_convergence_every > 0 || (scenario.map_hole_end_sec > 0.0 && !scenario.has_real_map_hole)) {
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
      // Shift along the corridor axis of the current leg (truth heading), on either leg.
      const Eigen::Vector3d heading = course.truth_at(current_time_sec).rotation().col(0);
      result.target_to_source.translation() += scenario.false_shift_m * heading;
      result.inlier_ratio = 0.9;
      result.converged = true;
    });
  }
  Eigen::Isometry3d initial_pose = truth_at(0.0);
  initial_pose.translation().y() += scenario.initial_offset_m;
  localizer.initialize(initial_pose);
  bool has_injected_rebuild_failure = false;
  std::optional<Eigen::Vector3d> previous_output_position;

  std::mt19937 random_generator(11);
  std::normal_distribution<double> noise(0.0, 1.0);
  Eigen::Isometry3d odom = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d wheel_only = truth_at(0.0);
  Report report;
  std::optional<double> pitch_step_begin_sec;
  bool has_observed_pitch_step = false;
  if (scenario.has_ramp && scenario.ramp_blend_m == 0.0) {
    report.pitch_step_recovery_sec = std::numeric_limits<double>::infinity();
  }
  bool has_reinitialized = false;
  double previous_accepted_gps_stamp_sec = -1.0;
  const int step_count = static_cast<int>(course.total_sec() / kDt);
  for (int step = 0; step <= step_count; ++step) {
    const double time_sec = step * kDt;
    if (scenario.stop_sec >= 0.0 && time_sec > scenario.stop_sec) break;
    current_time_sec = time_sec;
    const Eigen::Isometry3d truth = truth_at(time_sec);
    report.final_truth = truth;
    if (scenario.has_ramp && scenario.ramp_blend_m == 0.0 &&
        !pitch_step_begin_sec && truth.translation().x() >= 5.0) {
      pitch_step_begin_sec = 5.0 / kSpeed;
    }
    std::optional<double> gyro_increment;
    if (step > 0) {
      const Eigen::Isometry3d relative = truth_at(time_sec - kDt).inverse() * truth;
      constexpr double kGyroBiasResidualRadPerSec = 2.0e-4;
      gyro_increment = std::atan2(relative.rotation()(1, 0), relative.rotation()(0, 0)) +
                       kGyroBiasResidualRadPerSec * kDt;
      const double distance = relative.translation().norm();
      Eigen::Isometry3d drifted = relative;
      drifted.translation() *= scenario.wheel_scale;
      drifted.translation().z() = 0.0;
      drifted.linear() = Eigen::AngleAxisd(std::atan2(relative.rotation()(1, 0), relative.rotation()(0, 0)) +
                                               distance * (0.5 * M_PI / 180.0 / 10.0),
                                           Eigen::Vector3d::UnitZ())
                             .toRotationMatrix();     // 0.5 deg / 10 m yaw drift
      if (time_sec >= scenario.slip_begin_sec && time_sec < scenario.slip_end_sec) {
        drifted.linear() = Eigen::Matrix3d::Identity();
      }
      odom = odom * drifted;
      wheel_only = wheel_only * drifted;
    }
    if (time_sec > scenario.wheel_gap_begin_sec && time_sec < scenario.wheel_gap_end_sec) continue;
    // Scans are fed BEFORE the wheel sample of the same stamp so that the queue has to wait.
    if (step > 0 && step % static_cast<int>(scenario.scan_period_sec / kDt) == 0 && time_sec >= kScanDuration) {
      localizer.on_scan(time_sec, world.synthesize_scan(truth_at, time_sec - kScanDuration, kScanDuration, true, 360, 8));
    }
    const bool is_gps_time = step % static_cast<int>(1.0 / kDt) == 13;  // not aligned with scans
    const bool is_gps_off = time_sec >= scenario.gps_off_begin_sec && time_sec <= scenario.gps_off_end_sec;
    if (is_gps_time && !is_gps_off) {
      const Eigen::Isometry3d gps_truth = truth_at(time_sec);
      ++gps_counter;
      const double jump = (scenario.gps_outlier_every > 0 && gps_counter % scenario.gps_outlier_every == 0) ? 20.0 : 0.0;
      GpsFix fix = fix_at_enu(jump + gps_truth.translation().x() + scenario.gps_noise_m * noise(random_generator) + scenario.gps_bias_east_m,
                              gps_truth.translation().y() + scenario.gps_noise_m * noise(random_generator));
      if (scenario.gps_noise_m != 1.0 || scenario.gps_sigma_m != 1.0) {
        fix.has_known_covariance = true;  // GST available: the receiver reports its own sigma
        fix.known_std_m = scenario.gps_sigma_m != 1.0 ? scenario.gps_sigma_m : scenario.gps_noise_m;
      }
      localizer.on_gps(time_sec, fix);
      if (scenario.is_gps_enabled) ++report.gps_submitted_count;
    }
    if (scenario.fail_optimization_at_sec >= 0.0 && std::abs(time_sec - scenario.fail_optimization_at_sec) < kDt / 2) {
      localizer.inject_optimization_failure_for_test();
    }
    const bool should_reinitialize =
        (scenario.reinitialize_after_lost && report.saw_lost) ||
        (scenario.reinitialize_after_gap && time_sec >= scenario.wheel_gap_end_sec);
    if (should_reinitialize && !has_reinitialized) {
      localizer.initialize(truth);
      has_reinitialized = true;
    }
    if (scenario.rebuild_failure_mode != 0 && !has_injected_rebuild_failure && time_sec > 8.0) {
      localizer.inject_map_rebuild_failure_for_test(scenario.rebuild_failure_mode);
      has_injected_rebuild_failure = true;
    }
    const auto output = scenario.gives_gyro_increments ? localizer.on_wheel_odom(time_sec, odom, gyro_increment)
                                                       : localizer.on_wheel_odom(time_sec, odom);
    const double accepted_gps_stamp_sec = localizer.last_accepted_gps_stamp_sec();
    if (accepted_gps_stamp_sec >= 0.0 && accepted_gps_stamp_sec != previous_accepted_gps_stamp_sec) {
      ++report.gps_accepted_by_interval[accepted_gps_stamp_sec < 10.0 ? 0 : (accepted_gps_stamp_sec < 20.0 ? 1 : 2)];
    }
    previous_accepted_gps_stamp_sec = accepted_gps_stamp_sec;
    if (has_reinitialized && output) report.published_after_reinitialize = true;
    report.scan_ms_max = std::max(report.scan_ms_max, localizer.last_scan_time_ms());
    report.match_ms_max = std::max(report.match_ms_max, localizer.last_match_time_ms());
    report.rebuild_ms_max = std::max(report.rebuild_ms_max, localizer.last_submap_rebuild_time_ms());
    const bool is_hole = time_sec >= scenario.map_hole_begin_sec && time_sec <= scenario.map_hole_end_sec;
    if (report.saw_lost && output && !has_reinitialized) report.published_after_lost = true;
    if (!output) {
      if (output.has_value() == false && time_sec > 3.0 && report.saw_odom_only) report.saw_lost = true;
      continue;
    }
    
    const double pitch_error_deg =
        std::abs(std::asin(-output->map_to_base.rotation()(2, 0)) - std::asin(-truth.rotation()(2, 0))) * 180.0 / M_PI;
    report.samples.push_back({time_sec, truth.translation().x(), pitch_error_deg,
        (output->map_to_base.translation() - truth.translation()).dot(truth.rotation().col(0)),
        output->position_std_m,
        localizer.last_match_eigenvalue_ratio().value_or(std::numeric_limits<double>::quiet_NaN()),
        localizer.last_match_weakest_axis().value_or(Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN()))});
    if (pitch_step_begin_sec && std::isinf(report.pitch_step_recovery_sec) &&
        truth.translation().x() <= 15.0 && pitch_error_deg <= 2.0) {
      report.pitch_step_recovery_sec = has_observed_pitch_step ? time_sec - *pitch_step_begin_sec : 0.0;
    }
    if (pitch_step_begin_sec) has_observed_pitch_step = true;
    if (scenario.has_ramp && scenario.ramp_blend_m == 0.0) {
      if (time_sec >= 5.0 / kSpeed + 0.3 && time_sec < 15.0 / kSpeed) {
        report.max_pitch_error_after_step_entry_deg =
            std::max(report.max_pitch_error_after_step_entry_deg, pitch_error_deg);
        ++report.pitch_samples_after_step_entry;
      }
      if (time_sec >= 15.0 / kSpeed + 0.3) {
        report.max_pitch_error_after_step_exit_deg =
            std::max(report.max_pitch_error_after_step_exit_deg, pitch_error_deg);
        ++report.pitch_samples_after_step_exit;
      }
    }
    report.max_pitch_error_deg = std::max(report.max_pitch_error_deg, pitch_error_deg);
    if (truth.translation().x() >= 5.0 && truth.translation().x() <= 15.0) {
      report.max_pitch_error_on_ramp_deg = std::max(report.max_pitch_error_on_ramp_deg, pitch_error_deg);
    }
    report.saw_large_correction |= output->is_large_correction;
    if (previous_output_position) {
      report.max_output_speed_mps = std::max(
          report.max_output_speed_mps,
          (output->map_to_base.translation() - *previous_output_position).head<2>().norm() / kDt);  // z is not slewed
    }
    previous_output_position = output->map_to_base.translation();
    const double position_error = (output->map_to_base.translation() - truth.translation()).head<2>().norm();
    // Yaw only: the full rotation angle also contains roll/pitch noise (the 2 deg seen at 3 s in the
    // first version of this test was tilt, not yaw).
    const Eigen::Matrix3d rotation_error = truth.rotation().transpose() * output->map_to_base.rotation();
    const double yaw_error = std::abs(std::atan2(rotation_error(1, 0), rotation_error(0, 0))) * 180.0 / M_PI;
    report.max_position_error_m = std::max(report.max_position_error_m, position_error);
    report.final_position_error_m = position_error;
    report.max_yaw_error_deg = std::max(report.max_yaw_error_deg, yaw_error);
    if (is_hole) report.max_error_in_hole_m = std::max(report.max_error_in_hole_m, position_error);
    report.saw_gps_only |= output->state == LocalizationState::kGpsOnly && is_hole;
    report.saw_odom_only |= output->state == LocalizationState::kOdomOnly;
    if (time_sec >= scenario.slip_begin_sec && time_sec <= scenario.slip_end_sec) {
      report.saw_odom_only_during_slip |= output->state == LocalizationState::kOdomOnly;
    }
    report.saw_conflict |= output->is_gps_conflict;
    if (time_sec > scenario.map_hole_end_sec && output->state == LocalizationState::kTracking) report.saw_tracking_after_hole = true;
    report.is_tracking_at_end = output->state == LocalizationState::kTracking;
  }
  report.gps_time_gap_rejection_count = localizer.gps_time_gap_rejection_count();
  const int gps_accepted_count = report.gps_accepted_by_interval[0] +
      report.gps_accepted_by_interval[1] + report.gps_accepted_by_interval[2];
  report.gps_acceptance_ratio = report.gps_submitted_count > 0
      ? static_cast<double>(gps_accepted_count) / report.gps_submitted_count : 0.0;
  report.last_error = localizer.last_error();
  report.lidar_keyframe_poses = localizer.lidar_keyframe_poses_for_test();
  report.wheel_only_end_error_m = (wheel_only.translation() - truth_at(course.total_sec()).translation()).head<2>().norm();
  const auto* test_info = ::testing::UnitTest::GetInstance()->current_test_info();
  std::printf("[ MEASURED ] scenario=%s error_m=%.9g final_error_m=%.9g pitch_error_ramp_deg=%.9g pitch_step_recovery_sec=%.9g pitch_error_after_step_entry_deg=%.9g pitch_error_after_step_exit_deg=%.9g pitch_samples_after_step_entry=%d pitch_samples_after_step_exit=%d scan_period_sec=%.9g wheel_sigma=%.9g gps_sigma=%.9g degeneracy_ratio=%.9g pillars=%d gps_accepted_before_10s=%d gps_accepted_10_to_20s=%d gps_accepted_after_20s=%d gps_submitted_count=%d gps_acceptance_ratio=%.9g gps_time_gap_rejection_count=%zu gps_gate_rejection_count=%zu max_keyframe_interval_sec=%.9g max_gps_factors_per_keyframe=%zu saw_conflict=%d sample_columns=time_sec,truth_x_m,pitch_error_deg,along_track_error_m,position_std_m,eigen_ratio,weak_axis_x,weak_axis_y,weak_axis_z\n",
      test_info->name(), report.max_position_error_m, report.final_position_error_m,
      report.max_pitch_error_on_ramp_deg, report.pitch_step_recovery_sec,
      report.max_pitch_error_after_step_entry_deg, report.max_pitch_error_after_step_exit_deg,
      report.pitch_samples_after_step_entry, report.pitch_samples_after_step_exit,
      scenario.scan_period_sec, scenario.wheel_sigma_translation_per_m,
      scenario.gps_sigma_m != 1.0 ? scenario.gps_sigma_m : scenario.gps_noise_m,
      scenario.degeneracy_ratio, scenario.has_pillars, report.gps_accepted_by_interval[0],
      report.gps_accepted_by_interval[1], report.gps_accepted_by_interval[2],
      report.gps_submitted_count, report.gps_acceptance_ratio,
      report.gps_time_gap_rejection_count, localizer.gps_gate_rejection_count(),
      localizer.max_keyframe_interval_sec(), localizer.max_gps_factors_per_keyframe(), report.saw_conflict);
  for (const auto& sample : report.samples) {
    std::printf("[ SAMPLE ] scenario=%s %.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g\n",
        test_info->name(), sample.time_sec, sample.truth_x_m, sample.pitch_error_deg,
        sample.along_track_error_m, sample.position_std_m, sample.eigenvalue_ratio,
        sample.weakest_axis.x(), sample.weakest_axis.y(), sample.weakest_axis.z());
  }
  return report;
}

}  // namespace

// A at the real 10 Hz scan rate, on a course with along-track map features. The boxes are thin
// (0.2 m) and wide (1 m) so their faces perpendicular to travel raise the translation Hessian's
// smallest eigenvalue without adding much lateral surface. 1.25 m spacing was chosen from the
// [ SAMPLE ] eigen_ratio output: 10 m spacing stayed below 0.02 on the straight legs.
TEST(Localizer, FusionStaysCloseToTruthAtTenHertz) {
  Scenario scenario;
  scenario.use_spec_course = true;
  scenario.wheel_scale = 1.01;  // the Issue's wheel model: 1 % scale, 0.5 deg / 10 m yaw
  scenario.scan_period_sec = 0.1;
  scenario.has_pillars = true;
  scenario.pillar_half_length_m = 0.1;
  scenario.pillar_half_width_m = 0.5;
  scenario.pillar_spacing_m = 1.25;
  scenario.pillar_both_sides = true;
  const Report report = run(scenario);
  double min_eigen_ratio = std::numeric_limits<double>::infinity();
  int finite_count = 0;
  int nan_after_first_finite = 0;
  double first_nan_after_finite_sec = std::numeric_limits<double>::quiet_NaN();
  double last_nan_after_finite_sec = std::numeric_limits<double>::quiet_NaN();
  for (const Measurement& sample : report.samples) {
    if (std::isfinite(sample.eigenvalue_ratio)) {
      ++finite_count;
      min_eigen_ratio = std::min(min_eigen_ratio, sample.eigenvalue_ratio);
    } else if (finite_count > 0) {
      // localizer.cpp leaves the ratio empty when the Hessian is non-finite or its largest
      // eigenvalue is <= 0. After the first match, that is exactly the degeneracy to catch.
      ++nan_after_first_finite;
      if (!std::isfinite(first_nan_after_finite_sec)) first_nan_after_finite_sec = sample.time_sec;
      last_nan_after_finite_sec = sample.time_sec;
    }
  }
  std::printf("A: max pos err %.3f m, max yaw err %.2f deg, wheel-only end err %.2f m, "
              "min eigen ratio %.4f (finite %d, nan after first finite %d), scan max %.1f ms\n",
              report.max_position_error_m, report.max_yaw_error_deg,
              report.wheel_only_end_error_m, min_eigen_ratio, finite_count,
              nan_after_first_finite, report.scan_ms_max);
  if (nan_after_first_finite > 0) {
    std::printf("A: non-finite eigen_ratio after the first finite sample: %d samples, t=%.3f..%.3f s\n",
                nan_after_first_finite, first_nan_after_finite_sec, last_nan_after_finite_sec);
  }
  EXPECT_GT(report.wheel_only_end_error_m, 2.0);  // the Issue's condition: drift exists
  EXPECT_LT(report.max_position_error_m, 0.2);
  EXPECT_LT(report.max_yaw_error_deg, 2.0);
  EXPECT_TRUE(report.is_tracking_at_end);
  EXPECT_LT(report.scan_ms_max, 450.0);
  EXPECT_GT(finite_count, 0);
  EXPECT_EQ(nan_after_first_finite, 0);
  EXPECT_GE(min_eigen_ratio, 0.02);  // the scene keeps the along-track direction observable
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
  scenario.use_spec_course = true;
  scenario.wheel_scale = 1.01;
  scenario.map_hole_begin_sec = 50.0 / kSpeed;   // x = 50 m ...
  scenario.map_hole_end_sec = 100.0 / kSpeed;    // ... to x = 100 m: the Issue's 50 m hole
  scenario.has_real_map_hole = true;
  const Report report = run(scenario);
  std::printf("F1: hole err %.3f m gps_only=%d tracking_after=%d\n", report.max_error_in_hole_m,
              report.saw_gps_only, report.saw_tracking_after_hole);
  EXPECT_LT(report.max_error_in_hole_m, 1.5);  // GPS sigma 1 m
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

// Review high 3: an optimization failure restarts the window from the current estimate; the pose
// must stay near truth (no uninitialized pose reaches the sub-map) and the cause is reported.
TEST(Localizer, RecoversFromOptimizationFailure) {
  Scenario scenario;
  scenario.fail_optimization_at_sec = 12.3;
  const Report report = run(scenario);
  EXPECT_NE(report.last_error.find("window restarted"), std::string::npos);
  EXPECT_EQ(report.last_error.find("unknown node"), std::string::npos);  // no stale node index
  EXPECT_LT(report.max_position_error_m, 0.3);
  EXPECT_TRUE(report.is_tracking_at_end);
}

// Issue #45: after a wheel-odometry gap longer than the 30 s fixed-lag window the window drops every
// old node, so the LiDAR sub-map must drop those keyframes too. Before the fix they were all drawn at
// the newest pose and last_error stayed at "pose requested for unknown node".
Scenario make_wheel_gap_scenario() {
  Scenario scenario;
  scenario.use_spec_course = true;
  scenario.wheel_scale = 1.01;
  scenario.scan_period_sec = 0.1;
  scenario.wheel_gap_begin_sec = 20.0;
  scenario.wheel_gap_end_sec = 80.0;  // 60 s: twice the window
  return scenario;
}

TEST(Localizer, WheelGapLongerThanTheWindowKeepsTheLidarSubmapPoses) {
  Scenario scenario = make_wheel_gap_scenario();
  scenario.stop_sec = 86.0;  // about five keyframes after the gap
  const Report report = run(scenario);
  EXPECT_EQ(report.last_error.find("unknown node"), std::string::npos) << report.last_error;
  EXPECT_TRUE(report.last_error.empty()) << report.last_error;
  const std::vector<Eigen::Isometry3d>& poses = report.lidar_keyframe_poses;
  ASSERT_GE(poses.size(), 3u);
  // The robot drives along +x at kSpeed: successive keyframes are at least 0.3 m further on, and none
  // is further behind the robot than 10 keyframe-seconds (6 m) plus a 1 m margin.
  constexpr double kMaxDistanceBehindRobotM = 6.0 + 1.0;
  for (std::size_t index = 0; index < poses.size(); ++index) {
    EXPECT_LE((poses[index].translation() - report.final_truth.translation()).norm(), kMaxDistanceBehindRobotM)
        << "keyframe " << index;
    if (index == 0) continue;
    EXPECT_GT(poses[index].translation().x() - poses[index - 1].translation().x(), 0.3)
        << "keyframe " << index << " is stacked on its predecessor";
  }
}

TEST(Localizer, ReturnsToTrackingAfterWheelGapAndInitialPose) {
  Scenario scenario = make_wheel_gap_scenario();
  scenario.reinitialize_after_gap = true;
  scenario.stop_sec = 120.0;
  const Report report = run(scenario);
  EXPECT_TRUE(report.published_after_reinitialize);
  EXPECT_TRUE(report.is_tracking_at_end);
  EXPECT_LT(report.final_position_error_m, 0.5);
  EXPECT_TRUE(report.last_error.empty()) << report.last_error;
}

// Review high 4: initialize() is the way out of LOST.
TEST(Localizer, InitializeLeavesLost) {
  Scenario scenario;
  scenario.has_map = false;
  scenario.is_gps_enabled = false;
  scenario.odom_only_max_distance_m = 8.0;
  scenario.reinitialize_after_lost = true;
  const Report report = run(scenario);
  EXPECT_TRUE(report.saw_lost);
  EXPECT_TRUE(report.published_after_reinitialize);
}

// Review high 6: the scan queue is bounded, also before initialize() and while wheel odometry is
// silent.
TEST(Localizer, ScanQueueIsBounded) {
  Localizer localizer(LocalizerParameters{}, PointList{}, std::nullopt);
  const std::vector<TimedPoint> scan(10, TimedPoint{Eigen::Vector3f(1, 0, 0), 0.0F});
  for (int index = 0; index < 100; ++index) localizer.on_scan(index * 0.1, scan);
  EXPECT_EQ(localizer.pending_scan_count(), 0u);  // not initialized: nothing can consume them
  localizer.initialize(Eigen::Isometry3d::Identity());
  for (int index = 0; index < 100; ++index) localizer.on_scan(index * 0.1, scan);
  EXPECT_LE(localizer.pending_scan_count(), 20u);
}

// Review medium 6: INPUT_STALE is observable without a wheel sample.
TEST(Localizer, PollStateReportsStaleWheel) {
  Localizer localizer(LocalizerParameters{}, PointList{}, std::nullopt);
  EXPECT_EQ(localizer.poll_state(0.0), LocalizationState::kInitializing);
  localizer.initialize(Eigen::Isometry3d::Identity());
  localizer.on_wheel_odom(1.0, Eigen::Isometry3d::Identity());
  EXPECT_EQ(localizer.poll_state(1.5), LocalizationState::kInputStale);
}

// Review medium 7 (D at pipeline level): 20 m GPS jumps go through the Localizer's node insertion
// and Mahalanobis gate; the pose must not move.
TEST(Localizer, GpsJumpsAreGatedInThePipeline) {
  Scenario scenario;
  scenario.gps_outlier_every = 5;
  const Report report = run(scenario);
  std::printf("D: max pos err %.3f m\n", report.max_position_error_m);
  EXPECT_LT(report.max_position_error_m, 0.3);
  EXPECT_TRUE(report.is_tracking_at_end);
}

// Review medium 7 (I at pipeline level): start 0.9 m off truth; the map match pulls the pose in,
// but the published pose may only move at wheel speed + the 0.2 m/s slew, and the pending
// correction is flagged (threshold lowered to 0.5 m for the test) until it has been applied.
TEST(Localizer, InitialOffsetIsSlewedNotJumped) {
  Scenario scenario;
  scenario.initial_offset_m = 0.9;
  scenario.large_correction_threshold_m = 0.5;
  const Report report = run(scenario);
  std::printf("I: max output speed %.3f m/s, large=%d, final err %.3f m\n", report.max_output_speed_mps,
              report.saw_large_correction, report.final_position_error_m);
  EXPECT_LE(report.max_output_speed_mps, kSpeed * 1.01 + 0.2 + 0.05);
  EXPECT_TRUE(report.saw_large_correction);
  EXPECT_LT(report.final_position_error_m, 0.2);
}

// H at pipeline level: the wheel odometry is blind to the 10 degree climb (dz = pitch = 0, weak
// constraint); lidar odometry and map matching must supply roll/pitch/z. Truth includes z and pitch.
TEST(Localizer, ClimbsA10DegreeRampAtTenHertz) {
  Scenario scenario;
  scenario.has_ramp = true;
  scenario.scan_period_sec = 0.1;
  const Report report = run(scenario);
  std::printf("H: max pos err %.3f m, pitch err over x in [5, 15] m: %.2f deg\n",
              report.max_position_error_m, report.max_pitch_error_on_ramp_deg);
  EXPECT_FALSE(report.samples.empty());
  EXPECT_LE(report.max_pitch_error_on_ramp_deg, 2.0);  // the Issue's +-2 deg over the whole climb
  EXPECT_LT(report.max_position_error_m, 0.2);
  EXPECT_TRUE(report.is_tracking_at_end);
}

// Review high 1: the pitch changes by 10 degrees at once (the LiDAR odometry gate used to reject
// anything above 0.1 rad of full rotation). The judgement is the settling time: the first sample
// within 2 degrees after the step at x = 5 m must be <= 0.3 s, and after 0.3 s from each step
// (entry at x = 5 m, exit at x = 15 m) every sample must stay within 2 degrees.
TEST(Localizer, RecoversFromPitchStepWithinThreeTenthsAtTenHertz) {
  Scenario scenario;
  scenario.has_ramp = true;
  scenario.ramp_blend_m = 0.0;
  scenario.scan_period_sec = 0.1;
  const Report report = run(scenario);
  std::printf("H step: recovery %.3f s, entry max %.3f deg, exit max %.3f deg, max pos err %.3f m\n",
              report.pitch_step_recovery_sec, report.max_pitch_error_after_step_entry_deg,
              report.max_pitch_error_after_step_exit_deg, report.max_position_error_m);
  EXPECT_LE(report.pitch_step_recovery_sec, 0.3);
  EXPECT_GT(report.pitch_samples_after_step_entry, 0);
  EXPECT_GT(report.pitch_samples_after_step_exit, 0);
  EXPECT_LE(report.max_pitch_error_after_step_entry_deg, 2.0);
  EXPECT_LE(report.max_pitch_error_after_step_exit_deg, 2.0);
  EXPECT_LT(report.max_position_error_m, 0.2);
  EXPECT_TRUE(report.is_tracking_at_end);
}

// Review medium 1: a corridor whose ends cannot be seen. The Issue's 1 % wheel scale error gives
// ~1.2 m of along-track drift over 120 m, and the 0.5 deg / 10 m yaw drift adds a lateral error
// (wheel-only end error 6.41 m). The map factor must not freeze that drift (its sliding direction
// is weak) so GPS (sigma 1 m) pulls the pose back.
TEST(Localizer, OpenCorridorIsCorrectedByGpsNotFrozenByMap) {
  Scenario scenario;
  scenario.is_open_corridor = true;
  scenario.wheel_scale = 1.01;
  const Report report = run(scenario);
  std::printf("open corridor: wheel-only end err %.2f m, max pos err %.2f m, final %.2f m\n",
              report.wheel_only_end_error_m, report.max_position_error_m, report.final_position_error_m);
  EXPECT_FALSE(report.samples.empty());
  EXPECT_TRUE(report.is_tracking_at_end);
  EXPECT_GT(report.wheel_only_end_error_m, 1.0);
  EXPECT_LT(report.final_position_error_m, 1.0);
  EXPECT_LT(report.max_position_error_m, 1.0);
}

// Review medium 3: a failing prior-map rebuild (worker exception, or thread start failure) keeps
// the old target, records the cause and does not stop the localizer.
TEST(Localizer, MapRebuildFailureKeepsOldTarget) {
  for (const int mode : {1, 2}) {
    Scenario scenario;
    scenario.map_rebuild_distance_m = 5.0;
    scenario.rebuild_failure_mode = mode;
    const Report report = run(scenario);
    EXPECT_NE(report.last_error.find("prior-map rebuild"), std::string::npos) << "mode " << mode;
    EXPECT_LT(report.max_position_error_m, 0.3) << "mode " << mode;
    EXPECT_TRUE(report.is_tracking_at_end) << "mode " << mode;
  }
}

// The design case from the #2 review (1-4 / L2-c): with a 5 % wheel error and a precise GPS
// (sigma 0.1 m, CLAS Fixed equivalent), the old gate rejected the precise fixes in the corridor and
// the pose diverged with the wheel error. The trusted-fix path must keep it bounded.
TEST(Localizer, OpenCorridorWithPreciseGpsAndFivePercentWheel) {
  Scenario scenario;
  scenario.is_open_corridor = true;
  scenario.wheel_scale = 1.05;
  scenario.gps_noise_m = 0.1;
  scenario.gps_sigma_m = 0.1;
  const Report report = run(scenario);
  std::printf("open corridor (5 %% wheel, GPS 0.1 m): max pos err %.2f m, final %.2f m, accepted %.2f\n",
              report.max_position_error_m, report.final_position_error_m,
              report.gps_acceptance_ratio);
  EXPECT_FALSE(report.samples.empty());
  EXPECT_LT(report.final_position_error_m, 0.7);
  EXPECT_LT(report.max_position_error_m, 0.7);
  EXPECT_GE(report.gps_acceptance_ratio, 0.8);
}

TEST(LocalizerMeasurements, PillarsAppearInMapAndScan) {
  SyntheticWorld world({{0.0, 0.0}, {40.0, 0.0}}, false);
  const auto pose_at = [](double) {
    return arc_pose(0.0, Eigen::Vector2d(9.0, 0.0), 0.0, 0.0, 0.0);
  };
  const auto original_map_size = world.sample_map(0.2).size();
  const auto original_scan = world.synthesize_scan(pose_at, 0.0, 0.1, true, 360, 8);
  world.add_pillars();
  EXPECT_GT(world.sample_map(0.2).size(), original_map_size);
  const auto pillar_scan = world.synthesize_scan(pose_at, 0.0, 0.1, true, 360, 8);
  bool scan_changed = original_scan.size() != pillar_scan.size();
  for (std::size_t index = 0; index < std::min(original_scan.size(), pillar_scan.size()); ++index) {
    scan_changed |= (original_scan[index].position - pillar_scan[index].position).norm() > 0.01F;
  }
  EXPECT_TRUE(scan_changed);
}

TEST(Localizer, RejectsGpsBeyondKeyframeIntervalAndMargin) {
  Localizer localizer(LocalizerParameters{}, PointList{}, kOrigin);
  const Eigen::Isometry3d stationary_pose = Eigen::Isometry3d::Identity();
  localizer.initialize(stationary_pose);
  localizer.on_wheel_odom(0.0, stationary_pose);
  localizer.on_scan(0.0, std::vector<TimedPoint>(10, TimedPoint{Eigen::Vector3f(1, 0, 0), 0.0F}));
  localizer.on_wheel_odom(0.1, stationary_pose);
  GpsFix fix = fix_at_enu(0.0, 0.0);
  fix.has_known_covariance = true;
  fix.known_std_m = 0.1;
  localizer.on_gps(1.2, fix);
  localizer.on_wheel_odom(1.2, stationary_pose);
  EXPECT_EQ(localizer.gps_time_gap_rejection_count(), 0u);
  EXPECT_DOUBLE_EQ(localizer.last_accepted_gps_stamp_sec(), 1.2);
  localizer.on_gps(1.21, fix);
  localizer.on_wheel_odom(1.21, stationary_pose);
  EXPECT_EQ(localizer.gps_time_gap_rejection_count(), 1u);
  EXPECT_DOUBLE_EQ(localizer.last_accepted_gps_stamp_sec(), 1.2);
}

TEST(Localizer, CompensatesWheelDisplacementBetweenGpsAndKeyframe) {
  LocalizerParameters parameters;
  parameters.corrector.slew_mps = 100.0;
  Localizer localizer(parameters, PointList{}, kOrigin);
  Eigen::Isometry3d initial_pose = Eigen::Isometry3d::Identity();
  initial_pose.linear() = Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  localizer.initialize(initial_pose);
  localizer.on_wheel_odom(0.0, Eigen::Isometry3d::Identity());
  localizer.on_scan(0.0, std::vector<TimedPoint>(10, TimedPoint{Eigen::Vector3f(1, 0, 0), 0.0F}));
  localizer.on_wheel_odom(0.1, Eigen::Isometry3d::Identity());
  Eigen::Isometry3d wheel_pose = Eigen::Isometry3d::Identity();
  wheel_pose.translation().x() = 2.0;
  GpsFix fix = fix_at_enu(0.0, 2.0);
  fix.has_known_covariance = true;
  fix.known_std_m = 0.1;
  localizer.on_gps(0.4, fix);
  const auto output = localizer.on_wheel_odom(0.4, wheel_pose);
  ASSERT_TRUE(output);
  EXPECT_DOUBLE_EQ(localizer.last_accepted_gps_stamp_sec(), 0.4);
  EXPECT_NEAR(output->map_to_base.translation().x(), 0.0, 0.02);
  EXPECT_NEAR(output->map_to_base.translation().y(), 2.0, 0.02);
  EXPECT_EQ(localizer.gps_time_gap_rejection_count(), 0u);
}

TEST(Localizer, CountsFirstMapMatchRejectionReason) {
  const SyntheticWorld world(kShortCourse.centerline());
  LocalizerParameters parameters;
  parameters.keyframe_time_s = 0.1;
  parameters.match_period_s = 0.0;
  Localizer localizer(parameters, world.sample_map(0.2), std::nullopt);
  const Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  localizer.initialize(pose);
  int attempt = 0;
  localizer.set_map_match_hook([&](MatchResult& result) {
    ++attempt;
    result.converged = attempt != 1;
    result.inlier_ratio = attempt == 2 ? 0.0 : 1.0;
    result.target_to_source = pose;
    if (attempt == 3) result.target_to_source.translation().x() = 10.0;
    if (attempt == 4) result.target_to_source.linear() = Eigen::AngleAxisd(1.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    result.information = Eigen::Matrix<double, 6, 6>::Identity();
  });
  const auto truth = [&](double) { return pose; };
  for (int index = 0; index <= 4; ++index) {
    const double stamp = index * 0.2;
    localizer.on_wheel_odom(stamp, pose);
    localizer.on_scan(stamp, world.synthesize_scan(truth, stamp, 0.0, false));
    localizer.on_wheel_odom(stamp + 0.01, pose);
  }
  const MapMatchStatistics counts = localizer.map_match_statistics();
  EXPECT_EQ(counts.attempted_total, 4U);
  EXPECT_EQ(counts.accepted_total, 0U);
  EXPECT_EQ(counts.rejected_not_converged_total, 1U);
  EXPECT_EQ(counts.rejected_inlier_total, 1U);
  EXPECT_EQ(counts.rejected_translation_gate_total, 1U);
  EXPECT_EQ(counts.rejected_rotation_gate_total, 1U);
}

TEST(Localizer, DegenerateMapMatchDoesNotRefreshAbsoluteObservation) {
  const SyntheticWorld world(kShortCourse.centerline());
  LocalizerParameters parameters;
  parameters.keyframe_time_s = 0.1;
  parameters.match_period_s = 0.0;
  Localizer localizer(parameters, world.sample_map(0.2), std::nullopt);
  const Eigen::Isometry3d stationary_pose = Eigen::Isometry3d::Identity();
  localizer.initialize(stationary_pose);
  int match_count = 0;
  bool degenerate = true;
  localizer.set_map_match_hook([&](MatchResult& result) {
    ++match_count;
    result.converged = true;
    result.inlier_ratio = 1.0;
    result.target_to_source = stationary_pose;
    result.information = Eigen::Matrix<double, 6, 6>::Identity();
    if (degenerate) result.information(3, 3) = 0.001;
  });
  const auto truth = [&](double) { return stationary_pose; };
  for (int index = 0; index <= 2; ++index) {
    const double stamp = index * 0.2;
    localizer.on_wheel_odom(stamp, stationary_pose);
    localizer.on_scan(stamp, world.synthesize_scan(truth, stamp, 0.0, false));
    localizer.on_wheel_odom(stamp + 0.01, stationary_pose);
  }
  ASSERT_GT(match_count, 0);
  EXPECT_DOUBLE_EQ(localizer.last_absolute_observation_stamp_sec(), 0.0);
  degenerate = false;
  localizer.on_wheel_odom(0.6, stationary_pose);
  localizer.on_scan(0.6, world.synthesize_scan(truth, 0.6, 0.0, false));
  localizer.on_wheel_odom(0.61, stationary_pose);
  EXPECT_DOUBLE_EQ(localizer.last_absolute_observation_stamp_sec(), 0.6);
}


TEST(Localizer, KeyframeTimeComparisonToleratesTimestampRounding) {
  Localizer localizer(LocalizerParameters{}, PointList{}, std::nullopt);
  const Eigen::Isometry3d stationary_pose = Eigen::Isometry3d::Identity();
  localizer.initialize(stationary_pose);
  for (const double stamp : {0.2, 1.1999, 1.2, 2.2, 3.2}) {
    localizer.on_wheel_odom(stamp, stationary_pose);
    localizer.on_scan(stamp, std::vector<TimedPoint>(10, TimedPoint{Eigen::Vector3f(1, 0, 0), 0.0F}));
    localizer.on_wheel_odom(stamp, stationary_pose);
    if (stamp == 1.1999) {
      EXPECT_DOUBLE_EQ(localizer.max_keyframe_interval_sec(), 0.0);
    }
  }
  EXPECT_NEAR(localizer.max_keyframe_interval_sec(), 1.0, 1e-12);
}

TEST(Localizer, EventErrorExpiresAndRetainsCumulativeCount) {
  LocalizerParameters parameters;
  parameters.last_error_hold_s = 1.0;
  Localizer localizer(parameters, PointList{}, std::nullopt);
  const Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  localizer.initialize(pose);
  localizer.poll_state(10.0);
  localizer.on_wheel_odom(2.0, pose);
  localizer.on_wheel_odom(1.0, pose);
  EXPECT_EQ(localizer.error_events_total(), 1U);
  EXPECT_NE(localizer.last_error().find("time went backwards"), std::string::npos);
  localizer.poll_state(10.9);
  EXPECT_FALSE(localizer.last_error().empty());
  localizer.on_wheel_odom(0.5, pose);
  EXPECT_EQ(localizer.error_events_total(), 2U);
  localizer.poll_state(11.0);
  EXPECT_FALSE(localizer.last_error().empty());
  localizer.poll_state(12.0);
  EXPECT_TRUE(localizer.last_error().empty());
  EXPECT_EQ(localizer.error_events_total(), 2U);
}

TEST(Localizer, ScanTimeStatisticsIncludeDeskewFailuresAndResetAfterReading) {
  Localizer localizer(LocalizerParameters{}, PointList{}, std::nullopt);
  const Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  localizer.initialize(pose);
  localizer.on_wheel_odom(1.0, pose);
  // The scan precedes the first wheel sample and cannot be deskewed.
  localizer.on_scan(0.5, {TimedPoint{Eigen::Vector3f(1, 0, 0), 0.0F}});
  localizer.on_wheel_odom(1.1, pose);
  localizer.on_scan(1.1, std::vector<TimedPoint>(10, TimedPoint{Eigen::Vector3f(1, 0, 0), 0.0F}));
  localizer.on_wheel_odom(1.1, pose);

  const ScanTimeStatistics statistics = localizer.take_scan_time_statistics();
  EXPECT_EQ(statistics.count, 2U);
  EXPECT_GT(statistics.total_ms, 0.0);
  EXPECT_GT(statistics.maximum_ms, 0.0);
  EXPECT_LE(statistics.maximum_ms, statistics.total_ms);
  const ScanTimeStatistics reset = localizer.take_scan_time_statistics();
  EXPECT_EQ(reset.count, 0U);
  EXPECT_DOUBLE_EQ(reset.total_ms, 0.0);
  EXPECT_DOUBLE_EQ(reset.maximum_ms, 0.0);
}

TEST(Localizer, AcceptsOneHertzGpsAtLateKeyframePhases) {
  for (const double phase : {0.6, 0.7, 0.8, 0.9}) {
    Localizer localizer(LocalizerParameters{}, PointList{}, kOrigin);
    const auto pose = Eigen::Isometry3d::Identity();
    localizer.initialize(pose);
    GpsFix fix = fix_at_enu(0.0, 0.0);
    fix.has_known_covariance = true;
    fix.known_std_m = 0.1;
    int accepted = 0;
    for (int second = 0; second < 10; ++second) {
      localizer.on_wheel_odom(second, pose);
      localizer.on_scan(second, std::vector<TimedPoint>(10, TimedPoint{Eigen::Vector3f(1, 0, 0), 0.0F}));
      localizer.on_wheel_odom(second, pose);
      localizer.on_gps(second + phase, fix);
      localizer.on_wheel_odom(second + phase, pose);
      if (localizer.last_accepted_gps_stamp_sec() == second + phase) ++accepted;
    }
    EXPECT_GE(accepted, 8);
    std::printf("[ MEASURED ] gps_phase_sec=%.1f accepted=%d submitted=10\n", phase, accepted);
  }
}

TEST(Localizer, PreciseBiasedGpsConflictsWithoutQualityRejectionsDilutingWindow) {
  LocalizerParameters parameters;
  parameters.gps_conflict_window = 4;
  Localizer localizer(parameters, PointList{}, kOrigin);
  const auto pose = Eigen::Isometry3d::Identity();
  localizer.initialize(pose);
  GpsFix fix = fix_at_enu(3.0, 0.0);
  fix.has_known_covariance = true;
  fix.known_std_m = 0.1;
  std::optional<LocalizerOutput> output;
  for (int index = 0; index < 10; ++index) {
    const double stamp = index * 0.1;
    localizer.on_wheel_odom(stamp, pose);
    localizer.on_scan(stamp, std::vector<TimedPoint>(10, TimedPoint{Eigen::Vector3f(1, 0, 0), 0.0F}));
    fix.status = (index == 1 || index == 4 || index == 7 || index == 9) ? GpsFixStatus::kFix : GpsFixStatus::kNoFix;
    localizer.on_gps(stamp, fix);
    output = localizer.on_wheel_odom(stamp, pose);
  }
  ASSERT_TRUE(output);
  EXPECT_EQ(localizer.gps_gate_rejection_count(), 4u);
  EXPECT_TRUE(output->is_gps_conflict);
  EXPECT_LT(localizer.last_accepted_gps_stamp_sec(), 0.0);
}

TEST(LocalizerMeasurements, TenHertzGpsFactorsAccumulateOnOneKeyframe) {
  Localizer localizer(LocalizerParameters{}, PointList{}, kOrigin);
  const auto pose = Eigen::Isometry3d::Identity();
  localizer.initialize(pose);
  localizer.on_wheel_odom(0.0, pose);
  localizer.on_scan(0.0, std::vector<TimedPoint>(10, TimedPoint{Eigen::Vector3f(1, 0, 0), 0.0F}));
  localizer.on_wheel_odom(0.0, pose);
  GpsFix fix = fix_at_enu(0.0, 0.0);
  for (int index = 0; index < 12; ++index) {
    const double stamp = index * 0.1;
    localizer.on_gps(stamp, fix);
    localizer.on_wheel_odom(stamp, pose);
  }
  EXPECT_EQ(localizer.max_gps_factors_per_keyframe(), 12u);
  std::printf("[ MEASURED ] gps_hz=10 keyframes=1 duration_sec=1.1 max_gps_factors_per_keyframe=%zu\n", localizer.max_gps_factors_per_keyframe());
}

TEST(Localizer, DegenerateCorridorWithoutGpsDoesNotBecomeLostOver150Meters) {
  Scenario scenario;
  scenario.is_open_corridor = true;
  scenario.open_corridor_length_m = 150.0;
  // This covers ideal wheel scale (1.0) while map matches remain fresh.
  // With 5% scale drift, matching was lost and the estimator became LOST; see MEASUREMENTS.md.
  scenario.wheel_scale = 1.0;
  scenario.is_gps_enabled = false;
  const Report report = run(scenario);
  EXPECT_TRUE(report.is_tracking_at_end);
  ASSERT_FALSE(report.samples.empty());
  EXPECT_GT(report.samples.back().time_sec, 249.0);
  EXPECT_GT(report.samples.back().position_std_m, 2.0);
}

// This 10 Hz case monitors tracking; it also passes with develop-equivalent settings.
TEST(Localizer, WheelYawSlipDuringTurnKeepsTrackingAndYawCloseToTruth) {
  Scenario scenario;
  scenario.has_pillars = true;
  scenario.degeneracy_ratio = 0.0;
  scenario.scan_period_sec = 0.1;
  scenario.correction_slew_rps = 2.0;
  scenario.is_gps_enabled = false;
  scenario.wheel_scale = 1.0;
  scenario.slip_gate_yaw_rad = 0.35;
  scenario.wheel_yaw_slip_threshold_rad = 0.1;
  scenario.slip_begin_sec = 20.0 / kSpeed + 0.5;
  scenario.slip_end_sec = scenario.slip_begin_sec + 0.8;
  scenario.stop_sec = scenario.slip_end_sec + 2.0;
  const Report report = run(scenario);
  std::printf("[ MEASURED ] wheel_yaw_slip_max_error_deg=%.9g\n", report.max_yaw_error_deg);
  EXPECT_FALSE(report.saw_odom_only_during_slip);
  EXPECT_TRUE(report.is_tracking_at_end);
  EXPECT_LT(report.max_yaw_error_deg, 5.5);
}

// T2 (Issue #92): the same drive with gyro increments must not get worse (the first turn slips, so the
// gyro may not be verified yet while it slips).
TEST(LocalizerGyroDrive, T2_TheExistingSlipDriveDoesNotGetWorseWithTheGyro) {
  Scenario scenario;
  scenario.has_pillars = true;
  scenario.degeneracy_ratio = 0.0;
  scenario.scan_period_sec = 0.1;
  scenario.correction_slew_rps = 2.0;
  scenario.is_gps_enabled = false;
  scenario.wheel_scale = 1.0;
  scenario.slip_gate_yaw_rad = 0.35;
  scenario.wheel_yaw_slip_threshold_rad = 0.1;
  scenario.slip_begin_sec = 20.0 / kSpeed + 0.5;
  scenario.slip_end_sec = scenario.slip_begin_sec + 0.8;
  scenario.stop_sec = scenario.slip_end_sec + 2.0;
  const Report without_gyro = run(scenario);
  scenario.gives_gyro_increments = true;
  const Report with_gyro = run(scenario);
  std::printf("[ MEASURED ] T2 max yaw error without gyro %.3f deg, with gyro %.3f deg\n",
              without_gyro.max_yaw_error_deg, with_gyro.max_yaw_error_deg);
  EXPECT_LE(with_gyro.max_yaw_error_deg, without_gyro.max_yaw_error_deg + 0.2);
  EXPECT_LT(with_gyro.max_yaw_error_deg, 5.5);
  EXPECT_TRUE(with_gyro.is_tracking_at_end);
}

// Sparse observations expose the effect of slip handling; retain the 10 Hz test above.
TEST(Localizer, WheelYawSlipWithSparseScansKeepsYawBounded) {
  Scenario scenario;
  scenario.has_pillars = true;
  scenario.degeneracy_ratio = 0.0;
  scenario.scan_period_sec = 0.3;
  scenario.correction_slew_rps = 2.0;
  scenario.is_gps_enabled = false;
  scenario.wheel_scale = 1.0;
  scenario.slip_gate_yaw_rad = 0.35;
  scenario.wheel_yaw_slip_threshold_rad = 0.1;
  scenario.slip_begin_sec = 20.0 / kSpeed + 0.5;
  scenario.slip_end_sec = scenario.slip_begin_sec + 0.8;
  scenario.stop_sec = scenario.slip_end_sec + 2.0;
  const Report report = run(scenario);
  std::printf("[ MEASURED ] wheel_yaw_slip_max_error_deg=%.9g\n", report.max_yaw_error_deg);
  EXPECT_FALSE(report.saw_odom_only_during_slip);
  EXPECT_TRUE(report.is_tracking_at_end);
  EXPECT_LT(report.max_yaw_error_deg, 10.0);
}

TEST(Localizer, RejectsInvalidSlipWheelRateWindow) {
  for (double invalid : {0.0, -0.1, std::numeric_limits<double>::infinity(),
                         std::numeric_limits<double>::quiet_NaN()}) {
    LocalizerParameters parameters;
    parameters.lidar_odometry.slip_wheel_rate_window_s = invalid;
    EXPECT_THROW(Localizer localizer(parameters, PointList{}, std::nullopt), std::invalid_argument);
  }
}

// ---- Without a gyro increment the Localizer must behave exactly as before (Issue #92, section 4.1). ----
namespace {

// Compares a run with the recording of the code before the gyro existed (commit a1771d0; see
// test/data/README.md). Where the floating point rounds as it did when recording, every output must match
// bit for bit. Elsewhere (another compiler or CPU: fused multiply-add changes the last bits) the state of
// every output must still be identical and every number must agree within 1e-4.
void expect_matches_recording_before_the_gyro(bool has_prior_map, const std::vector<RecordedOutput>& run) {
  const std::filesystem::path path =
      std::filesystem::path(__FILE__).parent_path() / "data" / golden_file_name(has_prior_map);
  const std::vector<GoldenOutput> golden = read_golden(path.string());
  ASSERT_FALSE(golden.empty()) << path;
  ASSERT_EQ(golden.size(), run.size());
  std::size_t first_different_hash = golden.size();
  for (std::size_t index = 0; index < golden.size(); ++index) {
    if (hash_output(run[index]) != golden[index].hash) {
      first_different_hash = index;
      break;
    }
  }
  if (first_different_hash == golden.size()) return;
#if defined(__APPLE__)
  ADD_FAILURE() << "output " << first_different_hash << " differs from the recording before the gyro";
#else
  constexpr double kToleranceOtherPlatform = 1e-4;
  for (std::size_t index = 0; index < golden.size(); ++index) {
    EXPECT_EQ(static_cast<int>(run[index].state), golden[index].state) << "output " << index;
    const std::array<double, 7> values = output_values(run[index]);
    for (std::size_t column = 0; column < values.size(); ++column) {
      EXPECT_NEAR(values[column], golden[index].values[column], kToleranceOtherPlatform)
          << "output " << index << " value " << column;
    }
  }
#endif
}

enum class WheelCall { kTwoArguments, kNoGyroIncrement };

std::vector<RecordedOutput> run_exactly(bool has_prior_map, WheelCall wheel_call,
                                        GyroConsistencyState initial_state = GyroConsistencyState::kUnverified,
                                        GyroFusionStatus* final_status = nullptr) {
  return drive_short_course(
      has_prior_map,
      [&](LocalizerParameters& parameters) {
        parameters.initial_gyro_consistency = initial_state;
        if (wheel_call == WheelCall::kNoGyroIncrement) {
          parameters.graph.gyro_noise_density_rad_s_sqrt_hz = 0.5;
          parameters.graph.gyro_bias_sigma_rad_s = 0.25;
          parameters.graph.gyro_scale_sigma = 0.3;
          parameters.gyro_yaw_prediction_enabled = true;
        }
      },
      [&](Localizer& localizer, double time_sec, const Eigen::Isometry3d& odom) {
        return wheel_call == WheelCall::kTwoArguments ? localizer.on_wheel_odom(time_sec, odom)
                                                      : localizer.on_wheel_odom(time_sec, odom, std::nullopt);
      },
      [&](Localizer& localizer) {
        if (final_status) *final_status = localizer.gyro_fusion_status();
      });
}

void expect_unchanged_by_missing_gyro_increments(bool has_prior_map) {
  const std::vector<RecordedOutput> two_arguments = run_exactly(has_prior_map, WheelCall::kTwoArguments);
  ASSERT_FALSE(two_arguments.empty());
  expect_matches_recording_before_the_gyro(has_prior_map, two_arguments);
  // Increments given as "none", with other gyro noise parameters.
  GyroFusionStatus status;
  expect_matches_recording_before_the_gyro(
      has_prior_map, run_exactly(has_prior_map, WheelCall::kNoGyroIncrement, GyroConsistencyState::kUnverified, &status));
  EXPECT_EQ(status.rotation_factors_added_total, 0U);
  EXPECT_EQ(status.yaw_increments_gyro_total, 0U);
  EXPECT_EQ(status.consistency, GyroConsistencyState::kUnverified);
  // A gyro that is already active but never gets an increment must not change anything either.
  expect_matches_recording_before_the_gyro(
      has_prior_map, run_exactly(has_prior_map, WheelCall::kNoGyroIncrement, GyroConsistencyState::kActive));
}

}  // namespace

// Writes the recording when LIGHTNAV_WRITE_GOLDEN=<directory> is set; otherwise it does nothing. Run it on
// a work tree of commit a1771d0 to record the outputs of the code before the gyro (test/data/README.md).
TEST(GoldenRecording, WriteWhenRequested) {
  const char* const directory = std::getenv("LIGHTNAV_WRITE_GOLDEN");
  if (directory == nullptr) return;
  write_golden_recordings(directory);
}

TEST(LocalizerGyro, MappingWithoutGnssIsIdenticalWhenNoGyroIncrementIsGiven) {
  expect_unchanged_by_missing_gyro_increments(false);
}

TEST(LocalizerGyro, LocalizationWithAPriorMapIsIdenticalWhenNoGyroIncrementIsGiven) {
  expect_unchanged_by_missing_gyro_increments(true);
}

TEST(LocalizerGyro, InitializeCarriesOverTheStateAndTheCountOfLostVotes) {
  for (const GyroConsistencyState state :
       {GyroConsistencyState::kUnverified, GyroConsistencyState::kActive, GyroConsistencyState::kSuspended,
        GyroConsistencyState::kDisabledInconsistent}) {
    LocalizerParameters parameters;
    parameters.initial_gyro_consistency = state;
    parameters.initial_gyro_outvoted_total = 2;
    Localizer localizer(parameters, PointList{}, std::nullopt);
    EXPECT_EQ(localizer.gyro_fusion_status().consistency, state);
    EXPECT_EQ(localizer.gyro_fusion_status().consistency_outvoted_total, 2U);
    for (int rebuild = 0; rebuild < 2; ++rebuild) {
      localizer.initialize(Eigen::Isometry3d::Identity());
      EXPECT_EQ(localizer.gyro_fusion_status().consistency, state);
      EXPECT_EQ(localizer.gyro_fusion_status().consistency_outvoted_total, 2U);
    }
  }
}

// ---- Yaw replacement, rotation factors and the raw wheel pose (Issue #92, section 3.3 steps 2, 4, 5). ----
namespace {

constexpr double kTurnYawScale = 1.4;  // the wheel over-reports every yaw change by 40 %

double yaw_of_pose(const Eigen::Isometry3d& pose) {
  return std::atan2(pose.rotation()(1, 0), pose.rotation()(0, 0));
}

double yaw_error_deg(const Eigen::Isometry3d& estimate, const Eigen::Isometry3d& truth) {
  const Eigen::Matrix3d difference = truth.rotation().transpose() * estimate.rotation();
  return std::abs(std::atan2(difference(1, 0), difference(0, 0))) * 180.0 / M_PI;
}

// Straight wheel steps along +x with a yaw change per step; no scans, so nothing but the odometry acts.
Eigen::Isometry3d planar_step(double distance_m, double yaw_rad) {
  Eigen::Isometry3d step = Eigen::Isometry3d::Identity();
  step.translation().x() = distance_m;
  step.linear() = Eigen::AngleAxisd(yaw_rad, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  return step;
}

// Without scans the state stops publishing after a short while, so the drive stays short.
constexpr int kStepsWithoutScans = 20;
constexpr double kRawYawPerStepRad = 0.01;
constexpr double kTrueYawPerStepRad = 0.005;

Eigen::Isometry3d drive_without_scans(GyroConsistencyState initial_state, bool is_prediction_enabled,
                                      bool should_give_gyro_increments) {
  LocalizerParameters parameters;
  parameters.initial_gyro_consistency = initial_state;
  parameters.gyro_yaw_prediction_enabled = is_prediction_enabled;
  Localizer localizer(parameters, PointList{}, std::nullopt);
  localizer.initialize(Eigen::Isometry3d::Identity());
  Eigen::Isometry3d odom = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d last_output = Eigen::Isometry3d::Identity();
  for (int step = 0; step <= kStepsWithoutScans; ++step) {
    if (step > 0) odom = odom * planar_step(0.01, kRawYawPerStepRad);
    const auto output = should_give_gyro_increments
        ? localizer.on_wheel_odom(step * kDt, odom, step > 0 ? std::optional<double>(kTrueYawPerStepRad) : std::nullopt)
        : localizer.on_wheel_odom(step * kDt, odom);
    if (output) last_output = output->map_to_base;
  }
  return last_output;
}

}  // namespace

TEST(LocalizerGyro, ActiveGyroReplacesTheWheelYawOfTheOutput) {
  const double raw_total_rad = kStepsWithoutScans * kRawYawPerStepRad;
  const double true_total_rad = kStepsWithoutScans * kTrueYawPerStepRad;
  const Eigen::Isometry3d replaced = drive_without_scans(GyroConsistencyState::kActive, true, true);
  EXPECT_NEAR(yaw_of_pose(replaced), true_total_rad, 1e-9);
  const Eigen::Isometry3d raw = drive_without_scans(GyroConsistencyState::kUnverified, true, false);
  EXPECT_NEAR(yaw_of_pose(raw), raw_total_rad, 1e-9);
}

TEST(LocalizerGyro, GyroIsNotUsedUnlessActiveAndEnabled) {
  const Eigen::Isometry3d raw = drive_without_scans(GyroConsistencyState::kUnverified, true, false);
  // (kSuspended is not listed: initialize() turns it into kActive.)
  for (const GyroConsistencyState state : {GyroConsistencyState::kUnverified,
                                           GyroConsistencyState::kDisabledInconsistent}) {
    EXPECT_EQ(drive_without_scans(state, true, true).matrix(), raw.matrix());
  }
  EXPECT_EQ(drive_without_scans(GyroConsistencyState::kActive, false, true).matrix(), raw.matrix());
}

namespace {

struct GyroRunOptions {
  GyroConsistencyState initial_state = GyroConsistencyState::kActive;
  bool is_prediction_enabled = true;
  double wheel_yaw_scale = kTurnYawScale;
  double gyro_yaw_scale = 1.0;
  int missing_gyro_every = 0;  // every n-th wheel sample comes without a gyro increment
  bool is_lidar_rejected = true;
};

struct GyroRunResult {
  double final_yaw_error_deg;
  GyroFusionStatus status;
};

// The 40 m short course in mapping mode without GNSS. The LiDAR odometry is rejected (unless asked
// otherwise) so that only the wheel and the gyro decide the heading.
GyroRunResult run_with_gyro(const GyroRunOptions& options) {
  const SyntheticWorld world(kShortCourse.centerline(), true);
  LocalizerParameters parameters;
  parameters.mapping = true;
  parameters.initial_gyro_consistency = options.initial_state;
  parameters.gyro_yaw_prediction_enabled = options.is_prediction_enabled;
  Localizer localizer(parameters, PointList{}, std::nullopt);
  if (options.is_lidar_rejected) {
    localizer.set_lidar_match_hook_for_test([](MatchResult& result) { result.is_accepted = false; });
  }
  const auto truth_at = [](double time_sec) { return kShortCourse.truth_at(time_sec); };
  localizer.initialize(truth_at(0.0));
  Eigen::Isometry3d odom = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d last_output = truth_at(0.0);
  const int step_count = static_cast<int>(kShortCourse.total_sec() / kDt);
  for (int step = 0; step <= step_count; ++step) {
    const double time_sec = step * kDt;
    std::optional<double> gyro_increment;
    if (step > 0) {
      const Eigen::Isometry3d relative = truth_at(time_sec - kDt).inverse() * truth_at(time_sec);
      const double true_yaw = std::atan2(relative.rotation()(1, 0), relative.rotation()(0, 0));
      odom = odom * planar_step(relative.translation().norm(), true_yaw * options.wheel_yaw_scale);
      const bool is_missing = options.missing_gyro_every > 0 && step % options.missing_gyro_every == 0;
      if (!is_missing) gyro_increment = true_yaw * options.gyro_yaw_scale;
    }
    if (step > 0 && step % static_cast<int>(0.2 / kDt) == 0 && time_sec >= kScanDuration) {
      localizer.on_scan(time_sec, world.synthesize_scan(truth_at, time_sec - kScanDuration, kScanDuration, true, 360, 8));
    }
    const auto output = localizer.on_wheel_odom(time_sec, odom, gyro_increment);
    if (output) last_output = output->map_to_base;
  }
  return {yaw_error_deg(last_output, truth_at(kShortCourse.total_sec())), localizer.gyro_fusion_status()};
}

}  // namespace

TEST(LocalizerGyro, ActiveGyroKeepsTheHeadingWhenTheWheelOverReportsTheTurn) {
  GyroRunOptions without_gyro_use;
  without_gyro_use.initial_state = GyroConsistencyState::kDisabledInconsistent;  // the gyro stays unused
  const GyroRunResult wheel_only = run_with_gyro(without_gyro_use);
  EXPECT_GT(wheel_only.final_yaw_error_deg, 20.0);  // 40 % of a 90 degree turn
  EXPECT_EQ(wheel_only.status.rotation_factors_added_total, 0U);

  const GyroRunResult with_gyro = run_with_gyro(GyroRunOptions{});
  EXPECT_LT(with_gyro.final_yaw_error_deg, 2.0);
  EXPECT_GT(with_gyro.status.rotation_factors_added_total, 0U);
  EXPECT_EQ(with_gyro.status.rotation_factors_skipped_total, 0U);
  EXPECT_GT(with_gyro.status.yaw_increments_gyro_total, 0U);
  EXPECT_EQ(with_gyro.status.yaw_increments_wheel_total, 0U);
  ASSERT_TRUE(with_gyro.status.last_yaw_deltas_rad.has_value());
  EXPECT_TRUE(std::isnan((*with_gyro.status.last_yaw_deltas_rad)[2]));  // the LiDAR match was rejected
}

TEST(LocalizerGyro, SlipIsCountedFromTheRawWheelYaw) {
  const GyroRunResult with_gyro = run_with_gyro(GyroRunOptions{});
  EXPECT_GT(with_gyro.status.slip_detected_total, 0U);
  EXPECT_GT(with_gyro.status.slip_max_rad, 0.05);
  GyroRunOptions exact_wheel;
  exact_wheel.wheel_yaw_scale = 1.0;
  const GyroRunResult without_slip = run_with_gyro(exact_wheel);
  EXPECT_EQ(without_slip.status.slip_detected_total, 0U);
  EXPECT_LT(without_slip.status.slip_max_rad, 0.05);
}

TEST(LocalizerGyro, RotationFactorsNeedNinetyFivePercentCoverage) {
  GyroRunOptions every_twentieth_missing;  // 5 % missing: exactly on the edge is allowed, 4 % is
  every_twentieth_missing.missing_gyro_every = 25;
  const GyroRunResult nearly_covered = run_with_gyro(every_twentieth_missing);
  EXPECT_GT(nearly_covered.status.rotation_factors_added_total, 0U);
  EXPECT_EQ(nearly_covered.status.rotation_factors_skipped_total, 0U);

  GyroRunOptions every_fifth_missing;
  every_fifth_missing.missing_gyro_every = 5;
  const GyroRunResult poorly_covered = run_with_gyro(every_fifth_missing);
  EXPECT_EQ(poorly_covered.status.rotation_factors_added_total, 0U);
  EXPECT_GT(poorly_covered.status.rotation_factors_skipped_total, 0U);
  EXPECT_GT(poorly_covered.status.yaw_increments_wheel_total, 0U);
}

TEST(LocalizerGyro, PredictionCanBeSwitchedOffWhileRotationFactorsStay) {
  GyroRunOptions factors_only;
  factors_only.is_prediction_enabled = false;
  const GyroRunResult result = run_with_gyro(factors_only);
  EXPECT_GT(result.status.rotation_factors_added_total, 0U);
  EXPECT_EQ(result.status.yaw_increments_gyro_total, 0U);
  EXPECT_LT(result.final_yaw_error_deg, 10.0);  // the factors alone pull the 40 % error down a lot
}

// ---- The consistency judge (Issue #92, section 3.3 step 3). ROS and GTSAM free. ----
namespace {

GyroIntervalObservation covered_interval(double wheel_rad, double gyro_rad,
                                         std::optional<double> lidar_rad = std::nullopt, double duration_sec = 1.0) {
  GyroIntervalObservation interval;
  interval.wheel_yaw_rad = wheel_rad;
  interval.gyro_yaw_rad = gyro_rad;
  interval.lidar_yaw_rad = lidar_rad;
  interval.duration_sec = duration_sec;
  interval.is_gyro_covered = true;
  return interval;
}

// Feeds `count` identical intervals and returns the last result.
GyroConsistencyResult feed(GyroConsistencyJudge& judge, int count, const GyroIntervalObservation& interval) {
  GyroConsistencyResult result;
  for (int repetition = 0; repetition < count; ++repetition) {
    result = judge.observe_interval(interval);
    if (result.event != GyroConsistencyEvent::kNone) break;
  }
  return result;
}

// A judge that has been confirmed once (by a turn that LiDAR measured).
GyroConsistencyJudge active_judge() {
  return GyroConsistencyJudge(GyroConsistencyState::kActive);
}

}  // namespace

TEST(GyroConsistencyJudge, SlowTurnIsConfirmedByAccumulatingTheRotation) {
  GyroConsistencyJudge judge;
  // 0.05 rad per interval never reaches 0.2 rad alone; the sum does at the fifth interval.
  const GyroConsistencyResult result = feed(judge, 10, covered_interval(0.05, 0.05));
  EXPECT_EQ(result.event, GyroConsistencyEvent::kConfirmed);
  EXPECT_EQ(judge.state(), GyroConsistencyState::kActive);
  EXPECT_NEAR(result.verified_gyro_yaw_rad, 0.25, 1e-12);
  EXPECT_NEAR(result.verified_duration_sec, 5.0, 1e-12);
  EXPECT_EQ(judge.confirmed_total(), 1U);
  EXPECT_FALSE(judge.is_accumulating());
}

TEST(GyroConsistencyJudge, NothingIsDecidedBelowTheRotationThreshold) {
  GyroConsistencyJudge judge;
  EXPECT_EQ(feed(judge, 4, covered_interval(0.05, 0.05)).event, GyroConsistencyEvent::kNone);
  EXPECT_EQ(judge.state(), GyroConsistencyState::kUnverified);
  EXPECT_NEAR(judge.sums().wheel_rad, 0.2, 1e-12);
  EXPECT_NEAR(judge.sums().gyro_rad, 0.2, 1e-12);
}

TEST(GyroConsistencyJudge, WithoutLidarTheGyroMustRoughlyMatchTheWheel) {
  const auto verdict_for = [](double wheel_rad, double gyro_rad) {
    GyroConsistencyJudge judge;
    return judge.observe_interval(covered_interval(wheel_rad, gyro_rad)).event;
  };
  EXPECT_EQ(verdict_for(0.3, 0.3), GyroConsistencyEvent::kConfirmed);
  EXPECT_EQ(verdict_for(0.3, 0.3 * 1.15), GyroConsistencyEvent::kConfirmed);
  EXPECT_EQ(verdict_for(0.3, 0.3 * 0.65), GyroConsistencyEvent::kConfirmed);
  EXPECT_EQ(verdict_for(0.3, -0.3), GyroConsistencyEvent::kRejected);        // sign reversed
  EXPECT_EQ(verdict_for(0.3, 0.001), GyroConsistencyEvent::kRejected);       // another axis
  EXPECT_EQ(verdict_for(0.3, 0.3 * 57.3), GyroConsistencyEvent::kRejected);  // degrees instead of radians
  EXPECT_EQ(verdict_for(0.3, 0.3 * 2.0), GyroConsistencyEvent::kRejected);
  EXPECT_EQ(verdict_for(0.3, 0.3 * 1.3), GyroConsistencyEvent::kRejected);   // above 1.2
  EXPECT_EQ(verdict_for(0.3, 0.3 * 0.5), GyroConsistencyEvent::kRejected);   // below 0.6
  // The Issue's cases (w, g, no LiDAR).
  EXPECT_EQ(verdict_for(0.4, 0.2), GyroConsistencyEvent::kRejected);
  EXPECT_EQ(verdict_for(0.4, 0.52), GyroConsistencyEvent::kRejected);
  EXPECT_EQ(verdict_for(0.7, 1.0), GyroConsistencyEvent::kRejected);  // wheel 1.4x, gyro 2x: no longer passes
  EXPECT_EQ(verdict_for(0.7, 0.75), GyroConsistencyEvent::kConfirmed);
}

TEST(GyroConsistencyJudge, WithLidarTheGyroMustAgreeWithLidarEvenWhenTheWheelAgreesWithTheGyro) {
  // The wheel over-reports by 2x and so does a gyro of the wrong scale; LiDAR says 0.3.
  GyroConsistencyJudge wrong_scale;
  EXPECT_EQ(wrong_scale.observe_interval(covered_interval(0.6, 0.6, 0.3)).event, GyroConsistencyEvent::kRejected);
  EXPECT_EQ(wrong_scale.state(), GyroConsistencyState::kUnverified);
  GyroConsistencyJudge correct_gyro;
  EXPECT_EQ(correct_gyro.observe_interval(covered_interval(0.6, 0.3, 0.3)).event, GyroConsistencyEvent::kConfirmed);
}

TEST(GyroConsistencyJudge, AgreementIsTheLargerOfFiftyMilliradAndTwentyPercent) {
  // |L| = 0.5: tolerance 0.1.  |L| = 0.2 ... the floor 0.05 applies below |b| = 0.25.
  const auto verdict_for = [](double lidar_rad, double gyro_rad) {
    GyroConsistencyJudge judge;
    return judge.observe_interval(covered_interval(lidar_rad, gyro_rad, lidar_rad)).event;
  };
  EXPECT_EQ(verdict_for(0.5, 0.59), GyroConsistencyEvent::kConfirmed);
  EXPECT_EQ(verdict_for(0.5, 0.41), GyroConsistencyEvent::kConfirmed);
  EXPECT_EQ(verdict_for(0.5, 0.61), GyroConsistencyEvent::kRejected);
  EXPECT_EQ(verdict_for(0.5, 0.39), GyroConsistencyEvent::kRejected);
  EXPECT_EQ(verdict_for(0.21, 0.259), GyroConsistencyEvent::kConfirmed);  // floor: 0.05
  EXPECT_EQ(verdict_for(0.21, 0.27), GyroConsistencyEvent::kRejected);
}

TEST(GyroConsistencyJudge, AccumulationStartsOverAfter20SecondsAndOnUncoveredIntervals) {
  GyroConsistencyJudge judge;
  // 8 s intervals turning 0.05 rad: after 24 s the sums are dropped before the threshold is reached.
  EXPECT_EQ(feed(judge, 3, covered_interval(0.05, 0.05, std::nullopt, 8.0)).event, GyroConsistencyEvent::kNone);
  EXPECT_FALSE(judge.is_accumulating());
  EXPECT_EQ(judge.sums().wheel_rad, 0.0);
  EXPECT_EQ(judge.state(), GyroConsistencyState::kUnverified);

  feed(judge, 2, covered_interval(0.05, 0.05));
  EXPECT_TRUE(judge.is_accumulating());
  GyroIntervalObservation uncovered = covered_interval(0.05, 0.05);
  uncovered.is_gyro_covered = false;
  EXPECT_EQ(judge.observe_interval(uncovered).event, GyroConsistencyEvent::kNone);
  EXPECT_FALSE(judge.is_accumulating());
}

TEST(GyroConsistencyJudge, TurnsWithoutLidarInSomeIntervalsStillNeedTheLidarOnesToAgree) {
  GyroConsistencyJudge judge;
  // LiDAR saw 0.1 rad where the gyro said 0.3; the wheel-only interval alone would pass.
  judge.observe_interval(covered_interval(0.1, 0.3, 0.1));
  EXPECT_EQ(judge.state(), GyroConsistencyState::kUnverified);
  EXPECT_EQ(judge.rejected_total(), 0U);  // the sums are still below 0.2 rad
  const GyroConsistencyResult result = judge.observe_interval(covered_interval(0.2, 0.2));
  EXPECT_EQ(result.event, GyroConsistencyEvent::kRejected);  // G_l = 0.3 does not agree with L = 0.1
}

TEST(GyroConsistencyJudge, ThreeRejectionsRaiseTheWarningAndALaterConfirmationClearsIt) {
  GyroConsistencyJudge judge;
  for (int rejection = 0; rejection < 3; ++rejection) {
    EXPECT_FALSE(judge.is_inconsistent_warning());
    EXPECT_EQ(judge.observe_interval(covered_interval(0.3, -0.3)).event, GyroConsistencyEvent::kRejected);
  }
  EXPECT_TRUE(judge.is_inconsistent_warning());
  EXPECT_EQ(judge.state(), GyroConsistencyState::kUnverified);
  EXPECT_EQ(judge.rejected_total(), 3U);
  EXPECT_EQ(judge.observe_interval(covered_interval(0.3, 0.3)).event, GyroConsistencyEvent::kConfirmed);
  EXPECT_FALSE(judge.is_inconsistent_warning());
  EXPECT_EQ(judge.state(), GyroConsistencyState::kActive);
}

TEST(GyroConsistencyJudge, ActiveGyroIsOnlyStoppedWhenWheelAndLidarAgreeAgainstIt) {
  GyroConsistencyJudge judge = active_judge();
  // The wheel slipped (disagrees with LiDAR) and the gyro agrees with LiDAR: nothing happens.
  EXPECT_EQ(judge.observe_interval(covered_interval(0.6, 0.3, 0.3)).event, GyroConsistencyEvent::kNone);
  // Both the wheel and the gyro disagree with LiDAR: no majority.
  EXPECT_EQ(judge.observe_interval(covered_interval(0.6, 0.9, 0.3)).event, GyroConsistencyEvent::kNone);
  // Wheel without a LiDAR value: a wheel alone never stops the gyro (the wheel may spin in place).
  EXPECT_EQ(judge.observe_interval(covered_interval(0.0, 0.5)).event, GyroConsistencyEvent::kNone);
  EXPECT_EQ(judge.state(), GyroConsistencyState::kActive);
  // An interval the gyro did not cover cannot outvote it.
  GyroIntervalObservation uncovered = covered_interval(0.3, 0.9, 0.3);
  uncovered.is_gyro_covered = false;
  EXPECT_EQ(judge.observe_interval(uncovered).event, GyroConsistencyEvent::kNone);
  // Wheel and LiDAR agree, the gyro does not.
  EXPECT_EQ(judge.observe_interval(covered_interval(0.3, 0.6, 0.3)).event, GyroConsistencyEvent::kOutvoted);
  EXPECT_EQ(judge.state(), GyroConsistencyState::kSuspended);
  EXPECT_EQ(judge.outvoted_total(), 1U);
}

TEST(GyroConsistencyJudge, SuspendedGyroComesBackOnlyThroughLidar) {
  GyroConsistencyJudge judge = active_judge();
  ASSERT_EQ(judge.observe_interval(covered_interval(0.3, 0.6, 0.3)).event, GyroConsistencyEvent::kOutvoted);
  // Wheel and gyro agree with each other without LiDAR: not enough after a suspension.
  EXPECT_EQ(feed(judge, 5, covered_interval(0.3, 0.3)).event, GyroConsistencyEvent::kNone);
  EXPECT_EQ(judge.state(), GyroConsistencyState::kSuspended);
  EXPECT_EQ(judge.observe_interval(covered_interval(0.3, 0.3, 0.3)).event, GyroConsistencyEvent::kConfirmed);
  EXPECT_EQ(judge.state(), GyroConsistencyState::kActive);
}

TEST(GyroConsistencyJudge, ThirdOutvoteDisablesTheGyroForGood) {
  GyroConsistencyJudge judge = active_judge();
  for (int round = 0; round < 2; ++round) {
    ASSERT_EQ(judge.observe_interval(covered_interval(0.3, 0.6, 0.3)).event, GyroConsistencyEvent::kOutvoted);
    ASSERT_EQ(judge.state(), GyroConsistencyState::kSuspended);
    ASSERT_EQ(judge.observe_interval(covered_interval(0.3, 0.3, 0.3)).event, GyroConsistencyEvent::kConfirmed);
  }
  EXPECT_EQ(judge.observe_interval(covered_interval(0.3, 0.6, 0.3)).event, GyroConsistencyEvent::kOutvoted);
  EXPECT_EQ(judge.state(), GyroConsistencyState::kDisabledInconsistent);
  EXPECT_EQ(judge.outvoted_total(), 3U);
  EXPECT_EQ(judge.observe_interval(covered_interval(0.3, 0.3, 0.3)).event, GyroConsistencyEvent::kNone);
  EXPECT_EQ(judge.state(), GyroConsistencyState::kDisabledInconsistent);
}

TEST(GyroConsistencyJudge, ASuspendedGyroIsStillVotedOnAndThreeLossesDisableIt) {
  GyroConsistencyJudge judge = active_judge();
  ASSERT_EQ(judge.observe_interval(covered_interval(0.3, 0.6, 0.3)).event, GyroConsistencyEvent::kOutvoted);
  EXPECT_EQ(judge.observe_interval(covered_interval(0.3, 0.6, 0.3)).event, GyroConsistencyEvent::kOutvoted);
  EXPECT_EQ(judge.state(), GyroConsistencyState::kSuspended);
  EXPECT_EQ(judge.observe_interval(covered_interval(0.3, 0.6, 0.3)).event, GyroConsistencyEvent::kOutvoted);
  EXPECT_EQ(judge.state(), GyroConsistencyState::kDisabledInconsistent);
  EXPECT_EQ(judge.outvoted_total(), 3U);
}

TEST(GyroConsistencyJudge, LostVotesAreCountedInTotalAndNeverStartOver) {
  GyroConsistencyJudge judge = active_judge();
  ASSERT_EQ(judge.observe_interval(covered_interval(0.3, 0.6, 0.3)).event, GyroConsistencyEvent::kOutvoted);
  ASSERT_EQ(judge.observe_interval(covered_interval(0.3, 0.6, 0.3)).event, GyroConsistencyEvent::kOutvoted);
  ASSERT_EQ(judge.observe_interval(covered_interval(0.3, 0.3, 0.3)).event, GyroConsistencyEvent::kConfirmed);
  EXPECT_EQ(judge.outvoted_total(), 2U);  // coming back does not reset the count
  EXPECT_EQ(judge.observe_interval(covered_interval(0.3, 0.6, 0.3)).event, GyroConsistencyEvent::kOutvoted);
  EXPECT_EQ(judge.state(), GyroConsistencyState::kDisabledInconsistent);
}

TEST(GyroConsistencyJudge, AStartWithTwoLostVotesIsDisabledByTheNextOne) {
  GyroConsistencyJudge judge(GyroConsistencyState::kActive, 2);
  EXPECT_EQ(judge.outvoted_total(), 2U);
  EXPECT_EQ(judge.observe_interval(covered_interval(0.3, 0.6, 0.3)).event, GyroConsistencyEvent::kOutvoted);
  EXPECT_EQ(judge.state(), GyroConsistencyState::kDisabledInconsistent);
  EXPECT_EQ(GyroConsistencyJudge(GyroConsistencyState::kActive, 3).state(), GyroConsistencyState::kDisabledInconsistent);
}

TEST(GyroConsistencyJudge, AJudgeStartedSuspendedComesBackOnlyThroughLidar) {
  GyroConsistencyJudge judge(GyroConsistencyState::kSuspended);
  EXPECT_EQ(judge.state(), GyroConsistencyState::kSuspended);
  EXPECT_EQ(judge.observe_interval(covered_interval(0.7, 0.5)).event, GyroConsistencyEvent::kNone);
  EXPECT_EQ(judge.state(), GyroConsistencyState::kSuspended);
  EXPECT_EQ(judge.observe_interval(covered_interval(0.4, 0.4, 0.4)).event, GyroConsistencyEvent::kConfirmed);
  EXPECT_EQ(judge.state(), GyroConsistencyState::kActive);
}

TEST(GyroConsistencyJudge, SumsOfTheVerificationAreReportedPerKind) {
  GyroConsistencyJudge judge;
  judge.observe_interval(covered_interval(0.05, 0.04, 0.045));
  judge.observe_interval(covered_interval(0.02, 0.03));
  EXPECT_NEAR(judge.sums().wheel_rad, 0.07, 1e-12);
  EXPECT_NEAR(judge.sums().gyro_rad, 0.07, 1e-12);
  EXPECT_NEAR(judge.sums().lidar_rad, 0.045, 1e-12);
  EXPECT_NEAR(judge.sums().gyro_on_lidar_rad, 0.04, 1e-12);
  EXPECT_TRUE(judge.sums().has_interval_without_lidar);
}

// ---- The judge inside the Localizer ----
TEST(LocalizerGyro, AccurateGyroIsConfirmedDuringTheFirstTurnAndCorrectsTheWholeTurn) {
  GyroRunOptions options;
  options.initial_state = GyroConsistencyState::kUnverified;
  const GyroRunResult result = run_with_gyro(options);  // the wheel over-reports the turn by 40 %
  EXPECT_EQ(result.status.consistency, GyroConsistencyState::kActive);
  EXPECT_GE(result.status.consistency_confirmed_total, 1U);
  EXPECT_FALSE(result.status.is_inconsistent_warning);
  EXPECT_GT(result.status.rotation_factors_added_total, 0U);
  EXPECT_FALSE(result.status.verification_sums_rad.has_value());  // only while verifying
  // Confirmed on the first keyframe past 0.2 rad of the turn; the span factor repairs that part.
  EXPECT_LT(result.final_yaw_error_deg, 3.0);
}

TEST(LocalizerGyro, GyroWithAReversedSignIsNeverUsed) {
  GyroRunOptions reversed;
  reversed.initial_state = GyroConsistencyState::kUnverified;
  reversed.wheel_yaw_scale = 1.0;
  reversed.gyro_yaw_scale = -1.0;
  const GyroRunResult result = run_with_gyro(reversed);
  EXPECT_EQ(result.status.consistency, GyroConsistencyState::kUnverified);
  EXPECT_GE(result.status.consistency_rejected_total, 1U);
  EXPECT_EQ(result.status.consistency_confirmed_total, 0U);
  EXPECT_EQ(result.status.rotation_factors_added_total, 0U);
  EXPECT_EQ(result.status.yaw_increments_gyro_total, 0U);
  EXPECT_LT(result.final_yaw_error_deg, 5.0);  // the wheel alone (accurate here) still holds the heading
  ASSERT_TRUE(result.status.verification_sums_rad.has_value());
}

TEST(LocalizerGyro, NoGyroIncrementsLeaveTheJudgeUntouched) {
  GyroRunOptions no_gyro;
  no_gyro.initial_state = GyroConsistencyState::kUnverified;
  no_gyro.missing_gyro_every = 1;
  const GyroRunResult result = run_with_gyro(no_gyro);
  EXPECT_EQ(result.status.consistency, GyroConsistencyState::kUnverified);
  EXPECT_EQ(result.status.consistency_rejected_total, 0U);
  EXPECT_EQ(result.status.consistency_confirmed_total, 0U);
}

// ---- Effects of the gyro on a drive (Issue #92, section 4.3). ----
namespace {

using TimeWindow = std::pair<double, double>;
// The gyro increment of one wheel sample from the standard one (truth + bias + noise); nullopt = none.
using GyroTransform = std::function<std::optional<double>(double time_sec, double standard_increment_rad)>;

struct GyroDriveOptions {
  SegmentedCourse course = make_long_drive_course();
  double wheel_yaw_scale = 1.0;
  double wheel_yaw_drift_rad_per_m = 0.5 * M_PI / 180.0 / 10.0;  // the Issue's wheel model
  std::vector<TimeWindow> wheel_slip_windows;       // the wheel reports no rotation
  TimeWindow wheel_spin_window{-1.0, -1.0};         // the wheel reports 0.5 rad/s, the truth does not turn
  GyroTransform gyro_transform;                     // empty = the standard increment
  bool has_gyro = true;                             // false: the two argument on_wheel_odom
  std::vector<TimeWindow> lidar_rejected_windows;
  GyroConsistencyState initial_state = GyroConsistencyState::kUnverified;
  bool is_prediction_enabled = true;
  double optimization_failure_sec = -1.0;
  double wheel_time_backwards_sec = -1.0;
  double initialize_sec = -1.0;
  std::optional<std::size_t> initialize_when_outvoted_total;  // initialize() once the lost votes reach this
  bool has_prior_map = false;  // false: mapping without a map or GNSS (never reaches TRACKING)
  bool should_save_map = false;  // mapping only: save_map() after the drive and read each stage back
};

struct DriveSample {
  double time_sec;
  double yaw_error_deg;
  LocalizationState state;
  Eigen::Isometry3d map_to_base;
};

struct GyroDriveResult {
  std::vector<DriveSample> samples;
  GyroFusionStatus status;
  std::optional<double> first_active_sec;
  bool saw_suspended = false;
  // Yaw error (deg) of every keyframe in trajectory.csv, by save stage ("preliminary", "optimized", "final").
  std::map<std::string, std::vector<double>> saved_yaw_errors_deg;
  std::optional<GyroFusionStatus> status_before_initialize;
  std::optional<GyroFusionStatus> status_after_initialize;
  std::optional<GyroFusionStatus> status_before_backwards;
  std::optional<GyroFusionStatus> status_after_backwards;
  std::size_t scans_processed = 0;
  std::size_t max_pending_scans = 0;
  bool is_tracking_at_end = false;
  double max_yaw_error_deg(double from_sec, double to_sec) const {
    double maximum = 0.0;
    for (const DriveSample& sample : samples) {
      if (sample.time_sec >= from_sec && sample.time_sec <= to_sec) maximum = std::max(maximum, sample.yaw_error_deg);
    }
    return maximum;
  }
  double max_yaw_error_deg() const { return max_yaw_error_deg(0.0, 1.0e9); }
};

// Reads the yaw errors of the keyframes of trajectory.csv (columns: stamp, x, y, z, roll, pitch, yaw, ...).
std::vector<double> read_trajectory_yaw_errors_deg(const std::filesystem::path& file,
                                                   const std::function<Eigen::Isometry3d(double)>& truth_at) {
  std::ifstream stream(file);
  std::string line;
  std::getline(stream, line);  // header
  std::vector<double> errors;
  while (std::getline(stream, line)) {
    if (line.empty()) continue;
    std::stringstream line_stream(line);
    std::string field;
    std::vector<double> values;
    while (std::getline(line_stream, field, ',')) values.push_back(std::stod(field));
    if (values.size() < 7) continue;
    const double truth_yaw = yaw_of_pose(truth_at(values[0]));
    const double difference = std::remainder(values[6] - truth_yaw, 2.0 * M_PI);
    errors.push_back(std::abs(difference) * 180.0 / M_PI);
  }
  return errors;
}

void save_and_read_yaw_errors(Localizer& localizer, const std::function<Eigen::Isometry3d(double)>& truth_at,
                              GyroDriveResult& result) {
  const std::filesystem::path directory =
      std::filesystem::path(::testing::TempDir()) / ("gyro_drive_save_" + std::to_string(std::rand()));
  std::filesystem::create_directories(directory);
  MapOutputParameters output_parameters;
  output_parameters.map_output_dir = directory.string();
  output_parameters.map_voxel_size_m = 0.2;
  output_parameters.batch_max_iterations = 100;
  output_parameters.created = "2026-01-01T00:00:00Z";
  MapOutputResult output_result;
  std::string error;
  const bool is_saved = localizer.save_map(output_parameters, output_result, error, [&](const std::string& line) {
    const std::string stage = line.substr(0, line.find_first_of(" :"));
    result.saved_yaw_errors_deg[stage] = read_trajectory_yaw_errors_deg(directory / "trajectory.csv", truth_at);
  });
  EXPECT_TRUE(is_saved) << error;
  std::error_code ignored;
  std::filesystem::remove_all(directory, ignored);
}

GyroDriveResult run_gyro_drive(const GyroDriveOptions& options) {
  const SegmentedCourse& course = options.course;
  const SyntheticWorld world(course.centerline(), true);
  LocalizerParameters parameters;
  parameters.mapping = !options.has_prior_map;
  parameters.initial_gyro_consistency = options.initial_state;
  parameters.gyro_yaw_prediction_enabled = options.is_prediction_enabled;
  Localizer localizer(parameters, options.has_prior_map ? world.sample_map(0.2) : PointList{}, std::nullopt);
  double current_time_sec = 0.0;
  const auto is_in_any = [](const std::vector<TimeWindow>& windows, double time_sec) {
    return std::any_of(windows.begin(), windows.end(),
                       [&](const TimeWindow& window) { return time_sec >= window.first && time_sec <= window.second; });
  };
  localizer.set_lidar_match_hook_for_test([&](MatchResult& result) {
    if (is_in_any(options.lidar_rejected_windows, current_time_sec)) result.is_accepted = false;
  });
  const auto truth_at = [&course](double time_sec) { return course.truth_at(time_sec); };
  localizer.initialize(truth_at(0.0));
  std::mt19937 random_generator(5);
  std::normal_distribution<double> noise(0.0, 1.0);
  constexpr double kBiasResidualRadPerSec = 2.0e-4;
  constexpr double kNoiseDensity = 1.0e-3;

  GyroDriveResult result;
  Eigen::Isometry3d odom = Eigen::Isometry3d::Identity();
  const int step_count = static_cast<int>(course.total_sec() / kDt);
  const int steps_per_scan = static_cast<int>(0.1 / kDt);
  bool has_failed_optimization = false;
  bool has_gone_backwards = false;
  bool has_initialized = false;
  for (int step = 0; step <= step_count; ++step) {
    const double time_sec = step * kDt;
    current_time_sec = time_sec;
    std::optional<double> gyro_increment;
    if (step > 0) {
      const Eigen::Isometry3d relative = truth_at(time_sec - kDt).inverse() * truth_at(time_sec);
      const double true_yaw = std::atan2(relative.rotation()(1, 0), relative.rotation()(0, 0));
      const double distance = relative.translation().norm();
      double wheel_yaw = true_yaw * options.wheel_yaw_scale + distance * options.wheel_yaw_drift_rad_per_m;
      if (is_in_any(options.wheel_slip_windows, time_sec)) wheel_yaw = 0.0;
      if (time_sec >= options.wheel_spin_window.first && time_sec <= options.wheel_spin_window.second) {
        wheel_yaw = 0.5 * kDt;
      }
      odom = odom * planar_step(distance, wheel_yaw);
      const double standard = true_yaw + kBiasResidualRadPerSec * kDt +
                              kNoiseDensity * std::sqrt(kDt) * noise(random_generator);
      gyro_increment = options.gyro_transform ? options.gyro_transform(time_sec, standard) : standard;
    }
    if (step > 0 && step % steps_per_scan == 0 && time_sec >= kScanDuration) {
      localizer.on_scan(time_sec, world.synthesize_scan(truth_at, time_sec - kScanDuration, kScanDuration, true, 360, 8));
    }
    if (options.optimization_failure_sec >= 0.0 && !has_failed_optimization && time_sec >= options.optimization_failure_sec) {
      localizer.inject_optimization_failure_for_test();
      has_failed_optimization = true;
    }
    if (options.initialize_sec >= 0.0 && !has_initialized && time_sec >= options.initialize_sec) {
      localizer.initialize(truth_at(time_sec));
      has_initialized = true;
    }
    if (options.wheel_time_backwards_sec >= 0.0 && !has_gone_backwards && time_sec >= options.wheel_time_backwards_sec) {
      result.status_before_backwards = localizer.gyro_fusion_status();
      localizer.on_wheel_odom(time_sec - 1.0, odom, std::nullopt);  // one sample from the past
      result.status_after_backwards = localizer.gyro_fusion_status();
      has_gone_backwards = true;
    }
    const auto output = options.has_gyro ? localizer.on_wheel_odom(time_sec, odom, gyro_increment)
                                         : localizer.on_wheel_odom(time_sec, odom);
    result.max_pending_scans = std::max(result.max_pending_scans, localizer.pending_scan_count());
    const GyroFusionStatus status = localizer.gyro_fusion_status();
    if (status.consistency == GyroConsistencyState::kActive && !result.first_active_sec) result.first_active_sec = time_sec;
    if (options.initialize_when_outvoted_total && !result.status_after_initialize &&
        status.consistency_outvoted_total == *options.initialize_when_outvoted_total &&
        status.consistency == GyroConsistencyState::kSuspended) {
      result.status_before_initialize = status;
      localizer.initialize(truth_at(time_sec));
      result.status_after_initialize = localizer.gyro_fusion_status();
    }
    result.saw_suspended |= status.consistency_outvoted_total > 0;
    if (output) {
      result.samples.push_back({time_sec, yaw_error_deg(output->map_to_base, truth_at(time_sec)), output->state,
                                output->map_to_base});
      result.is_tracking_at_end = output->state == LocalizationState::kTracking;
    }
  }
  result.status = localizer.gyro_fusion_status();
  result.scans_processed = localizer.take_scan_time_statistics().count;
  if (options.should_save_map) save_and_read_yaw_errors(localizer, truth_at, result);
  return result;
}

// 5 m, stop, in-place 90 degree turn at 0.5 rad/s, then a leg long enough (8 m = 13 s) for the
// "10 seconds after the turn" check; the Issue's 5 m would end 8 s after the turn.
SegmentedCourse make_t1_course() {
  return SegmentedCourse({5.0, 8.0}, {1.0}, 0.6, 0.5, 2.0);
}

GyroDriveOptions t1_options() {
  GyroDriveOptions options;
  options.course = make_t1_course();
  const TimeWindow turn = options.course.turn_windows().front();
  options.wheel_yaw_scale = 1.4;
  options.lidar_rejected_windows = {{turn.first, turn.second + 0.5}};
  return options;
}

TimeWindow slip_in_turn(const SegmentedCourse& course, std::size_t turn_index) {
  const double begin_sec = course.turn_windows()[turn_index].first + 0.5;
  return {begin_sec, begin_sec + 0.8};  // as in the existing slip scenario: 0.8 s without rotation
}

GyroDriveOptions long_drive_with_slip_in_turn(std::size_t turn_index) {
  GyroDriveOptions options;
  options.wheel_slip_windows = {slip_in_turn(options.course, turn_index)};
  return options;
}

GyroTransform scaled_gyro(double scale) {
  return [scale](double, double standard) { return std::optional<double>(standard * scale); };
}

}  // namespace

TEST(LocalizerGyroDrive, T1_SlippingInPlaceTurnIsReproducedWithoutTheGyro) {
  GyroDriveOptions options = t1_options();
  options.has_gyro = false;
  const GyroDriveResult result = run_gyro_drive(options);
  std::printf("[ MEASURED ] T1 no gyro final yaw error %.3f deg\n", result.samples.back().yaw_error_deg);
  EXPECT_GT(result.samples.back().yaw_error_deg, 15.0);
}

TEST(LocalizerGyroDrive, T1_ActiveWithinOneAndAHalfSecondsAndTheYawFollowsAfterTheTurn) {
  const GyroDriveOptions options = t1_options();
  const TimeWindow turn = options.course.turn_windows().front();
  const GyroDriveResult result = run_gyro_drive(options);
  ASSERT_TRUE(result.first_active_sec.has_value());
  std::printf("[ MEASURED ] T1 active %.2f s after the turn began; error after turn+3 s %.3f deg\n",
              *result.first_active_sec - turn.first, result.max_yaw_error_deg(turn.second + 3.0, 1.0e9));
  EXPECT_LE(*result.first_active_sec, turn.first + 1.5);
  EXPECT_LT(result.max_yaw_error_deg(turn.second + 3.0, 1.0e9), 2.0);
  EXPECT_EQ(result.status.consistency_outvoted_total, 0U);  // T10
}

TEST(LocalizerGyroDrive, T1_WithoutPredictionTheRotationFactorsStillPullTheYawBack) {
  GyroDriveOptions options = t1_options();
  options.is_prediction_enabled = false;
  const TimeWindow turn = options.course.turn_windows().front();
  const GyroDriveResult result = run_gyro_drive(options);
  ASSERT_GT(result.samples.back().time_sec, turn.second + 10.0);
  std::printf("[ MEASURED ] T1 no prediction error after turn+10 s %.3f deg\n",
              result.max_yaw_error_deg(turn.second + 10.0, 1.0e9));
  EXPECT_LT(result.max_yaw_error_deg(turn.second + 10.0, 1.0e9), 2.0);
}

TEST(LocalizerGyroDrive, T1b_ASlipAfterAnEarlierCleanTurnIsCorrectedFromItsStart) {
  GyroDriveOptions options;  // long drive: turn 0 clean (becomes active), turn 1 slips
  options.wheel_slip_windows = {slip_in_turn(options.course, 1)};
  const GyroDriveResult result = run_gyro_drive(options);
  const double slip_begin = options.wheel_slip_windows.front().first;
  EXPECT_LT(result.first_active_sec.value_or(1.0e9), slip_begin);
  std::printf("[ MEASURED ] T1b/T2b max yaw error from slip start %.3f deg\n", result.max_yaw_error_deg(slip_begin, 1.0e9));
  EXPECT_LT(result.max_yaw_error_deg(slip_begin, 1.0e9), 2.0);
  // T6: the slip is counted although the gyro replaced the yaw (the raw wheel is used for w).
  EXPECT_GE(result.status.slip_detected_total, 1U);
  EXPECT_GE(result.status.slip_max_rad, 0.1);
  // T10: a correct gyro is never outvoted.
  EXPECT_EQ(result.status.consistency_outvoted_total, 0U);
  EXPECT_EQ(result.status.consistency, GyroConsistencyState::kActive);
}

TEST(LocalizerGyroDrive, T2b_WithAPriorMapTheDriveStaysTrackingAndTheYawWithin2Degrees) {
  GyroDriveOptions options = long_drive_with_slip_in_turn(1);
  options.has_prior_map = true;
  const GyroDriveResult result = run_gyro_drive(options);
  const double slip_begin = options.wheel_slip_windows.front().first;
  std::printf("[ MEASURED ] T2b (prior map) max yaw error from slip start %.3f deg\n", result.max_yaw_error_deg(slip_begin, 1.0e9));
  EXPECT_LT(result.max_yaw_error_deg(slip_begin, 1.0e9), 2.0);
  EXPECT_TRUE(result.is_tracking_at_end) << "final state " << static_cast<int>(result.samples.back().state);
  EXPECT_EQ(result.status.consistency_outvoted_total, 0U);
}

TEST(LocalizerGyroDrive, T6_NoSlipMeansNoCountOnADriveWithAnAccurateWheel) {
  GyroDriveOptions options;
  options.wheel_yaw_drift_rad_per_m = 0.0;
  const GyroDriveResult result = run_gyro_drive(options);
  EXPECT_EQ(result.status.slip_detected_total, 0U);
}

TEST(LocalizerGyroDrive, T4b_ScatteredMissingIncrementsAreToleratedUpToFivePercent) {
  GyroDriveOptions two_percent = long_drive_with_slip_in_turn(1);
  int counter = 0;
  two_percent.gyro_transform = [&counter](double, double standard) {
    return ++counter % 50 == 0 ? std::nullopt : std::optional<double>(standard);
  };
  const GyroDriveResult tolerated = run_gyro_drive(two_percent);
  EXPECT_EQ(tolerated.status.rotation_factors_skipped_total, 0U);
  EXPECT_LT(tolerated.max_yaw_error_deg(slip_in_turn(two_percent.course, 1).first, 1.0e9), 2.0);

  GyroDriveOptions ten_percent = long_drive_with_slip_in_turn(1);
  int other_counter = 0;
  ten_percent.initial_state = GyroConsistencyState::kActive;  // 10 % misses would never even verify
  ten_percent.gyro_transform = [&other_counter](double, double standard) {
    return ++other_counter % 10 == 0 ? std::nullopt : std::optional<double>(standard);
  };
  const GyroDriveResult skipped = run_gyro_drive(ten_percent);
  EXPECT_EQ(skipped.status.rotation_factors_added_total, 0U);
  EXPECT_GT(skipped.status.rotation_factors_skipped_total, 0U);
}

TEST(LocalizerGyroDrive, T4_AnOutageOfFiveSecondsDoesNotStopTheOutputOrHoldBackScans) {
  GyroDriveOptions reference_options = long_drive_with_slip_in_turn(1);
  const GyroDriveResult reference = run_gyro_drive(reference_options);
  GyroDriveOptions options = long_drive_with_slip_in_turn(1);
  const double outage_begin = slip_in_turn(options.course, 1).second + 8.0;
  const double outage_end = outage_begin + 5.0;
  options.gyro_transform = [=](double time_sec, double standard) {
    return time_sec >= outage_begin && time_sec < outage_end ? std::nullopt : std::optional<double>(standard);
  };
  const GyroDriveResult result = run_gyro_drive(options);
  EXPECT_EQ(result.scans_processed, reference.scans_processed);
  EXPECT_LT(result.max_pending_scans, 20U);
  EXPECT_GT(result.status.rotation_factors_skipped_total, 0U);
  EXPECT_GT(result.status.rotation_factors_added_total, reference.status.rotation_factors_added_total - 8U);
  double largest_jump_deg = 0.0;
  for (std::size_t index = 1; index < result.samples.size(); ++index) {
    const Eigen::Matrix3d difference = result.samples[index - 1].map_to_base.rotation().transpose() *
                                       result.samples[index].map_to_base.rotation();
    largest_jump_deg = std::max(largest_jump_deg, std::abs(std::atan2(difference(1, 0), difference(0, 0))) * 180.0 / M_PI);
  }
  // Turning at 0.4 rad/s moves 0.46 deg per 20 ms by itself; a jump is anything beyond that.
  EXPECT_LT(largest_jump_deg, 0.5 + 0.46);
}

TEST(LocalizerGyroDrive, T5_AGyroIncrementDelayedByOneSampleStillHoldsTheYaw) {
  GyroDriveOptions options = long_drive_with_slip_in_turn(1);
  double previous = 0.0;
  options.gyro_transform = [&previous](double, double standard) {
    const double delayed = previous;
    previous = standard;
    return std::optional<double>(delayed);
  };
  const GyroDriveResult result = run_gyro_drive(options);
  const TimeWindow slip = slip_in_turn(options.course, 1);
  EXPECT_LT(result.max_yaw_error_deg(options.course.turn_windows()[1].second + 1.0, 1.0e9), 2.0);
  (void)slip;
}

TEST(LocalizerGyroDrive, T7_ASignReversedGyroNeverActivatesAndTheOutputEqualsTheWheelOnlyRun) {
  GyroDriveOptions wheel_only = long_drive_with_slip_in_turn(0);
  wheel_only.has_gyro = false;
  const GyroDriveResult reference = run_gyro_drive(wheel_only);
  GyroDriveOptions options = long_drive_with_slip_in_turn(0);
  options.gyro_transform = scaled_gyro(-1.0);
  const GyroDriveResult result = run_gyro_drive(options);
  EXPECT_FALSE(result.first_active_sec.has_value());
  EXPECT_EQ(result.status.rotation_factors_added_total, 0U);
  EXPECT_GE(result.status.consistency_rejected_total, 3U);
  EXPECT_TRUE(result.status.is_inconsistent_warning);
  ASSERT_EQ(result.samples.size(), reference.samples.size());
  for (std::size_t index = 0; index < result.samples.size(); ++index) {
    EXPECT_EQ(result.samples[index].map_to_base.matrix(), reference.samples[index].map_to_base.matrix()) << index;
  }
}

TEST(LocalizerGyroDrive, T8_ADoubleScaleOrNoiseOnlyGyroNeverActivatesAndTheOutputEqualsTheWheelOnlyRun) {
  GyroDriveOptions wheel_only = long_drive_with_slip_in_turn(0);
  wheel_only.has_gyro = false;
  const GyroDriveResult reference = run_gyro_drive(wheel_only);
  const std::vector<GyroTransform> wrong_gyros = {
      scaled_gyro(2.0),
      [](double, double standard) { return std::optional<double>(standard * 0.0 + 1.0e-4); }};  // another axis
  for (const GyroTransform& transform : wrong_gyros) {
    GyroDriveOptions options = long_drive_with_slip_in_turn(0);
    options.gyro_transform = transform;
    const GyroDriveResult result = run_gyro_drive(options);
    EXPECT_FALSE(result.first_active_sec.has_value());
    EXPECT_EQ(result.status.rotation_factors_added_total, 0U);
    ASSERT_EQ(result.samples.size(), reference.samples.size());
    for (std::size_t index = 0; index < result.samples.size(); ++index) {
      EXPECT_EQ(result.samples[index].map_to_base.matrix(), reference.samples[index].map_to_base.matrix()) << index;
    }
  }
}

TEST(LocalizerGyroDrive, T8b_ADoubleScaleGyroBesideAWheelThatOverReportsNeverActivates) {
  GyroDriveOptions wheel_only = t1_options();  // wheel 1.4x, no LiDAR during the turn
  wheel_only.has_gyro = false;
  const GyroDriveResult reference = run_gyro_drive(wheel_only);
  GyroDriveOptions options = t1_options();
  options.gyro_transform = scaled_gyro(2.0);  // the ratio to the wheel is about 1.43: above 1.2
  const GyroDriveResult result = run_gyro_drive(options);
  EXPECT_FALSE(result.first_active_sec.has_value());
  EXPECT_EQ(result.status.rotation_factors_added_total, 0U);
  ASSERT_EQ(result.samples.size(), reference.samples.size());
  for (std::size_t index = 0; index < result.samples.size(); ++index) {
    EXPECT_EQ(result.samples[index].map_to_base.matrix(), reference.samples[index].map_to_base.matrix()) << index;
  }
}

TEST(LocalizerGyroDrive, T8c_TheLimitThatRemainsIsMeasuredNotJudged) {
  GyroDriveOptions options = t1_options();  // wheel 1.4x: a 1.5x gyro has a ratio of 1.07, inside the range
  options.gyro_transform = scaled_gyro(1.5);
  const GyroDriveResult result = run_gyro_drive(options);
  ASSERT_FALSE(result.samples.empty());
  std::printf("[ MEASURED ] T8c active=%d max_yaw_error_deg=%.3f final_yaw_error_deg=%.3f outvoted_total=%zu final_state=%d\n",
              result.first_active_sec.has_value(), result.max_yaw_error_deg(), result.samples.back().yaw_error_deg,
              result.status.consistency_outvoted_total, static_cast<int>(result.status.consistency));
  EXPECT_TRUE(result.first_active_sec.has_value());
}

TEST(LocalizerGyroDrive, T9_AGyroThatGoesWrongLaterIsSuspendedAndEventuallyDisabled) {
  GyroDriveOptions wheel_only;
  wheel_only.has_gyro = false;
  const GyroDriveResult reference = run_gyro_drive(wheel_only);
  GyroDriveOptions options;
  const double second_turn_begin = options.course.turn_windows()[1].first;
  options.gyro_transform = [=](double time_sec, double standard) {
    return std::optional<double>(time_sec >= second_turn_begin - 2.0 ? 2.0 * standard : standard);
  };
  const GyroDriveResult result = run_gyro_drive(options);
  EXPECT_TRUE(result.first_active_sec.has_value());
  EXPECT_TRUE(result.saw_suspended);
  EXPECT_GE(result.status.consistency_outvoted_total, 1U);
  const double suspended_after_sec = second_turn_begin;
  EXPECT_LE(result.max_yaw_error_deg(suspended_after_sec + 5.0, 1.0e9),
            reference.max_yaw_error_deg(suspended_after_sec + 5.0, 1.0e9) + 1.0);
  std::printf("[ MEASURED ] T9 outvoted_total=%zu final_state=%d\n", result.status.consistency_outvoted_total,
              static_cast<int>(result.status.consistency));
}

TEST(LocalizerGyroDrive, T9_InitializeInTheMiddleKeepsTheSuspensionAndTheLostVotes) {
  GyroDriveOptions options;
  const double second_turn_begin = options.course.turn_windows()[1].first;
  options.gyro_transform = [=](double time_sec, double standard) {
    return std::optional<double>(time_sec >= second_turn_begin - 2.0 ? 2.0 * standard : standard);
  };
  options.initialize_when_outvoted_total = 2;
  const GyroDriveResult result = run_gyro_drive(options);
  ASSERT_TRUE(result.status_after_initialize.has_value());
  EXPECT_EQ(result.status_after_initialize->consistency, GyroConsistencyState::kSuspended);
  EXPECT_EQ(result.status_after_initialize->consistency_outvoted_total, 2U);
  // The next lost vote disables the gyro for good.
  EXPECT_EQ(result.status.consistency, GyroConsistencyState::kDisabledInconsistent);
  EXPECT_GE(result.status.consistency_outvoted_total, 3U);
}

TEST(LocalizerGyroDrive, T10_AWheelSpinningInPlaceDoesNotStopACorrectGyro) {
  GyroDriveOptions options;
  const double spin_begin = options.course.turn_windows()[0].second + 3.0;
  options.wheel_spin_window = {spin_begin, spin_begin + 3.0};
  options.lidar_rejected_windows = {{spin_begin, spin_begin + 3.5}};
  const GyroDriveResult result = run_gyro_drive(options);
  EXPECT_EQ(result.status.consistency, GyroConsistencyState::kActive);
  EXPECT_EQ(result.status.consistency_outvoted_total, 0U);
  EXPECT_LT(result.max_yaw_error_deg(spin_begin, spin_begin + 3.0), 2.0);
}

TEST(LocalizerGyroDrive, DisabledInconsistentStartMatchesTheWheelOnlyRunExactly) {
  GyroDriveOptions wheel_only = long_drive_with_slip_in_turn(0);
  wheel_only.has_gyro = false;
  const GyroDriveResult reference = run_gyro_drive(wheel_only);
  GyroDriveOptions options = long_drive_with_slip_in_turn(0);
  options.initial_state = GyroConsistencyState::kDisabledInconsistent;
  const GyroDriveResult result = run_gyro_drive(options);
  EXPECT_EQ(result.status.rotation_factors_added_total, 0U);
  ASSERT_EQ(result.samples.size(), reference.samples.size());
  for (std::size_t index = 0; index < result.samples.size(); ++index) {
    EXPECT_EQ(result.samples[index].map_to_base.matrix(), reference.samples[index].map_to_base.matrix()) << index;
  }
}

TEST(LocalizerGyroDrive, ActiveStateSurvivesInitializeAndFactorsContinueWithoutVerifyingAgain) {
  GyroDriveOptions options = long_drive_with_slip_in_turn(1);
  options.initialize_sec = options.course.turn_windows()[0].second + 3.0;
  const GyroDriveResult result = run_gyro_drive(options);
  EXPECT_EQ(result.status.consistency, GyroConsistencyState::kActive);
  EXPECT_EQ(result.status.consistency_confirmed_total, 0U);  // no new confirmation after initialize()
  EXPECT_GT(result.status.rotation_factors_added_total, 0U);
}

TEST(LocalizerGyroDrive, WindowRebuildAndTimeGoingBackwardsDoNotBreakTheFusion) {
  GyroDriveOptions failure = long_drive_with_slip_in_turn(1);
  failure.optimization_failure_sec = failure.course.turn_windows()[1].first - 3.0;
  const GyroDriveResult rebuilt = run_gyro_drive(failure);
  EXPECT_GT(rebuilt.status.rotation_factors_added_total, 0U);
  EXPECT_LT(rebuilt.max_yaw_error_deg(), 3.0);

  GyroDriveOptions backwards = long_drive_with_slip_in_turn(1);
  backwards.wheel_time_backwards_sec = backwards.course.turn_windows()[1].first - 3.0;
  const GyroDriveResult went_back = run_gyro_drive(backwards);
  EXPECT_FALSE(went_back.samples.empty());
  EXPECT_GT(went_back.samples.back().time_sec, backwards.wheel_time_backwards_sec);
}

TEST(LocalizerGyroDrive, T1_SavedTrajectoryKeepsEveryKeyframeYawWithin2Degrees) {
  GyroDriveOptions options = t1_options();
  options.should_save_map = true;
  const GyroDriveResult with_gyro = run_gyro_drive(options);
  ASSERT_TRUE(with_gyro.saved_yaw_errors_deg.count("optimized"));
  ASSERT_TRUE(with_gyro.saved_yaw_errors_deg.count("final"));
  for (const char* stage : {"optimized", "final"}) {
    const std::vector<double>& errors = with_gyro.saved_yaw_errors_deg.at(stage);
    ASSERT_FALSE(errors.empty()) << stage;
    const double maximum = *std::max_element(errors.begin(), errors.end());
    std::printf("[ MEASURED ] T1 saved trajectory (%s) max yaw error %.3f deg\n", stage, maximum);
    EXPECT_LT(maximum, 2.0) << stage;
  }
  // Without the prediction the rotation factors alone must still do it.
  options.is_prediction_enabled = false;
  const GyroDriveResult factors_only = run_gyro_drive(options);
  for (const char* stage : {"optimized", "final"}) {
    const std::vector<double>& errors = factors_only.saved_yaw_errors_deg.at(stage);
    ASSERT_FALSE(errors.empty()) << stage;
    EXPECT_LT(*std::max_element(errors.begin(), errors.end()), 2.0) << stage;
  }
  // The wheel alone is far off, so the check above has something to prove.
  GyroDriveOptions wheel_only = t1_options();
  wheel_only.has_gyro = false;
  wheel_only.should_save_map = true;
  const GyroDriveResult reference = run_gyro_drive(wheel_only);
  EXPECT_GT(*std::max_element(reference.saved_yaw_errors_deg.at("final").begin(),
                              reference.saved_yaw_errors_deg.at("final").end()), 15.0);
}

TEST(LocalizerGyroDrive, T10b_AWheelSpinningInPlaceWhileLidarIsAcceptedKeepsTheGyroActive) {
  GyroDriveOptions options;
  const double spin_begin = options.course.turn_windows()[0].second + 3.0;
  options.wheel_spin_window = {spin_begin, spin_begin + 3.0};  // LiDAR and gyro both say "not turning"
  const GyroDriveResult result = run_gyro_drive(options);
  EXPECT_EQ(result.status.consistency, GyroConsistencyState::kActive);
  EXPECT_EQ(result.status.consistency_outvoted_total, 0U);
  EXPECT_LT(result.max_yaw_error_deg(spin_begin, spin_begin + 3.0), 2.0);
}

// T3 (Issue #92 version 3.1): the first leg is short so that the first turn verifies the gyro early;
// the long straight legs afterwards then run with the gyro active.
GyroDriveResult run_t3(double first_leg_m, double gyro_bias_rad_per_s, bool has_gyro) {
  GyroDriveOptions options;
  options.course = SegmentedCourse({first_leg_m, 30.0, 30.0, 30.0, 30.0, 30.0}, {1.0, 1.0, -1.0, 1.0, -1.0}, 0.6, 0.4);
  options.wheel_yaw_drift_rad_per_m = 1.5 * M_PI / 180.0 / 10.0;  // three times the Issue's wheel model
  options.has_gyro = has_gyro;
  if (gyro_bias_rad_per_s != 2.0e-4) {
    // The standard increment contains 2e-4 rad/s of bias; replace it by the requested one.
    options.gyro_transform = [gyro_bias_rad_per_s](double, double standard) {
      return std::optional<double>(standard + (gyro_bias_rad_per_s - 2.0e-4) * kDt);
    };
  }
  return run_gyro_drive(options);
}

TEST(LocalizerGyroDrive, T3_ALongDriveWithATripledWheelYawBiasEndsAsWellAsTheGyroAllows) {
  const GyroDriveResult reference = run_t3(5.0, 2.0e-4, false);
  const GyroDriveResult with_gyro = run_t3(5.0, 2.0e-4, true);
  const double wheel_only_final = reference.samples.back().yaw_error_deg;
  const double gyro_final = with_gyro.samples.back().yaw_error_deg;
  std::printf("[ MEASURED ] T3 final yaw error without gyro %.3f deg, with gyro %.3f deg (limit %.3f)\n",
              wheel_only_final, gyro_final, std::max(1.0, 0.5 * wheel_only_final));
  EXPECT_LE(gyro_final, std::max(1.0, 0.5 * wheel_only_final));
  EXPECT_EQ(with_gyro.status.consistency_outvoted_total, 0U);
}

// Measurements for the PR (no pass / fail): without the gyro bias, and with the first leg left at 30 m.
TEST(LocalizerGyroDrive, T3_MeasurementsForThePullRequest) {
  const GyroDriveResult no_bias = run_t3(5.0, 0.0, true);
  const GyroDriveResult long_first_leg = run_t3(30.0, 2.0e-4, true);
  const GyroDriveResult long_first_leg_reference = run_t3(30.0, 2.0e-4, false);
  std::printf("[ MEASURED ] T3 gyro bias 0: final yaw error %.3f deg\n", no_bias.samples.back().yaw_error_deg);
  std::printf("[ MEASURED ] T3 first leg 30 m: final yaw error %.3f deg with gyro, %.3f deg without\n",
              long_first_leg.samples.back().yaw_error_deg, long_first_leg_reference.samples.back().yaw_error_deg);
  SUCCEED();
}

// ---- Review of PR #96 ----
TEST(WheelYawSamples, ASampleOnAKeyframeTimeIsSplitInProportionToTheTimeOnEachSide) {
  // 50 Hz samples turning at 0.5 rad/s (0.01 rad each, covered by an exact gyro of 0.005 rad each).
  std::deque<WheelYawSample> samples;
  for (int index = 1; index <= 100; ++index) samples.push_back({index * 0.02, 0.02, true, 0.005, 0.01});
  // The keyframe at 1.01 s falls in the middle of the sample (1.00, 1.02].
  const WheelYawInterval before = summarize_wheel_yaw_samples(samples, 0.0, 1.01);
  const WheelYawInterval after = summarize_wheel_yaw_samples(samples, 1.01, 2.0);
  EXPECT_NEAR(before.total_sec, 1.01, 1e-12);
  EXPECT_NEAR(after.total_sec, 0.99, 1e-12);
  EXPECT_NEAR(before.raw_wheel_yaw_sum_rad, 0.5 * 1.01, 1e-12);
  EXPECT_NEAR(after.raw_wheel_yaw_sum_rad, 0.5 * 0.99, 1e-12);
  EXPECT_NEAR(before.gyro_yaw_sum_rad, 0.25 * 1.01, 1e-12);
  EXPECT_NEAR(before.gyro_yaw_sum_rad + after.gyro_yaw_sum_rad, 0.25 * 2.0, 1e-12);
  EXPECT_TRUE(before.is_sufficiently_covered());
}

TEST(WheelYawSamples, AnUncoveredPartOfASplitSampleStaysWithItsSide) {
  std::deque<WheelYawSample> samples = {{0.02, 0.02, true, 0.005, 0.01}, {0.04, 0.02, false, 0.0, 0.01}};
  const WheelYawInterval first = summarize_wheel_yaw_samples(samples, 0.0, 0.03);
  EXPECT_NEAR(first.covered_sec, 0.02, 1e-12);
  EXPECT_NEAR(first.total_sec, 0.03, 1e-12);
  EXPECT_NEAR(first.uncovered_wheel_yaw_sum_rad, 0.005, 1e-12);
  EXPECT_NEAR(first.measured_yaw_rad(), 0.005 + 0.005, 1e-12);
}

TEST(LocalizerGyroDrive, TheOutputDoesNotJumpWhenTheWheelTimeGoesBackwardsWhileTheGyroIsUsed) {
  GyroDriveOptions options;
  options.initial_state = GyroConsistencyState::kActive;
  options.wheel_yaw_scale = 1.4;  // the raw wheel yaw is 36 degrees off after the first turn
  options.wheel_time_backwards_sec = options.course.turn_windows()[0].second + 3.0;
  const GyroDriveResult result = run_gyro_drive(options);
  double largest_step_deg = 0.0;
  for (std::size_t index = 1; index < result.samples.size(); ++index) {
    if (std::abs(result.samples[index].time_sec - options.wheel_time_backwards_sec) > 0.2) continue;
    const Eigen::Matrix3d difference = result.samples[index - 1].map_to_base.rotation().transpose() *
                                       result.samples[index].map_to_base.rotation();
    largest_step_deg = std::max(largest_step_deg, std::abs(std::atan2(difference(1, 0), difference(0, 0))) * 180.0 / M_PI);
  }
  std::printf("[ MEASURED ] largest output yaw step around the backwards sample %.4f deg\n", largest_step_deg);
  EXPECT_LT(largest_step_deg, 0.2);
  EXPECT_LT(result.max_yaw_error_deg(options.wheel_time_backwards_sec, 1.0e9), 2.0);
}

TEST(LocalizerGyroDrive, TheVerificationSumsStartOverWhenTheWheelTimeGoesBackwards) {
  GyroDriveOptions options;
  options.course = SegmentedCourse({4.0, 4.0}, {1.0}, 0.6, 0.1);  // a slow turn: 0.1 rad per keyframe
  options.wheel_yaw_scale = 1.0;
  options.wheel_time_backwards_sec = options.course.turn_windows()[0].first + 1.7;
  const GyroDriveResult result = run_gyro_drive(options);
  ASSERT_TRUE(result.status_before_backwards.has_value());
  ASSERT_TRUE(result.status_before_backwards->verification_sums_rad.has_value());
  EXPECT_GT(std::abs((*result.status_before_backwards->verification_sums_rad)[0]), 0.05);
  ASSERT_TRUE(result.status_after_backwards->verification_sums_rad.has_value());
  EXPECT_EQ(*result.status_after_backwards->verification_sums_rad, Eigen::Vector3d::Zero());
}
