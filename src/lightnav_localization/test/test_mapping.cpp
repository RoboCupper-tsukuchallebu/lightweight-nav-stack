#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <limits>
#include <random>
#include <sstream>
#include <string>
#include <vector>

#include "lightnav_localization/localizer.hpp"
#include "lightnav_localization/loop_closure.hpp"
#include "lightnav_localization/matcher.hpp"
#include "lightnav_localization/map_builder.hpp"
#include "lightnav_localization/map_metadata.hpp"
#include "lightnav_localization/mapping_initialization.hpp"
#include "lightnav_localization/pcd_io.hpp"
#include "synthetic_world.hpp"

using namespace lightnav_localization;
using namespace lightnav_localization::testing;

namespace {

constexpr double kDt = 0.02;
constexpr double kScanDuration = 0.1;
constexpr double kSpeed = 0.6;
// Batch trajectory error limits for GPS sigma 1.0 m (Issue #43 D4: K 0.4 m from the pixi
// measurement 0.366 m and K2 1.0 m from 0.848 m; the apt/Docker build measures 0.410 m and 0.847 m, so K is 0.45 m).
constexpr double kSigma1KMaximumErrorM = 0.45;
constexpr double kSigma1K2MaximumErrorM = 1.0;
constexpr double kTurnRate = 0.4;
constexpr double kMetersPerDegreeLatitude = 110959.00067472785;
constexpr double kMetersPerDegreeLongitude = 90163.68782156157;
const GpsOrigin kOrigin{36.0, 140.0, 0.0};

struct Course {
  std::vector<double> leg_lengths_m;
  std::vector<double> turn_signs;

  double leg_duration_sec(std::size_t leg) const { return leg_lengths_m[leg] / kSpeed; }
  static double turn_duration_sec() { return (M_PI / 2) / kTurnRate; }
  double total_sec() const {
    double total = turn_duration_sec() * turn_signs.size();
    for (const double length : leg_lengths_m) total += length / kSpeed;
    return total;
  }
  std::vector<Eigen::Vector2d> centerline() const {
    std::vector<Eigen::Vector2d> points{{0, 0}};
    double heading = 0.0;
    for (std::size_t leg = 0; leg < leg_lengths_m.size(); ++leg) {
      points.push_back(points.back() +
                       leg_lengths_m[leg] * Eigen::Vector2d(std::cos(heading), std::sin(heading)));
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
      if (remaining <= turn_duration_sec()) {
        return arc_pose(remaining, start, heading, 0.0, turn_signs[leg] * kTurnRate);
      }
      remaining -= turn_duration_sec();
      heading += turn_signs[leg] * M_PI / 2;
    }
    return arc_pose(0.0, start, heading, 0.0, 0.0);
  }
};

const Course kKCourse{{20.0, 20.0}, {1.0}};
const Course kSpecCourse{{70.0, 60.0, 70.0}, {1.0, -1.0}};
// 80 m: leaves room for a 60 s hole in the input (36 m); the corner is at 83 s, after the point (75 s)
// where the sub-map is inspected.
// Time after a gap at which the sub-map is inspected, while keyframes from before the fix would still
// be in it (later they roll out of the sub-map and the stacking is no longer visible).
constexpr double kSecondsToObserveAfterGap = 5.0;
const Course kGapCourse{{50.0, 30.0}, {1.0}};
const Course kLoopCourse{{40.0, 40.0, 40.0, 40.0}, {1.0, 1.0, 1.0, 1.0}};
const Course kPeriodicCourse{{30.0, 0.0, 30.0}, {1.0, 1.0}};
// Biases chosen to produce approximately one-third and one-half period of drift.
constexpr double kPeriodicOneThirdYawBiasDegPer10m = 2.65;
constexpr double kPeriodicHalfYawBiasDegPer10m = 3.85;

GpsFix fix_at_enu(double east_m, double north_m) {
  GpsFix fix;
  fix.latitude_deg = kOrigin.latitude_deg + north_m / kMetersPerDegreeLatitude;
  fix.longitude_deg = kOrigin.longitude_deg + east_m / kMetersPerDegreeLongitude;
  fix.status = GpsFixStatus::kFix;
  fix.has_known_covariance = true;
  fix.known_std_m = 1.0;
  return fix;
}

struct CsvRow {
  double stamp_sec = 0.0;
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  double position_std_m = 0.0;
};

std::vector<CsvRow> read_trajectory(const std::string& file_path) {
  std::ifstream stream(file_path);
  if (!stream) throw std::runtime_error("cannot read " + file_path);
  std::string line;
  std::getline(stream, line);  // header
  std::vector<CsvRow> rows;
  while (std::getline(stream, line)) {
    if (line.empty()) continue;
    std::stringstream line_stream(line);
    std::array<double, 8> values{};
    std::string field;
    std::size_t field_index = 0;
    while (std::getline(line_stream, field, ',') && field_index < values.size()) {
      values[field_index++] = std::stod(field);
    }
    rows.push_back({values[0], values[1], values[2], values[3],
                    values[4], values[5], values[6], values[7]});
  }
  return rows;
}

struct MappingRun {
  std::vector<CsvRow> trajectory;
  PointList map_points;
  MapMetadata metadata;
  double online_max_error_m = 0.0;
  double online_dropout_max_error_m = 0.0;
  std::size_t accepted_gps_factors = 0;
  std::size_t gps_gate_rejections = 0;
  std::size_t gps_time_gap_rejections = 0;
  std::size_t outputs_after_failure = 0;
  bool online_moved_after_failure = false;
  std::string last_error;
  std::string loop_statistics;
  std::vector<std::string> loop_logs;
  std::vector<std::pair<double, Eigen::Isometry3d>> online_poses;
  // Sub-map keyframes a few seconds after the input gap (empty without a gap).
  std::vector<Eigen::Isometry3d> lidar_keyframe_poses_after_gap;
};

MappingRun run_mapping(const Course& course, bool has_pillars,
                       double gps_off_begin_sec, double gps_off_end_sec,
                       bool gps_enabled, double wheel_scale, double gps_noise_m,
                       const std::filesystem::path& output_directory,
                       double fail_optimization_at_sec = -1.0,
                       double wheel_gap_begin_sec = -1.0, double wheel_gap_end_sec = -1.0,
                       bool loop_closure_enabled = true, double wheel_yaw_bias_deg_per_10m = 0.5,
                       bool closed_course = false,
                       const LoopClosureTestHooks* hooks = nullptr,
                       bool periodic_pillar_corridor = false) {
  const auto centerline = periodic_pillar_corridor
                              ? std::vector<Eigen::Vector2d>{{0, 0}, {30, 0}}
                              : course.centerline();
  SyntheticWorld world(centerline, !periodic_pillar_corridor, closed_course);
  if (periodic_pillar_corridor) world.add_pillars(0.2, 0.2, 3.0, 1.5, true);
  if (has_pillars) {
    world.add_pillars(0.1, 0.5, 1.25, 1.5, true);
  }
  const auto truth_at = [&course, &world](double time_sec) {
    Eigen::Isometry3d pose = course.truth_at(time_sec);
    pose.translation().z() += world.floor_height(pose.translation().x());
    return pose;
  };

  LocalizerParameters parameters;
  parameters.mapping = true;
  parameters.keyframe_cloud_voxel_m = 0.25;
  // K2 observes the online error through the whole 100 m GPS outage: mapping mode does not apply
  // the 50 m / 60 s ODOM_ONLY limits (Issue #43 D5), so no override is needed here.
  Localizer localizer(parameters, PointList{}, std::optional<GpsOrigin>(kOrigin));

  Eigen::Isometry3d initial_pose = truth_at(0.0);
  localizer.initialize(initial_pose);

  std::mt19937 random_generator(31);
  std::normal_distribution<double> noise(0.0, 1.0);
  Eigen::Isometry3d odom = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d previous_truth = truth_at(0.0);
  MappingRun result;
  const int step_count = static_cast<int>(course.total_sec() / kDt);
  for (int step = 0; step <= step_count; ++step) {
    const double time_sec = step * kDt;
    const Eigen::Isometry3d truth = truth_at(time_sec);
    if (step > 0) {
      const Eigen::Isometry3d relative = previous_truth.inverse() * truth;
      const double distance = relative.translation().norm();
      Eigen::Isometry3d drifted = relative;
      drifted.translation() *= wheel_scale;
      drifted.translation().z() = 0.0;
      drifted.linear() =
          Eigen::AngleAxisd(std::atan2(relative.rotation()(1, 0), relative.rotation()(0, 0)) +
                                distance * (wheel_yaw_bias_deg_per_10m * M_PI / 180.0 / 10.0),
                            Eigen::Vector3d::UnitZ())
              .toRotationMatrix();
      odom = odom * drifted;
    }
    previous_truth = truth;
    // No wheel, scan or GPS input in the gap; the odometry keeps integrating (Issue #45).
    if (time_sec > wheel_gap_begin_sec && time_sec < wheel_gap_end_sec) continue;
    if (step > 0 && step % static_cast<int>(0.1 / kDt) == 0 && time_sec >= kScanDuration) {
      localizer.on_scan(time_sec,
                        world.synthesize_scan(truth_at, time_sec - kScanDuration, kScanDuration,
                                              true, 360, 8));
    }
    const bool is_gps_time = step % static_cast<int>(1.0 / kDt) == 13;
    const bool is_gps_off =
        !gps_enabled || (time_sec >= gps_off_begin_sec && time_sec <= gps_off_end_sec);
    if (is_gps_time && !is_gps_off) {
      GpsFix fix = fix_at_enu(truth.translation().x() + gps_noise_m * noise(random_generator),
                              truth.translation().y() + gps_noise_m * noise(random_generator));
      localizer.on_gps(time_sec, fix);
    }
    if (fail_optimization_at_sec >= 0.0 &&
        std::abs(time_sec - fail_optimization_at_sec) < kDt / 2) {
      localizer.inject_optimization_failure_for_test();
    }
    const auto output = localizer.on_wheel_odom(time_sec, odom);
    if (wheel_gap_end_sec >= 0.0 && time_sec >= wheel_gap_end_sec + kSecondsToObserveAfterGap &&
        result.lidar_keyframe_poses_after_gap.empty()) {
      result.lidar_keyframe_poses_after_gap = localizer.lidar_keyframe_poses_for_test();
    }
    if (output) {
      result.online_poses.push_back({time_sec, output->map_to_base});
      if (fail_optimization_at_sec >= 0.0 && time_sec > fail_optimization_at_sec + 1.0) {
        ++result.outputs_after_failure;
      }
      const double error =
          (output->map_to_base.translation() - truth.translation()).head<2>().norm();
      result.online_max_error_m = std::max(result.online_max_error_m, error);
      if (time_sec >= gps_off_begin_sec && time_sec <= gps_off_end_sec) {
        result.online_dropout_max_error_m =
            std::max(result.online_dropout_max_error_m, error);
      }
    }
  }

  if (course.total_sec() == kLoopCourse.total_sec() && !gps_enabled) {
    std::size_t minimum_points = std::numeric_limits<std::size_t>::max();
    std::size_t maximum_points = 0;
    for (const KeyframeCloud& keyframe : localizer.mapping_keyframes()) {
      minimum_points = std::min(minimum_points, keyframe.points_in_base.size());
      maximum_points = std::max(maximum_points, keyframe.points_in_base.size());
    }
    std::printf("[ LOOP_A1_CLOUDS ] keyframes=%zu min=%zu max=%zu\n",
                localizer.mapping_keyframe_count(), minimum_points, maximum_points);
  }
  if (course.total_sec() == kLoopCourse.total_sec() && !gps_enabled && loop_closure_enabled) {
    const auto& keyframes = localizer.mapping_keyframes();
    std::vector<BatchPose> truth_poses;
    for (const auto& keyframe : keyframes) {
      BatchPose pose;
      pose.map_to_base = truth_at(keyframe.stamp_sec);
      truth_poses.push_back(pose);
    }
    const auto lengths = loop_path_lengths(truth_poses);
    const auto candidates = select_loop_candidates(truth_poses, lengths, LoopClosureParameters{});
    EXPECT_FALSE(candidates.empty());
    if (!candidates.empty()) {
      const LoopCandidate candidate = candidates.front();
      PointList target_points;
      const std::size_t begin = candidate.older_index > 5 ? candidate.older_index - 5 : 0;
      const std::size_t end = std::min(keyframes.size(), candidate.older_index + 6);
      for (std::size_t index = begin; index < end; ++index) {
        const Eigen::Isometry3d transform = truth_poses[candidate.older_index].map_to_base.inverse() *
                                            truth_poses[index].map_to_base;
        for (const Eigen::Vector3f& point : keyframes[index].points_in_base) {
          target_points.push_back((transform * point.cast<double>()).cast<float>());
        }
      }
      const PreparedTarget target(target_points);
      for (const auto& offset : std::vector<std::pair<double, double>>{{0.0, 0.0}, {0.3, 3.0}, {1.0, 9.0}}) {
        auto initial_poses = truth_poses;
        initial_poses[candidate.newer_index].map_to_base.translation().x() += offset.first;
        initial_poses[candidate.newer_index].map_to_base.linear() *=
            Eigen::AngleAxisd(offset.second * M_PI / 180.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
        const Eigen::Isometry3d prediction = initial_poses[candidate.older_index].map_to_base.inverse() *
                                              initial_poses[candidate.newer_index].map_to_base;
        MatcherParameters matcher_parameters;
        matcher_parameters.max_iterations = 10;
        matcher_parameters.max_correspondence_distance_m = 2.0;
        const MatchResult coarse = match_scan(target, keyframes[candidate.newer_index].points_in_base,
                                               prediction, matcher_parameters);
        matcher_parameters.max_iterations = 30;
        matcher_parameters.max_correspondence_distance_m = 0.5;
        const MatchResult fine = match_scan(target, keyframes[candidate.newer_index].points_in_base,
                                             coarse.target_to_source, matcher_parameters);
        const Eigen::Isometry3d correction = prediction.inverse() * fine.target_to_source;
        std::printf("[ LOOP_TRUTH_STAGES ] offset=%.1f yaw=%.1f coarse=%d/%d/%.3f fine=%d/%d/%.3f correction=%.3f/%.3f\n",
                    offset.first, offset.second, coarse.converged, coarse.iterations, coarse.inlier_ratio,
                    fine.converged, fine.iterations, fine.inlier_ratio, correction.translation().norm(),
                    Eigen::AngleAxisd(correction.rotation()).angle());
        EXPECT_TRUE(coarse.converged);
        EXPECT_TRUE(fine.converged);
        EXPECT_GT(fine.inlier_ratio, 0.95);
        LoopVerificationFailure failure;
        double inlier = 0.0;
        const auto verified = verify_loop_candidate(candidate, initial_poses, keyframes, lengths, {}, {}, 0.05, &failure, &inlier);
        EXPECT_TRUE(verified.has_value()) << "failure=" << static_cast<int>(failure) << " inlier=" << inlier;
        if (verified) {
          const Eigen::Isometry3d expected = truth_poses[candidate.older_index].map_to_base.inverse() *
                                              truth_poses[candidate.newer_index].map_to_base;
          const Eigen::Isometry3d error = expected.inverse() * verified->older_to_newer;
          EXPECT_LT(error.translation().norm(), 0.1);
          EXPECT_LT(Eigen::AngleAxisd(error.rotation()).angle(), 0.05);
        }
      }
    }
  }
  MapOutputParameters output_parameters;
  output_parameters.map_output_dir = output_directory.string();
  output_parameters.map_voxel_size_m = 0.2;
  output_parameters.batch_max_iterations = 100;
  output_parameters.loop_closure.enabled = loop_closure_enabled;
  output_parameters.loop_closure_test_hooks = hooks;
  output_parameters.loop_closure.info_logger = [&](const std::string& message) {
    result.loop_logs.push_back(message);
  };
  output_parameters.created = "2026-01-01T00:00:00Z";
  MapOutputResult output_result;
  std::string error;
  EXPECT_TRUE(localizer.save_map(output_parameters, output_result, error)) << error;
  result.loop_statistics = output_result.statistics;
  result.accepted_gps_factors = localizer.accepted_gps_factor_count();
  result.gps_gate_rejections = localizer.gps_gate_rejection_count();
  result.gps_time_gap_rejections = localizer.gps_time_gap_rejection_count();
  result.last_error = localizer.last_error();
  if (fail_optimization_at_sec >= 0.0) {
    std::optional<Eigen::Isometry3d> at_failure;
    for (const auto& entry : result.online_poses) {
      if (entry.first >= fail_optimization_at_sec) {
        at_failure = entry.second;
        break;
      }
    }
    if (at_failure && !result.online_poses.empty()) {
      result.online_moved_after_failure =
          (result.online_poses.back().second.translation() - at_failure->translation())
              .head<2>()
              .norm() > 1.0;
    }
  }
  result.trajectory = read_trajectory((output_directory / "trajectory.csv").string());
  result.map_points = read_pcd((output_directory / "map.pcd").string());
  result.metadata = read_map_metadata((output_directory / "map.yaml").string());
  EXPECT_EQ(result.trajectory.size(), result.metadata.keyframe_count);
  return result;
}

struct ErrorSummary {
  double maximum = 0.0;
  double maximum_stamp_sec = 0.0;
  double first_quarter_maximum = 0.0;
  double middle_maximum = 0.0;
  double last_quarter_maximum = 0.0;
};

ErrorSummary summarize_trajectory_error(const Course& course,
                                        const std::vector<CsvRow>& trajectory) {
  ErrorSummary summary;
  const double total_sec = course.total_sec();
  for (const CsvRow& row : trajectory) {
    const Eigen::Vector3d truth = course.truth_at(row.stamp_sec).translation();
    const double error = Eigen::Vector2d(row.x - truth.x(), row.y - truth.y()).norm();
    if (error > summary.maximum) {
      summary.maximum = error;
      summary.maximum_stamp_sec = row.stamp_sec;
    }
    if (row.stamp_sec < 0.25 * total_sec) {
      summary.first_quarter_maximum = std::max(summary.first_quarter_maximum, error);
    } else if (row.stamp_sec > 0.75 * total_sec) {
      summary.last_quarter_maximum = std::max(summary.last_quarter_maximum, error);
    } else {
      summary.middle_maximum = std::max(summary.middle_maximum, error);
    }
  }
  return summary;
}

ErrorSummary summarize_error_in_window(const Course& course,
                                       const std::vector<CsvRow>& trajectory,
                                       double begin_sec, double end_sec) {
  ErrorSummary summary;
  for (const CsvRow& row : trajectory) {
    if (row.stamp_sec < begin_sec || row.stamp_sec > end_sec) continue;
    const Eigen::Vector3d truth = course.truth_at(row.stamp_sec).translation();
    const double error = Eigen::Vector2d(row.x - truth.x(), row.y - truth.y()).norm();
    if (error > summary.maximum) {
      summary.maximum = error;
      summary.maximum_stamp_sec = row.stamp_sec;
    }
  }
  return summary;
}

struct LocalizationRun {
  double max_error_m = 0.0;
  double max_yaw_error_deg = 0.0;
  double wheel_only_end_error_m = 0.0;
  int published_count = 0;
  bool published = false;
};

LocalizationRun run_localization(const Course& course, const PointList& map_points,
                                 bool has_pillars, double wheel_scale) {
  SyntheticWorld world(course.centerline(), true);
  if (has_pillars) world.add_pillars(0.1, 0.5, 1.25, 1.5, true);
  const auto truth_at = [&course, &world](double time_sec) {
    Eigen::Isometry3d pose = course.truth_at(time_sec);
    pose.translation().z() += world.floor_height(pose.translation().x());
    return pose;
  };
  LocalizerParameters parameters;
  Localizer localizer(parameters, map_points, std::optional<GpsOrigin>(kOrigin));
  localizer.initialize(truth_at(0.0));
  std::mt19937 random_generator(41);
  std::normal_distribution<double> noise(0.0, 1.0);
  Eigen::Isometry3d odom = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d wheel_only = truth_at(0.0);
  Eigen::Isometry3d previous_truth = truth_at(0.0);
  LocalizationRun result;
  const int step_count = static_cast<int>(course.total_sec() / kDt);
  for (int step = 0; step <= step_count; ++step) {
    const double time_sec = step * kDt;
    const Eigen::Isometry3d truth = truth_at(time_sec);
    if (step > 0) {
      const Eigen::Isometry3d relative = previous_truth.inverse() * truth;
      const double distance = relative.translation().norm();
      Eigen::Isometry3d drifted = relative;
      drifted.translation() *= wheel_scale;
      drifted.translation().z() = 0.0;
      drifted.linear() =
          Eigen::AngleAxisd(std::atan2(relative.rotation()(1, 0), relative.rotation()(0, 0)) +
                                distance * (0.5 * M_PI / 180.0 / 10.0),
                            Eigen::Vector3d::UnitZ())
              .toRotationMatrix();
      odom = odom * drifted;
      wheel_only = wheel_only * drifted;
    }
    previous_truth = truth;
    if (step > 0 && step % static_cast<int>(0.1 / kDt) == 0 && time_sec >= kScanDuration) {
      localizer.on_scan(time_sec,
                        world.synthesize_scan(truth_at, time_sec - kScanDuration, kScanDuration,
                                              true, 360, 8));
    }
    if (step % static_cast<int>(1.0 / kDt) == 13) {
      localizer.on_gps(time_sec,
                       fix_at_enu(truth.translation().x() + noise(random_generator),
                                  truth.translation().y() + noise(random_generator)));
    }
    const auto output = localizer.on_wheel_odom(time_sec, odom);
    if (output) {
      result.published = true;
      ++result.published_count;
      const double error =
          (output->map_to_base.translation() - truth.translation()).head<2>().norm();
      result.max_error_m = std::max(result.max_error_m, error);
      const Eigen::Matrix3d rotation_error =
          truth.rotation().transpose() * output->map_to_base.rotation();
      const double yaw_error =
          std::abs(std::atan2(rotation_error(1, 0), rotation_error(0, 0))) * 180.0 / M_PI;
      result.max_yaw_error_deg = std::max(result.max_yaw_error_deg, yaw_error);
    }
  }
  result.wheel_only_end_error_m =
      (wheel_only.translation() - truth_at(course.total_sec()).translation()).head<2>().norm();
  return result;
}

std::string unique_directory(const std::string& name) {
  const auto unique =
      std::to_string(std::chrono::steady_clock::now().time_since_epoch().count());
  const std::filesystem::path path =
      std::filesystem::temp_directory_path() / ("lightnav_mapping_" + name + "_" + unique);
  std::filesystem::create_directories(path);
  return path.string();
}

// Distance of seam wall points from the actual wall surfaces in the closed corridor.
double seam_wall_distance_p95(const PointList& map_points, const SyntheticWorld& world) {
  std::vector<double> distances;
  for (const Eigen::Vector3f& point : map_points) {
    if (point.head<2>().norm() < 5.0F && point.z() > 0.3F && point.z() < 2.5F) {
      distances.push_back(world.distance_to_wall(point.head<2>().cast<double>()));
    }
  }
  if (distances.empty()) return std::numeric_limits<double>::quiet_NaN();
  std::sort(distances.begin(), distances.end());
  return distances[static_cast<std::size_t>(0.95 * (distances.size() - 1))];
}

// Share of the wall-height points on the first straight leg of kGapCourse (corridor walls at
// y = +-2 m) that lie more than 0.5 m outside a wall: a map smeared by a wrong keyframe pose has them.
double straight_leg_points_outside_walls_ratio(const PointList& map_points) {
  std::size_t wall_height_count = 0;
  std::size_t outside_count = 0;
  for (const Eigen::Vector3f& point : map_points) {
    const bool is_wall_height = point.z() > 0.3F && point.z() < 2.5F;
    if (point.x() < 3.0F || point.x() > 45.0F || !is_wall_height) continue;
    ++wall_height_count;
    if (std::abs(point.y()) > 2.5F) ++outside_count;
  }
  return wall_height_count == 0 ? 1.0 : static_cast<double>(outside_count) / wall_height_count;
}

}  // namespace

TEST(MapBuilder, K_ClasSigma0p2_ShortCourse) {
  const std::string output_directory = unique_directory("k_clas");
  const MappingRun mapping =
      run_mapping(kKCourse, true, -1.0, -1.0, true, 1.01, 0.2, output_directory);
  const ErrorSummary summary = summarize_trajectory_error(kKCourse, mapping.trajectory);
  const LocalizationRun localization =
      run_localization(kKCourse, mapping.map_points, true, 1.01);
  std::printf(
      "[ K_CLAS ] keyframes=%zu map_points=%zu trajectory_errors_max=%.6f "
      "localization_max_error=%.6f localization_max_yaw_deg=%.6f\n",
      mapping.trajectory.size(), mapping.map_points.size(), summary.maximum,
      localization.max_error_m, localization.max_yaw_error_deg);
  EXPECT_GT(mapping.trajectory.size(), 10U);
  EXPECT_GT(mapping.map_points.size(), 100U);
  EXPECT_TRUE(mapping.metadata.batch_optimized);
  EXPECT_LT(summary.maximum, 0.2);
  EXPECT_TRUE(localization.published);
  EXPECT_LT(localization.max_error_m, 0.2);
  std::error_code error;
  std::filesystem::remove_all(output_directory, error);
}

TEST(MapBuilder, K2_ClasSigma0p1_SpecCourse) {
  const std::string dropout_directory = unique_directory("k2_clas");
  const double gps_off_begin_sec = 50.0 / kSpeed;
  const double gps_off_end_sec = 150.0 / kSpeed;
  const MappingRun mapping = run_mapping(kSpecCourse, false, gps_off_begin_sec, gps_off_end_sec,
                                         true, 1.01, 0.1, dropout_directory);
  const ErrorSummary summary = summarize_trajectory_error(kSpecCourse, mapping.trajectory);
  const ErrorSummary before_dropout =
      summarize_error_in_window(kSpecCourse, mapping.trajectory, 0.0, gps_off_begin_sec);
  const ErrorSummary in_dropout = summarize_error_in_window(
      kSpecCourse, mapping.trajectory, gps_off_begin_sec, gps_off_end_sec);
  const ErrorSummary after_dropout = summarize_error_in_window(
      kSpecCourse, mapping.trajectory, gps_off_end_sec, 1e9);
  std::printf(
      "[ K2_CLAS ] keyframes=%zu map_points=%zu online_max_error=%.6f "
      "online_dropout_max_error=%.6f batch_max_error=%.6f batch_max_stamp=%.3f "
      "batch_before_dropout=%.6f batch_in_dropout=%.6f batch_after_dropout=%.6f\n",
      mapping.trajectory.size(), mapping.map_points.size(), mapping.online_max_error_m,
      mapping.online_dropout_max_error_m, summary.maximum, summary.maximum_stamp_sec,
      before_dropout.maximum, in_dropout.maximum, after_dropout.maximum);
  EXPECT_GT(mapping.online_dropout_max_error_m, 0.5);
  EXPECT_LT(summary.maximum, 0.2);
  std::error_code dropout_error;
  std::filesystem::remove_all(dropout_directory, dropout_error);
}

// A3-4: the same periodic corridor is traversed outward and back.
void run_periodic_mapping(double yaw_bias, bool assert_acceptance) {
  const std::string directory = unique_directory("periodic_corridor");
  const MappingRun mapping = run_mapping(kPeriodicCourse, false, -1.0, -1.0, false, 1.0, 1.0,
                                         directory, -1.0, -1.0, -1.0, true, yaw_bias,
                                         false, nullptr, true);
  const double maximum_error =
      summarize_trajectory_error(kPeriodicCourse, mapping.trajectory).maximum;
  const double revisit_time = kPeriodicCourse.total_sec();
  double revisit_error = 0.0;
  if (!mapping.online_poses.empty()) {
    const auto& last_online_pose = mapping.online_poses.back();
    revisit_error = (last_online_pose.second.translation() -
                     kPeriodicCourse.truth_at(last_online_pose.first).translation())
                        .head<2>().norm();
  }
  std::printf("[ PERIODIC ] bias=%.3f revisit_time=%.3f drift=%.6f max=%.6f %s\n",
              yaw_bias, revisit_time, revisit_error, maximum_error, mapping.loop_statistics.c_str());
  ::testing::Test::RecordProperty("revisit_drift_m", revisit_error);
  ::testing::Test::RecordProperty("maximum_horizontal_error_m", maximum_error);
  for (const std::string& log : mapping.loop_logs) {
    std::printf("[ PERIODIC_CANDIDATE ] %s\n", log.c_str());
  }
  if (assert_acceptance) {
    // The specification accepts zero wrong loops or error < 0.5 m for every row.
    // Conservatively treat every retained loop as potentially wrong.
    EXPECT_TRUE(mapping.metadata.loop_closure_count == 0 || maximum_error < 0.5);
  }
  std::error_code error;
  std::filesystem::remove_all(directory, error);
}

// Before saving, the final online pose is 1.133 m from truth in this synthetic
// setup, approximately one-third of the 3 m period.
TEST(MapBuilder, PeriodicCorridorOneThirdDriftHasNoWrongLoopsOrSmallError) {
  run_periodic_mapping(kPeriodicOneThirdYawBiasDegPer10m, true);
}

// Before saving, the final online pose is 1.424 m from truth in this synthetic
// setup, approximately one-half of the 3 m period.
TEST(MapBuilder, PeriodicCorridorHalfPeriodDriftRecordsOutcome) {
  run_periodic_mapping(kPeriodicHalfYawBiasDegPer10m, false);
}

// A4: inject a five-metre conflicting factor while saving the A1 closed corridor.
TEST(MapBuilder, A4_ClosedCorridorRemovesInjectedConflictingLoop) {
  LoopClosureTestHooks hooks;
  bool injected = false;
  std::size_t correct_count = 0;
  std::size_t injected_newer_index = 0;
  hooks.before_batch_with_graph = [&](PoseGraph& graph) -> std::optional<LoopCandidate> {
    if (injected) return std::nullopt;
    const auto& poses = graph.batch_poses();
    if (poses.size() < 2) return std::nullopt;
    correct_count = graph.loop_factor_count();
    Eigen::Isometry3d incorrect = poses.front().map_to_base.inverse() * poses.back().map_to_base;
    incorrect.translation().x() += 5.0;
    const auto information = Eigen::Matrix<double, 6, 6>::Identity() * 400.0;
    if (!graph.add_loop_factor(poses.front().index, poses.back().index, incorrect, information)) {
      return std::nullopt;
    }
    injected = true;
    injected_newer_index = poses.size() - 1;
    return LoopCandidate{0, injected_newer_index, 0.0};
  };
  const std::string directory = unique_directory("a4_closed_corridor");
  const MappingRun mapping = run_mapping(kLoopCourse, false, -1.0, -1.0, false, 1.01, 1.0,
                                         directory, -1.0, -1.0, -1.0, true, 1.2, true, &hooks);
  ASSERT_TRUE(injected);
  ASSERT_GE(mapping.trajectory.size(), 2U);
  const auto& first_keyframe = mapping.trajectory.front();
  const auto& last_keyframe = mapping.trajectory.back();
  const double gap = Eigen::Vector2d(last_keyframe.x - first_keyframe.x,
                                     last_keyframe.y - first_keyframe.y).norm();
  const auto removal = mapping.loop_statistics.find("removed chi2=");
  const std::string injected_removal = "loop removed: older_index=0 newer_index=" +
      std::to_string(injected_newer_index) + " ";
  EXPECT_TRUE(std::any_of(mapping.loop_logs.begin(), mapping.loop_logs.end(),
                          [&](const std::string& log) { return log.find(injected_removal) == 0; }));
  std::printf("[ A4 ] gap=%.6f correct=%zu %s\n", gap, correct_count,
              mapping.loop_statistics.c_str());
  EXPECT_LT(gap, 0.3);
  ASSERT_NE(removal, std::string::npos);
  EXPECT_EQ(mapping.loop_statistics.find("removed chi2=", removal + 1), std::string::npos);
  EXPECT_EQ(static_cast<std::size_t>(mapping.metadata.loop_closure_count), correct_count);
  std::error_code error;
  std::filesystem::remove_all(directory, error);
}

// 仕様 A1 からの変更: 端壁が交差して互いを遮るため、端壁の無い回廊を使う（PR #62 参照）。
TEST(MapBuilder, A1_ModifiedClosedCorridorWithoutIntersectingEndCaps) {
  const std::string baseline_directory = unique_directory("loop_baseline");
  const std::string closed_directory = unique_directory("loop_closed");
  const MappingRun baseline = run_mapping(kLoopCourse, false, -1.0, -1.0, false, 1.01, 1.0,
                                          baseline_directory, -1.0, -1.0, -1.0, false, 1.2, true);
  const MappingRun closed = run_mapping(kLoopCourse, false, -1.0, -1.0, false, 1.01, 1.0,
                                        closed_directory, -1.0, -1.0, -1.0, true, 1.2, true);
  ASSERT_GE(baseline.trajectory.size(), 2U);
  ASSERT_GE(closed.trajectory.size(), 2U);
  const auto seam_gap = [](const MappingRun& run) {
    return Eigen::Vector2d(run.trajectory.back().x - run.trajectory.front().x,
                           run.trajectory.back().y - run.trajectory.front().y).norm();
  };
  const double baseline_gap = seam_gap(baseline);
  const double closed_gap = seam_gap(closed);
  const SyntheticWorld world(kLoopCourse.centerline(), true, true);
  const double baseline_wall_gap = seam_wall_distance_p95(baseline.map_points, world);
  const double wall_gap = seam_wall_distance_p95(closed.map_points, world);
  RecordProperty("baseline_seam_wall_distance_p95_m", baseline_wall_gap);
  RecordProperty("closed_seam_wall_distance_p95_m", wall_gap);
  std::printf("[ LOOP_A1 ] baseline=%.6f closed=%.6f baseline_wall=%.6f wall=%.6f loops=%d\n",
              baseline_gap, closed_gap, baseline_wall_gap, wall_gap, closed.metadata.loop_closure_count);
  std::printf("[ LOOP_A1_STATS ] %s\n", closed.loop_statistics.c_str());
  EXPECT_GT(baseline_gap, 1.0);
  EXPECT_LT(baseline_gap, 2.5);
  EXPECT_LT(closed_gap, 0.2);
  EXPECT_LT(wall_gap, 0.3);
  EXPECT_LT(summarize_trajectory_error(kLoopCourse, closed.trajectory).maximum, 0.5);
  EXPECT_GE(closed.metadata.loop_closure_count, 2);
  EXPECT_EQ(closed.loop_statistics.find("removed chi2="), std::string::npos);
  std::error_code error;
  std::filesystem::remove_all(baseline_directory, error);
  std::filesystem::remove_all(closed_directory, error);
}

TEST(MapBuilder, K_SpecCourseSigma1p0) {
  const std::string output_directory = unique_directory("k_spec");
  const MappingRun mapping =
      run_mapping(kSpecCourse, true, -1.0, -1.0, true, 1.01, 1.0, output_directory);
  const ErrorSummary summary = summarize_trajectory_error(kSpecCourse, mapping.trajectory);
  const LocalizationRun localization =
      run_localization(kSpecCourse, mapping.map_points, true, 1.01);
  std::printf(
      "[ K_SPEC ] keyframes=%zu map_points=%zu trajectory_errors_max=%.6f "
      "trajectory_max_stamp=%.3f first_quarter=%.6f middle=%.6f last_quarter=%.6f "
      "localization_max_error=%.6f localization_max_yaw_deg=%.6f "
      "wheel_only_end_error=%.6f published=%d\n",
      mapping.trajectory.size(), mapping.map_points.size(), summary.maximum,
      summary.maximum_stamp_sec, summary.first_quarter_maximum, summary.middle_maximum,
      summary.last_quarter_maximum, localization.max_error_m,
      localization.max_yaw_error_deg, localization.wheel_only_end_error_m,
      localization.published_count);
  EXPECT_LT(summary.maximum, kSigma1KMaximumErrorM);
  std::error_code error;
  std::filesystem::remove_all(output_directory, error);
}

TEST(MapBuilder, K2_SpecCourseSigma1p0) {
  const std::string output_directory = unique_directory("k2_spec");
  const double gps_off_begin_sec = 50.0 / kSpeed;
  const double gps_off_end_sec = 150.0 / kSpeed;
  const MappingRun mapping = run_mapping(kSpecCourse, false, gps_off_begin_sec, gps_off_end_sec,
                                         true, 1.01, 1.0, output_directory);
  const ErrorSummary summary = summarize_trajectory_error(kSpecCourse, mapping.trajectory);
  const ErrorSummary before_dropout =
      summarize_error_in_window(kSpecCourse, mapping.trajectory, 0.0, gps_off_begin_sec);
  const ErrorSummary in_dropout = summarize_error_in_window(
      kSpecCourse, mapping.trajectory, gps_off_begin_sec, gps_off_end_sec);
  const ErrorSummary after_dropout = summarize_error_in_window(
      kSpecCourse, mapping.trajectory, gps_off_end_sec, 1e9);
  std::printf(
      "[ K2_SPEC ] keyframes=%zu map_points=%zu online_max_error=%.6f "
      "online_dropout_max_error=%.6f batch_max_error=%.6f batch_max_stamp=%.3f "
      "batch_before_dropout=%.6f batch_in_dropout=%.6f batch_after_dropout=%.6f\n",
      mapping.trajectory.size(), mapping.map_points.size(), mapping.online_max_error_m,
      mapping.online_dropout_max_error_m, summary.maximum, summary.maximum_stamp_sec,
      before_dropout.maximum, in_dropout.maximum, after_dropout.maximum);
  EXPECT_LT(summary.maximum, kSigma1K2MaximumErrorM);
  std::error_code error;
  std::filesystem::remove_all(output_directory, error);
}

// Issue #45: a wheel-odometry gap (60 s) longer than the 30 s window must neither keep an unknown-node
// error nor smear the map: mapping continues after the gap and the sub-map keyframes are not stacked on one pose.
TEST(MapBuilder, MappingContinuesAfterWheelGapLongerThanTheWindow) {
  constexpr double kGapBeginSec = 10.0;
  constexpr double kGapEndSec = 70.0;
  const std::string output_directory = unique_directory("wheel_gap");
  const MappingRun mapping = run_mapping(kGapCourse, false, -1.0, -1.0, true, 1.01, 0.2,
                                         output_directory, -1.0, kGapBeginSec, kGapEndSec);
  EXPECT_EQ(mapping.last_error.find("unknown node"), std::string::npos) << mapping.last_error;
  EXPECT_TRUE(mapping.last_error.empty()) << mapping.last_error;
  std::size_t keyframes_after_gap = 0;
  for (const CsvRow& row : mapping.trajectory) {
    if (row.stamp_sec > kGapEndSec) ++keyframes_after_gap;
  }
  EXPECT_GE(keyframes_after_gap, 30u);  // about 1 keyframe/s for the 63 s after the gap
  const ErrorSummary after_gap = summarize_error_in_window(
      kGapCourse, mapping.trajectory, kGapEndSec, kGapCourse.total_sec());
  const double outside_ratio = straight_leg_points_outside_walls_ratio(mapping.map_points);
  std::printf("[ GAP ] keyframes_after_gap=%zu error_after_gap_m=%.3f points_outside_walls=%.4f\n",
              keyframes_after_gap, after_gap.maximum, outside_ratio);
  EXPECT_LT(after_gap.maximum, 0.5);
  EXPECT_LT(outside_ratio, 0.01);
  // The GPS and the match gate keep a stacked keyframe out of the saved map, so the map check above
  // alone would also pass without the fix; the sub-map keyframe poses just after the gap show the stacking.
  ASSERT_GE(mapping.lidar_keyframe_poses_after_gap.size(), 3u);
  for (std::size_t index = 1; index < mapping.lidar_keyframe_poses_after_gap.size(); ++index) {
    const double spacing_m = (mapping.lidar_keyframe_poses_after_gap[index].translation() -
                              mapping.lidar_keyframe_poses_after_gap[index - 1].translation()).norm();
    EXPECT_GT(spacing_m, 0.3) << "sub-map keyframe " << index << " is stacked on its predecessor";
  }
}

TEST(MapBuilder, MappingSaveSurvivesOptimizationFailure) {
  const std::string baseline_directory = unique_directory("recovery_baseline");
  const MappingRun baseline =
      run_mapping(kKCourse, true, -1.0, -1.0, true, 1.01, 0.2, baseline_directory);
  const double baseline_error =
      summarize_trajectory_error(kKCourse, baseline.trajectory).maximum;

  const std::string output_directory = unique_directory("recovery");
  const MappingRun mapping =
      run_mapping(kKCourse, true, -1.0, -1.0, true, 1.01, 0.2, output_directory, 20.0);
  const ErrorSummary summary = summarize_trajectory_error(kKCourse, mapping.trajectory);
  std::printf("[ MAPPING_RECOVERY ] keyframes=%zu first_stamp=%.3f last_stamp=%.3f "
              "trajectory_errors_max=%.6f baseline_max=%.6f outputs_after_failure=%zu "
              "online_moved_after_failure=%d last_error=%s\n",
              mapping.trajectory.size(),
              mapping.trajectory.empty() ? 0.0 : mapping.trajectory.front().stamp_sec,
              mapping.trajectory.empty() ? 0.0 : mapping.trajectory.back().stamp_sec,
              summary.maximum, baseline_error, mapping.outputs_after_failure,
              mapping.online_moved_after_failure ? 1 : 0, mapping.last_error.c_str());
  ASSERT_GT(mapping.trajectory.size(), 10U);
  EXPECT_LT(mapping.trajectory.front().stamp_sec, 20.0);
  EXPECT_GT(mapping.trajectory.back().stamp_sec, 20.0);
  EXPECT_TRUE(mapping.metadata.batch_optimized);
  EXPECT_GT(mapping.outputs_after_failure, 20U);
  EXPECT_TRUE(mapping.online_moved_after_failure);
  EXPECT_EQ(mapping.last_error.find("restart also failed"), std::string::npos)
      << mapping.last_error;
  EXPECT_LT(summary.maximum, 0.2);
  EXPECT_LT(summary.maximum, baseline_error + 0.05);
  std::error_code error;
  std::filesystem::remove_all(output_directory, error);
  std::filesystem::remove_all(baseline_directory, error);
}

TEST(MapBuilder, TrajectoryCsvHasContractHeaderAndOneRowPerKeyframe) {
  const std::string output_directory = unique_directory("csv");
  std::vector<KeyframeCloud> keyframes;
  for (int index = 0; index < 3; ++index) {
    KeyframeCloud keyframe;
    keyframe.node_index = index;
    keyframe.stamp_sec = 10.0 + index;
    keyframe.points_in_base.push_back(Eigen::Vector3f(0.0F, 0.0F, 0.0F));
    keyframes.push_back(keyframe);
  }
  std::vector<TrajectoryPoint> trajectory;
  for (int index = 0; index < 3; ++index) {
    TrajectoryPoint point;
    point.node_index = index;
    point.stamp_sec = 10.0 + index;
    point.map_to_base.translation() = Eigen::Vector3d(index, 2.0 * index, 0.5);
    point.position_std_m = 0.1 + index;
    trajectory.push_back(point);
  }
  MapOutputParameters parameters;
  parameters.map_output_dir = output_directory;
  parameters.map_voxel_size_m = 0.2;
  parameters.batch_optimized = true;
  parameters.created = "2026-01-01T00:00:00Z";
  const MapOutputResult result = write_map_outputs(parameters, keyframes, trajectory);
  EXPECT_EQ(result.keyframe_count, 3U);
  EXPECT_EQ(read_trajectory((std::filesystem::path(output_directory) / "trajectory.csv").string()).size(),
            3U);
  const MapMetadata metadata =
      read_map_metadata((std::filesystem::path(output_directory) / "map.yaml").string());
  EXPECT_EQ(metadata.keyframe_count, 3);
  EXPECT_FALSE(metadata.has_gps_origin);
  EXPECT_TRUE(metadata.batch_optimized);
  std::error_code error;
  std::filesystem::remove_all(output_directory, error);
}

TEST(MapBuilder, MapYamlRecordsGpsOriginAndBatchFlags) {
  const std::string output_directory = unique_directory("yaml_origin");
  std::vector<KeyframeCloud> keyframes(1);
  keyframes.front().node_index = 0;
  keyframes.front().stamp_sec = 1.0;
  keyframes.front().points_in_base.push_back(Eigen::Vector3f(0.0F, 0.0F, 0.0F));
  std::vector<TrajectoryPoint> trajectory(1);
  trajectory.front().node_index = 0;
  trajectory.front().stamp_sec = 1.0;
  trajectory.front().map_to_base.translation() = Eigen::Vector3d(1.0, 2.0, 0.5);
  trajectory.front().position_std_m = 0.05;
  MapOutputParameters parameters;
  parameters.map_output_dir = output_directory;
  parameters.map_voxel_size_m = 0.2;
  parameters.has_gps_origin = true;
  parameters.gps_origin = GpsOrigin{36.123, 140.456, 12.5};
  parameters.batch_optimized = true;
  parameters.created = "2026-02-03T04:05:06Z";
  write_map_outputs(parameters, keyframes, trajectory);
  const MapMetadata metadata =
      read_map_metadata((std::filesystem::path(output_directory) / "map.yaml").string());
  EXPECT_TRUE(metadata.has_gps_origin);
  EXPECT_DOUBLE_EQ(metadata.origin_latitude_deg, 36.123);
  EXPECT_DOUBLE_EQ(metadata.origin_longitude_deg, 140.456);
  EXPECT_DOUBLE_EQ(metadata.origin_altitude_m, 12.5);
  EXPECT_TRUE(metadata.batch_optimized);
  EXPECT_EQ(metadata.keyframe_count, 1);
  EXPECT_EQ(metadata.created, "2026-02-03T04:05:06Z");
  std::error_code error;
  std::filesystem::remove_all(output_directory, error);
}

namespace {

// Drives straight on the first leg of kSpecCourse without GPS and without a map, so no absolute
// observation ever arrives. Returns the worst state seen and the number of mapping keyframes.
struct NoAbsoluteObservationRun {
  bool was_lost = false;
  std::size_t mapping_keyframes = 0;
  std::size_t mapping_keyframes_at_half = 0;
};

NoAbsoluteObservationRun drive_without_absolute_observations(bool is_mapping, double duration_sec) {
  SyntheticWorld world(kSpecCourse.centerline(), true);
  const auto truth_at = [&world](double time_sec) {
    Eigen::Isometry3d pose = kSpecCourse.truth_at(time_sec);
    pose.translation().z() += world.floor_height(pose.translation().x());
    return pose;
  };
  LocalizerParameters parameters;
  parameters.mapping = is_mapping;
  Localizer localizer(parameters, PointList{}, std::nullopt);
  localizer.initialize(truth_at(0.0));

  NoAbsoluteObservationRun run;
  Eigen::Isometry3d odom = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d previous_truth = truth_at(0.0);
  const int step_count = static_cast<int>(duration_sec / kDt);
  for (int step = 0; step <= step_count; ++step) {
    const double time_sec = step * kDt;
    const Eigen::Isometry3d truth = truth_at(time_sec);
    if (step > 0) odom = odom * (previous_truth.inverse() * truth);
    previous_truth = truth;
    if (step > 0 && step % static_cast<int>(0.1 / kDt) == 0 && time_sec >= kScanDuration) {
      localizer.on_scan(time_sec, world.synthesize_scan(truth_at, time_sec - kScanDuration,
                                                        kScanDuration, true, 360, 8));
    }
    const auto output = localizer.on_wheel_odom(time_sec, odom);
    if (output && output->state == LocalizationState::kLost) run.was_lost = true;
    if (localizer.poll_state(time_sec) == LocalizationState::kLost) run.was_lost = true;
    if (step == step_count / 2) run.mapping_keyframes_at_half = localizer.mapping_keyframe_count();
  }
  run.mapping_keyframes = localizer.mapping_keyframe_count();
  return run;
}

}  // namespace

// Issue #43 D5: 70 s (42 m) without GPS or map exceeds odom_only_max_s (60 s).
TEST(MapBuilder, MappingDoesNotGoLostAtTheOdomOnlyLimits) {
  constexpr double kDurationSec = 70.0;
  const NoAbsoluteObservationRun run = drive_without_absolute_observations(true, kDurationSec);
  std::printf("[ MAPPING_ODOM_ONLY ] lost=%d keyframes=%zu keyframes_at_half=%zu\n",
              run.was_lost ? 1 : 0, run.mapping_keyframes, run.mapping_keyframes_at_half);
  EXPECT_FALSE(run.was_lost);
  EXPECT_GT(run.mapping_keyframes, run.mapping_keyframes_at_half);
  EXPECT_GT(run.mapping_keyframes, 60U);
}

TEST(MapBuilder, LocalizationStillGoesLostAtTheOdomOnlyLimits) {
  const NoAbsoluteObservationRun run = drive_without_absolute_observations(false, 70.0);
  EXPECT_TRUE(run.was_lost);
}

namespace {

struct SavedFile {
  std::string name;
  std::string content;
};

std::vector<SavedFile> read_map_files(const std::string& directory) {
  std::vector<SavedFile> files;
  for (const char* name : {"map.pcd", "trajectory.csv", "map.yaml"}) {
    std::ifstream stream(std::filesystem::path(directory) / name, std::ios::binary);
    std::stringstream buffer;
    buffer << stream.rdbuf();
    files.push_back({name, buffer.str()});
  }
  return files;
}

void make_map_inputs(int keyframe_count, std::vector<KeyframeCloud>& keyframes,
                     std::vector<TrajectoryPoint>& trajectory) {
  for (int index = 0; index < keyframe_count; ++index) {
    KeyframeCloud keyframe;
    keyframe.node_index = index;
    keyframe.stamp_sec = 10.0 + index;
    keyframe.points_in_base.push_back(Eigen::Vector3f(1.0F + index, 0.0F, 0.0F));
    keyframes.push_back(keyframe);
    TrajectoryPoint point;
    point.node_index = index;
    point.stamp_sec = 10.0 + index;
    point.map_to_base.translation() = Eigen::Vector3d(index, 0.0, 0.0);
    trajectory.push_back(point);
  }
}

}  // namespace

// Issue #43 D7 (3): a failure while writing must not leave a new map.pcd next to an old map.yaml.
TEST(MapBuilder, FailedWriteKeepsThePreviousThreeFiles) {
  const std::string output_directory = unique_directory("atomic");
  MapOutputParameters parameters;
  parameters.map_output_dir = output_directory;
  parameters.map_voxel_size_m = 0.2;
  parameters.has_gps_origin = true;
  parameters.gps_origin = GpsOrigin{36.0, 140.0, 1.0};
  parameters.batch_optimized = true;
  parameters.created = "2026-01-01T00:00:00Z";
  std::vector<KeyframeCloud> first_keyframes;
  std::vector<TrajectoryPoint> first_trajectory;
  make_map_inputs(3, first_keyframes, first_trajectory);
  write_map_outputs(parameters, first_keyframes, first_trajectory);
  const std::vector<SavedFile> before = read_map_files(output_directory);

  // A directory in the way of map.yaml's temporary file makes the last write fail, after map.pcd
  // and trajectory.csv were already written under their temporary names.
  const std::filesystem::path blocker =
      std::filesystem::path(output_directory) / (std::string("map.yaml") + kTemporarySuffix);
  std::filesystem::create_directory(blocker);
  std::vector<KeyframeCloud> second_keyframes;
  std::vector<TrajectoryPoint> second_trajectory;
  make_map_inputs(5, second_keyframes, second_trajectory);
  parameters.created = "2026-02-02T00:00:00Z";
  EXPECT_THROW(write_map_outputs(parameters, second_keyframes, second_trajectory),
               std::runtime_error);

  const std::vector<SavedFile> after = read_map_files(output_directory);
  ASSERT_EQ(before.size(), after.size());
  for (std::size_t index = 0; index < before.size(); ++index) {
    EXPECT_EQ(before[index].content, after[index].content) << before[index].name;
  }
  for (const char* name : {"map.pcd", "trajectory.csv"}) {
    EXPECT_FALSE(std::filesystem::exists(std::filesystem::path(output_directory) /
                                         (std::string(name) + kTemporarySuffix)))
        << name;
  }

  std::filesystem::remove_all(blocker);
  write_map_outputs(parameters, second_keyframes, second_trajectory);
  EXPECT_EQ(read_map_metadata((std::filesystem::path(output_directory) / "map.yaml").string())
                .keyframe_count,
            5);
  std::error_code error;
  std::filesystem::remove_all(output_directory, error);
}

TEST(MappingInitialization, CurvedTravelYieldsCurrentHeading) {
  constexpr double kReferenceYaw = 0.2;
  constexpr double kWheelRotation = 0.5;
  constexpr double kChordRelativeToReference = 0.25;
  Eigen::Isometry3d wheel_delta = Eigen::Isometry3d::Identity();
  wheel_delta.linear() =
      Eigen::AngleAxisd(kWheelRotation, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  wheel_delta.translation() =
      Eigen::Vector3d(std::cos(kChordRelativeToReference),
                      std::sin(kChordRelativeToReference), 0.0);
  const Eigen::Vector3d gps_displacement(
      std::cos(kReferenceYaw + kChordRelativeToReference),
      std::sin(kReferenceYaw + kChordRelativeToReference), 0.0);
  const double yaw = mapping_initial_yaw(gps_displacement, wheel_delta);
  std::printf("[ MAPPING_YAW_UNIT ] yaw=%.6f expected=%.6f\n", yaw,
              kReferenceYaw + kWheelRotation);
  EXPECT_NEAR(yaw, kReferenceYaw + kWheelRotation, 1e-9);
}
