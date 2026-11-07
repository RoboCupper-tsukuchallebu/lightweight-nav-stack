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
#include "lightnav_localization/map_builder.hpp"
#include "lightnav_localization/map_metadata.hpp"
#include "lightnav_localization/pcd_io.hpp"
#include "synthetic_world.hpp"

using namespace lightnav_localization;
using namespace lightnav_localization::testing;

namespace {

constexpr double kDt = 0.02;
constexpr double kScanDuration = 0.1;
constexpr double kSpeed = 0.6;
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
const Course kK2Course{{70.0, 60.0, 70.0}, {1.0, -1.0}};
const Course kLoopCourse{{40.0, 40.0, 40.0, 40.0}, {1.0, 1.0, 1.0, 1.0}};

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
  std::vector<std::pair<double, Eigen::Isometry3d>> online_poses;
};

MappingRun run_mapping(const Course& course, bool has_pillars,
                       double gps_off_begin_sec, double gps_off_end_sec,
                       bool gps_enabled, double wheel_scale, double gps_noise_m,
                       const std::filesystem::path& output_directory) {
  SyntheticWorld world(course.centerline(), true);
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
  // K2 must observe the online error through the whole 100 m GPS outage; the production
  // default would declare LOST after 50 m / 60 s and stop publishing.
  parameters.state.odom_only_max_distance_m = 200.0;
  parameters.state.odom_only_max_s = 10000.0;
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
                                distance * (0.5 * M_PI / 180.0 / 10.0),
                            Eigen::Vector3d::UnitZ())
              .toRotationMatrix();
      odom = odom * drifted;
    }
    previous_truth = truth;
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
    const auto output = localizer.on_wheel_odom(time_sec, odom);
    if (output) {
      result.online_poses.push_back({time_sec, output->map_to_base});
      const double error =
          (output->map_to_base.translation() - truth.translation()).head<2>().norm();
      result.online_max_error_m = std::max(result.online_max_error_m, error);
      if (time_sec >= gps_off_begin_sec && time_sec <= gps_off_end_sec) {
        result.online_dropout_max_error_m =
            std::max(result.online_dropout_max_error_m, error);
      }
    }
  }

  MapOutputParameters output_parameters;
  output_parameters.map_output_dir = output_directory.string();
  output_parameters.map_voxel_size_m = 0.2;
  output_parameters.batch_max_iterations = 100;
  output_parameters.created = "2026-01-01T00:00:00Z";
  MapOutputResult output_result;
  std::string error;
  EXPECT_TRUE(localizer.save_map(output_parameters, output_result, error)) << error;
  result.accepted_gps_factors = localizer.accepted_gps_factor_count();
  result.gps_gate_rejections = localizer.gps_gate_rejection_count();
  result.gps_time_gap_rejections = localizer.gps_time_gap_rejection_count();
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
      const double error =
          (output->map_to_base.translation() - truth.translation()).head<2>().norm();
      result.max_error_m = std::max(result.max_error_m, error);
    }
  }
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

// Largest gap between clusters of wall points near the loop start (the double-wall seam).
double start_cap_double_wall_gap(const PointList& map_points) {
  std::vector<double> x_values;
  for (const Eigen::Vector3f& point : map_points) {
    if (std::abs(point.y()) < 1.2F && std::abs(point.x()) < 3.0F && point.z() > 0.3F &&
        point.z() < 2.5F) {
      x_values.push_back(point.x());
    }
  }
  if (x_values.size() < 4) return std::numeric_limits<double>::quiet_NaN();
  std::sort(x_values.begin(), x_values.end());
  double largest_gap = 0.0;
  double left_mean = x_values.front();
  double right_sum = 0.0;
  std::size_t right_count = 0;
  for (std::size_t index = 1; index < x_values.size(); ++index) {
    if (x_values[index] - x_values[index - 1] > 0.15) {
      left_mean = x_values[index - 1];
      right_sum = x_values[index];
      right_count = 1;
      for (std::size_t next = index + 1; next < x_values.size(); ++next) {
        if (x_values[next] - x_values[next - 1] > 0.15) break;
        right_sum += x_values[next];
        ++right_count;
      }
      largest_gap = std::max(largest_gap, right_sum / right_count - left_mean);
      break;
    }
  }
  return largest_gap;
}

}  // namespace

TEST(MapBuilder, K_MappingMapLocalizesAndTrajectoryMatchesTruth) {
  const std::string output_directory = unique_directory("k");
  const MappingRun mapping =
      run_mapping(kKCourse, true, -1.0, -1.0, true, 1.01, 0.2, output_directory);
  const ErrorSummary summary = summarize_trajectory_error(kKCourse, mapping.trajectory);
  const double trajectory_error = summary.maximum;
  const LocalizationRun localization =
      run_localization(kKCourse, mapping.map_points, true, 1.01);
  std::printf(
      "[ K ] keyframes=%zu map_points=%zu trajectory_errors_max=%.6f "
      "trajectory_max_stamp=%.3f first_quarter=%.6f middle=%.6f last_quarter=%.6f "
      "stamps=[%.3f,%.3f] localization_max_error=%.6f online_max_error=%.6f\n",
      mapping.trajectory.size(), mapping.map_points.size(), trajectory_error,
      summary.maximum_stamp_sec, summary.first_quarter_maximum, summary.middle_maximum,
      summary.last_quarter_maximum,
      mapping.trajectory.empty() ? 0.0 : mapping.trajectory.front().stamp_sec,
      mapping.trajectory.empty() ? 0.0 : mapping.trajectory.back().stamp_sec,
      localization.max_error_m, mapping.online_max_error_m);
  EXPECT_GT(mapping.trajectory.size(), 10U);
  EXPECT_GT(mapping.map_points.size(), 100U);
  EXPECT_TRUE(mapping.metadata.batch_optimized);
  EXPECT_LT(trajectory_error, 0.2);
  EXPECT_TRUE(localization.published);
  EXPECT_LT(localization.max_error_m, 0.2);
  std::error_code error;
  std::filesystem::remove_all(output_directory, error);
}

TEST(MapBuilder, K2_BatchFixesGpsDropoutAndReportsLoopSeam) {
  const std::string dropout_directory = unique_directory("k2_dropout");
  const double gps_off_begin_sec = 50.0 / kSpeed;
  const double gps_off_end_sec = 150.0 / kSpeed;
  const MappingRun mapping = run_mapping(kK2Course, false, gps_off_begin_sec, gps_off_end_sec,
                                         true, 1.01, 0.1, dropout_directory);
  const ErrorSummary summary = summarize_trajectory_error(kK2Course, mapping.trajectory);
  const ErrorSummary before_dropout =
      summarize_error_in_window(kK2Course, mapping.trajectory, 0.0, gps_off_begin_sec);
  const ErrorSummary in_dropout = summarize_error_in_window(
      kK2Course, mapping.trajectory, gps_off_begin_sec, gps_off_end_sec);
  const ErrorSummary after_dropout = summarize_error_in_window(
      kK2Course, mapping.trajectory, gps_off_end_sec, 1e9);
  const double batch_error = summary.maximum;
  std::printf(
      "[ K2 ] keyframes=%zu map_points=%zu online_max_error=%.6f "
      "online_dropout_max_error=%.6f batch_max_error=%.6f batch_max_stamp=%.3f "
      "batch_before_dropout=%.6f batch_in_dropout=%.6f batch_after_dropout=%.6f "
      "dropout_time=[%.3f,%.3f] gps_factors=%zu gps_gate_rejections=%zu "
      "gps_time_gap_rejections=%zu\n",
      mapping.trajectory.size(), mapping.map_points.size(), mapping.online_max_error_m,
      mapping.online_dropout_max_error_m, batch_error, summary.maximum_stamp_sec,
      before_dropout.maximum, in_dropout.maximum, after_dropout.maximum, gps_off_begin_sec,
      gps_off_end_sec, mapping.accepted_gps_factors, mapping.gps_gate_rejections,
      mapping.gps_time_gap_rejections);
  EXPECT_GT(mapping.online_dropout_max_error_m, 0.5);
  EXPECT_LT(batch_error, 0.2);
  std::error_code dropout_error;
  std::filesystem::remove_all(dropout_directory, dropout_error);

  const std::string seam_directory = unique_directory("k2_seam");
  const MappingRun loop =
      run_mapping(kLoopCourse, false, 80.0 / kSpeed, 1e9, true, 1.01, 1.0, seam_directory);
  ASSERT_GE(loop.trajectory.size(), 2U);
  const CsvRow& first = loop.trajectory.front();
  const CsvRow& last = loop.trajectory.back();
  const double seam_pose_gap =
      Eigen::Vector2d(last.x - first.x, last.y - first.y).norm();
  const double double_wall_gap_m = start_cap_double_wall_gap(loop.map_points);
  std::printf("[ K2_SEAM ] keyframes=%zu seam_pose_gap_m=%.6f double_wall_gap_m=%.6f\n",
              loop.trajectory.size(), seam_pose_gap, double_wall_gap_m);
  EXPECT_GT(loop.trajectory.size(), 10U);
  std::error_code seam_error;
  std::filesystem::remove_all(seam_directory, seam_error);
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
