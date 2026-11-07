#include "lightnav_localization/map_builder.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <map>
#include <stdexcept>

#include "lightnav_localization/downsample.hpp"
#include "lightnav_localization/map_metadata.hpp"

namespace lightnav_localization {
namespace {

constexpr std::size_t kMaxTrajectoryRows = 1000000;

double roll_of(const Eigen::Isometry3d& pose) {
  return std::atan2(pose.rotation()(2, 1), pose.rotation()(2, 2));
}
double pitch_of(const Eigen::Isometry3d& pose) {
  return std::asin(std::clamp(-pose.rotation()(2, 0), -1.0, 1.0));
}
double yaw_of(const Eigen::Isometry3d& pose) {
  return std::atan2(pose.rotation()(1, 0), pose.rotation()(0, 0));
}

std::string join_path(const std::string& directory, const std::string& name) {
  return (std::filesystem::path(directory) / name).string();
}

}  // namespace

PointList build_map_points(const std::vector<KeyframeCloud>& keyframes,
                           const std::vector<TrajectoryPoint>& trajectory,
                           double map_voxel_size_m) {
  if (!(map_voxel_size_m > 0.0) || !std::isfinite(map_voxel_size_m)) {
    throw std::runtime_error("map_voxel_size_m must be positive and finite");
  }
  std::map<int, Eigen::Isometry3d> poses;
  for (const TrajectoryPoint& point : trajectory) poses[point.node_index] = point.map_to_base;
  PointList assembled;
  for (const KeyframeCloud& keyframe : keyframes) {
    const auto found = poses.find(keyframe.node_index);
    if (found == poses.end()) continue;
    assembled.reserve(assembled.size() + keyframe.points_in_base.size());
    for (const Eigen::Vector3f& point : keyframe.points_in_base) {
      const Eigen::Vector3d transformed =
          found->second.linear() * point.cast<double>() + found->second.translation();
      assembled.push_back(transformed.cast<float>());
    }
  }
  return downsample_voxel(assembled, map_voxel_size_m);
}

void write_trajectory_csv(const std::string& file_path,
                          const std::vector<TrajectoryPoint>& trajectory) {
  if (trajectory.size() > kMaxTrajectoryRows) {
    throw std::runtime_error("trajectory has too many rows");
  }
  std::ofstream stream(file_path, std::ios::out | std::ios::trunc);
  if (!stream) throw std::runtime_error("cannot open trajectory.csv: " + file_path);
  stream << "stamp_sec,x,y,z,roll,pitch,yaw,position_std_m\n";
  stream << std::setprecision(12);
  for (const TrajectoryPoint& point : trajectory) {
    const Eigen::Vector3d position = point.map_to_base.translation();
    stream << point.stamp_sec << ',' << position.x() << ',' << position.y() << ','
           << position.z() << ',' << roll_of(point.map_to_base) << ','
           << pitch_of(point.map_to_base) << ',' << yaw_of(point.map_to_base) << ','
           << point.position_std_m << '\n';
  }
  stream.flush();
  if (!stream) throw std::runtime_error("cannot write trajectory.csv: " + file_path);
}

MapOutputResult write_map_outputs(const MapOutputParameters& parameters,
                                  const std::vector<KeyframeCloud>& keyframes,
                                  const std::vector<TrajectoryPoint>& trajectory) {
  if (parameters.map_output_dir.empty()) {
    throw std::runtime_error("map_output_dir is empty");
  }
  if (!(parameters.map_voxel_size_m > 0.0) || !std::isfinite(parameters.map_voxel_size_m)) {
    throw std::runtime_error("map_voxel_size_m must be positive and finite");
  }
  std::error_code directory_error;
  std::filesystem::create_directories(parameters.map_output_dir, directory_error);
  if (directory_error) {
    throw std::runtime_error("cannot create map_output_dir: " + directory_error.message());
  }

  MapOutputResult result;
  result.keyframe_count = keyframes.size();
  PointList map_points =
      build_map_points(keyframes, trajectory, parameters.map_voxel_size_m);
  result.map_point_count = map_points.size();

  write_pcd(join_path(parameters.map_output_dir, "map.pcd"), map_points, PcdEncoding::kBinary);
  write_trajectory_csv(join_path(parameters.map_output_dir, "trajectory.csv"), trajectory);

  MapMetadata metadata;
  metadata.has_gps_origin = parameters.has_gps_origin;
  if (parameters.has_gps_origin) {
    metadata.origin_latitude_deg = parameters.gps_origin.latitude_deg;
    metadata.origin_longitude_deg = parameters.gps_origin.longitude_deg;
    metadata.origin_altitude_m = parameters.gps_origin.altitude_m;
  }
  metadata.voxel_size_m = parameters.map_voxel_size_m;
  metadata.created = parameters.created;
  metadata.keyframe_count = static_cast<int>(keyframes.size());
  metadata.batch_optimized = true;
  write_map_metadata(join_path(parameters.map_output_dir, "map.yaml"), metadata);
  return result;
}

}  // namespace lightnav_localization
