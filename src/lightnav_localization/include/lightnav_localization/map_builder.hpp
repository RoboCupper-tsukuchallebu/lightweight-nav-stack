#pragma once

// Pure mapping output logic (Issue #30): assemble keyframe clouds into a map, write map.pcd,
// map.yaml and trajectory.csv. No ROS types, so the node and the gtest share the same code.

#include <Eigen/Geometry>
#include <cstddef>
#include <string>
#include <vector>

#include "lightnav_localization/gps_input.hpp"
#include "lightnav_localization/loop_closure.hpp"
#include "lightnav_localization/pcd_io.hpp"

namespace lightnav_localization {

// Keyframe cloud after deskewing, expressed in the base frame at `stamp_sec`.
struct KeyframeCloud {
  int node_index = 0;
  double stamp_sec = 0.0;
  PointList points_in_base;
};

// One row of trajectory.csv: the batch-optimized pose of a keyframe.
struct TrajectoryPoint {
  int node_index = 0;
  double stamp_sec = 0.0;
  Eigen::Isometry3d map_to_base = Eigen::Isometry3d::Identity();
  double position_std_m = 0.0;
};

struct MapOutputParameters {
  std::string map_output_dir;
  double map_voxel_size_m = 0.2;
  int batch_max_iterations = 100;
  bool batch_optimized = false;
  int loop_closure_count = 0;
  LoopClosureParameters loop_closure;
  const LoopClosureTestHooks* loop_closure_test_hooks = nullptr;
  bool has_gps_origin = false;
  GpsOrigin gps_origin;
  std::string created;  // ISO-8601 UTC, e.g. 2026-01-02T03:04:05Z
};

struct MapOutputResult {
  std::size_t keyframe_count = 0;
  std::size_t map_point_count = 0;
  std::string warning;
  std::string statistics;
};

// Transforms every keyframe cloud by its trajectory pose, concatenates them and voxel-downsamples
// the result at `map_voxel_size_m`. Keyframes without a trajectory row are skipped.
PointList build_map_points(const std::vector<KeyframeCloud>& keyframes,
                           const std::vector<TrajectoryPoint>& trajectory,
                           double map_voxel_size_m);

// Writes the contract header and one row per keyframe. Throws std::runtime_error on I/O failure.
void write_trajectory_csv(const std::string& file_path,
                          const std::vector<TrajectoryPoint>& trajectory);

// Appended to each output name while it is being written.
constexpr const char* kTemporarySuffix = ".tmp";

// Creates `map_output_dir` if needed and writes map.pcd (binary, x y z), map.yaml and
// trajectory.csv. Each file is written under its name + kTemporarySuffix, then renamed in the order
// map.pcd, trajectory.csv, map.yaml. If writing fails, the previous three files are left untouched.
// Throws std::runtime_error on failure.
MapOutputResult write_map_outputs(const MapOutputParameters& parameters,
                                  const std::vector<KeyframeCloud>& keyframes,
                                  const std::vector<TrajectoryPoint>& trajectory);

}  // namespace lightnav_localization
