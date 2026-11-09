#pragma once

// Live-preview helpers for localization_node (Foxglove / RViz). Not used for the saved map: map.pcd is
// still built from the batch-optimized keyframes.

#include <Eigen/Geometry>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <unordered_set>

#include "lightnav_localization/pcd_io.hpp"

namespace lightnav_localization {

// Keeps the first point that falls into each voxel, up to `max_points` voxels (later voxels are dropped).
// ponytail: first point instead of the voxel mean; use downsample_voxel if the preview looks too noisy.
class VoxelMapAccumulator {
 public:
  VoxelMapAccumulator(double voxel_size_m, std::size_t max_points);

  void add(const PointList& points_in_map);
  void clear();
  const PointList& points() const { return points_; }
  // Increases whenever points() changed, so a publisher can skip an unchanged map.
  std::uint64_t version() const { return version_; }

 private:
  double voxel_size_m_;
  std::size_t max_points_;
  std::unordered_set<std::uint64_t> occupied_voxels_;
  PointList points_;
  std::uint64_t version_ = 0;
};

struct TimedPose {
  double stamp_sec = 0.0;
  Eigen::Isometry3d map_to_base = Eigen::Isometry3d::Identity();
};

// Records a pose when it is at least `min_step_m` away (xy) from the previous recorded pose; the oldest
// pose is dropped beyond `max_poses`.
class PathRecorder {
 public:
  PathRecorder(double min_step_m, std::size_t max_poses);

  void add(double stamp_sec, const Eigen::Isometry3d& map_to_base);
  void clear();
  const std::deque<TimedPose>& poses() const { return poses_; }
  std::uint64_t version() const { return version_; }

 private:
  double min_step_m_;
  std::size_t max_poses_;
  std::deque<TimedPose> poses_;
  std::uint64_t version_ = 0;
};

}  // namespace lightnav_localization
