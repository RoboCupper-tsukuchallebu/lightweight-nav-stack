#include "lightnav_localization/visualization.hpp"

#include <cmath>

namespace lightnav_localization {
namespace {

constexpr int kVoxelIndexBits = 21;
constexpr std::int64_t kVoxelIndexOffset = std::int64_t{1} << (kVoxelIndexBits - 1);
constexpr std::uint64_t kVoxelIndexMask = (std::uint64_t{1} << kVoxelIndexBits) - 1;

std::uint64_t pack_voxel_index(double coordinate_m, double voxel_size_m) {
  const auto index = static_cast<std::int64_t>(std::floor(coordinate_m / voxel_size_m));
  return static_cast<std::uint64_t>(index + kVoxelIndexOffset) & kVoxelIndexMask;
}

}  // namespace

VoxelMapAccumulator::VoxelMapAccumulator(double voxel_size_m, std::size_t max_points)
    : voxel_size_m_(voxel_size_m), max_points_(max_points) {}

void VoxelMapAccumulator::add(const PointList& points_in_map) {
  const std::size_t previous_count = points_.size();
  for (const Eigen::Vector3f& point : points_in_map) {
    if (points_.size() >= max_points_) break;
    if (!point.allFinite()) continue;
    const std::uint64_t key = (pack_voxel_index(point.x(), voxel_size_m_) << (2 * kVoxelIndexBits)) |
                              (pack_voxel_index(point.y(), voxel_size_m_) << kVoxelIndexBits) |
                              pack_voxel_index(point.z(), voxel_size_m_);
    if (occupied_voxels_.insert(key).second) points_.push_back(point);
  }
  if (points_.size() != previous_count) ++version_;
}

void VoxelMapAccumulator::clear() {
  occupied_voxels_.clear();
  points_.clear();
  ++version_;
}

PathRecorder::PathRecorder(double min_step_m, std::size_t max_poses)
    : min_step_m_(min_step_m), max_poses_(max_poses) {}

void PathRecorder::add(double stamp_sec, const Eigen::Isometry3d& map_to_base) {
  if (!map_to_base.matrix().allFinite()) return;
  if (!poses_.empty()) {
    const Eigen::Vector3d step = map_to_base.translation() - poses_.back().map_to_base.translation();
    if (step.head<2>().norm() < min_step_m_) return;
  }
  poses_.push_back(TimedPose{stamp_sec, map_to_base});
  while (poses_.size() > max_poses_) poses_.pop_front();
  ++version_;
}

void PathRecorder::clear() {
  poses_.clear();
  ++version_;
}

}  // namespace lightnav_localization
