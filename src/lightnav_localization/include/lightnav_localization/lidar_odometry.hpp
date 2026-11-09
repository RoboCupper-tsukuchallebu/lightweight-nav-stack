#pragma once

#include <Eigen/Geometry>
#include <deque>
#include <vector>

#include "lightnav_localization/matcher.hpp"
#include "lightnav_localization/pcd_io.hpp"

namespace lightnav_localization {

struct LidarOdometryParameters {
  int submap_keyframes = 10;
  double submap_voxel_size_m = 0.25;
  int max_iterations = 20;
  double inlier_ratio_min = 0.6;
  double max_correspondence_distance_m = 1.0;
  double gate_translation_m = 0.3;
  double gate_translation_ratio = 0.2;
  double gate_yaw_rad = 0.1;
  double slip_gate_yaw_rad = 0.35;
};

// Scan-to-submap odometry. Keyframe clouds are stored in the base frame with their graph pose, so
// the submap can be rebuilt after the graph moved the poses.
class LidarOdometry {
 public:
  explicit LidarOdometry(const LidarOdometryParameters& parameters) : parameters_(parameters) {}

  // Adds a keyframe and rebuilds the sub-map (oldest keyframe dropped beyond submap_keyframes).
  void add_keyframe(const Eigen::Isometry3d& map_to_base, const PointList& cloud_in_base);
  // Updates stored poses after optimization (index 0 = oldest kept keyframe).
  void update_keyframe_poses(const std::vector<Eigen::Isometry3d>& map_to_base_poses);
  void rebuild_submap();
  // Drops the `count` oldest keyframes (all of them when count exceeds the kept ones) and rebuilds
  // the sub-map. Used when the graph no longer holds the poses of those keyframes.
  void drop_oldest_keyframes(std::size_t count);

  // Matches `scan_in_base` against the sub-map from `prediction` (map_to_base). The gate allows
  // gate_translation_m + gate_translation_ratio * moved_since_keyframe_m.
  MatchResult match(const PointList& scan_in_base, const Eigen::Isometry3d& prediction,
                    double moved_since_keyframe_m) const;

  bool has_submap() const { return submap_.is_valid(); }
  std::size_t keyframe_count() const { return keyframes_.size(); }
  // map_to_base of the kept keyframes, oldest first.
  std::vector<Eigen::Isometry3d> keyframe_poses() const;

 private:
  struct Keyframe {
    Eigen::Isometry3d map_to_base;
    PointList cloud_in_base;
  };
  LidarOdometryParameters parameters_;
  std::deque<Keyframe> keyframes_;
  PreparedTarget submap_;
};

}  // namespace lightnav_localization
