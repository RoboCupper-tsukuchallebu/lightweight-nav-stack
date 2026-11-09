#include "lightnav_localization/lidar_odometry.hpp"

#include <algorithm>
#include <cmath>

#include "lightnav_localization/downsample.hpp"

namespace lightnav_localization {

void LidarOdometry::add_keyframe(const Eigen::Isometry3d& map_to_base,
                                 const PointList& cloud_in_base) {
  keyframes_.push_back({map_to_base, cloud_in_base});
  while (static_cast<int>(keyframes_.size()) > parameters_.submap_keyframes) keyframes_.pop_front();
  rebuild_submap();
}

void LidarOdometry::update_keyframe_poses(const std::vector<Eigen::Isometry3d>& map_to_base_poses) {
  const std::size_t count = std::min(map_to_base_poses.size(), keyframes_.size());
  for (std::size_t offset = 0; offset < count; ++offset) {
    keyframes_[keyframes_.size() - count + offset].map_to_base =
        map_to_base_poses[map_to_base_poses.size() - count + offset];
  }
}

void LidarOdometry::drop_oldest_keyframes(std::size_t count) {
  if (count == 0) return;
  keyframes_.erase(keyframes_.begin(),
                   keyframes_.begin() + static_cast<std::ptrdiff_t>(std::min(count, keyframes_.size())));
  rebuild_submap();
}

std::vector<Eigen::Isometry3d> LidarOdometry::keyframe_poses() const {
  std::vector<Eigen::Isometry3d> poses;
  poses.reserve(keyframes_.size());
  for (const Keyframe& keyframe : keyframes_) poses.push_back(keyframe.map_to_base);
  return poses;
}

void LidarOdometry::rebuild_submap() {
  PointList merged;
  for (const Keyframe& keyframe : keyframes_) {
    for (const auto& point : keyframe.cloud_in_base) merged.push_back(keyframe.map_to_base.cast<float>() * point);
  }
  submap_ = PreparedTarget(downsample_voxel(merged, parameters_.submap_voxel_size_m));
}

MatchResult LidarOdometry::match(const PointList& scan_in_base, const Eigen::Isometry3d& prediction,
                                 double moved_since_keyframe_m, double wheel_yaw_rate_rad_s) const {
  MatcherParameters matcher;
  matcher.max_correspondence_distance_m = parameters_.max_correspondence_distance_m;
  matcher.max_iterations = parameters_.max_iterations;
  matcher.inlier_ratio_min = parameters_.inlier_ratio_min;
  matcher.gate_translation_m =
      parameters_.gate_translation_m + parameters_.gate_translation_ratio * moved_since_keyframe_m;
  matcher.gate_rotation_rad = M_PI;  // the Issue gates yaw only; roll/pitch come from this match
  MatchResult result = match_scan(submap_, scan_in_base, prediction, matcher);
  const Eigen::Matrix3d difference = prediction.rotation().transpose() * result.target_to_source.rotation();
  const double yaw_difference = std::abs(std::atan2(difference(1, 0), difference(0, 0)));
  const double yaw_gate = std::abs(wheel_yaw_rate_rad_s) < parameters_.slip_wheel_yaw_rate_rad_s
      ? std::max(parameters_.gate_yaw_rad, parameters_.slip_gate_yaw_rad) : parameters_.gate_yaw_rad;
  result.is_accepted = result.is_accepted && yaw_difference <= yaw_gate;
  return result;
}

}  // namespace lightnav_localization
