#include "lightnav_localization/deskew.hpp"

namespace lightnav_localization {

std::optional<PointList> deskew_scan(const std::vector<TimedPoint>& scan, double header_stamp_sec,
                                     const PoseBuffer& odom_to_base,
                                     const Eigen::Isometry3d& base_to_lidar) {
  const auto reference_pose = odom_to_base.interpolate(header_stamp_sec);
  if (!reference_pose) return std::nullopt;
  const Eigen::Isometry3d base_at_reference_from_odom = reference_pose->inverse();

  PointList corrected;
  corrected.reserve(scan.size());
  for (const TimedPoint& point : scan) {
    const auto point_pose = odom_to_base.interpolate(header_stamp_sec + point.time_offset_sec);
    if (!point_pose) return std::nullopt;
    const Eigen::Vector3d in_base_at_point_time = base_to_lidar * point.position.cast<double>();
    corrected.push_back(
        (base_at_reference_from_odom * (*point_pose) * in_base_at_point_time).cast<float>());
  }
  return corrected;
}

}  // namespace lightnav_localization
