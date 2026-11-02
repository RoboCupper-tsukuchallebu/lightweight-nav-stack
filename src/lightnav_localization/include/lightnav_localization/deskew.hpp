#pragma once

#include <Eigen/Geometry>
#include <optional>
#include <vector>

#include "lightnav_localization/pcd_io.hpp"
#include "lightnav_localization/pose_buffer.hpp"

namespace lightnav_localization {

struct TimedPoint {
  Eigen::Vector3f position;  // LiDAR frame
  float time_offset_sec;     // relative to header.stamp; may be negative or positive
};

// Moves every point to the base_link frame at t_ref = header_stamp_sec.
// The pose used for a point is odom_to_base at (header_stamp_sec + time_offset_sec), so the sign of
// time_offset_sec is never assumed. Returns nullopt (scan must be dropped) if the odometry buffer
// does not cover one of the point times.
std::optional<PointList> deskew_scan(const std::vector<TimedPoint>& scan, double header_stamp_sec,
                                     const PoseBuffer& odom_to_base,
                                     const Eigen::Isometry3d& base_to_lidar);

}  // namespace lightnav_localization
