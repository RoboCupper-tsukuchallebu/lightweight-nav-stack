#pragma once

#include <Eigen/Geometry>

namespace lightnav_localization {

// Initial mapping yaw from one straight-ish displacement:
//   gps_displacement_m is the map-frame displacement of the GPS antenna (or base) and
//   wheel_delta is the wheel-odometry pose of the current sample in the reference frame.
// The displacement gives the chord heading; adding the accumulated wheel rotation makes the
// result the current heading, so a turn during init_travel_m is not lost.
double mapping_initial_yaw(const Eigen::Vector3d& gps_displacement_m,
                           const Eigen::Isometry3d& wheel_delta);

}  // namespace lightnav_localization
