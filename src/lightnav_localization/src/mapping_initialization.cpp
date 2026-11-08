#include "lightnav_localization/mapping_initialization.hpp"

#include <cmath>

namespace lightnav_localization {

double mapping_initial_yaw(const Eigen::Vector3d& gps_displacement_m,
                           const Eigen::Isometry3d& wheel_delta) {
  const double gps_heading = std::atan2(gps_displacement_m.y(), gps_displacement_m.x());
  const double wheel_chord =
      std::atan2(wheel_delta.translation().y(), wheel_delta.translation().x());
  const double wheel_rotation =
      std::atan2(wheel_delta.rotation()(1, 0), wheel_delta.rotation()(0, 0));
  return gps_heading - wheel_chord + wheel_rotation;
}

}  // namespace lightnav_localization
