#pragma once

#include <Eigen/Geometry>
#include <string>

#include "lightnav_localization/pcd_io.hpp"

namespace lightnav_localization {

// Parameters of the localization-mode initial yaw search (Issue #2 section 8).
struct InitialAlignmentParameters {
  int scan_count = 3;
  double yaw_step_deg = 15.0;
  double coarse_correspondence_m = 2.0;
  double fine_correspondence_m = 0.5;
  double inlier_ratio_min = 0.6;
  double search_radius_m = 100.0;
  int coarse_max_iterations = 10;
  int fine_max_iterations = 30;
};

struct InitialAlignmentResult {
  bool accepted = false;
  double yaw_rad = 0.0;
  double inlier_ratio = 0.0;
  // Pose of the accumulated scan in the map frame when accepted; translation is the prediction
  // passed in (the initial position), only the yaw is searched here.
  Eigen::Isometry3d map_to_base = Eigen::Isometry3d::Identity();
  std::string error;
};

// Coarse yaw search at `yaw_step_deg` around `position`, then a fine match from the best coarse
// candidate. `scan` is in the same frame as the map after deskewing (base frame), and `position`
// is the initial base position in the map frame. Pure logic; no ROS types.
InitialAlignmentResult align_initial_yaw(const PointList& map_points, const PointList& scan,
                                         const Eigen::Vector3d& position,
                                         const InitialAlignmentParameters& parameters);

}  // namespace lightnav_localization
