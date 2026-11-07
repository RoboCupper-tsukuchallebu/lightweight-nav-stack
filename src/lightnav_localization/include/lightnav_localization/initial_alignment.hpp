#pragma once

#include <Eigen/Geometry>
#include <optional>
#include <string>

#include "lightnav_localization/matcher.hpp"
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
  // Refined map->base pose from the fine match. Both the yaw and the translation are used by the
  // node: the reference position may be a GPS antenna position (see `lever_arm_in_base`).
  Eigen::Isometry3d map_to_base = Eigen::Isometry3d::Identity();
  std::string error;
};

// Crops the map around `reference_position` and builds the reusable GICP target for the initial
// alignment. Returns nullopt when there are not enough map points or the target cannot be built.
std::optional<PreparedTarget> build_initial_alignment_target(
    const PointList& map_points, const Eigen::Vector3d& reference_position, double radius_m);

// Coarse yaw search at `yaw_step_deg`, then a fine match from the best coarse candidate. `scan` is
// in the same frame as the map after deskewing (base frame). `reference_position` is the initial
// position in the map frame; when it is an antenna position, `lever_arm_in_base` is the antenna
// position in the base frame and is removed for every yaw candidate before matching.
InitialAlignmentResult align_initial_yaw(const PointList& map_points, const PointList& scan,
                                         const Eigen::Vector3d& reference_position,
                                         const InitialAlignmentParameters& parameters,
                                         const Eigen::Vector3d& lever_arm_in_base =
                                             Eigen::Vector3d::Zero());

// Same search against a prebuilt target (the node caches this while the initial position does not
// change).
InitialAlignmentResult align_initial_yaw(const PreparedTarget& target, const PointList& scan,
                                         const Eigen::Vector3d& reference_position,
                                         const InitialAlignmentParameters& parameters,
                                         const Eigen::Vector3d& lever_arm_in_base =
                                             Eigen::Vector3d::Zero());

}  // namespace lightnav_localization
