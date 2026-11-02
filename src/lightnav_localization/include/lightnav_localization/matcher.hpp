#pragma once

#include <Eigen/Geometry>

#include "lightnav_localization/pcd_io.hpp"

namespace lightnav_localization {

struct MatcherParameters {
  double max_correspondence_distance_m = 1.0;
  int max_iterations = 30;
  double inlier_ratio_min = 0.5;
  double gate_translation_m = 1.0;  // allowed difference between result and prediction
  double gate_rotation_rad = 0.2;
};

struct MatchResult {
  Eigen::Isometry3d target_to_source = Eigen::Isometry3d::Identity();  // pose of source in target
  bool converged = false;
  double inlier_ratio = 0.0;
  Eigen::Matrix<double, 6, 6> information = Eigen::Matrix<double, 6, 6>::Zero();  // GICP Hessian
  bool is_accepted = false;  // converged && inlier ratio && gate vs. the prediction
};

// GICP of `source` against `target`, started from `prediction`. Single-threaded.
// Both clouds need >= 10 points, otherwise the result is not converged.
MatchResult match_scan(const PointList& target, const PointList& source,
                       const Eigen::Isometry3d& prediction, const MatcherParameters& parameters);

}  // namespace lightnav_localization
