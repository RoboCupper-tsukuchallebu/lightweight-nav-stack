#include "lightnav_localization/initial_alignment.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace lightnav_localization {
namespace {

bool is_finite_pose(const Eigen::Isometry3d& pose) { return pose.matrix().allFinite(); }

InitialAlignmentResult run_alignment(const PreparedTarget& target, const PointList& scan,
                                     const Eigen::Vector3d& reference_position,
                                     const InitialAlignmentParameters& parameters,
                                     const Eigen::Vector3d& lever_arm_in_base) {
  InitialAlignmentResult result;
  result.map_to_base = Eigen::Isometry3d::Identity();
  result.map_to_base.translation() = reference_position;
  if (!reference_position.allFinite() || !lever_arm_in_base.allFinite()) {
    result.error = "initial position or lever arm is not finite";
    return result;
  }
  if (scan.size() < 10) {
    result.error = "not enough scan points for initial yaw alignment";
    return result;
  }
  if (!(parameters.yaw_step_deg > 0.0) || !(parameters.search_radius_m > 0.0) ||
      !(parameters.inlier_ratio_min > 0.0)) {
    result.error = "invalid initial yaw alignment parameters";
    return result;
  }

  MatcherParameters coarse_parameters;
  coarse_parameters.max_correspondence_distance_m = parameters.coarse_correspondence_m;
  coarse_parameters.max_iterations = parameters.coarse_max_iterations;
  coarse_parameters.inlier_ratio_min = 0.0;
  coarse_parameters.gate_translation_m = std::numeric_limits<double>::max();
  coarse_parameters.gate_rotation_rad = std::numeric_limits<double>::max();

  const int candidate_count =
      std::max(1, static_cast<int>(std::lround(360.0 / parameters.yaw_step_deg)));
  bool has_candidate = false;
  double best_inlier_ratio = -1.0;
  Eigen::Isometry3d best_pose = Eigen::Isometry3d::Identity();
  for (int candidate = 0; candidate < candidate_count; ++candidate) {
    const double yaw = 2.0 * M_PI * static_cast<double>(candidate) /
                       static_cast<double>(candidate_count);
    const Eigen::Matrix3d rotation =
        Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    Eigen::Isometry3d prediction = Eigen::Isometry3d::Identity();
    prediction.translation() = reference_position - rotation * lever_arm_in_base;
    prediction.linear() = rotation;
    const MatchResult match = match_scan(target, scan, prediction, coarse_parameters);
    if (!match.converged || !is_finite_pose(match.target_to_source) ||
        !std::isfinite(match.inlier_ratio)) {
      continue;
    }
    if (!has_candidate || match.inlier_ratio > best_inlier_ratio) {
      has_candidate = true;
      best_inlier_ratio = match.inlier_ratio;
      best_pose = match.target_to_source;
    }
  }
  if (!has_candidate) {
    result.error = "initial yaw coarse match did not converge";
    return result;
  }

  MatcherParameters fine_parameters = coarse_parameters;
  fine_parameters.max_correspondence_distance_m = parameters.fine_correspondence_m;
  fine_parameters.max_iterations = parameters.fine_max_iterations;
  const MatchResult fine = match_scan(target, scan, best_pose, fine_parameters);
  result.inlier_ratio = fine.inlier_ratio;
  if (!fine.converged || !is_finite_pose(fine.target_to_source) ||
      !std::isfinite(fine.inlier_ratio) || fine.inlier_ratio < parameters.inlier_ratio_min) {
    result.error = "initial yaw fine match inlier ratio below the minimum";
    return result;
  }

  result.accepted = true;
  result.map_to_base = fine.target_to_source;
  result.yaw_rad = std::atan2(fine.target_to_source.rotation()(1, 0),
                             fine.target_to_source.rotation()(0, 0));
  return result;
}

}  // namespace

std::optional<PreparedTarget> build_initial_alignment_target(
    const PointList& map_points, const Eigen::Vector3d& reference_position, double radius_m) {
  if (!reference_position.allFinite() || !(radius_m > 0.0)) return std::nullopt;
  PointList cropped_map;
  for (const Eigen::Vector3f& point : map_points) {
    if (!point.allFinite()) continue;
    if ((point.head<2>().cast<double>() - reference_position.head<2>()).norm() <= radius_m) {
      cropped_map.push_back(point);
    }
  }
  if (cropped_map.size() < 10) return std::nullopt;
  PreparedTarget target(cropped_map);
  if (!target.is_valid()) return std::nullopt;
  return target;
}

InitialAlignmentResult align_initial_yaw(const PointList& map_points, const PointList& scan,
                                         const Eigen::Vector3d& reference_position,
                                         const InitialAlignmentParameters& parameters,
                                         const Eigen::Vector3d& lever_arm_in_base) {
  const std::optional<PreparedTarget> target = build_initial_alignment_target(
      map_points, reference_position, parameters.search_radius_m);
  if (!target) {
    InitialAlignmentResult result;
    result.error = "not enough map points near the initial position";
    return result;
  }
  return run_alignment(*target, scan, reference_position, parameters, lever_arm_in_base);
}

InitialAlignmentResult align_initial_yaw(const PreparedTarget& target, const PointList& scan,
                                         const Eigen::Vector3d& reference_position,
                                         const InitialAlignmentParameters& parameters,
                                         const Eigen::Vector3d& lever_arm_in_base) {
  return run_alignment(target, scan, reference_position, parameters, lever_arm_in_base);
}

}  // namespace lightnav_localization
