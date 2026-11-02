#pragma once

#include <Eigen/Geometry>
#include <memory>

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
  int iterations = 0;
  double inlier_ratio = 0.0;
  Eigen::Matrix<double, 6, 6> information = Eigen::Matrix<double, 6, 6>::Zero();  // GICP Hessian
  bool is_accepted = false;  // converged && inlier ratio && gate vs. the prediction
};

// Target cloud with covariances and KdTree, built once and reused by many match_scan() calls
// (sub-map rebuilds and scan matching run on different periods). Immutable after construction.
class PreparedTarget {
 public:
  PreparedTarget() = default;
  explicit PreparedTarget(const PointList& points);
  bool is_valid() const { return implementation_ != nullptr; }

  struct Implementation;  // holds small_gicp types; opaque to users of this header

 private:
  friend MatchResult match_scan(const PreparedTarget&, const PointList&, const Eigen::Isometry3d&,
                                const MatcherParameters&);
  std::shared_ptr<const Implementation> implementation_;
};

// GICP of `source` against a prepared target, started from `prediction`. Single-threaded.
// Source covariances are estimated here. Needs >= 10 points on both sides, otherwise the result
// is not converged.
MatchResult match_scan(const PreparedTarget& target, const PointList& source,
                       const Eigen::Isometry3d& prediction, const MatcherParameters& parameters);

}  // namespace lightnav_localization
