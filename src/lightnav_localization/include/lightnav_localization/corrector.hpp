#pragma once

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>

namespace lightnav_localization {

struct CorrectorParameters {
  double slew_mps = 0.2;
  double slew_rps = 0.1;
  double large_correction_threshold_m = 1.0;
};

// Moves the published map->base pose toward the target map->base pose at a limited speed. The
// difference is taken at the base pose (not on T_map_odom), so a yaw correction far from the odom
// origin does not swing the output by (distance x angle).
class Corrector {
 public:
  explicit Corrector(const CorrectorParameters& parameters) : parameters_(parameters) {}

  void reset(const Eigen::Isometry3d& map_to_odom) { applied_ = target_ = map_to_odom; }
  void set_target(const Eigen::Isometry3d& map_to_odom) { target_ = map_to_odom; }

  // `odom_to_base` is the current wheel odometry pose. Non-positive elapsed time does not move.
  const Eigen::Isometry3d& step(double elapsed_sec, const Eigen::Isometry3d& odom_to_base) {
    if (elapsed_sec <= 0.0) return applied_;
    const Eigen::Isometry3d remaining = remaining_at_base(odom_to_base);
    const double distance = remaining.translation().norm();
    const Eigen::AngleAxisd rotation(remaining.rotation());
    const double angle = rotation.angle();
    const double translation_ratio =
        distance > 1e-12 ? std::min(1.0, parameters_.slew_mps * elapsed_sec / distance) : 1.0;
    const double rotation_ratio =
        angle > 1e-12 ? std::min(1.0, parameters_.slew_rps * elapsed_sec / angle) : 1.0;
    Eigen::Isometry3d increment = Eigen::Isometry3d::Identity();
    increment.translation() = translation_ratio * remaining.translation();
    increment.linear() =
        Eigen::AngleAxisd(rotation_ratio * angle, rotation.axis()).toRotationMatrix();
    const Eigen::Isometry3d new_map_to_base = applied_ * odom_to_base * increment;
    applied_ = new_map_to_base * odom_to_base.inverse();
    return applied_;
  }

  const Eigen::Isometry3d& applied() const { return applied_; }
  double pending_correction_m(const Eigen::Isometry3d& odom_to_base) const {
    return remaining_at_base(odom_to_base).translation().norm();
  }
  bool is_large_correction(const Eigen::Isometry3d& odom_to_base) const {
    return pending_correction_m(odom_to_base) > parameters_.large_correction_threshold_m;
  }

 private:
  // Target base pose expressed in the currently published base frame.
  Eigen::Isometry3d remaining_at_base(const Eigen::Isometry3d& odom_to_base) const {
    return (applied_ * odom_to_base).inverse() * (target_ * odom_to_base);
  }

  CorrectorParameters parameters_;
  Eigen::Isometry3d applied_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d target_ = Eigen::Isometry3d::Identity();
};

}  // namespace lightnav_localization
