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

// Applies T_map_odom toward a target at a limited speed so the output never jumps.
class Corrector {
 public:
  explicit Corrector(const CorrectorParameters& parameters) : parameters_(parameters) {}

  void reset(const Eigen::Isometry3d& map_to_odom) { applied_ = target_ = map_to_odom; }
  void set_target(const Eigen::Isometry3d& map_to_odom) { target_ = map_to_odom; }

  // Moves the applied transform toward the target by at most slew * elapsed_sec.
  const Eigen::Isometry3d& step(double elapsed_sec) {
    const Eigen::Isometry3d remaining = applied_.inverse() * target_;
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
    applied_ = applied_ * increment;
    return applied_;
  }

  const Eigen::Isometry3d& applied() const { return applied_; }
  double pending_correction_m() const { return (applied_.inverse() * target_).translation().norm(); }
  bool is_large_correction() const {
    return pending_correction_m() > parameters_.large_correction_threshold_m;
  }

 private:
  CorrectorParameters parameters_;
  Eigen::Isometry3d applied_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d target_ = Eigen::Isometry3d::Identity();
};

}  // namespace lightnav_localization
