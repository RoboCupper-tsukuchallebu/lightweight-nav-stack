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
// Only x, y and yaw are slewed (the 2D pose the downstream adapter projects to). z, roll and pitch
// follow the target immediately (tentative decision, see the PR).
class Corrector {
 public:
  explicit Corrector(const CorrectorParameters& parameters) : parameters_(parameters) {}

  void reset(const Eigen::Isometry3d& map_to_odom) { applied_ = target_ = map_to_odom; }
  void set_target(const Eigen::Isometry3d& map_to_odom) { target_ = map_to_odom; }
  void set_planar_target(const Eigen::Isometry3d& map_to_odom) {
    const double yaw_difference = yaw_of(map_to_odom) - yaw_of(target_);
    target_.translation().head<2>() = map_to_odom.translation().head<2>();
    target_.linear() =
        Eigen::AngleAxisd(yaw_difference, Eigen::Vector3d::UnitZ()).toRotationMatrix() *
        target_.rotation();
  }
  const Eigen::Isometry3d& target() const { return target_; }

  // `odom_to_base` is the current wheel odometry pose. Non-positive elapsed time does not move.
  const Eigen::Isometry3d& step(double elapsed_sec, const Eigen::Isometry3d& odom_to_base) {
    if (elapsed_sec <= 0.0) return applied_;
    const Eigen::Isometry3d current = applied_ * odom_to_base;
    const Eigen::Isometry3d goal = target_ * odom_to_base;
    const Eigen::Vector2d offset = (goal.translation() - current.translation()).head<2>();
    const double distance = offset.norm();
    const double translation_ratio =
        distance > 1e-12 ? std::min(1.0, parameters_.slew_mps * elapsed_sec / distance) : 1.0;
    const double yaw_difference = wrap(yaw_of(goal) - yaw_of(current));
    const double yaw_step = std::clamp(yaw_difference, -parameters_.slew_rps * elapsed_sec,
                                       parameters_.slew_rps * elapsed_sec);
    Eigen::Isometry3d next = goal;  // z, roll, pitch from the goal
    next.translation().head<2>() = current.translation().head<2>() + translation_ratio * offset;
    // goal rotation = Rz(yaw_goal) * tilt; keep the tilt, set the yaw to the slewed value.
    const double yaw_next = yaw_of(current) + yaw_step;
    next.linear() = Eigen::AngleAxisd(yaw_next - yaw_of(goal), Eigen::Vector3d::UnitZ()).toRotationMatrix() *
                    goal.rotation();
    applied_ = next * odom_to_base.inverse();
    return applied_;
  }

  const Eigen::Isometry3d& applied() const { return applied_; }
  // Horizontal distance still to be applied at the base.
  double pending_correction_m(const Eigen::Isometry3d& odom_to_base) const {
    return ((target_ * odom_to_base).translation() - (applied_ * odom_to_base).translation()).head<2>().norm();
  }
  bool is_large_correction(const Eigen::Isometry3d& odom_to_base) const {
    return pending_correction_m(odom_to_base) > parameters_.large_correction_threshold_m;
  }

 private:
  static double yaw_of(const Eigen::Isometry3d& pose) {
    return std::atan2(pose.rotation()(1, 0), pose.rotation()(0, 0));
  }
  static double wrap(double angle) { return std::atan2(std::sin(angle), std::cos(angle)); }

  CorrectorParameters parameters_;
  Eigen::Isometry3d applied_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d target_ = Eigen::Isometry3d::Identity();
};

}  // namespace lightnav_localization
