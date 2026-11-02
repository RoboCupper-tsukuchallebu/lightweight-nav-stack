#pragma once

#include <Eigen/Eigenvalues>
#include <algorithm>

namespace lightnav_localization {

struct InformationLimits {
  double covariance_scale = 1.0;  // information = hessian / scale
  double sigma_floor_m = 0.02;
  double sigma_floor_rad = 0.005;
  double sigma_max_m = 10.0;  // a degenerate direction is weakened to this sigma, not removed
  double sigma_max_rad = 1.0;
};

// Pose factor information from a small_gicp Hessian. small_gicp's twist is (rotation xyz,
// translation xyz) applied as T * exp(delta) (see third_party/small_gicp util/lie.hpp and
// gicp_factor.hpp), which is the same order and perturbation side as GTSAM Pose3. The rotation and
// translation 3x3 blocks are eigen-decomposed separately (cross terms are dropped) and each
// eigenvalue is clamped to [1/sigma_max^2, 1/sigma_floor^2]. A direction the scan cannot observe
// (a corridor axis) has a tiny eigenvalue and so keeps a weak factor instead of a confident one.
inline Eigen::Matrix<double, 6, 6> information_from_hessian(const Eigen::Matrix<double, 6, 6>& hessian,
                                                            const InformationLimits& limits) {
  Eigen::Matrix<double, 6, 6> information = Eigen::Matrix<double, 6, 6>::Zero();
  const auto clamp_block = [&](int offset, double sigma_floor, double sigma_max) {
    const Eigen::Matrix3d block = hessian.block<3, 3>(offset, offset) / limits.covariance_scale;
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(0.5 * (block + block.transpose()));
    Eigen::Vector3d eigenvalues = solver.eigenvalues();
    for (int axis = 0; axis < 3; ++axis) {
      eigenvalues[axis] = std::clamp(eigenvalues[axis], 1.0 / (sigma_max * sigma_max),
                                     1.0 / (sigma_floor * sigma_floor));
    }
    information.block<3, 3>(offset, offset) =
        solver.eigenvectors() * eigenvalues.asDiagonal() * solver.eigenvectors().transpose();
  };
  clamp_block(0, limits.sigma_floor_rad, limits.sigma_max_rad);
  clamp_block(3, limits.sigma_floor_m, limits.sigma_max_m);
  return information;
}

}  // namespace lightnav_localization
