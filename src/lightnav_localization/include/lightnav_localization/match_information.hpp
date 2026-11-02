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
  // A direction whose Hessian eigenvalue is below this fraction of its block's largest one is
  // degenerate. Measured on an open corridor at 30-60 m range: sliding 0.022 vs the next axis 0.2.
  // small_gicp normalizes every point covariance to eigenvalues (1e-3, 1, 1), so a point constrains
  // its normal ~1000x more than the surface directions. The absolute H is therefore not an
  // information measure (a sliding direction still has H ~ 0.5 N); the ratio is.
  double degeneracy_ratio = 0.05;           // translation block
  double rotation_degeneracy_ratio = 0.0;  // rotation is constrained by the whole scene (lever arm); not tested
};

// Pose factor information from a small_gicp Hessian. small_gicp's twist is (rotation xyz,
// translation xyz) applied as T * exp(delta) (third_party/small_gicp util/lie.hpp,
// gicp_factor.hpp), the same order and perturbation side as GTSAM Pose3.
// The translation and rotation 3x3 blocks are decided SEPARATELY: within one block the units are
// the same, so the eigenvalue ratio does not depend on the scan range (a 6x6 mix made the result
// depend on lever arms, i.e. on distance). In each block a direction below degeneracy_ratio times
// the block's largest eigenvalue is set to sigma_max, the others are capped at the floor sigma.
// Rotation/translation cross terms are dropped.
inline Eigen::Matrix<double, 6, 6> information_from_hessian(const Eigen::Matrix<double, 6, 6>& hessian,
                                                            const InformationLimits& limits) {
  Eigen::Matrix<double, 6, 6> information = Eigen::Matrix<double, 6, 6>::Zero();
  const auto process_block = [&](int offset, double sigma_floor, double sigma_max, double ratio) {
    const Eigen::Matrix3d block = hessian.block<3, 3>(offset, offset) / limits.covariance_scale;
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(0.5 * (block + block.transpose()));
    Eigen::Vector3d eigenvalues = solver.eigenvalues();
    const double largest = eigenvalues.maxCoeff();
    const double strongest = 1.0 / (sigma_floor * sigma_floor);
    const double weakest = 1.0 / (sigma_max * sigma_max);
    for (int axis = 0; axis < 3; ++axis) {
      const bool is_degenerate = eigenvalues[axis] < ratio * largest;
      eigenvalues[axis] = is_degenerate ? weakest : std::clamp(eigenvalues[axis], weakest, strongest);
    }
    information.block<3, 3>(offset, offset) =
        solver.eigenvectors() * eigenvalues.asDiagonal() * solver.eigenvectors().transpose();
  };
  process_block(0, limits.sigma_floor_rad, limits.sigma_max_rad, limits.rotation_degeneracy_ratio);
  process_block(3, limits.sigma_floor_m, limits.sigma_max_m, limits.degeneracy_ratio);
  return information;
}

}  // namespace lightnav_localization
