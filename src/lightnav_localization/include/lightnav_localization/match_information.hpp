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
  // A direction whose Hessian eigenvalue is below this fraction of the largest one is degenerate.
  // small_gicp normalizes every point covariance to eigenvalues (1e-3, 1, 1), so a point constrains
  // its normal ~1000x more than the surface directions. The absolute H is therefore not an
  // information measure (a sliding direction still has H ~ 0.5 N); the ratio is.
  double degeneracy_ratio = 0.01;
};

// Pose factor information from a small_gicp Hessian. small_gicp's twist is (rotation xyz,
// translation xyz) applied as T * exp(delta) (third_party/small_gicp util/lie.hpp,
// gicp_factor.hpp), the same order and perturbation side as GTSAM Pose3.
// The 6x6 matrix is made dimensionless with the floor sigmas, eigen-decomposed as a whole (so
// mixed rotation/translation degeneracies are found), degenerate eigen-directions are set to the
// weak level (sigma_max) and the rest are capped at the floor sigma.
inline Eigen::Matrix<double, 6, 6> information_from_hessian(const Eigen::Matrix<double, 6, 6>& hessian,
                                                            const InformationLimits& limits) {
  Eigen::Matrix<double, 6, 1> floor_sigma;
  floor_sigma << Eigen::Vector3d::Constant(limits.sigma_floor_rad),
      Eigen::Vector3d::Constant(limits.sigma_floor_m);
  const Eigen::Matrix<double, 6, 6> scaling = floor_sigma.asDiagonal();
  const Eigen::Matrix<double, 6, 6> normalized = scaling * (hessian / limits.covariance_scale) * scaling;
  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> solver(
      0.5 * (normalized + normalized.transpose()));
  const double weak_level = std::min(std::pow(limits.sigma_floor_m / limits.sigma_max_m, 2),
                                     std::pow(limits.sigma_floor_rad / limits.sigma_max_rad, 2));
  const double largest = solver.eigenvalues().maxCoeff();
  Eigen::Matrix<double, 6, 1> eigenvalues = solver.eigenvalues();
  for (int axis = 0; axis < 6; ++axis) {
    const bool is_degenerate = eigenvalues[axis] < limits.degeneracy_ratio * largest;
    eigenvalues[axis] = is_degenerate ? weak_level : std::clamp(eigenvalues[axis], weak_level, 1.0);
  }
  const Eigen::Matrix<double, 6, 6> clamped =
      solver.eigenvectors() * eigenvalues.asDiagonal() * solver.eigenvectors().transpose();
  const Eigen::Matrix<double, 6, 6> inverse_scaling = floor_sigma.cwiseInverse().asDiagonal();
  return inverse_scaling * clamped * inverse_scaling;
}

}  // namespace lightnav_localization
