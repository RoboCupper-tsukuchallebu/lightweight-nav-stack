#pragma once

#include <Eigen/Geometry>
#include <map>
#include <memory>
#include <optional>

namespace lightnav_localization {

inline Eigen::Matrix<double, 6, 6> diagonal_information(double sigma_m, double sigma_rad) {
  Eigen::Matrix<double, 6, 6> information = Eigen::Matrix<double, 6, 6>::Zero();
  information.topLeftCorner<3, 3>().diagonal().setConstant(1.0 / (sigma_rad * sigma_rad));
  information.bottomRightCorner<3, 3>().diagonal().setConstant(1.0 / (sigma_m * sigma_m));
  return information;
}

struct PoseGraphParameters {
  double window_s = 30.0;
  double huber_k = 1.0;
  double wheel_sigma_translation_per_m = 0.02;
  double wheel_sigma_yaw_per_rad = 0.1;
  double wheel_sigma_yaw_per_m = 0.01;
  double planar_sigma_z_per_m = 0.05;
  double planar_sigma_tilt_rad = 0.05;
  double sigma_floor_m = 0.01;  // keeps a stationary wheel factor from becoming a hard constraint
  double sigma_floor_rad = 0.005;
};

// Pose3 node chain inside gtsam::IncrementalFixedLagSmoother. GTSAM types stay in the .cpp.
class PoseGraph {
 public:
  explicit PoseGraph(const PoseGraphParameters& parameters);
  ~PoseGraph();
  PoseGraph(PoseGraph&&) noexcept;
  PoseGraph& operator=(PoseGraph&&) noexcept;

  // First node. roll/pitch ~ 0 +- 0.1 rad and z ~ 0 +- 1 m are added as in the spec.
  int add_first_node(double stamp_sec, const Eigen::Isometry3d& pose, double sigma_xy_m,
                     double sigma_yaw_rad);
  // Wheel odometry: relative motion (2D lifted to 3D) from the previous node to the new one.
  int add_node_with_wheel_factor(double stamp_sec, const Eigen::Isometry3d& guess,
                                 const Eigen::Isometry3d& wheel_relative);
  // `information` is 6x6 in GTSAM Pose3 order (rotation xyz, then translation xyz).
  void add_lidar_odometry_factor(int from_index, int to_index, const Eigen::Isometry3d& relative,
                                 const Eigen::Matrix<double, 6, 6>& information);
  void add_map_factor(int index, const Eigen::Isometry3d& pose,
                      const Eigen::Matrix<double, 6, 6>& information);
  void add_gps_factor(int index, const Eigen::Vector3d& position, const Eigen::Vector3d& sigma_m);
  // Optimizes. Returns false if GTSAM threw; the caller then rebuilds from the current estimate.
  bool optimize();

  // Estimate of a node; nullopt after it left the window.
  std::optional<Eigen::Isometry3d> pose(int index) const;
  // sqrt of the (x, y) marginal variances and the yaw standard deviation.
  std::optional<Eigen::Vector3d> position_and_yaw_std(int index) const;
  int latest_index() const { return latest_index_; }

 private:
  struct Implementation;
  std::unique_ptr<Implementation> implementation_;
  int latest_index_ = -1;
};

}  // namespace lightnav_localization
