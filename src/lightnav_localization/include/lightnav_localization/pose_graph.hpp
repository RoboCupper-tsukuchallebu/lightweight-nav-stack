#pragma once

#include <Eigen/Geometry>
#include <map>
#include <memory>
#include <string>
#include <optional>
#include <vector>

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
  double wheel_yaw_slip_threshold_rad = 0.1;
  double wheel_sigma_yaw_per_rad = 0.1;
  double wheel_sigma_yaw_per_m = 0.01;
  std::string wheel_robust_kernel = "huber";
  double wheel_robust_k = 8.0;
  double planar_sigma_z_per_m = 0.05;
  double planar_sigma_tilt_rad = 0.05;
  double sigma_floor_m = 0.01;  // keeps a stationary wheel factor from becoming a hard constraint
  double sigma_floor_rad = 0.005;
  // Keeps every factor and initial value so that mapping can batch-optimize the full graph.
  // Off by default: localization keeps the fixed-lag window only.
  bool record_history = false;
};

double wheel_yaw_sigma(const PoseGraphParameters& parameters, const Eigen::Isometry3d& wheel_relative,
                       double yaw_disagreement_rad);

struct BatchPose {
  int index = 0;
  double stamp_sec = 0.0;
  Eigen::Isometry3d map_to_base = Eigen::Isometry3d::Identity();
  std::optional<Eigen::Vector3d> position_and_yaw_std;
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
  // Mapping recovery: replaces the fixed-lag window with a new node at `pose`, keeping the
  // recorded factor history (when enabled) and continuing with a non-reused node index. A weak
  // between factor connects `previous_pose` (the last estimate) to the new node.
  int restart_window(double stamp_sec, const Eigen::Isometry3d& pose,
                     const Eigen::Isometry3d& previous_pose, double sigma_xy_m,
                     double sigma_yaw_rad);
  // Wheel odometry: relative motion (2D lifted to 3D) from the previous node to the new one.
  int add_node_with_wheel_factor(double stamp_sec, const Eigen::Isometry3d& guess,
                                 const Eigen::Isometry3d& wheel_relative, double yaw_disagreement_rad = 0.0);
  // `information` is 6x6 in GTSAM Pose3 order (rotation xyz, then translation xyz).
  void add_lidar_odometry_factor(int from_index, int to_index, const Eigen::Isometry3d& relative,
                                 const Eigen::Matrix<double, 6, 6>& information);
  void add_map_factor(int index, const Eigen::Isometry3d& pose,
                      const Eigen::Matrix<double, 6, 6>& information);
  // Uses only map-frame x/y and sigma x/y; altitude is intentionally ignored.
  void add_gps_factor(int index, const Eigen::Vector3d& position, const Eigen::Vector3d& sigma_m);
  // Optimizes. Returns false if GTSAM threw; the caller then rebuilds from the current estimate.
  bool optimize();
  // Batch re-optimization of every retained factor (record_history must be on). Fills batch_poses().
  bool batch_optimize(int max_iterations);
  const std::vector<BatchPose>& batch_poses() const;

  // Estimate of a node; nullopt after it left the window.
  std::optional<Eigen::Isometry3d> pose(int index) const;
  // sqrt of map-frame (x, y) marginal variances and the local yaw standard deviation.
  std::optional<Eigen::Vector3d> position_and_yaw_std(int index) const;
  int latest_index() const { return latest_index_; }
  // Message of the last exception caught in optimize()/covariance (empty if none).
  const std::string& last_error() const { return last_error_; }
  // Test seam: the next optimize() fails as if GTSAM had thrown.
  void inject_failure_once() { should_fail_once_ = true; }

 private:
  struct Implementation;
  std::unique_ptr<Implementation> implementation_;
  int latest_index_ = -1;
  std::string last_error_;
  bool should_fail_once_ = false;
  mutable std::string covariance_error_;
};

}  // namespace lightnav_localization
