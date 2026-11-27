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
  double wheel_yaw_slip_threshold_rad = 0.0;
  double wheel_sigma_yaw_per_rad = 0.1;
  double wheel_sigma_yaw_per_m = 0.01;
  std::string wheel_robust_kernel = "huber";
  double wheel_robust_k = 8.0;
  double planar_sigma_z_per_m = 0.05;
  double planar_sigma_tilt_rad = 0.05;
  // Absolute references for a ground vehicle; 0 disables each. The planar terms above only limit the
  // change between two nodes (a random walk), so roll, pitch and height drift without bound when
  // nothing observes them (no IMU, GPS altitude unused). These pull every node towards level and
  // towards the height of the first node.
  double absolute_sigma_tilt_rad = 0.0;
  double absolute_sigma_z_m = 0.0;
  double sigma_floor_m = 0.01;  // keeps a stationary wheel factor from becoming a hard constraint
  double sigma_floor_rad = 0.005;
  // Keeps every factor and initial value so that mapping can batch-optimize the full graph.
  // Off by default: localization keeps the fixed-lag window only.
  bool record_history = false;
  // Gyro yaw rotation factor (add_rotation_factor / gyro_yaw_sigma). The white noise is larger than a
  // bench measurement (about 3.4e-5) to cover vibration; the bias sigma is what is left of the
  // estimated bias; the scale sigma is a sensitivity error proportional to the rotation.
  double gyro_noise_density_rad_s_sqrt_hz = 1.0e-3;
  double gyro_bias_sigma_rad_s = 5.0e-4;
  double gyro_scale_sigma = 0.01;
};

double wheel_yaw_sigma(const PoseGraphParameters& parameters, const Eigen::Isometry3d& wheel_relative,
                       double yaw_disagreement_rad);

// Standard deviation of the yaw change measured by the gyro over `interval_sec`:
// max(sigma_floor_rad, sqrt(noise^2 T + (bias T)^2 + (scale |dyaw|)^2 + uncovered^2)).
// `uncovered_wheel_yaw_rad` is the wheel yaw change over the part of the interval the gyro did not cover.
// Throws std::invalid_argument when an argument is not finite or `interval_sec` is negative.
double gyro_yaw_sigma(const PoseGraphParameters& parameters, double interval_sec, double yaw_delta_rad,
                      double uncovered_wheel_yaw_rad);

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
  // `minimum_yaw_sigma_rad` raises the yaw standard deviation of the wheel factor (0 = unchanged): used
  // so that a wheel yaw that slipped does not pull against a gyro rotation factor.
  int add_node_with_wheel_factor(double stamp_sec, const Eigen::Isometry3d& guess,
                                 const Eigen::Isometry3d& wheel_relative, double yaw_disagreement_rad = 0.0,
                                 double minimum_yaw_sigma_rad = 0.0);
  // `information` is 6x6 in GTSAM Pose3 order (rotation xyz, then translation xyz).
  void add_lidar_odometry_factor(int from_index, int to_index, const Eigen::Isometry3d& relative,
                                 const Eigen::Matrix<double, 6, 6>& information);
  // Relative yaw measured by a gyro between two nodes (both still in the window): a between factor on
  // the yaw only. Translation, roll and pitch are left unconstrained (very large standard deviations).
  // The yaw error is the z component of the Pose3 log of the relative pose, which equals the yaw error
  // only while roll and pitch are small (ground vehicle); on a steep slope it is a close approximation.
  // Enters the batch history when record_history is on. Call before optimize().
  void add_rotation_factor(int from_index, int to_index, double yaw_delta_rad, double yaw_sigma_rad);
  void add_map_factor(int index, const Eigen::Isometry3d& pose,
                      const Eigen::Matrix<double, 6, 6>& information);
  // Uses only map-frame x/y and sigma x/y; altitude is intentionally ignored.
  void add_gps_factor(int index, const Eigen::Vector3d& position, const Eigen::Vector3d& sigma_m);
  // Optimizes. Returns false if GTSAM threw; the caller then rebuilds from the current estimate.
  bool optimize();
  // Relative pose between two recorded nodes found by loop-closure detection. It enters only the batch
  // history (the online window no longer holds the earlier node). Needs record_history.
  void add_loop_closure_factor(int from_index, int to_index, const Eigen::Isometry3d& relative,
                               double sigma_translation_m, double sigma_rotation_rad);
  // Batch re-optimization of every retained factor (record_history must be on). Fills batch_poses().
  // Starts from the previous batch result when there is one. `should_compute_marginals` false leaves
  // position_and_yaw_std empty (the marginals of a long drive take most of the time).
  bool batch_optimize(int max_iterations, bool should_compute_marginals = true);
  // The loop closure factors added since the last remove_loop_closure_factors().
  std::size_t loop_closure_factor_count() const;
  // Drops every loop closure factor and puts batch_poses() / the batch initial guess back to what they
  // were before the first one was added (the optimized map of the save stage before the loop closing).
  void remove_loop_closure_factors();
  // Test seam: the batch_optimize() call after `successful_calls` more calls fails as if GTSAM had thrown.
  void inject_batch_failure_after(int successful_calls) { batch_failure_countdown_ = successful_calls; }
  const std::vector<BatchPose>& batch_poses() const;
  // Every recorded node at the estimate it had when it left the online window (the nodes still in the
  // window: their latest estimate). No optimisation runs, so this is available at once;
  // position_and_yaw_std is empty. Empty without record_history.
  std::vector<BatchPose> recorded_poses() const;

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
  int batch_failure_countdown_ = -1;
  mutable std::string covariance_error_;
};

}  // namespace lightnav_localization
