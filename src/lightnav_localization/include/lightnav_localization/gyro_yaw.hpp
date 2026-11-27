#pragma once

#include <Eigen/Geometry>
#include <cstddef>
#include <deque>
#include <optional>

namespace lightnav_localization {

struct GyroYawParameters {
  double max_gap_s = 0.05;
  double max_rate_rad_s = 10.0;
  double stationary_window_s = 2.0;
  double bias_max_abs_rad_s = 0.05;
  double bias_max_std_rad_s = 0.1;
  double up_axis_warn_deg = 10.0;
  double timeout_s = 0.5;
};

enum class GyroYawState { kWaitingForData, kWaitingForStationary, kReady, kStale };

struct GyroYawStatus {
  GyroYawState state = GyroYawState::kWaitingForData;
  double sample_rate_hz = 0.0;
  std::optional<double> last_sample_age_s;  // newest wheel stamp - newest IMU stamp (never negative)
  std::optional<Eigen::Vector3d> gyro_bias_rad_s;
  std::optional<double> yaw_bias_rad_s;
  std::optional<double> bias_age_s;
  std::size_t bias_updates_total = 0;
  std::size_t bias_rejected_total = 0;
  std::optional<Eigen::Vector3d> up_axis_in_imu;
  std::optional<double> tf_up_axis_disagreement_deg;
  bool is_up_axis_warning = false;
  std::optional<double> yaw_rate_rad_s;
  std::size_t dropped_samples_total = 0;
};

// Integrates the yaw (rotation about the robot's up axis) measured by an external gyro.
//
// The up axis is the direction of gravity measured while the wheels report standstill, and the
// gyro bias is the mean angular velocity of the same standstill windows. Depends on Eigen only
// (no ROS, no GTSAM); all times are header stamps in seconds, never the wall clock.
class GyroYawTracker {
 public:
  // Throws std::invalid_argument unless every parameter is finite and positive.
  explicit GyroYawTracker(const GyroYawParameters& parameters);

  // Samples with a non-finite value, a rate above max_rate_rad_s or a stamp that does not advance
  // are dropped and counted. A stamp that moves backwards also discards the held samples; the
  // bias and the up axis are kept.
  void add_imu_sample(double stamp_sec, const Eigen::Vector3d& angular_velocity_rad_s,
                      const Eigen::Vector3d& linear_acceleration_m_s2);

  // Standstill is judged from the pose difference of consecutive wheel poses. Also the reference
  // time of the timeout.
  void add_wheel_pose(double stamp_sec, const Eigen::Isometry3d& odom_to_base);

  // Yaw change between the two stamps (trapezoid rule over the linearly interpolated rate).
  // nullopt when the bias is unknown, the range starts before the oldest held sample or ends more
  // than 0.03 s after the newest one, or a gap longer than max_gap_s lies inside it.
  std::optional<double> integrate_yaw(double from_sec, double to_sec) const;

  // The up axis seen through the TF tree. Only compared with the gravity-based axis.
  void set_expected_up_axis(const std::optional<Eigen::Vector3d>& up_axis_in_imu);

  GyroYawStatus status() const;

 private:
  struct ImuSample {
    double stamp_sec;
    Eigen::Vector3d angular_velocity_rad_s;
    Eigen::Vector3d linear_acceleration_m_s2;
  };

  GyroYawParameters parameters_;
  std::deque<ImuSample> samples_;
  std::size_t dropped_samples_total_ = 0;
  std::optional<double> latest_wheel_stamp_sec_;
  std::optional<Eigen::Isometry3d> previous_wheel_pose_;
  std::optional<double> stationary_window_start_sec_;
  std::optional<Eigen::Vector3d> gyro_bias_rad_s_;
  std::optional<double> bias_stamp_sec_;
  std::optional<Eigen::Vector3d> up_axis_in_imu_;
  std::optional<Eigen::Vector3d> expected_up_axis_in_imu_;
  std::size_t bias_updates_total_ = 0;
  std::size_t bias_rejected_total_ = 0;

  void prune_old_samples();
  void evaluate_stationary_windows(double wheel_stamp_sec);
  void evaluate_window(double window_start_sec, double window_end_sec);
  double yaw_rate_of(const Eigen::Vector3d& angular_velocity_rad_s) const;
};

}  // namespace lightnav_localization
