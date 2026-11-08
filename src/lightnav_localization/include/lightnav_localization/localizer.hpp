#pragma once

#include <Eigen/Geometry>
#include <deque>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "lightnav_localization/corrector.hpp"
#include "lightnav_localization/deskew.hpp"
#include "lightnav_localization/gps_input.hpp"
#include "lightnav_localization/lidar_odometry.hpp"
#include "lightnav_localization/map_builder.hpp"
#include "lightnav_localization/match_information.hpp"
#include "lightnav_localization/pose_buffer.hpp"
#include "lightnav_localization/pose_graph.hpp"
#include "lightnav_localization/state_machine.hpp"

namespace lightnav_localization {

struct LocalizerParameters {
  PoseGraphParameters graph;
  LidarOdometryParameters lidar_odometry;
  CorrectorParameters corrector;
  StateMachineParameters state;
  GpsParameters gps;
  InformationLimits lidar_odometry_information;
  InformationLimits map_information{1.0, 0.05, 0.01, 10.0, 1.0};
  MatcherParameters map_matcher;
  Eigen::Isometry3d lidar_to_base = Eigen::Isometry3d::Identity();
  double voxel_size_m = 0.5;
  double keyframe_distance_m = 1.0;
  double keyframe_time_s = 1.0;
  double match_period_s = 0.5;
  double gps_keyframe_max_gap_s = 1.2;  // default keyframe interval 1.0 s + 0.2 s margin
  int gps_conflict_window = 10;
  double gps_conflict_ratio = 0.5;
  double drift_rate = 0.02;
  double map_submap_rebuild_distance_m = 50.0;
  double initial_sigma_xy_m = 0.5;
  double initial_sigma_yaw_rad = 0.1;
  // Mapping mode: excludes the prior-map factor (empty map already does that), keeps every
  // keyframe cloud after deskewing and enables the full factor history for the batch save.
  bool mapping = false;
  double keyframe_cloud_voxel_m = 0.25;
};

struct LocalizerOutput {
  Eigen::Isometry3d map_to_base = Eigen::Isometry3d::Identity();
  LocalizationState state = LocalizationState::kInitializing;
  double position_std_m = 0.0;
  bool is_large_correction = false;
  bool is_gps_conflict = false;
  double pending_correction_m = 0.0;
};

// Thread contract: every public method must be called from ONE thread (the node's executor, with a
// mutually exclusive callback group). Internally one worker thread rebuilds the prior-map target
// every 50 m; it only reads the immutable map and hands the result back through a future that the
// calling thread collects, so no lock is needed and the 50 Hz path never waits for the rebuild.
// ROS-free pipeline: deskew -> lidar odometry -> prior-map match -> GPS -> pose graph -> slew.
class Localizer {
 public:
  // `map_points` may be empty (no prior map). `gps_origin` empty disables GPS.
  Localizer(const LocalizerParameters& parameters, const PointList& map_points,
            const std::optional<GpsOrigin>& gps_origin);
  ~Localizer();

  // Starts (or restarts after LOST) from this pose, discarding graph, queues and sub-map.
  void initialize(const Eigen::Isometry3d& map_to_base);

  // Output for this wheel sample (nullopt while the state withholds odom). Also drains queued
  // scans/GPS that the odometry now covers.
  std::optional<LocalizerOutput> on_wheel_odom(double stamp_sec, const Eigen::Isometry3d& odom_to_base);
  // Queued until wheel odometry covers the end of the scan (stamp + largest time offset).
  void on_scan(double header_stamp_sec, std::vector<TimedPoint> scan);
  void on_gps(double stamp_sec, const GpsFix& fix);

  // State at `now_sec` without a wheel sample (INPUT_STALE when the wheel stopped). Not
  // thread-safe: Localizer must be called from one thread (the ROS node's executor).
  LocalizationState poll_state(double now_sec);
  const std::string& last_error() const;
  std::size_t pending_scan_count() const;
  void inject_optimization_failure_for_test();
  // Next prior-map rebuild: 1 = the worker throws, 2 = starting the worker throws system_error.
  void inject_map_rebuild_failure_for_test(int mode);

  double last_scan_time_ms() const;
  double last_match_time_ms() const;
  double last_submap_rebuild_time_ms() const;
  // Read-only diagnostics for LocalizationStatus.
  double last_inlier_ratio() const;
  double last_lidar_odom_inlier_ratio() const;
  double last_map_match_stamp_sec() const;
  double odom_only_distance_m() const;
  // map-frame (x, y) standard deviations and the local yaw standard deviation of the latest node.
  std::optional<Eigen::Vector3d> latest_position_and_yaw_std() const;
  // Raw translation Hessian diagnostics; absent before a valid match. Axis is in the scan frame
  // and its sign is arbitrary. initialize() clears both values.
  std::optional<double> last_match_eigenvalue_ratio() const;
  std::optional<Eigen::Vector3d> last_match_weakest_axis() const;
  double last_accepted_gps_stamp_sec() const;
  std::size_t gps_time_gap_rejection_count() const;
  std::size_t gps_gate_rejection_count() const;
  // Gate rejections among the last `gps_conflict_window` fixes; counts the fixes the gate would
  // have rejected, including those a trusted fix let through (Issue #43 section 4.3).
  std::size_t gps_conflict_window_reject_count() const;
  std::size_t max_gps_factors_per_keyframe() const;
  double max_keyframe_interval_sec() const;
  double last_absolute_observation_stamp_sec() const;
  // Test seam: may alter a prior-map match result before the gate (e.g. inject a false convergence).
  void set_map_match_hook(std::function<void(MatchResult&)> hook);

  // Mapping support. Only meaningful when LocalizerParameters::mapping is true.
  const std::vector<KeyframeCloud>& mapping_keyframes() const;
  std::size_t mapping_keyframe_count() const;
  std::size_t accepted_gps_factor_count() const;
  // Stops keeping new keyframe clouds after a LOST; the retained partial map can still be saved.
  void interrupt_mapping();
  // Batch re-optimizes every retained factor and writes map.pcd / map.yaml / trajectory.csv.
  bool save_map(const MapOutputParameters& parameters, MapOutputResult& result,
                std::string& error);

 private:
  struct Implementation;
  std::unique_ptr<Implementation> implementation_;
};

}  // namespace lightnav_localization
