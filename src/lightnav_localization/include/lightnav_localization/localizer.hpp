#pragma once

#include <Eigen/Geometry>
#include <cstddef>
#include <deque>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "lightnav_localization/corrector.hpp"
#include "lightnav_localization/deskew.hpp"
#include "lightnav_localization/gps_input.hpp"
#include "lightnav_localization/gyro_consistency.hpp"
#include "lightnav_localization/lidar_odometry.hpp"
#include "lightnav_localization/loop_closure.hpp"
#include "lightnav_localization/map_builder.hpp"
#include "lightnav_localization/match_information.hpp"
#include "lightnav_localization/pose_buffer.hpp"
#include "lightnav_localization/pose_graph.hpp"
#include "lightnav_localization/state_machine.hpp"

namespace lightnav_localization {

// Receives one line per finished stage of Localizer::save_map().
using MapSaveProgress = std::function<void(const std::string&)>;

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
  double last_error_hold_s = 10.0;
  double map_submap_rebuild_distance_m = 50.0;
  double initial_sigma_xy_m = 0.5;
  double initial_sigma_yaw_rad = 0.1;
  // Mapping mode: excludes the prior-map factor (empty map already does that), keeps every
  // keyframe cloud after deskewing and enables the full factor history for the batch save.
  bool mapping = false;
  double keyframe_cloud_voxel_m = 0.25;
  // Mapping only: closes loops in the batch optimisation of save_map().
  LoopClosureParameters loop_closure;
  // Replace the wheel yaw increments by the gyro ones while the gyro is verified (Issue #92). When false
  // the gyro only adds rotation factors. Has no effect until an increment is passed to on_wheel_odom.
  bool gyro_yaw_prediction_enabled = true;
  // State to start from: the node passes the remembered kActive or kDisabledInconsistent here (anything
  // else starts from kUnverified).
  GyroConsistencyState initial_gyro_consistency = GyroConsistencyState::kUnverified;
};

struct LocalizerOutput {
  Eigen::Isometry3d map_to_base = Eigen::Isometry3d::Identity();
  LocalizationState state = LocalizationState::kInitializing;
  double position_std_m = 0.0;
  bool is_large_correction = false;
  bool is_gps_conflict = false;
  double pending_correction_m = 0.0;
};

struct MapMatchStatistics {
  std::size_t attempted_total = 0;
  std::size_t accepted_total = 0;
  std::size_t rejected_not_converged_total = 0;
  std::size_t rejected_inlier_total = 0;
  std::size_t rejected_translation_gate_total = 0;
  std::size_t rejected_rotation_gate_total = 0;
  double last_gate_scale = 1.0;
};

// Read-only view of the gyro yaw fusion for /diagnostics and tests.
struct GyroFusionStatus {
  GyroConsistencyState consistency = GyroConsistencyState::kUnverified;
  bool is_inconsistent_warning = false;  // still kUnverified after three rejected checks
  std::size_t consistency_confirmed_total = 0;
  std::size_t consistency_outvoted_total = 0;
  std::size_t consistency_rejected_total = 0;
  std::optional<Eigen::Vector3d> last_yaw_deltas_rad;    // wheel, gyro, lidar (NaN without lidar)
  std::optional<Eigen::Vector3d> verification_sums_rad;  // W, G, L; only while kUnverified / kSuspended
  std::size_t yaw_increments_gyro_total = 0;
  std::size_t yaw_increments_wheel_total = 0;
  std::size_t rotation_factors_added_total = 0;
  std::size_t rotation_factors_skipped_total = 0;
  std::size_t slip_detected_total = 0;
  double slip_max_rad = 0.0;
};

struct ScanTimeStatistics {
  std::size_t count = 0;
  double total_ms = 0.0;
  double maximum_ms = 0.0;
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

  // Starts (or restarts after LOST) from this pose, discarding graph, queues and sub-map. The gyro
  // consistency state is carried over: kActive and kDisabledInconsistent stay, kSuspended becomes kActive.
  void initialize(const Eigen::Isometry3d& map_to_base);

  // Output for this wheel sample (nullopt while the state withholds odom). Also drains queued
  // scans/GPS that the odometry now covers. `gyro_yaw_delta_rad` is the gyro yaw change from the
  // previous wheel sample to this one; nullopt means "no increment for this sample". A Localizer that
  // has never received a value produces exactly the same output as before the gyro was introduced.
  std::optional<LocalizerOutput> on_wheel_odom(double stamp_sec, const Eigen::Isometry3d& odom_to_base,
                                               std::optional<double> gyro_yaw_delta_rad = std::nullopt);
  // Queued until wheel odometry covers the end of the scan (stamp + largest time offset).
  void on_scan(double header_stamp_sec, std::vector<TimedPoint> scan);
  void on_gps(double stamp_sec, const GpsFix& fix);

  // State at `now_sec` without a wheel sample (INPUT_STALE when the wheel stopped). Not
  // thread-safe: Localizer must be called from one thread (the ROS node's executor).
  LocalizationState poll_state(double now_sec);
  const std::string& last_error() const;
  std::size_t error_events_total() const;
  std::size_t pending_scan_count() const;
  void inject_optimization_failure_for_test();
  // Next prior-map rebuild: 1 = the worker throws, 2 = starting the worker throws system_error.
  void inject_map_rebuild_failure_for_test(int mode);
  // Poses (map_to_base, oldest first) of the keyframes in the LiDAR odometry sub-map.
  std::vector<Eigen::Isometry3d> lidar_keyframe_poses_for_test() const;

  double last_scan_time_ms() const;
  ScanTimeStatistics take_scan_time_statistics();
  double last_match_time_ms() const;
  double last_submap_rebuild_time_ms() const;
  // Read-only diagnostics for LocalizationStatus.
  MapMatchStatistics map_match_statistics() const;
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
  // Test seams like set_map_match_hook; both survive initialize().
  // May alter the LiDAR odometry match result before it is used.
  void set_lidar_match_hook_for_test(std::function<void(MatchResult&)> hook);
  // Called at the very start of scan processing (to delay it).
  void set_scan_processing_hook_for_test(std::function<void()> hook);
  GyroFusionStatus gyro_fusion_status() const;

  // Mapping support. Only meaningful when LocalizerParameters::mapping is true.
  const std::vector<KeyframeCloud>& mapping_keyframes() const;
  std::size_t mapping_keyframe_count() const;
  std::size_t accepted_gps_factor_count() const;
  // Stops keeping new keyframe clouds after a LOST; the retained partial map can still be saved.
  void interrupt_mapping();
  // Writes map.pcd / map.yaml / trajectory.csv in three stages, each replacing the files of the one
  // before, so a save that is cut short (the stop time limit of the container) leaves the latest
  // finished stage instead of nothing:
  //   1. preliminary: the poses as the online estimate left them (map.yaml batch_optimized: false),
  //   2. optimized: after the batch optimisation of every retained factor (position_std_m is nan),
  //   3. final: after the loop closures (when enabled) and with the standard deviations.
  // `report_progress` is called with one line as soon as a stage is on disk.
  // On failure the files of the stages that finished stay; `error` says what failed.
  bool save_map(const MapOutputParameters& parameters, MapOutputResult& result,
                std::string& error, const MapSaveProgress& report_progress = {});

 private:
  struct Implementation;
  std::unique_ptr<Implementation> implementation_;
};

}  // namespace lightnav_localization
