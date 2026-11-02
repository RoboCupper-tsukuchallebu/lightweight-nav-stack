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
  double gps_node_gap_s = 0.2;       // beyond this a GPS fix gets its own node
  int gps_conflict_count = 5;
  double drift_rate = 0.02;
  double initial_sigma_xy_m = 0.5;
  double initial_sigma_yaw_rad = 0.1;
};

struct LocalizerOutput {
  Eigen::Isometry3d map_to_base = Eigen::Isometry3d::Identity();
  LocalizationState state = LocalizationState::kInitializing;
  double position_std_m = 0.0;
  bool is_large_correction = false;
  bool is_gps_conflict = false;
  double pending_correction_m = 0.0;
};

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

  double last_scan_time_ms() const;
  double last_match_time_ms() const;
  double last_submap_rebuild_time_ms() const;
  // Test seam: may alter a prior-map match result before the gate (e.g. inject a false convergence).
  void set_map_match_hook(std::function<void(MatchResult&)> hook);

 private:
  struct Implementation;
  std::unique_ptr<Implementation> implementation_;
};

}  // namespace lightnav_localization
