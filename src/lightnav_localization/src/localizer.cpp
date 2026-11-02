#include "lightnav_localization/localizer.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>
#include <string>

#include "lightnav_localization/downsample.hpp"

namespace lightnav_localization {
namespace {

constexpr double kMapSubmapRadiusM = 100.0;
constexpr double kMapSubmapRebuildDistanceM = 50.0;
constexpr double kWheelBufferSec = 3.0;
constexpr double kNeverSec = -1e9;
constexpr std::size_t kMaxPendingScans = 20;
constexpr std::size_t kMaxPendingGps = 20;

using Clock = std::chrono::steady_clock;
double elapsed_ms(Clock::time_point start) {
  return std::chrono::duration<double, std::milli>(Clock::now() - start).count();
}

double rotation_angle_rad(const Eigen::Isometry3d& transform) {
  return Eigen::AngleAxisd(transform.rotation()).angle();
}

struct NodeRecord {
  int index;
  double stamp;
  Eigen::Isometry3d odom_to_base;
};
struct PendingScan {
  double stamp;
  double ready_stamp;
  std::vector<TimedPoint> scan;
};
struct PendingGps {
  double stamp;
  GpsFix fix;
};

}  // namespace

struct Localizer::Implementation {
  Implementation(const LocalizerParameters& parameters_in, const PointList& map_points_in,
                 const std::optional<GpsOrigin>& gps_origin_in)
      : parameters(parameters_in),
        map_points(map_points_in),
        graph(std::make_unique<PoseGraph>(parameters_in.graph)),
        lidar(parameters_in.lidar_odometry),
        corrector(parameters_in.corrector),
        machine(parameters_in.state),
        conflict(parameters_in.gps_conflict_count),
        gps_origin(gps_origin_in) {
    if (gps_origin) gps.emplace(*gps_origin, parameters_in.gps);
  }

  LocalizerParameters parameters;
  PointList map_points;
  std::unique_ptr<PoseGraph> graph;
  LidarOdometry lidar;
  Corrector corrector;
  StateMachine machine;
  GpsConflictCounter conflict;
  std::optional<GpsOrigin> gps_origin;
  std::optional<GpsConverter> gps;
  PoseBuffer buffer;
  PreparedTarget map_target;
  Eigen::Vector2d map_target_center = Eigen::Vector2d::Zero();
  bool has_map_target = false;

  std::optional<Eigen::Isometry3d> requested_initial_pose;
  bool is_started = false;
  bool is_graph_started = false;
  double last_wheel_sec = kNeverSec;
  double last_scan_sec = kNeverSec;
  double last_map_sec = kNeverSec;
  double last_gps_sec = kNeverSec;
  double last_match_attempt_sec = kNeverSec;
  double absolute_reference_sec = 0.0;
  double distance_since_absolute_m = 0.0;
  Eigen::Isometry3d last_wheel_pose = Eigen::Isometry3d::Identity();
  double position_std_m = 0.0;

  double last_keyframe_sec = 0.0;  // last node that carries a scan (GPS-only nodes do not count)
  Eigen::Isometry3d last_keyframe_odom = Eigen::Isometry3d::Identity();
  std::deque<NodeRecord> nodes;
  std::deque<int> lidar_nodes;
  std::map<int, Eigen::Isometry3d> pose_cache;
  std::vector<PendingScan> pending_scans;
  std::vector<PendingGps> pending_gps;
  std::function<void(MatchResult&)> map_match_hook;
  std::string last_error;
  double scan_time_ms = 0.0;
  double match_time_ms = 0.0;
  double submap_rebuild_time_ms = 0.0;

  Eigen::Isometry3d pose_of(int index) {
    if (const auto pose = graph->pose(index)) return pose_cache[index] = *pose;
    const auto cached = pose_cache.find(index);
    if (cached != pose_cache.end()) return cached->second;
    last_error = "internal: pose requested for unknown node " + std::to_string(index);
    return pose_cache.empty() ? Eigen::Isometry3d::Identity() : pose_cache.rbegin()->second;
  }

  // Optimizes; returns true when the optimization failed and the window was restarted from the
  // current estimate (node indices are renumbered from 0 and the lidar sub-map is dropped, so
  // callers must re-read nodes.back().index).
  bool refresh_after_optimize() {
    bool was_restarted = false;
    if (!graph->optimize()) {
      last_error = "pose graph optimization failed, window restarted: " + graph->last_error();
      const Eigen::Isometry3d current = pose_cache.at(nodes.back().index);
      const NodeRecord latest_node = nodes.back();
      graph = std::make_unique<PoseGraph>(parameters.graph);
      graph->add_first_node(latest_node.stamp, current, parameters.initial_sigma_xy_m,
                            parameters.initial_sigma_yaw_rad);
      nodes.clear();
      nodes.push_back({0, latest_node.stamp, latest_node.odom_to_base});
      lidar = LidarOdometry(parameters.lidar_odometry);
      lidar_nodes.clear();
      pose_cache.clear();
      pose_cache[0] = current;
      if (!graph->optimize()) last_error += " (restart also failed)";
      was_restarted = true;
    }
    const int latest = nodes.back().index;
    if (const auto pose = graph->pose(latest)) pose_cache[latest] = *pose;
    if (const auto std_values = graph->position_and_yaw_std(latest)) {
      position_std_m = std::max((*std_values)[0], (*std_values)[1]);
    }
    corrector.set_target(pose_cache[latest] * nodes.back().odom_to_base.inverse());
    while (nodes.size() > 1 && nodes.front().stamp < nodes.back().stamp - parameters.graph.window_s) {
      pose_cache.erase(nodes.front().index);
      nodes.pop_front();
    }
    return was_restarted;
  }

  void sync_lidar_submap() {
    const auto start = Clock::now();
    std::vector<Eigen::Isometry3d> poses;
    for (const int index : lidar_nodes) poses.push_back(pose_of(index));
    lidar.update_keyframe_poses(poses);
    lidar.rebuild_submap();
    submap_rebuild_time_ms = elapsed_ms(start);
  }

  void ensure_map_target(const Eigen::Vector3d& center) {
    if (map_points.empty()) return;
    if (has_map_target && (center.head<2>() - map_target_center).norm() < kMapSubmapRebuildDistanceM) {
      return;
    }
    PointList cropped;
    for (const auto& point : map_points) {
      if ((point.head<2>().cast<double>() - center.head<2>()).norm() <= kMapSubmapRadiusM) {
        cropped.push_back(point);
      }
    }
    map_target = PreparedTarget(cropped);
    map_target_center = center.head<2>();
    has_map_target = map_target.is_valid();
  }

  void mark_absolute(double stamp) {
    absolute_reference_sec = stamp;
    distance_since_absolute_m = 0.0;
  }

  void try_map_match(int node_index, double stamp, const Eigen::Isometry3d& estimate,
                     const PointList& cloud) {
    if (map_points.empty() || stamp - last_match_attempt_sec < parameters.match_period_s) return;
    last_match_attempt_sec = stamp;
    ensure_map_target(estimate.translation());
    if (!has_map_target) return;
    const auto start = Clock::now();
    MatchResult result = match_scan(map_target, cloud, estimate, parameters.map_matcher);
    match_time_ms = elapsed_ms(start);
    if (map_match_hook) map_match_hook(result);
    // Without a map match or GPS the estimate is less certain, so the gate widens with it.
    const bool is_tracking = stamp - last_map_sec <= parameters.state.tracking_window_s;
    const double gate_scale = is_tracking ? 1.0 : std::max(1.0, 3.0 * position_std_m);
    const Eigen::Isometry3d difference = estimate.inverse() * result.target_to_source;
    const bool is_accepted = result.converged && result.inlier_ratio >= parameters.map_matcher.inlier_ratio_min &&
                             difference.translation().norm() <= parameters.map_matcher.gate_translation_m * gate_scale &&
                             rotation_angle_rad(difference) <= parameters.map_matcher.gate_rotation_rad * gate_scale;
    if (!is_accepted) return;
    graph->add_map_factor(node_index, result.target_to_source,
                          information_from_hessian(result.information, parameters.map_information));
    last_map_sec = stamp;
    mark_absolute(stamp);
  }

  void process_scan(const PendingScan& pending) {
    const auto start = Clock::now();
    const auto deskewed = deskew_scan(pending.scan, pending.stamp, buffer, parameters.lidar_to_base);
    const auto odom_now = buffer.interpolate(pending.stamp);
    if (!deskewed || !odom_now) return;
    const PointList cloud = downsample_voxel(*deskewed, parameters.voxel_size_m);
    last_scan_sec = pending.stamp;

    if (!is_graph_started) {
      const Eigen::Isometry3d initial = corrector.applied() * *odom_now;
      graph->add_first_node(pending.stamp, initial, parameters.initial_sigma_xy_m,
                            parameters.initial_sigma_yaw_rad);
      graph->optimize();
      pose_cache[0] = initial;
      nodes.push_back({0, pending.stamp, *odom_now});
      lidar.add_keyframe(initial, cloud);
      lidar_nodes.push_back(0);
      last_keyframe_sec = pending.stamp;
      last_keyframe_odom = *odom_now;
      is_graph_started = true;
      scan_time_ms = elapsed_ms(start);
      return;
    }

    const NodeRecord last = nodes.back();
    const Eigen::Isometry3d last_pose = pose_of(last.index);
    const Eigen::Isometry3d wheel_relative = last.odom_to_base.inverse() * *odom_now;
    const Eigen::Isometry3d prediction = last_pose * wheel_relative;
    const double moved_m = wheel_relative.translation().norm();

    Eigen::Isometry3d best = prediction;
    std::optional<MatchResult> lidar_result;
    if (lidar.has_submap()) {
      MatchResult result = lidar.match(cloud, prediction, moved_m);
      if (result.is_accepted) {
        best = result.target_to_source;
        lidar_result = result;
        corrector.set_target(best * odom_now->inverse());
      }
    }

    const double moved_since_keyframe_m =
        (odom_now->translation() - last_keyframe_odom.translation()).norm();
    const bool is_keyframe = moved_since_keyframe_m >= parameters.keyframe_distance_m ||
                             pending.stamp - last_keyframe_sec >= parameters.keyframe_time_s;
    if (is_keyframe) {
      const int index = graph->add_node_with_wheel_factor(pending.stamp, best, wheel_relative);
      if (lidar_result) {
        graph->add_lidar_odometry_factor(
            last.index, index, last_pose.inverse() * best,
            information_from_hessian(lidar_result->information, parameters.lidar_odometry_information));
      }
      nodes.push_back({index, pending.stamp, *odom_now});
      last_keyframe_sec = pending.stamp;
      last_keyframe_odom = *odom_now;
      pose_cache[index] = best;
      try_map_match(index, pending.stamp, best, cloud);
      refresh_after_optimize();
      const int current_index = nodes.back().index;  // renumbered after a restart
      lidar.add_keyframe(pose_of(current_index), cloud);
      lidar_nodes.push_back(current_index);
      while (static_cast<int>(lidar_nodes.size()) > parameters.lidar_odometry.submap_keyframes) {
        lidar_nodes.pop_front();
      }
      sync_lidar_submap();
    }
    scan_time_ms = elapsed_ms(start);
  }

  void process_gps(const PendingGps& pending) {
    if (!gps || !is_graph_started) return;
    const NodeRecord* nearest = &nodes.front();
    for (const NodeRecord& node : nodes) {
      if (std::abs(node.stamp - pending.stamp) < std::abs(nearest->stamp - pending.stamp)) nearest = &node;
    }
    int node_index = nearest->index;
    Eigen::Isometry3d predicted;
    bool is_new_node = false;
    if (std::abs(nearest->stamp - pending.stamp) <= parameters.gps_node_gap_s) {
      predicted = pose_of(node_index);
    } else if (pending.stamp > nodes.back().stamp) {
      const auto odom_at = buffer.interpolate(pending.stamp);
      if (!odom_at) return;
      const Eigen::Isometry3d relative = nodes.back().odom_to_base.inverse() * *odom_at;
      predicted = pose_of(nodes.back().index) * relative;
      node_index = graph->add_node_with_wheel_factor(pending.stamp, predicted, relative);
      nodes.push_back({node_index, pending.stamp, *odom_at});
      pose_cache[node_index] = predicted;
      is_new_node = true;
    } else {
      return;  // late fix far from every node in the window
    }
    const GpsDecision decision = gps->evaluate(pending.fix, predicted, position_std_m);
    const bool is_tracking = pending.stamp - last_map_sec <= parameters.state.tracking_window_s;
    if (decision.rejection == GpsRejection::kGate) conflict.record(is_tracking);
    if (decision.measurement) {
      conflict.record(false);
      graph->add_gps_factor(node_index, decision.measurement->base_position, decision.measurement->sigma_m);
      last_gps_sec = pending.stamp;
      mark_absolute(pending.stamp);
    }
    if (is_new_node || decision.measurement) refresh_after_optimize();
  }

  StateInputs make_state_inputs(double now_sec) const {
    StateInputs inputs;
    inputs.now_sec = now_sec;
    inputs.last_wheel_sec = last_wheel_sec;
    inputs.last_scan_sec = last_scan_sec;
    inputs.last_map_match_sec = last_map_sec;
    inputs.last_gps_sec = last_gps_sec;
    inputs.odom_only_distance_m = distance_since_absolute_m;
    inputs.odom_only_duration_s = now_sec - absolute_reference_sec;
    inputs.position_std_m = std::hypot(position_std_m, parameters.drift_rate * distance_since_absolute_m);
    return inputs;
  }

  void drain() {
    while (true) {
      const double newest = buffer.size() > 0 ? last_wheel_sec : kNeverSec;
      auto scan_it = std::min_element(pending_scans.begin(), pending_scans.end(),
                                      [](const PendingScan& a, const PendingScan& b) { return a.stamp < b.stamp; });
      auto gps_it = std::min_element(pending_gps.begin(), pending_gps.end(),
                                     [](const PendingGps& a, const PendingGps& b) { return a.stamp < b.stamp; });
      const bool scan_ready = scan_it != pending_scans.end() && scan_it->ready_stamp <= newest;
      const bool gps_ready = gps_it != pending_gps.end() && gps_it->stamp <= newest;
      if (!scan_ready && !gps_ready) return;
      if (scan_ready && (!gps_ready || scan_it->stamp <= gps_it->stamp)) {
        PendingScan scan = std::move(*scan_it);
        pending_scans.erase(scan_it);
        process_scan(scan);
      } else {
        const PendingGps fix = *gps_it;
        pending_gps.erase(gps_it);
        process_gps(fix);
      }
    }
  }
};

Localizer::Localizer(const LocalizerParameters& parameters, const PointList& map_points,
                     const std::optional<GpsOrigin>& gps_origin)
    : implementation_(std::make_unique<Implementation>(parameters, map_points, gps_origin)) {}
Localizer::~Localizer() = default;

void Localizer::initialize(const Eigen::Isometry3d& map_to_base) {
  // Full reset (also the way out of LOST): graph, queues, sub-map, state machine and slew.
  auto fresh = std::make_unique<Implementation>(implementation_->parameters, implementation_->map_points,
                                                implementation_->gps_origin);
  fresh->map_match_hook = implementation_->map_match_hook;
  fresh->requested_initial_pose = map_to_base;
  implementation_ = std::move(fresh);
}

void Localizer::on_scan(double header_stamp_sec, std::vector<TimedPoint> scan) {
  // Before initialize() nobody can consume scans; dropping them keeps the queue from growing.
  if (!implementation_->requested_initial_pose) return;
  float latest_offset = 0.0F;
  for (const TimedPoint& point : scan) latest_offset = std::max(latest_offset, point.time_offset_sec);
  auto& queue = implementation_->pending_scans;
  queue.push_back({header_stamp_sec, header_stamp_sec + latest_offset, std::move(scan)});
  if (queue.size() > kMaxPendingScans) queue.erase(queue.begin());  // oldest first
}

void Localizer::on_gps(double stamp_sec, const GpsFix& fix) {
  auto& queue = implementation_->pending_gps;
  queue.push_back({stamp_sec, fix});
  if (queue.size() > kMaxPendingGps) queue.erase(queue.begin());
}

void Localizer::set_map_match_hook(std::function<void(MatchResult&)> hook) {
  implementation_->map_match_hook = std::move(hook);
}

LocalizationState Localizer::poll_state(double now_sec) {
  Implementation& self = *implementation_;
  if (!self.is_started) return LocalizationState::kInitializing;
  return self.machine.update(self.make_state_inputs(now_sec));
}

std::size_t Localizer::pending_scan_count() const { return implementation_->pending_scans.size(); }

const std::string& Localizer::last_error() const { return implementation_->last_error; }

void Localizer::inject_optimization_failure_for_test() { implementation_->graph->inject_failure_once(); }

double Localizer::last_scan_time_ms() const { return implementation_->scan_time_ms; }
double Localizer::last_match_time_ms() const { return implementation_->match_time_ms; }
double Localizer::last_submap_rebuild_time_ms() const { return implementation_->submap_rebuild_time_ms; }

std::optional<LocalizerOutput> Localizer::on_wheel_odom(double stamp_sec,
                                                        const Eigen::Isometry3d& odom_to_base) {
  Implementation& self = *implementation_;
  if (!self.requested_initial_pose) return std::nullopt;
  if (stamp_sec < self.last_wheel_sec) {
    self.buffer = PoseBuffer();
    self.pending_scans.clear();
    self.pending_gps.clear();
    self.last_error = "wheel odometry time went backwards: buffers cleared";
    self.last_wheel_sec = stamp_sec;
    self.last_wheel_pose = odom_to_base;
  }
  const double elapsed_sec = self.last_wheel_sec > kNeverSec ? stamp_sec - self.last_wheel_sec : 0.0;
  if (!self.is_started) {
    self.corrector.reset(*self.requested_initial_pose * odom_to_base.inverse());
    self.machine.start_tracking();
    self.last_scan_sec = stamp_sec;
    self.absolute_reference_sec = stamp_sec;
    self.last_wheel_pose = odom_to_base;
    self.is_started = true;
  }
  self.distance_since_absolute_m +=
      (odom_to_base.translation() - self.last_wheel_pose.translation()).head<2>().norm();
  self.last_wheel_pose = odom_to_base;
  self.last_wheel_sec = stamp_sec;
  self.buffer.add(stamp_sec, odom_to_base);
  self.buffer.prune_before(stamp_sec - kWheelBufferSec);
  // A scan older than the buffer can no longer be deskewed.
  const double oldest_usable_sec = stamp_sec - kWheelBufferSec;
  self.pending_scans.erase(
      std::remove_if(self.pending_scans.begin(), self.pending_scans.end(),
                     [&](const PendingScan& scan) { return scan.stamp < oldest_usable_sec; }),
      self.pending_scans.end());
  self.pending_gps.erase(
      std::remove_if(self.pending_gps.begin(), self.pending_gps.end(),
                     [&](const PendingGps& fix) { return fix.stamp < oldest_usable_sec; }),
      self.pending_gps.end());
  self.drain();

  const Eigen::Isometry3d map_to_odom = self.corrector.step(elapsed_sec, odom_to_base);
  const StateInputs inputs = self.make_state_inputs(stamp_sec);
  const LocalizationState state = self.machine.update(inputs);
  if (!self.machine.should_publish_odom()) return std::nullopt;

  LocalizerOutput output;
  output.map_to_base = map_to_odom * odom_to_base;
  output.state = state;
  output.position_std_m = inputs.position_std_m;
  output.is_large_correction = self.corrector.is_large_correction(odom_to_base);
  output.pending_correction_m = self.corrector.pending_correction_m(odom_to_base);
  output.is_gps_conflict = self.conflict.is_conflict();
  return output;
}

}  // namespace lightnav_localization
