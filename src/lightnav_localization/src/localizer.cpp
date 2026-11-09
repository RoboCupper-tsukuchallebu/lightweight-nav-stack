#include "lightnav_localization/localizer.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <future>
#include <limits>
#include <map>
#include <stdexcept>
#include <string>
#include <system_error>

#include "lightnav_localization/downsample.hpp"

namespace lightnav_localization {
namespace {

constexpr double kMapSubmapRadiusM = 100.0;
constexpr double kWheelBufferSec = 3.0;
constexpr double kNeverSec = -1e9;
constexpr std::size_t kMaxPendingScans = 20;
constexpr std::size_t kMaxPendingGps = 20;
const std::string kUnknownNodeError = "internal: pose requested for unknown node ";

using Clock = std::chrono::steady_clock;
double elapsed_ms(Clock::time_point start) {
  return std::chrono::duration<double, std::milli>(Clock::now() - start).count();
}

double rotation_angle_rad(const Eigen::Isometry3d& transform) {
  return Eigen::AngleAxisd(transform.rotation()).angle();
}

// Eigen's Isometry inverse() leaves its local result matrix uninitialized until makeAffine()
// fills the last row, which GCC reports as -Wmaybe-uninitialized under -Werror. Build the same
// inverse explicitly from an Identity-initialized transform instead.
Eigen::Isometry3d invert_isometry(const Eigen::Isometry3d& transform) {
  Eigen::Isometry3d inverse = Eigen::Isometry3d::Identity();
  inverse.linear() = transform.linear().transpose();
  inverse.translation() = -(inverse.linear() * transform.translation());
  return inverse;
}

struct NodeRecord {
  int index;
  double stamp;
  Eigen::Isometry3d odom_to_base = Eigen::Isometry3d::Identity();
};
struct PendingScan {
  double stamp;
  double ready_stamp;
  std::vector<TimedPoint> scan;
};
struct MapTargetBuild {
  PreparedTarget target;
  Eigen::Vector2d center;
};
struct PendingGps {
  double stamp;
  GpsFix fix;
};

PoseGraphParameters history_parameters_for(const LocalizerParameters& parameters) {
  PoseGraphParameters graph_parameters = parameters.graph;
  graph_parameters.record_history = parameters.mapping;
  return graph_parameters;
}

// Mapping has no prior map and often loses GNSS for long stretches, so neither the ODOM_ONLY distance/time
// limits nor the position-uncertainty limit (the drift_rate term grows with the distance since the last
// absolute observation) may turn a long drive into LOST (Issue #43 D5). The localization mode keeps them.
StateMachineParameters state_parameters_for(const LocalizerParameters& parameters) {
  StateMachineParameters state_parameters = parameters.state;
  if (parameters.mapping) {
    state_parameters.odom_only_max_distance_m = std::numeric_limits<double>::infinity();
    state_parameters.odom_only_max_s = std::numeric_limits<double>::infinity();
    state_parameters.lost_position_std_m = std::numeric_limits<double>::infinity();
  }
  return state_parameters;
}

}  // namespace

struct Localizer::Implementation {
  Implementation(const LocalizerParameters& parameters_in, const PointList& map_points_in,
                 const std::optional<GpsOrigin>& gps_origin_in)
      : parameters(parameters_in),
        map_points(map_points_in),
        graph(std::make_unique<PoseGraph>(history_parameters_for(parameters_in))),
        lidar(parameters_in.lidar_odometry),
        corrector(parameters_in.corrector),
        machine(state_parameters_for(parameters_in)),
        conflict(parameters_in.gps_conflict_window, parameters_in.gps_conflict_ratio),
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
  std::future<MapTargetBuild> pending_map_target;
  Eigen::Vector2d map_target_center = Eigen::Vector2d::Zero();
  bool has_map_target = false;

  std::optional<Eigen::Isometry3d> requested_initial_pose;
  bool is_started = false;
  bool is_graph_started = false;
  double last_wheel_sec = kNeverSec;
  double last_scan_sec = kNeverSec;
  double last_map_sec = kNeverSec;
  double last_gps_sec = kNeverSec;
  std::size_t gps_time_gap_rejections = 0;
  std::size_t gps_gate_rejections = 0;
  std::map<int, std::size_t> gps_factors_per_keyframe;
  std::size_t max_gps_factors_per_keyframe = 0;
  std::size_t accepted_gps_factor_count = 0;
  double max_keyframe_interval_sec = 0.0;
  double last_match_attempt_sec = kNeverSec;
  double absolute_reference_sec = 0.0;
  double distance_since_absolute_m = 0.0;
  Eigen::Isometry3d last_wheel_pose = Eigen::Isometry3d::Identity();
  double position_std_m = 0.0;

  Eigen::Isometry3d last_scan_odom_for_gate = Eigen::Isometry3d::Identity();
  double last_keyframe_sec = 0.0;  // all graph nodes are scan keyframes
  Eigen::Isometry3d last_keyframe_odom = Eigen::Isometry3d::Identity();
  std::deque<NodeRecord> nodes;
  std::deque<int> lidar_nodes;
  std::map<int, Eigen::Isometry3d> pose_cache;
  std::vector<PendingScan> pending_scans;
  std::vector<PendingGps> pending_gps;
  std::function<void(MatchResult&)> map_match_hook;
  std::string last_error;
  int rebuild_failure_mode = 0;  // test seam: 1 = worker throws, 2 = launch throws
  double scan_time_ms = 0.0;
  double match_time_ms = 0.0;
  double submap_rebuild_time_ms = 0.0;
  double last_inlier_ratio = 0.0;
  double last_lidar_odom_inlier_ratio = 0.0;
  std::optional<Eigen::Vector3d> latest_position_and_yaw_std;
  std::vector<KeyframeCloud> mapping_keyframes;
  bool mapping_paused = false;
  std::optional<double> match_eigenvalue_ratio;
  std::optional<Eigen::Vector3d> match_weakest_axis;

  // nullopt for a node the window no longer holds: callers skip the work instead of using another
  // node's pose, and last_error says so until sync_lidar_submap() sees every pose again.
  std::optional<Eigen::Isometry3d> pose_of(int index) {
    if (const auto pose = graph->pose(index)) return pose_cache[index] = *pose;
    const auto cached = pose_cache.find(index);
    if (cached != pose_cache.end()) return cached->second;
    last_error = kUnknownNodeError + std::to_string(index);
    return std::nullopt;
  }

  // Optimizes; returns true when the optimization failed and the window was restarted from the
  // current estimate (node indices are renumbered from 0 and the lidar sub-map is dropped, so
  // callers must re-read nodes.back().index).
  bool refresh_after_optimize(bool preserve_target_tilt = false) {
    bool was_restarted = false;
    if (!graph->optimize()) {
      last_error = "pose graph optimization failed, window restarted: " + graph->last_error();
      const Eigen::Isometry3d current = pose_cache.at(nodes.back().index);
      const NodeRecord latest_node = nodes.back();
      gps_factors_per_keyframe.clear();
      if (parameters.mapping) {
        // Keep the retained factors and keyframe clouds: only the fixed-lag window is rebuilt,
        // and the new window continues with a fresh node index.
        const int restarted_index = graph->restart_window(
            latest_node.stamp, current, current, parameters.initial_sigma_xy_m,
            parameters.initial_sigma_yaw_rad);
        nodes.clear();
        nodes.push_back({restarted_index, latest_node.stamp, latest_node.odom_to_base});
        lidar = LidarOdometry(parameters.lidar_odometry);
        lidar_nodes.clear();
        pose_cache.clear();
        pose_cache[restarted_index] = current;
        if (!graph->optimize()) last_error += " (restart also failed)";
      } else {
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
      }
      was_restarted = true;
    }
    const int latest = nodes.back().index;
    if (const auto pose = graph->pose(latest)) pose_cache[latest] = *pose;
    if (const auto std_values = graph->position_and_yaw_std(latest)) {
      position_std_m = std::max((*std_values)[0], (*std_values)[1]);
      latest_position_and_yaw_std = *std_values;
    }
    const Eigen::Isometry3d optimized_target = pose_cache[latest] * invert_isometry(nodes.back().odom_to_base);
    if (preserve_target_tilt) {
      corrector.set_planar_target(optimized_target, last_wheel_pose);
    } else {
      corrector.set_target(optimized_target);
    }
    while (nodes.size() > 1 && nodes.front().stamp < nodes.back().stamp - parameters.graph.window_s) {
      gps_factors_per_keyframe.erase(nodes.front().index);
      pose_cache.erase(nodes.front().index);
      nodes.pop_front();
    }
    // The graph no longer holds the poses of LiDAR keyframes older than the window (a gap in the
    // input longer than window_s drops all of them), so the sub-map must forget them as well.
    std::size_t stale_lidar_keyframe_count = 0;
    while (!lidar_nodes.empty() && lidar_nodes.front() < nodes.front().index) {
      lidar_nodes.pop_front();
      ++stale_lidar_keyframe_count;
    }
    lidar.drop_oldest_keyframes(stale_lidar_keyframe_count);
    return was_restarted;
  }

  void sync_lidar_submap() {
    const auto start = Clock::now();
    std::vector<Eigen::Isometry3d> poses;
    for (const int index : lidar_nodes) {
      const auto pose = pose_of(index);
      if (!pose) return;  // keep the sub-map as it is rather than draw a keyframe at a wrong pose
      poses.push_back(*pose);
    }
    // Every pose is known again, so the unknown-node error no longer describes the state.
    if (last_error.rfind(kUnknownNodeError, 0) == 0) last_error.clear();
    lidar.update_keyframe_poses(poses);
    lidar.rebuild_submap();
    submap_rebuild_time_ms = elapsed_ms(start);
  }

  static MapTargetBuild build_map_target(const PointList& all_points, const Eigen::Vector2d& center) {
    PointList cropped;
    for (const auto& point : all_points) {
      if ((point.head<2>().cast<double>() - center).norm() <= kMapSubmapRadiusM) cropped.push_back(point);
    }
    return {PreparedTarget(cropped), center};
  }

  // The first target is built synchronously; later rebuilds (every 50 m) run on a worker thread so
  // the 50 Hz output is not stalled, and the old target is used until the new one is ready.
  void ensure_map_target(const Eigen::Vector3d& center) {
    if (map_points.empty()) return;
    if (pending_map_target.valid() &&
        pending_map_target.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
      MapTargetBuild built;
      try {
        built = pending_map_target.get();
      } catch (const std::exception& exception) {
        last_error = std::string("prior-map rebuild failed, keeping the old target: ") + exception.what();
        built.target = PreparedTarget();
      }
      if (built.target.is_valid()) {
        map_target = built.target;
        map_target_center = built.center;
        has_map_target = true;
      }
    }
    if (!has_map_target) {
      MapTargetBuild built = build_map_target(map_points, center.head<2>());
      map_target = built.target;
      map_target_center = built.center;
      has_map_target = map_target.is_valid();
      return;
    }
    if ((center.head<2>() - map_target_center).norm() < parameters.map_submap_rebuild_distance_m) return;
    if (pending_map_target.valid()) return;  // already rebuilding
    const Eigen::Vector2d rebuild_center = center.head<2>();
    try {
      if (rebuild_failure_mode == 2) throw std::system_error(std::make_error_code(std::errc::resource_unavailable_try_again));
      const bool should_fail = rebuild_failure_mode == 1;
      pending_map_target = std::async(std::launch::async, [this, rebuild_center, should_fail] {
        if (should_fail) throw std::runtime_error("injected rebuild failure");
        return build_map_target(map_points, rebuild_center);
      });
    } catch (const std::system_error& error) {
      // Thread could not be started: keep matching against the old target.
      last_error = std::string("prior-map rebuild could not start, keeping the old target: ") + error.what();
    }
    rebuild_failure_mode = 0;
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
    last_inlier_ratio = result.inlier_ratio;
    if (map_match_hook) map_match_hook(result);
    match_eigenvalue_ratio.reset();
    match_weakest_axis.reset();
    const Eigen::Matrix3d translation_hessian = result.information.block<3, 3>(3, 3);
    if (translation_hessian.allFinite()) {
      const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(
          0.5 * (translation_hessian + translation_hessian.transpose()));
      if (solver.info() == Eigen::Success && solver.eigenvalues()[2] > 0.0) {
        match_eigenvalue_ratio = solver.eigenvalues()[0] / solver.eigenvalues()[2];
        match_weakest_axis = solver.eigenvectors().col(0);
      }
    }
    // Without a map match or GPS the estimate is less certain, so the gate widens with it.
    const bool is_tracking = stamp - last_map_sec <= parameters.state.tracking_window_s;
    const double gate_scale = is_tracking ? 1.0 : std::max(1.0, 3.0 * position_std_m);
    const Eigen::Isometry3d difference = invert_isometry(estimate) * result.target_to_source;
    const bool is_accepted = result.converged && result.inlier_ratio >= parameters.map_matcher.inlier_ratio_min &&
                             difference.translation().norm() <= parameters.map_matcher.gate_translation_m * gate_scale &&
                             rotation_angle_rad(difference) <= parameters.map_matcher.gate_rotation_rad * gate_scale;
    if (!is_accepted) return;
    graph->add_map_factor(node_index, result.target_to_source,
                          information_from_hessian(result.information, parameters.map_information));
    last_map_sec = stamp;
    if (match_eigenvalue_ratio && *match_eigenvalue_ratio >= parameters.map_information.degeneracy_ratio) {
      mark_absolute(stamp);
    }
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
      if (const auto std_values = graph->position_and_yaw_std(0)) {
        latest_position_and_yaw_std = *std_values;
      }
      pose_cache[0] = initial;
      nodes.push_back({0, pending.stamp, *odom_now});
      lidar.add_keyframe(initial, cloud);
      lidar_nodes.push_back(0);
      last_scan_odom_for_gate = *odom_now;
      last_keyframe_sec = pending.stamp;
      last_keyframe_odom = *odom_now;
      is_graph_started = true;
      if (parameters.mapping && !mapping_paused) {
        mapping_keyframes.push_back(
            {0, pending.stamp, downsample_voxel(*deskewed, parameters.keyframe_cloud_voxel_m)});
      }
      scan_time_ms = elapsed_ms(start);
      return;
    }

    const NodeRecord last = nodes.back();
    const auto last_pose = pose_of(last.index);
    if (!last_pose) return;
    const Eigen::Isometry3d wheel_relative = invert_isometry(last.odom_to_base) * *odom_now;
    const Eigen::Isometry3d prediction = corrector.target() * *odom_now;

    Eigen::Isometry3d best = prediction;
    std::optional<MatchResult> lidar_result;
    constexpr double kWheelRateIntervalSec = 0.05;
    const auto recent_odom = buffer.interpolate(pending.stamp - kWheelRateIntervalSec);
    double wheel_yaw_rate = 1.0;
    if (recent_odom) {
      const Eigen::Isometry3d recent_relative = invert_isometry(*recent_odom) * *odom_now;
      wheel_yaw_rate = std::atan2(recent_relative.rotation()(1, 0), recent_relative.rotation()(0, 0)) / kWheelRateIntervalSec;
    }
    if (lidar.has_submap()) {
      // Keep the gate's scan-to-scan wheel distance independent of the optimized prediction.
      const Eigen::Isometry3d scan_relative = invert_isometry(last_scan_odom_for_gate) * *odom_now;
      MatchResult result = lidar.match(cloud, prediction, scan_relative.translation().norm(), wheel_yaw_rate);
      last_lidar_odom_inlier_ratio = result.inlier_ratio;
      if (result.is_accepted) {
        best = result.target_to_source;
        lidar_result = result;
        corrector.set_target(best * invert_isometry(*odom_now));
      }
    }

    const double moved_since_keyframe_m =
        (odom_now->translation() - last_keyframe_odom.translation()).norm();
    last_scan_odom_for_gate = *odom_now;
    // One microsecond covers timestamp subtraction rounding, not a scan-period delay.
    constexpr double kKeyframeTimeToleranceSec = 1e-6;
    const bool is_keyframe = moved_since_keyframe_m >= parameters.keyframe_distance_m ||
                             pending.stamp - last_keyframe_sec + kKeyframeTimeToleranceSec >= parameters.keyframe_time_s;
    if (is_keyframe) {
      double yaw_disagreement = 0.0;
      if (lidar_result) {
        const Eigen::Isometry3d lidar_relative = invert_isometry(*last_pose) * best;
        const Eigen::Matrix3d difference = wheel_relative.rotation().transpose() * lidar_relative.rotation();
        yaw_disagreement = std::abs(std::atan2(difference(1, 0), difference(0, 0)));
      }
      const int index = graph->add_node_with_wheel_factor(pending.stamp, best, wheel_relative, yaw_disagreement);
      if (lidar_result) {
        graph->add_lidar_odometry_factor(
            last.index, index, invert_isometry(*last_pose) * best,
            information_from_hessian(lidar_result->information, parameters.lidar_odometry_information));
      }
      nodes.push_back({index, pending.stamp, *odom_now});
      max_keyframe_interval_sec = std::max(max_keyframe_interval_sec, pending.stamp - last_keyframe_sec);
      last_keyframe_sec = pending.stamp;
      last_keyframe_odom = *odom_now;
      pose_cache[index] = best;
      try_map_match(index, pending.stamp, best, cloud);
      refresh_after_optimize();
      const int current_index = nodes.back().index;  // renumbered after a restart
      const auto current_pose = pose_of(current_index);
      if (!current_pose) return;
      lidar.add_keyframe(*current_pose, cloud);
      lidar_nodes.push_back(current_index);
      while (static_cast<int>(lidar_nodes.size()) > parameters.lidar_odometry.submap_keyframes) {
        lidar_nodes.pop_front();
      }
      sync_lidar_submap();
      if (parameters.mapping && !mapping_paused) {
        mapping_keyframes.push_back({current_index, pending.stamp,
                                     downsample_voxel(*deskewed, parameters.keyframe_cloud_voxel_m)});
      }
    }
    scan_time_ms = elapsed_ms(start);
  }

  void process_gps(const PendingGps& pending) {
    if (!gps || !is_graph_started) return;
    const NodeRecord* nearest = &nodes.front();
    for (const NodeRecord& node : nodes) {
      if (std::abs(node.stamp - pending.stamp) < std::abs(nearest->stamp - pending.stamp)) nearest = &node;
    }
    if (std::abs(nearest->stamp - pending.stamp) > parameters.gps_keyframe_max_gap_s) {
      ++gps_time_gap_rejections;
      return;
    }
    const auto odom_at_fix = buffer.interpolate(pending.stamp);
    if (!odom_at_fix) return;
    const int node_index = nearest->index;
    const auto found_pose = pose_of(node_index);
    if (!found_pose) return;
    const Eigen::Isometry3d keyframe_pose = *found_pose;
    const Eigen::Isometry3d predicted = keyframe_pose * invert_isometry(nearest->odom_to_base) * *odom_at_fix;
    const GpsDecision decision = gps->evaluate(pending.fix, predicted, position_std_m);
    if (decision.rejection == GpsRejection::kGate) ++gps_gate_rejections;
    if (decision.gate_would_reject) conflict.record(*decision.gate_would_reject);
    if (decision.measurement) {
      ++accepted_gps_factor_count;
      max_gps_factors_per_keyframe = std::max(max_gps_factors_per_keyframe, ++gps_factors_per_keyframe[node_index]);
      const Eigen::Vector3d displacement = predicted.translation() - keyframe_pose.translation();
      graph->add_gps_factor(node_index, decision.measurement->base_position - displacement,
                            decision.measurement->sigma_m);
      last_gps_sec = pending.stamp;
      mark_absolute(pending.stamp);
      refresh_after_optimize(true);
    }
  }

  StateInputs make_state_inputs(double now_sec) const {
    StateInputs inputs;
    inputs.allow_uncertain_map_tracking = true;
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

void Localizer::inject_map_rebuild_failure_for_test(int mode) { implementation_->rebuild_failure_mode = mode; }

std::vector<Eigen::Isometry3d> Localizer::lidar_keyframe_poses_for_test() const {
  return implementation_->lidar.keyframe_poses();
}

void Localizer::inject_optimization_failure_for_test() { implementation_->graph->inject_failure_once(); }

double Localizer::last_scan_time_ms() const { return implementation_->scan_time_ms; }
double Localizer::last_match_time_ms() const { return implementation_->match_time_ms; }
double Localizer::last_submap_rebuild_time_ms() const { return implementation_->submap_rebuild_time_ms; }
double Localizer::last_inlier_ratio() const { return implementation_->last_inlier_ratio; }
double Localizer::last_lidar_odom_inlier_ratio() const {
  return implementation_->last_lidar_odom_inlier_ratio;
}
double Localizer::last_map_match_stamp_sec() const { return implementation_->last_map_sec; }
double Localizer::odom_only_distance_m() const {
  return implementation_->distance_since_absolute_m;
}
std::optional<Eigen::Vector3d> Localizer::latest_position_and_yaw_std() const {
  return implementation_->latest_position_and_yaw_std;
}
std::optional<double> Localizer::last_match_eigenvalue_ratio() const {
  return implementation_->match_eigenvalue_ratio;
}
std::optional<Eigen::Vector3d> Localizer::last_match_weakest_axis() const {
  return implementation_->match_weakest_axis;
}
double Localizer::last_accepted_gps_stamp_sec() const { return implementation_->last_gps_sec; }
std::size_t Localizer::gps_time_gap_rejection_count() const { return implementation_->gps_time_gap_rejections; }
std::size_t Localizer::gps_gate_rejection_count() const { return implementation_->gps_gate_rejections; }
std::size_t Localizer::gps_conflict_window_reject_count() const {
  return static_cast<std::size_t>(implementation_->conflict.rejected_count());
}
std::size_t Localizer::max_gps_factors_per_keyframe() const { return implementation_->max_gps_factors_per_keyframe; }
double Localizer::max_keyframe_interval_sec() const { return implementation_->max_keyframe_interval_sec; }
double Localizer::last_absolute_observation_stamp_sec() const { return implementation_->absolute_reference_sec; }

const std::vector<KeyframeCloud>& Localizer::mapping_keyframes() const {
  return implementation_->mapping_keyframes;
}

std::size_t Localizer::mapping_keyframe_count() const {
  return implementation_->mapping_keyframes.size();
}

std::size_t Localizer::accepted_gps_factor_count() const {
  return implementation_->accepted_gps_factor_count;
}

void Localizer::interrupt_mapping() { implementation_->mapping_paused = true; }

bool Localizer::save_map(const MapOutputParameters& parameters, MapOutputResult& result,
                         std::string& error) {
  Implementation& self = *implementation_;
  if (!self.parameters.mapping) {
    error = "save_map requires mapping mode";
    return false;
  }
  if (!self.graph->batch_optimize(parameters.batch_max_iterations)) {
    error = "batch optimization failed: " + self.graph->last_error();
    return false;
  }
  std::map<int, const BatchPose*> poses;
  for (const BatchPose& pose : self.graph->batch_poses()) poses[pose.index] = &pose;
  std::vector<TrajectoryPoint> trajectory;
  trajectory.reserve(self.mapping_keyframes.size());
  for (const KeyframeCloud& keyframe : self.mapping_keyframes) {
    const auto found = poses.find(keyframe.node_index);
    if (found == poses.end()) continue;
    TrajectoryPoint point;
    point.node_index = keyframe.node_index;
    point.stamp_sec = keyframe.stamp_sec;
    point.map_to_base = found->second->map_to_base;
    if (found->second->position_and_yaw_std) {
      point.position_std_m =
          std::max((*found->second->position_and_yaw_std)[0],
                   (*found->second->position_and_yaw_std)[1]);
    } else {
      // A failed marginal solve must be visible in trajectory.csv, not read as a perfect 0 m.
      point.position_std_m = std::numeric_limits<double>::quiet_NaN();
    }
    trajectory.push_back(point);
  }
  MapOutputParameters output_parameters = parameters;
  output_parameters.batch_optimized = true;
  output_parameters.has_gps_origin = self.gps_origin.has_value();
  if (self.gps_origin) output_parameters.gps_origin = *self.gps_origin;
  try {
    result = write_map_outputs(output_parameters, self.mapping_keyframes, trajectory);
  } catch (const std::exception& exception) {
    error = exception.what();
    return false;
  }
  return true;
}

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
    self.corrector.reset(*self.requested_initial_pose * invert_isometry(odom_to_base));
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
