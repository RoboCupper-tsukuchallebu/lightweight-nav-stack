#pragma once

#include <chrono>
#include <deque>
#include <cmath>
#include <limits>
#include <memory>
#include <map>
#include <climits>
#include <optional>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "lightnav_controller/arbitrate.hpp"
#include "lightnav_controller/arc_markers.hpp"
#include "lightnav_controller/avoid_layer.hpp"
#include "lightnav_controller/follow_layer.hpp"
#include "lightnav_controller/freshness.hpp"
#include "lightnav_msgs/msg/follow_status.hpp"
#include "lightnav_msgs/msg/obstacle_status.hpp"
#include "lightnav_msgs/msg/path_with_id.hpp"
#include "lightnav_msgs/msg/velocity_command.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace lightnav_controller {

namespace {

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

bool rawGridMatches(const LocalGrid& inflated, const LocalGrid& raw, bool stamp_matches,
                    double age_s, double timeout_s) {
  return stamp_matches && age_s >= 0.0 && age_s <= timeout_s &&
         raw.width_cells == inflated.width_cells && raw.height_cells == inflated.height_cells &&
         raw.resolution_m == inflated.resolution_m && raw.origin_x_m == inflated.origin_x_m &&
         raw.origin_y_m == inflated.origin_y_m;
}

}  // namespace

struct ReceivedGrid {
  LocalGrid grid;
  rclcpp::Time received_at;
  builtin_interfaces::msg::Time stamp;
};

class RawGridBuffer {
 public:
  void clear() { grids_.clear(); }
  void push(ReceivedGrid grid) {
    grids_.push_back(std::move(grid));
    if (grids_.size() > 4) grids_.pop_front();
  }
  const LocalGrid* match(const ReceivedGrid& inflated, const rclcpp::Time& now, double timeout) const {
    for (auto it = grids_.rbegin(); it != grids_.rend(); ++it) {
      if (rawGridMatches(inflated.grid, it->grid, it->stamp == inflated.stamp,
                         (now - it->received_at).seconds(), timeout)) return &it->grid;
    }
    return nullptr;
  }
 private:
  std::deque<ReceivedGrid> grids_;
};

namespace {

double yawFromQuaternion(const geometry_msgs::msg::Quaternion& quaternion) {
  return std::atan2(2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
                    1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z));
}

}  // namespace

class ControllerNode : public rclcpp_lifecycle::LifecycleNode {
 public:
  explicit ControllerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
      : rclcpp_lifecycle::LifecycleNode("controller_node", options) {
    rcl_interfaces::msg::ParameterDescriptor structural;
    structural.read_only = true;
    structural.description = "Structural parameter fixed at construction";
    declare_parameter("grid_frame", "base_link", structural);
    declare_parameter("control_hz", 20.0, structural);
    declare_parameter("stop_zone_x_m", 1.0, structural);
    declare_parameter("stop_zone_y_half_m", 0.4, structural);
    declare_parameter("arc_markers_rate_hz", 2.0, structural);
    declare_parameter("lookahead_m", 1.0);
    declare_parameter("search_window_m", 5.0);
    declare_parameter("v_max", 0.6);
    declare_parameter("v_min", 0.1);
    declare_parameter("v_slow", 0.25);
    declare_parameter("w_max", 0.4);
    declare_parameter("w_rotate", 0.3);
    declare_parameter("k_end", 0.5);
    declare_parameter("goal_tolerance_m", 0.3);
    declare_parameter("rotate_in_place_heading_rad", 1.0);
    declare_parameter("pose_timeout_s", 0.5);
    declare_parameter("obstacle_status_timeout_s", 0.5);
    declare_parameter("grid_timeout_s", 0.5);
    declare_parameter("v_avoid", 0.25);
    declare_parameter("arc_candidate_count", 15);
    declare_parameter("arc_length_m", 2.0);
    declare_parameter("arc_step_m", 0.1);
    declare_parameter("max_path_deviation_m", std::numeric_limits<double>::quiet_NaN());
    declare_parameter("w_path", 1.0);
    declare_parameter("w_heading", 0.5);
    declare_parameter("w_smooth", 0.2);
    declare_parameter("avoid_trigger_m", 2.5);
    declare_parameter("avoid_release_m", 3.0);
    declare_parameter("rejoin_distance_m", 0.5);
    declare_parameter("avoid_release_hold_s", 1.0);
    declare_parameter("stuck_timeout_s", 10.0);
    for (const auto& name : safety_names_) {
      baseline_[name.first] = get_parameter(name.first).as_double();
    }
    validation_handle_ = add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter>& parameters) { return validateChange(parameters); });
    apply_handle_ = add_post_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter>& parameters) { applyChange(parameters); });
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State&) override {
    const auto validation = validateValues({}, true);
    if (!validation.empty()) {
      RCLCPP_ERROR(get_logger(), "%s", validation.c_str());
      return CallbackReturn::FAILURE;
    }
    const double max_path_deviation_m = get_parameter("max_path_deviation_m").as_double();
    if (!std::isfinite(max_path_deviation_m) || max_path_deviation_m <= 0.0) {
      RCLCPP_ERROR(get_logger(),
                   "Specify max_path_deviation_m in the course-specific yaml; "
                   "the value must be finite and positive (got %f)", max_path_deviation_m);
      return CallbackReturn::FAILURE;
    }
    arc_markers_rate_hz_ = get_parameter("arc_markers_rate_hz").as_double();
    if (!std::isfinite(arc_markers_rate_hz_) || arc_markers_rate_hz_ < 0.0) {
      RCLCPP_ERROR(get_logger(), "arc_markers_rate_hz must be finite and nonnegative");
      return CallbackReturn::FAILURE;
    }
    grid_frame_ = get_parameter("grid_frame").as_string();
    if (grid_frame_.empty()) {
      RCLCPP_ERROR(get_logger(), "parameter grid_frame must not be empty");
      return CallbackReturn::FAILURE;
    }
    const std::vector<std::string> positive_names = {
        "control_hz", "stop_zone_x_m", "stop_zone_y_half_m", "lookahead_m", "search_window_m", "v_max", "v_min", "v_slow",
        "w_max", "w_rotate", "k_end", "goal_tolerance_m", "rotate_in_place_heading_rad",
        "pose_timeout_s", "obstacle_status_timeout_s", "grid_timeout_s",
        "v_avoid", "arc_length_m", "arc_step_m", "w_path", "w_heading",
        "w_smooth", "avoid_trigger_m", "avoid_release_m", "rejoin_distance_m",
        "avoid_release_hold_s", "stuck_timeout_s"};
    for (const std::string& name : positive_names) {
      const double value = get_parameter(name).as_double();
      if (!std::isfinite(value) || value <= 0.0) {
        RCLCPP_ERROR(get_logger(), "parameter %s must be finite and positive (got %f)",
                     name.c_str(), value);
        return CallbackReturn::FAILURE;
      }
    }
    FollowParameters follow_parameters;
    follow_parameters.lookahead_m = get_parameter("lookahead_m").as_double();
    follow_parameters.search_window_m = get_parameter("search_window_m").as_double();
    follow_parameters.v_max_mps = get_parameter("v_max").as_double();
    follow_parameters.v_min_mps = get_parameter("v_min").as_double();
    follow_parameters.v_slow_mps = get_parameter("v_slow").as_double();
    follow_parameters.w_max_rps = get_parameter("w_max").as_double();
    follow_parameters.w_rotate_rps = get_parameter("w_rotate").as_double();
    follow_parameters.k_end_per_s = get_parameter("k_end").as_double();
    follow_parameters.goal_tolerance_m = get_parameter("goal_tolerance_m").as_double();
    follow_parameters.rotate_in_place_heading_rad =
        get_parameter("rotate_in_place_heading_rad").as_double();
    timeouts_.pose_timeout_s = get_parameter("pose_timeout_s").as_double();
    timeouts_.obstacle_status_timeout_s = get_parameter("obstacle_status_timeout_s").as_double();

    follow_layer_ = std::make_unique<FollowLayer>(follow_parameters);
    AvoidParameters avoid_parameters;
    avoid_parameters.stop_zone_x_m = get_parameter("stop_zone_x_m").as_double();
    avoid_parameters.stop_zone_y_half_m = get_parameter("stop_zone_y_half_m").as_double();
    avoid_parameters.v_avoid_mps = get_parameter("v_avoid").as_double();
    avoid_parameters.w_max_rps = follow_parameters.w_max_rps;
    avoid_parameters.arc_candidate_count = static_cast<int>(get_parameter("arc_candidate_count").as_int());
    if (avoid_parameters.arc_candidate_count < 1) {
      RCLCPP_ERROR(get_logger(), "parameter arc_candidate_count must be at least 1");
      return CallbackReturn::FAILURE;
    }
    avoid_parameters.arc_length_m = get_parameter("arc_length_m").as_double();
    avoid_parameters.arc_step_m = get_parameter("arc_step_m").as_double();
    avoid_parameters.max_path_deviation_m = max_path_deviation_m;
    avoid_parameters.w_path = get_parameter("w_path").as_double();
    avoid_parameters.w_heading = get_parameter("w_heading").as_double();
    avoid_parameters.w_smooth = get_parameter("w_smooth").as_double();
    avoid_parameters.avoid_trigger_m = get_parameter("avoid_trigger_m").as_double();
    avoid_parameters.avoid_release_m = get_parameter("avoid_release_m").as_double();
    avoid_parameters.rejoin_distance_m = get_parameter("rejoin_distance_m").as_double();
    avoid_parameters.avoid_release_hold_s = get_parameter("avoid_release_hold_s").as_double();
    avoid_parameters.stuck_timeout_s = get_parameter("stuck_timeout_s").as_double();
    timeouts_.grid_timeout_s = get_parameter("grid_timeout_s").as_double();
    avoid_parameters.goal_tolerance_m = follow_parameters.goal_tolerance_m;
    avoid_parameters.k_end_per_s = follow_parameters.k_end_per_s;
    avoid_layer_ = std::make_unique<ArcAvoidLayer>(avoid_parameters);
    last_grid_.reset();
    raw_grids_.clear();
    last_pose_.reset();
    last_obstacle_status_.reset();

    marker_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "/lightnav/controller/arc_candidates", rclcpp::QoS(1).reliable());
    command_publisher_ = create_publisher<lightnav_msgs::msg::VelocityCommand>(
        "/lightnav/cmd_vel_raw", rclcpp::QoS(10).reliable());
    status_publisher_ = create_publisher<lightnav_msgs::msg::FollowStatus>(
        "/lightnav/follow_status", rclcpp::QoS(10).reliable());
    path_subscription_ = create_subscription<lightnav_msgs::msg::PathWithId>(
        "/lightnav/path", rclcpp::QoS(1).reliable().transient_local(),
        [this](lightnav_msgs::msg::PathWithId::ConstSharedPtr message) { onPath(*message); });
    pose_subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "/lightnav/pose", rclcpp::QoS(10).reliable(),
        [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr message) {
          last_pose_ = ReceivedPose{*message, now()};
        });
    obstacle_subscription_ = create_subscription<lightnav_msgs::msg::ObstacleStatus>(
        "/lightnav/obstacle_status", rclcpp::QoS(10).reliable(),
        [this](lightnav_msgs::msg::ObstacleStatus::ConstSharedPtr message) {
          last_obstacle_status_ = ReceivedObstacleStatus{*message, now()};
        });
    raw_grid_subscription_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/lightnav/local_grid_raw", rclcpp::QoS(1).reliable(),
        [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr message) { onGrid(*message, true); });
    grid_subscription_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/lightnav/local_grid", rclcpp::QoS(1).reliable(),
        [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr message) { onGrid(*message); });
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State& state) override {
    LifecycleNode::on_activate(state);
    const auto period = std::chrono::duration<double>(1.0 / get_parameter("control_hz").as_double());
    timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(period),
                               [this]() { controlStep(); });
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& state) override {
    timer_.reset();
    LifecycleNode::on_deactivate(state);
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State&) override {
    releaseOutputs();
    return CallbackReturn::SUCCESS;
  }

  // Reached from any state, so it must stop the timer and the outputs itself.
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State&) override {
    releaseOutputs();
    return CallbackReturn::SUCCESS;
  }

  bool isTimerRunning() const { return timer_ != nullptr; }

  bool isReleased() const {
    return timer_ == nullptr && command_publisher_ == nullptr && status_publisher_ == nullptr;
  }

 private:
  using Values = std::map<std::string, rclcpp::Parameter>;

  std::string validateValues(const Values& changes, bool configuring) const {
    auto value = [this, &changes](const std::string& name) {
      const auto found = changes.find(name);
      return found == changes.end() ? get_parameter(name) : found->second;
    };
    for (const auto& entry : positive_names_) {
      const auto& name = entry.first;
      const double number = value(name).as_double();
      // The missing course-specific default is allowed until configure.
      if (!configuring && name == "max_path_deviation_m" &&
          changes.count(name) == 0 && !std::isfinite(number)) continue;
      if (!std::isfinite(number) || number <= 0.0) return name + ": must be finite and positive";
    }
    const auto count = value("arc_candidate_count").as_int();
    if (count < 1 || count > INT_MAX) return "arc_candidate_count: must be between 1 and INT_MAX";
    if (!(value("v_min").as_double() <= value("v_slow").as_double() &&
          value("v_slow").as_double() <= value("v_max").as_double()))
      return "v_min <= v_slow <= v_max is required";
    if (!(value("avoid_trigger_m").as_double() < value("avoid_release_m").as_double()))
      return "avoid_trigger_m must be less than avoid_release_m";
    return {};
  }

  rcl_interfaces::msg::SetParametersResult validateChange(
      const std::vector<rclcpp::Parameter>& parameters) const {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = false;
    Values changes;
    for (const auto& parameter : parameters) {
      const auto& name = parameter.get_name();
      if (name == "use_sim_time" || name == "grid_frame" || name == "control_hz" || name == "arc_markers_rate_hz") {
        result.reason = name + ": read-only after construction";
        break;
      }
      if (name == "arc_candidate_count") {
        if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) {
          result.reason = name + ": expected integer"; break;
        }
      } else if (positive_names_.count(name)) {
        if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
          result.reason = name + ": expected double"; break;
        }
        const auto safety = safety_names_.find(name);
        if (safety != safety_names_.end()) {
          const double requested = parameter.as_double();
          const double baseline = baseline_.at(name);
          if (!(std::isfinite(requested) && std::isfinite(baseline) &&
                (safety->second ? requested >= baseline : requested <= baseline))) {
            result.reason = name + ": must be finite and on the conservative side of the finite startup value";
            break;
          }
        }
      } else {
        result.reason = name + ": unsupported runtime parameter"; break;
      }
      changes.insert_or_assign(name, parameter);
    }
    if (result.reason.empty()) result.reason = validateValues(changes, false);
    result.successful = result.reason.empty();
    if (!result.successful) RCLCPP_WARN(get_logger(), "parameter rejected: %s", result.reason.c_str());
    return result;
  }

  void applyChange(const std::vector<rclcpp::Parameter>& parameters) {
    if (follow_layer_ && avoid_layer_) {
      FollowParameters follow;
      follow.lookahead_m = get_parameter("lookahead_m").as_double();
      follow.search_window_m = get_parameter("search_window_m").as_double();
      follow.v_max_mps = get_parameter("v_max").as_double();
      follow.v_min_mps = get_parameter("v_min").as_double();
      follow.v_slow_mps = get_parameter("v_slow").as_double();
      follow.w_max_rps = get_parameter("w_max").as_double();
      follow.w_rotate_rps = get_parameter("w_rotate").as_double();
      follow.k_end_per_s = get_parameter("k_end").as_double();
      follow.goal_tolerance_m = get_parameter("goal_tolerance_m").as_double();
      follow.rotate_in_place_heading_rad = get_parameter("rotate_in_place_heading_rad").as_double();
      AvoidParameters avoid;
      avoid.stop_zone_x_m = get_parameter("stop_zone_x_m").as_double();
      avoid.stop_zone_y_half_m = get_parameter("stop_zone_y_half_m").as_double();
      avoid.v_avoid_mps = get_parameter("v_avoid").as_double();
      avoid.w_max_rps = follow.w_max_rps;
      avoid.arc_candidate_count = static_cast<int>(get_parameter("arc_candidate_count").as_int());
      avoid.arc_length_m = get_parameter("arc_length_m").as_double();
      avoid.arc_step_m = get_parameter("arc_step_m").as_double();
      avoid.max_path_deviation_m = get_parameter("max_path_deviation_m").as_double();
      avoid.w_path = get_parameter("w_path").as_double();
      avoid.w_heading = get_parameter("w_heading").as_double();
      avoid.w_smooth = get_parameter("w_smooth").as_double();
      avoid.avoid_trigger_m = get_parameter("avoid_trigger_m").as_double();
      avoid.avoid_release_m = get_parameter("avoid_release_m").as_double();
      avoid.rejoin_distance_m = get_parameter("rejoin_distance_m").as_double();
      avoid.avoid_release_hold_s = get_parameter("avoid_release_hold_s").as_double();
      avoid.stuck_timeout_s = get_parameter("stuck_timeout_s").as_double();
      avoid.goal_tolerance_m = follow.goal_tolerance_m;
      avoid.k_end_per_s = follow.k_end_per_s;
      FreshnessTimeouts timeouts;
      timeouts.pose_timeout_s = get_parameter("pose_timeout_s").as_double();
      timeouts.obstacle_status_timeout_s = get_parameter("obstacle_status_timeout_s").as_double();
      timeouts.grid_timeout_s = get_parameter("grid_timeout_s").as_double();
      follow_layer_->updateParameters(follow);
      avoid_layer_->updateParameters(avoid);
      timeouts_ = timeouts;
    }
    for (const auto& parameter : parameters) {
      RCLCPP_INFO(get_logger(), "parameter changed: %s -> %s", parameter.get_name().c_str(),
                  parameter.value_to_string().c_str());
    }
  }

  void releaseOutputs() {
    timer_.reset();
    path_subscription_.reset();
    pose_subscription_.reset();
    obstacle_subscription_.reset();
    grid_subscription_.reset();
    raw_grid_subscription_.reset();
    marker_publisher_.reset();
    command_publisher_.reset();
    status_publisher_.reset();
    follow_layer_.reset();
    avoid_layer_.reset();
  }

  struct ReceivedPose {
    geometry_msgs::msg::PoseStamped message;
    rclcpp::Time received_at;
  };
  struct ReceivedObstacleStatus {
    lightnav_msgs::msg::ObstacleStatus message;
    rclcpp::Time received_at;
  };

  void onGrid(const nav_msgs::msg::OccupancyGrid& message, bool raw = false) {
    auto invalidate = [&]() { if (raw) raw_grids_.clear(); else last_grid_.reset(); };
    const auto& orientation = message.info.origin.orientation;
    const bool is_unrotated = std::fabs(orientation.x) < 1e-6 && std::fabs(orientation.y) < 1e-6 &&
                              std::fabs(orientation.z) < 1e-6 && std::fabs(orientation.w - 1.0) < 1e-6;
    if (message.header.frame_id != grid_frame_ || !is_unrotated ||
        !std::isfinite(message.info.resolution) || !std::isfinite(message.info.origin.position.x) ||
        !std::isfinite(message.info.origin.position.y)) {
      RCLCPP_WARN(get_logger(), "rejected local_grid: need frame %s, unrotated finite origin",
                  grid_frame_.c_str());
      invalidate();
      return;
    }
    LocalGrid grid;
    grid.width_cells = message.info.width;
    grid.height_cells = message.info.height;
    grid.resolution_m = message.info.resolution;
    grid.origin_x_m = message.info.origin.position.x;
    grid.origin_y_m = message.info.origin.position.y;
    grid.cells = message.data;
    if (grid.resolution_m <= 0.0 || grid.width_cells == 0 || grid.height_cells == 0 ||
        grid.cells.size() != grid.width_cells * grid.height_cells ||
        (raw && std::any_of(grid.cells.begin(), grid.cells.end(),
                            [](auto cell) { return cell != 0 && cell != 100; }))) {
      RCLCPP_WARN(get_logger(), "ignored malformed local_grid");
      invalidate();
      return;
    }
    ReceivedGrid received{std::move(grid), now(), message.header.stamp};
    if (raw) {
      raw_grids_.push(std::move(received));
    } else {
      last_grid_ = std::move(received);
    }
  }

  void onPath(const lightnav_msgs::msg::PathWithId& message) {
    std::vector<Point2D> points;
    points.reserve(message.path.poses.size());
    for (const auto& stamped_pose : message.path.poses) {
      points.push_back(Point2D{stamped_pose.pose.position.x, stamped_pose.pose.position.y});
    }
    if (!follow_layer_->tryAcceptPath(message.session_id, message.path_id, std::move(points))) {
      RCLCPP_DEBUG(get_logger(), "ignored path session=%u path_id=%u", message.session_id,
                   message.path_id);
    }
  }

  void controlStep() {
    if (follow_layer_ == nullptr) {
      return;
    }
    const rclcpp::Time current_time = now();
    InputAges ages;
    if (last_pose_.has_value()) {
      ages.pose_age_s = (current_time - last_pose_->received_at).seconds();
    }
    if (last_obstacle_status_.has_value()) {
      ages.obstacle_status_age_s = (current_time - last_obstacle_status_->received_at).seconds();
      ages.is_obstacle_status_input_stale = last_obstacle_status_->message.is_input_stale;
    }
    if (last_grid_.has_value()) {
      ages.grid_age_s = (current_time - last_grid_->received_at).seconds();
    }
    const Freshness freshness = evaluateFreshness(ages, timeouts_);
    const bool is_grid_fresh = freshness.is_grid_fresh;
    const bool is_pose_fresh = freshness.is_pose_fresh;
    const bool is_obstacle_status_fresh = freshness.is_obstacle_status_fresh;

    ArbitrationInput arbitration_input;
    arbitration_input.is_pose_fresh = is_pose_fresh;
    arbitration_input.is_grid_fresh = is_grid_fresh;
    FollowResult follow_result;
    if (is_pose_fresh) {
      const auto& pose_message = last_pose_->message.pose;
      const Pose2D pose{pose_message.position.x, pose_message.position.y,
                        yawFromQuaternion(pose_message.orientation)};
      FollowInputs follow_inputs;
      follow_inputs.is_obstacle_status_fresh = is_obstacle_status_fresh;
      if (is_obstacle_status_fresh) {
        follow_inputs.is_stop_zone_occupied = last_obstacle_status_->message.is_stop_zone_occupied;
        follow_inputs.is_slow_zone_occupied = last_obstacle_status_->message.is_slow_zone_occupied;
      }
      follow_result = follow_layer_->compute(pose, follow_inputs);
      arbitration_input.follow = follow_result.output;
      const LocalGrid* raw_grid = nullptr;
      if (is_grid_fresh) {
        raw_grid = raw_grids_.match(*last_grid_, current_time, timeouts_.grid_timeout_s);
      }
      arbitration_input.avoid = avoid_layer_->compute(AvoidContext{
          pose, follow_layer_->pathPoints(), follow_result.closest_index,
          arbitration_input.is_grid_fresh, is_grid_fresh ? &last_grid_->grid : nullptr,
          current_time.seconds(),
          follow_result.remaining_distance_m, follow_inputs.is_stop_zone_occupied,
          follow_inputs.is_obstacle_status_fresh, raw_grid});
    }
    const bool marker_active = arcMarkersActive(is_pose_fresh, arbitration_input.avoid.is_asserted);
    const auto marker_time = std::chrono::steady_clock::now().time_since_epoch();
    if (marker_publisher_ && marker_rate_.ready(
        std::chrono::duration<double>(marker_time).count(),
        arc_markers_rate_hz_, marker_publisher_->get_subscription_count() > 0)) {
      marker_publisher_->publish(makeArcMarkers(avoid_layer_->candidates(), avoid_layer_->selectedCandidate(),
          get_parameter("arc_length_m").as_double(), get_parameter("arc_step_m").as_double(),
          grid_frame_, current_time, marker_active, arbitration_input.avoid.reason,
          arc_markers_rate_hz_));
    }
    const ArbitrationResult result = arbitrate(arbitration_input);
    publish(result, follow_result);
  }

  void publish(const ArbitrationResult& result, const FollowResult& follow_result) {
    const rclcpp::Time stamp = now();
    lightnav_msgs::msg::VelocityCommand command;
    command.header.stamp = stamp;
    command.twist.linear.x = result.twist.linear_mps;
    command.twist.angular.z = result.twist.angular_rps;
    command.is_hard_stop = result.is_hard_stop;
    command.source_layer = static_cast<uint8_t>(result.source_layer);
    command.reason = static_cast<uint8_t>(result.reason);
    command_publisher_->publish(command);

    lightnav_msgs::msg::FollowStatus status;
    status.header.stamp = stamp;
    status.state = static_cast<uint8_t>(result.reason);
    status.active_layer = static_cast<uint8_t>(result.source_layer);
    status.session_id = follow_layer_->sessionId();
    status.path_id = follow_layer_->pathId();
    status.closest_index = static_cast<uint32_t>(follow_result.closest_index);
    status.remaining_distance_m = static_cast<float>(follow_result.remaining_distance_m);
    status.path_deviation_m = static_cast<float>(follow_result.path_deviation_m);
    status.commanded_linear_mps = static_cast<float>(result.twist.linear_mps);
    status.is_stuck = result.reason == StopReason::kStuck;
    status_publisher_->publish(status);
  }

  const std::map<std::string, bool> safety_names_ = {
      {"v_min", false}, {"max_path_deviation_m", false}, {"pose_timeout_s", false},
      {"obstacle_status_timeout_s", false}, {"grid_timeout_s", false},
      {"avoid_trigger_m", true}, {"avoid_release_m", true}, {"arc_step_m", false},
      {"arc_length_m", true}, {"rejoin_distance_m", false}, {"avoid_release_hold_s", true}};
  const std::map<std::string, bool> positive_names_ = {
      {"lookahead_m", true}, {"search_window_m", true}, {"v_max", true}, {"v_min", true},
      {"v_slow", true}, {"w_max", true}, {"w_rotate", true}, {"k_end", true},
      {"goal_tolerance_m", true}, {"rotate_in_place_heading_rad", true},
      {"pose_timeout_s", true}, {"obstacle_status_timeout_s", true}, {"grid_timeout_s", true},
      {"v_avoid", true}, {"arc_length_m", true}, {"arc_step_m", true},
      {"max_path_deviation_m", true}, {"w_path", true}, {"w_heading", true}, {"w_smooth", true},
      {"avoid_trigger_m", true}, {"avoid_release_m", true}, {"rejoin_distance_m", true},
      {"avoid_release_hold_s", true}, {"stuck_timeout_s", true}};
  std::map<std::string, double> baseline_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr validation_handle_;
  rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr apply_handle_;
  std::unique_ptr<FollowLayer> follow_layer_;
  std::unique_ptr<ArcAvoidLayer> avoid_layer_;
  FreshnessTimeouts timeouts_;
  std::string grid_frame_;
  std::optional<ReceivedGrid> last_grid_;
  RawGridBuffer raw_grids_;
  std::optional<ReceivedPose> last_pose_;
  std::optional<ReceivedObstacleStatus> last_obstacle_status_;
  double arc_markers_rate_hz_{0.0};
  ArcMarkerRate marker_rate_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<lightnav_msgs::msg::PathWithId>::SharedPtr path_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscription_;
  rclcpp::Subscription<lightnav_msgs::msg::ObstacleStatus>::SharedPtr obstacle_subscription_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_subscription_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr raw_grid_subscription_;
  rclcpp_lifecycle::LifecyclePublisher<lightnav_msgs::msg::VelocityCommand>::SharedPtr
      command_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<lightnav_msgs::msg::FollowStatus>::SharedPtr
      status_publisher_;
};

}  // namespace lightnav_controller

