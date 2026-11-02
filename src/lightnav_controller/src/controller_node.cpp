#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "lightnav_controller/arbitrate.hpp"
#include "lightnav_controller/avoid_layer.hpp"
#include "lightnav_controller/follow_layer.hpp"
#include "lightnav_msgs/msg/follow_status.hpp"
#include "lightnav_msgs/msg/obstacle_status.hpp"
#include "lightnav_msgs/msg/path_with_id.hpp"
#include "lightnav_msgs/msg/velocity_command.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace lightnav_controller {

namespace {

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

double yawFromQuaternion(const geometry_msgs::msg::Quaternion& quaternion) {
  return std::atan2(2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
                    1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z));
}

}  // namespace

class ControllerNode : public rclcpp_lifecycle::LifecycleNode {
 public:
  ControllerNode() : rclcpp_lifecycle::LifecycleNode("controller_node") {
    declare_parameter("control_hz", 20.0);
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
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State&) override {
    const std::vector<std::string> positive_names = {
        "control_hz", "lookahead_m", "search_window_m", "v_max", "v_min", "v_slow",
        "w_max", "w_rotate", "k_end", "goal_tolerance_m", "rotate_in_place_heading_rad",
        "pose_timeout_s", "obstacle_status_timeout_s"};
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
    pose_timeout_ = rclcpp::Duration::from_seconds(get_parameter("pose_timeout_s").as_double());
    obstacle_status_timeout_ =
        rclcpp::Duration::from_seconds(get_parameter("obstacle_status_timeout_s").as_double());

    follow_layer_ = std::make_unique<FollowLayer>(follow_parameters);
    avoid_layer_ = std::make_unique<NullAvoidLayer>();
    last_pose_.reset();
    last_obstacle_status_.reset();

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
    timer_.reset();
    path_subscription_.reset();
    pose_subscription_.reset();
    obstacle_subscription_.reset();
    command_publisher_.reset();
    status_publisher_.reset();
    follow_layer_.reset();
    avoid_layer_.reset();
    return CallbackReturn::SUCCESS;
  }

 private:
  struct ReceivedPose {
    geometry_msgs::msg::PoseStamped message;
    rclcpp::Time received_at;
  };
  struct ReceivedObstacleStatus {
    lightnav_msgs::msg::ObstacleStatus message;
    rclcpp::Time received_at;
  };

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
    const rclcpp::Time current_time = now();
    const bool is_pose_fresh =
        last_pose_.has_value() && (current_time - last_pose_->received_at) <= pose_timeout_;
    const bool is_obstacle_status_fresh =
        last_obstacle_status_.has_value() &&
        (current_time - last_obstacle_status_->received_at) <= obstacle_status_timeout_ &&
        !last_obstacle_status_->message.is_input_stale;

    ArbitrationInput arbitration_input;
    arbitration_input.is_pose_fresh = is_pose_fresh;
    // Grid subscription arrives with the avoid layer (PR4), so it is never fresh here.
    arbitration_input.is_grid_fresh = false;
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
      arbitration_input.avoid = avoid_layer_->compute(AvoidContext{
          pose, follow_layer_->pathPoints(), follow_result.closest_index,
          arbitration_input.is_grid_fresh});
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
    status.is_stuck = false;  // stuck detection arrives with the avoid layer (PR4)
    status_publisher_->publish(status);
  }

  std::unique_ptr<FollowLayer> follow_layer_;
  std::unique_ptr<AvoidLayer> avoid_layer_;
  rclcpp::Duration pose_timeout_{0, 0};
  rclcpp::Duration obstacle_status_timeout_{0, 0};
  std::optional<ReceivedPose> last_pose_;
  std::optional<ReceivedObstacleStatus> last_obstacle_status_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<lightnav_msgs::msg::PathWithId>::SharedPtr path_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscription_;
  rclcpp::Subscription<lightnav_msgs::msg::ObstacleStatus>::SharedPtr obstacle_subscription_;
  rclcpp_lifecycle::LifecyclePublisher<lightnav_msgs::msg::VelocityCommand>::SharedPtr
      command_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<lightnav_msgs::msg::FollowStatus>::SharedPtr
      status_publisher_;
};

}  // namespace lightnav_controller

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<lightnav_controller::ControllerNode>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
