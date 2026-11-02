#include <chrono>
#include <cmath>
#include <memory>

#include "geometry_msgs/msg/twist.hpp"
#include "lightnav_msgs/msg/obstacle_status.hpp"
#include "lightnav_msgs/msg/velocity_command.hpp"
#include "lightnav_safety_gate/safety_gate_logic.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace lightnav_safety_gate
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class SafetyGateNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  SafetyGateNode()
  : rclcpp_lifecycle::LifecycleNode("safety_gate_node")
  {
    declare_parameter("publish_hz", 20.0);
    declare_parameter("command_timeout_s", 0.3);
    declare_parameter("obstacle_status_timeout_s", 0.5);
    declare_parameter("v_max", 0.8);
    declare_parameter("w_max", 0.6);
    declare_parameter("acceleration_max_mps2", 0.5);
    declare_parameter("deceleration_max_mps2", 1.0);
    declare_parameter("angular_acceleration_max_rps2", 1.0);
    declare_parameter("angular_deceleration_max_rps2", 2.0);
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    publish_period_s_ = 1.0 / get_parameter("publish_hz").as_double();
    timeouts_.command_timeout_s = get_parameter("command_timeout_s").as_double();
    timeouts_.obstacle_status_timeout_s = get_parameter("obstacle_status_timeout_s").as_double();
    limits_.linear_max_mps = get_parameter("v_max").as_double();
    limits_.angular_max_rps = get_parameter("w_max").as_double();
    limits_.acceleration_max_mps2 = get_parameter("acceleration_max_mps2").as_double();
    limits_.deceleration_max_mps2 = get_parameter("deceleration_max_mps2").as_double();
    limits_.angular_acceleration_max_rps2 =
      get_parameter("angular_acceleration_max_rps2").as_double();
    limits_.angular_deceleration_max_rps2 =
      get_parameter("angular_deceleration_max_rps2").as_double();

    const bool are_parameters_valid =
      std::isfinite(publish_period_s_) && publish_period_s_ > 0.0 &&
      std::isfinite(timeouts_.command_timeout_s) && timeouts_.command_timeout_s > 0.0 &&
      std::isfinite(timeouts_.obstacle_status_timeout_s) &&
      timeouts_.obstacle_status_timeout_s > 0.0 && areLimitsValid(limits_);
    if (!are_parameters_valid) {
      RCLCPP_ERROR(get_logger(), "invalid parameters; configure rejected");
      return CallbackReturn::FAILURE;
    }

    const auto reliable_depth_10 = rclcpp::QoS(10).reliable().durability_volatile();
    command_publisher_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", reliable_depth_10);
    obstacle_subscription_ = create_subscription<lightnav_msgs::msg::ObstacleStatus>(
      "/lightnav/obstacle_status", reliable_depth_10,
      [this](lightnav_msgs::msg::ObstacleStatus::ConstSharedPtr message) {
        latest_obstacle_status_ = *message;
        obstacle_status_received_at_ = steady_clock_.now();
        has_obstacle_status_ = true;
      });
    command_subscription_ = create_subscription<lightnav_msgs::msg::VelocityCommand>(
      "/lightnav/cmd_vel_raw", reliable_depth_10,
      [this](lightnav_msgs::msg::VelocityCommand::ConstSharedPtr message) {
        handleVelocityCommand(*message);
      });
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override
  {
    rclcpp_lifecycle::LifecycleNode::on_activate(state);
    // Inputs received before activation are not trusted.
    has_obstacle_status_ = false;
    has_command_ = false;
    previous_output_ = PlanarVelocity{};
    is_active_ = true;
    timer_ = create_wall_timer(
      std::chrono::duration<double>(publish_period_s_), [this]() {publishOnce();});
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override
  {
    is_active_ = false;
    timer_.reset();
    publishVelocity(PlanarVelocity{});  // zero exactly once, then silence
    previous_output_ = PlanarVelocity{};
    rclcpp_lifecycle::LifecycleNode::on_deactivate(state);
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override
  {
    resetInterfaces();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override
  {
    is_active_ = false;
    timer_.reset();
    resetInterfaces();
    return CallbackReturn::SUCCESS;
  }

private:
  void resetInterfaces()
  {
    command_publisher_.reset();
    obstacle_subscription_.reset();
    command_subscription_.reset();
  }

  void handleVelocityCommand(const lightnav_msgs::msg::VelocityCommand & message)
  {
    const PlanarVelocity requested{message.twist.linear.x, message.twist.angular.z};
    if (!isFinite(requested)) {
      // Not recorded as received, so the command timeout takes the output to zero.
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000, "dropping non-finite velocity command");
      return;
    }
    latest_requested_ = requested;
    latest_is_hard_stop_ = message.is_hard_stop;
    command_received_at_ = steady_clock_.now();
    has_command_ = true;
  }

  void publishOnce()
  {
    const rclcpp::Time now = steady_clock_.now();
    EmergencyStopInputs inputs;
    inputs.is_active = is_active_;
    inputs.has_obstacle_status = has_obstacle_status_;
    inputs.obstacle_status_age_s = (now - obstacle_status_received_at_).seconds();
    inputs.is_stop_zone_occupied = latest_obstacle_status_.is_stop_zone_occupied;
    inputs.is_input_stale = latest_obstacle_status_.is_input_stale;
    inputs.has_command = has_command_;
    inputs.command_age_s = (now - command_received_at_).seconds();
    inputs.is_hard_stop = latest_is_hard_stop_;

    PlanarVelocity output;  // zero unless every stop condition is false
    if (!shouldEmergencyStop(inputs, timeouts_)) {
      output = limitRate(
        previous_output_, saturate(latest_requested_, limits_), limits_, publish_period_s_);
    }
    previous_output_ = output;
    publishVelocity(output);
  }

  void publishVelocity(const PlanarVelocity & velocity)
  {
    if (!command_publisher_) {
      return;
    }
    geometry_msgs::msg::Twist twist;
    twist.linear.x = velocity.linear_mps;
    twist.angular.z = velocity.angular_rps;
    command_publisher_->publish(twist);
  }

  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
  double publish_period_s_{0.05};
  EmergencyStopTimeouts timeouts_;
  SafetyGateLimits limits_;
  bool is_active_{false};

  lightnav_msgs::msg::ObstacleStatus latest_obstacle_status_;
  rclcpp::Time obstacle_status_received_at_{0, 0, RCL_STEADY_TIME};
  bool has_obstacle_status_{false};
  PlanarVelocity latest_requested_;
  bool latest_is_hard_stop_{false};
  rclcpp::Time command_received_at_{0, 0, RCL_STEADY_TIME};
  bool has_command_{false};
  PlanarVelocity previous_output_;

  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr command_publisher_;
  rclcpp::Subscription<lightnav_msgs::msg::ObstacleStatus>::SharedPtr obstacle_subscription_;
  rclcpp::Subscription<lightnav_msgs::msg::VelocityCommand>::SharedPtr command_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace lightnav_safety_gate

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<lightnav_safety_gate::SafetyGateNode>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
