#ifndef LIGHTNAV_SAFETY_GATE__SAFETY_GATE_NODE_HPP_
#define LIGHTNAV_SAFETY_GATE__SAFETY_GATE_NODE_HPP_

#include <chrono>
#include <cmath>
#include <memory>
#include <string>

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
  explicit SafetyGateNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp_lifecycle::LifecycleNode("safety_gate_node", options)
  {
    declareNumericParameter("publish_hz", 20.0);
    declareNumericParameter("command_timeout_s", 0.3);
    declareNumericParameter("obstacle_status_timeout_s", 0.5);
    declareNumericParameter("v_max", 0.8);
    declareNumericParameter("w_max", 0.6);
    declareNumericParameter("acceleration_max_mps2", 0.5);
    declareNumericParameter("deceleration_max_mps2", 1.0);
    declareNumericParameter("angular_acceleration_max_rps2", 1.0);
    declareNumericParameter("angular_deceleration_max_rps2", 2.0);
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    double publish_hz = 0.0;
    if (!readNumericParameter("publish_hz", publish_hz) ||
      !readNumericParameter("command_timeout_s", timeouts_.command_timeout_s) ||
      !readNumericParameter("obstacle_status_timeout_s", timeouts_.obstacle_status_timeout_s) ||
      !readNumericParameter("v_max", limits_.linear_max_mps) ||
      !readNumericParameter("w_max", limits_.angular_max_rps) ||
      !readNumericParameter("acceleration_max_mps2", limits_.acceleration_max_mps2) ||
      !readNumericParameter("deceleration_max_mps2", limits_.deceleration_max_mps2) ||
      !readNumericParameter(
        "angular_acceleration_max_rps2", limits_.angular_acceleration_max_rps2) ||
      !readNumericParameter(
        "angular_deceleration_max_rps2", limits_.angular_deceleration_max_rps2))
    {
      RCLCPP_ERROR(get_logger(), "parameter types must be integer or double; configure rejected");
      return CallbackReturn::FAILURE;
    }
    publish_period_s_ = 1.0 / publish_hz;

    const bool are_parameters_valid =
      std::isfinite(publish_hz) && publish_hz > 0.0 &&
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
  void declareNumericParameter(const std::string & name, double default_value)
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.dynamic_typing = true;
    descriptor.description = "A number; an integer is accepted as well as a double.";
    declare_parameter(name, rclcpp::ParameterValue(default_value), descriptor);
  }

  bool readNumericParameter(const std::string & name, double & value)
  {
    const rclcpp::Parameter parameter = get_parameter(name);
    if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
      value = static_cast<double>(parameter.as_int());
      return true;
    }
    if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      value = parameter.as_double();
      return true;
    }
    RCLCPP_ERROR(get_logger(), "parameter '%s' must be an integer or a double", name.c_str());
    return false;
  }

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
      RCLCPP_WARN_THROTTLE(
        get_logger(), steady_clock_, 1000, "dropping non-finite velocity command");
      if (message.is_hard_stop) {
        // A hard stop must not be lost just because the twist is garbage.
        latest_is_hard_stop_ = true;
        command_received_at_ = steady_clock_.now();
        has_command_ = true;
      }
      // Otherwise the command is not recorded as received, so it times out to zero.
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

#endif  // LIGHTNAV_SAFETY_GATE__SAFETY_GATE_NODE_HPP_
