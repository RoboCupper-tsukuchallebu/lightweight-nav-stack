#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "lightnav_msgs/msg/obstacle_status.hpp"
#include "lightnav_msgs/msg/velocity_command.hpp"
#include "lightnav_safety_gate/safety_gate_node.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{

using ObstacleStatus = lightnav_msgs::msg::ObstacleStatus;
using VelocityCommand = lightnav_msgs::msg::VelocityCommand;
using Twist = geometry_msgs::msg::Twist;
using std::chrono::milliseconds;

// Kept away from the default domain 42 so a running stack is never touched.
constexpr size_t kTestDomainId = 47;
constexpr double kLoopPeriodSeconds = 0.02;
const double kNaN = std::numeric_limits<double>::quiet_NaN();

ObstacleStatus healthyStatus()
{
  ObstacleStatus status;
  status.nearest_distance_m = std::numeric_limits<float>::infinity();
  return status;
}

VelocityCommand forwardCommand(double linear_mps = 0.3)
{
  VelocityCommand command;
  command.twist.linear.x = linear_mps;
  return command;
}

class SafetyGateNodeTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    rclcpp::InitOptions init_options;
    init_options.set_domain_id(kTestDomainId);
    rclcpp::init(0, nullptr, init_options);
  }
  static void TearDownTestSuite() {rclcpp::shutdown();}

  void SetUp() override
  {
    gate_ = std::make_shared<lightnav_safety_gate::SafetyGateNode>();
    peer_ = std::make_shared<rclcpp::Node>("safety_gate_test_peer");
    const auto qos = rclcpp::QoS(10).reliable().durability_volatile();
    status_publisher_ = peer_->create_publisher<ObstacleStatus>("/lightnav/obstacle_status", qos);
    command_publisher_ = peer_->create_publisher<VelocityCommand>("/lightnav/cmd_vel_raw", qos);
    twist_subscription_ = peer_->create_subscription<Twist>(
      "/cmd_vel", qos, [this](Twist::ConstSharedPtr message) {received_.push_back(*message);});
    executor_.add_node(gate_->get_node_base_interface());
    executor_.add_node(peer_);
    ASSERT_EQ(
      gate_->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  }

  void TearDown() override
  {
    gate_->shutdown();
    executor_.remove_node(peer_);
    executor_.remove_node(gate_->get_node_base_interface());
  }

  void activate()
  {
    ASSERT_EQ(gate_->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  }

  // Publishes the given inputs every loop (null = do not publish) while spinning.
  void run(
    double duration_s, const ObstacleStatus * status, const VelocityCommand * command)
  {
    const auto end = std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(duration_s));
    while (std::chrono::steady_clock::now() < end) {
      if (status != nullptr) {status_publisher_->publish(*status);}
      if (command != nullptr) {command_publisher_->publish(*command);}
      executor_.spin_some(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(kLoopPeriodSeconds)));
    }
  }

  // Waits for a condition instead of a fixed time; the limit only bounds a hang.
  template<typename Predicate>
  bool runUntil(
    Predicate is_satisfied, const ObstacleStatus * status, const VelocityCommand * command,
    double limit_s = 10.0)
  {
    const auto limit = std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(limit_s));
    while (std::chrono::steady_clock::now() < limit) {
      run(kLoopPeriodSeconds, status, command);
      if (is_satisfied()) {return true;}
    }
    return false;
  }

  bool hasNonZeroOutput() const
  {
    return !received_.empty() && received_.back().linear.x > 0.0;
  }

  // Drives the gate until it outputs a non-zero velocity (positive control).
  void reachMotion()
  {
    activate();
    const ObstacleStatus status = healthyStatus();
    const VelocityCommand command = forwardCommand();
    ASSERT_TRUE(runUntil([this] {return hasNonZeroOutput();}, &status, &command))
      << "positive control failed";
    received_.clear();
  }

  void expectLastIsZero(const char * label)
  {
    ASSERT_FALSE(received_.empty()) << label;
    EXPECT_EQ(received_.back().linear.x, 0.0) << label;
    EXPECT_EQ(received_.back().angular.z, 0.0) << label;
  }

  void expectAllZero()
  {
    ASSERT_FALSE(received_.empty());
    for (const Twist & twist : received_) {
      EXPECT_EQ(twist.linear.x, 0.0);
      EXPECT_EQ(twist.angular.z, 0.0);
    }
  }

  std::shared_ptr<lightnav_safety_gate::SafetyGateNode> gate_;
  std::shared_ptr<rclcpp::Node> peer_;
  rclcpp::Publisher<ObstacleStatus>::SharedPtr status_publisher_;
  rclcpp::Publisher<VelocityCommand>::SharedPtr command_publisher_;
  rclcpp::Subscription<Twist>::SharedPtr twist_subscription_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::vector<Twist> received_;
};

}  // namespace

TEST_F(SafetyGateNodeTest, healthyInputsProduceNonZeroOutput)
{
  reachMotion();
}

TEST_F(SafetyGateNodeTest, stopsWhenStopZoneOccupied)
{
  reachMotion();
  ObstacleStatus status = healthyStatus();
  status.is_stop_zone_occupied = true;
  const VelocityCommand command = forwardCommand();
  run(0.15, &status, &command);
  expectLastIsZero("stop zone");
}

TEST_F(SafetyGateNodeTest, stopsWhenInputIsStale)
{
  reachMotion();
  ObstacleStatus status = healthyStatus();
  status.is_input_stale = true;
  const VelocityCommand command = forwardCommand();
  run(0.15, &status, &command);
  expectLastIsZero("input stale");
}

TEST_F(SafetyGateNodeTest, stopsWhenObstacleStatusStopsArriving)
{
  reachMotion();
  const VelocityCommand command = forwardCommand();
  run(0.7, nullptr, &command);  // obstacle_status_timeout_s is 0.5
  expectLastIsZero("obstacle status timeout");
}

TEST_F(SafetyGateNodeTest, stopsWhenControllerStopsSending)
{
  reachMotion();
  const ObstacleStatus status = healthyStatus();
  run(0.5, &status, nullptr);  // command_timeout_s is 0.3
  expectLastIsZero("command timeout");
}

TEST_F(SafetyGateNodeTest, stopsOnHardStop)
{
  reachMotion();
  const ObstacleStatus status = healthyStatus();
  VelocityCommand command = forwardCommand();
  command.is_hard_stop = true;
  run(0.15, &status, &command);
  expectLastIsZero("hard stop");
}

TEST_F(SafetyGateNodeTest, nonFiniteCommandIsDroppedAndTimesOutToZero)
{
  reachMotion();
  const ObstacleStatus status = healthyStatus();
  VelocityCommand command = forwardCommand();
  command.twist.linear.x = kNaN;
  run(0.6, &status, &command);
  expectLastIsZero("non-finite");
  for (const Twist & twist : received_) {
    EXPECT_TRUE(std::isfinite(twist.linear.x));
  }
}

TEST_F(SafetyGateNodeTest, nonFiniteWithHardStopStopsImmediately)
{
  reachMotion();
  const ObstacleStatus status = healthyStatus();
  VelocityCommand command = forwardCommand();
  command.twist.linear.x = kNaN;
  command.is_hard_stop = true;
  run(0.12, &status, &command);  // well inside command_timeout_s (0.3)
  expectLastIsZero("non-finite + hard stop");
}

TEST_F(SafetyGateNodeTest, outputsZeroAfterActivateWithNoInputs)
{
  activate();
  run(0.3, nullptr, nullptr);
  expectAllZero();
}

TEST_F(SafetyGateNodeTest, outputsZeroWithObstacleStatusButNoCommand)
{
  activate();
  const ObstacleStatus status = healthyStatus();
  run(0.3, &status, nullptr);
  expectAllZero();
}

TEST_F(SafetyGateNodeTest, outputsZeroWithCommandButNoObstacleStatus)
{
  activate();
  const VelocityCommand command = forwardCommand();
  run(0.3, nullptr, &command);
  expectAllZero();
}

TEST_F(SafetyGateNodeTest, deactivateSendsOneZeroThenStaysSilent)
{
  reachMotion();
  const ObstacleStatus status = healthyStatus();
  const VelocityCommand command = forwardCommand();
  ASSERT_EQ(gate_->deactivate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  // Output published before the transition may still be in flight, so wait for the zero.
  const auto is_zero_received = [this] {
      return !received_.empty() && received_.back().linear.x == 0.0 &&
             received_.back().angular.z == 0.0;
    };
  ASSERT_TRUE(runUntil(is_zero_received, &status, &command));
  const size_t count_at_zero = received_.size();
  run(0.3, &status, &command);
  EXPECT_EQ(received_.size(), count_at_zero) << "published after the deactivate zero";
}

TEST_F(SafetyGateNodeTest, reactivationDoesNotReuseOldInputs)
{
  reachMotion();
  ASSERT_EQ(gate_->deactivate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  run(0.1, nullptr, nullptr);
  received_.clear();
  activate();
  run(0.3, nullptr, nullptr);
  ASSERT_FALSE(received_.empty());
  for (const Twist & twist : received_) {
    EXPECT_EQ(twist.linear.x, 0.0);
  }
}

TEST_F(SafetyGateNodeTest, linearAndAngularCommandsAreSaturatedAtVmaxAndWmax)
{
  activate();
  const ObstacleStatus status = healthyStatus();
  VelocityCommand command;
  command.twist.linear.x = 5.0;    // v_max 0.8
  command.twist.angular.z = -5.0;  // w_max 0.6
  const auto is_at_limits = [this] {
      return !received_.empty() && received_.back().linear.x == 0.8 &&
             received_.back().angular.z == -0.6;
    };
  ASSERT_TRUE(runUntil(is_at_limits, &status, &command));
  for (const Twist & twist : received_) {
    EXPECT_LE(twist.linear.x, 0.8);
    EXPECT_GE(twist.angular.z, -0.6);
  }
}

TEST_F(SafetyGateNodeTest, accelerationIsRateLimitedPerPublishPeriod)
{
  activate();
  const ObstacleStatus status = healthyStatus();
  VelocityCommand command;
  command.twist.linear.x = 0.6;
  command.twist.angular.z = 0.4;
  const auto is_at_target = [this] {
      return !received_.empty() && received_.back().linear.x == 0.6 &&
             received_.back().angular.z == 0.4;
    };
  ASSERT_TRUE(runUntil(is_at_target, &status, &command));
  ASSERT_GT(received_.size(), 5U) << "target reached in one step: no rate limit";
  constexpr double kPeriodSeconds = 0.05;  // publish_hz 20
  constexpr double kTolerance = 1e-9;
  double previous_linear = 0.0;
  double previous_angular = 0.0;
  for (const Twist & twist : received_) {
    EXPECT_LE(twist.linear.x - previous_linear, 0.5 * kPeriodSeconds + kTolerance);
    EXPECT_LE(twist.angular.z - previous_angular, 1.0 * kPeriodSeconds + kTolerance);
    previous_linear = twist.linear.x;
    previous_angular = twist.angular.z;
  }
}

TEST_F(SafetyGateNodeTest, decelerationOfOneAxisIsRateLimitedWhileOtherAxisKeepsMoving)
{
  reachMotion();
  const ObstacleStatus status = healthyStatus();
  const VelocityCommand forward_command = forwardCommand();
  VelocityCommand rotate_in_place;
  rotate_in_place.twist.angular.z = 0.3;  // linear target 0, but the twist is not all zero
  ASSERT_TRUE(runUntil(
      [this] {return !received_.empty() && received_.back().linear.x == 0.3;}, &status,
      &forward_command));
  received_.clear();
  ASSERT_TRUE(runUntil(
      [this] {return !received_.empty() && received_.back().linear.x == 0.0;}, &status,
      &rotate_in_place));
  ASSERT_GT(received_.size(), 2U) << "0.3 m/s dropped to 0 at once: no rate limit";
  constexpr double kPeriodSeconds = 0.05;
  double previous_linear = -1.0;
  for (const Twist & twist : received_) {
    if (previous_linear >= 0.0) {
      EXPECT_GE(twist.linear.x - previous_linear, -1.0 * kPeriodSeconds - 1e-9);
    }
    previous_linear = twist.linear.x;
  }
  EXPECT_GT(received_.back().angular.z, 0.0);
}

TEST_F(SafetyGateNodeTest, angularCommandMapsToAngularOutputWithSign)
{
  activate();
  const ObstacleStatus status = healthyStatus();
  for (const double angular_rps : {0.3, -0.3}) {
    received_.clear();
    VelocityCommand command;
    command.twist.angular.z = angular_rps;
    ASSERT_TRUE(runUntil(
        [this, angular_rps] {
          return !received_.empty() && received_.back().angular.z == angular_rps;
        },
        &status, &command));
    for (const Twist & twist : received_) {
      EXPECT_EQ(twist.linear.x, 0.0);
      EXPECT_EQ(twist.angular.x, 0.0);
      EXPECT_EQ(twist.angular.y, 0.0);
    }
  }
}
