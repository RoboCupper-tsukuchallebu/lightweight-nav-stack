#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <optional>

#include "lightnav_controller/controller_node.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"


class ControllerNodeLifecycle : public ::testing::Test {
 protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void SetUp() override { node_ = std::make_shared<lightnav_controller::ControllerNode>(); }

  void configureAndActivate() {
    ASSERT_EQ(node_->configure().label(), "inactive");
    ASSERT_EQ(node_->activate().label(), "active");
  }

  std::shared_ptr<lightnav_controller::ControllerNode> node_;
};

TEST_F(ControllerNodeLifecycle, ShutdownFromActiveStopsTimerAndOutputs) {
  configureAndActivate();
  ASSERT_FALSE(node_->isReleased());
  EXPECT_EQ(node_->shutdown().label(), "finalized");
  EXPECT_TRUE(node_->isReleased());
}

TEST_F(ControllerNodeLifecycle, ShutdownFromInactiveReleasesOutputs) {
  ASSERT_EQ(node_->configure().label(), "inactive");
  ASSERT_FALSE(node_->isReleased());
  EXPECT_EQ(node_->shutdown().label(), "finalized");
  EXPECT_TRUE(node_->isReleased());
}

TEST_F(ControllerNodeLifecycle, ShutdownFromUnconfiguredFinalizes) {
  EXPECT_EQ(node_->shutdown().label(), "finalized");
  EXPECT_TRUE(node_->isReleased());
}

TEST_F(ControllerNodeLifecycle, DeactivateStopsTimer) {
  configureAndActivate();
  ASSERT_EQ(node_->deactivate().label(), "inactive");
  EXPECT_FALSE(node_->isTimerRunning());
}

TEST_F(ControllerNodeLifecycle, CleanupReleasesOutputs) {
  configureAndActivate();
  node_->deactivate();
  ASSERT_EQ(node_->cleanup().label(), "unconfigured");
  EXPECT_TRUE(node_->isReleased());
}

TEST_F(ControllerNodeLifecycle, ConfigureFailsOnNonPositiveParameter) {
  node_->set_parameter(rclcpp::Parameter("v_max", -1.0));
  EXPECT_EQ(node_->configure().label(), "unconfigured");
}

namespace {

// Drives the real node over topics inside one process (domain taken from the env).
class ControllerNodeAvoidWiring : public ControllerNodeLifecycle {
 protected:
  void SetUp() override {
    ControllerNodeLifecycle::SetUp();
    node_->set_parameter(rclcpp::Parameter("grid_timeout_s", 0.3));
    configureAndActivate();
    helper_ = std::make_shared<rclcpp::Node>("controller_node_test_helper");
    path_publisher_ = helper_->create_publisher<lightnav_msgs::msg::PathWithId>(
        "/lightnav/path", rclcpp::QoS(1).reliable().transient_local());
    pose_publisher_ = helper_->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/lightnav/pose", rclcpp::QoS(10).reliable());
    status_publisher_ = helper_->create_publisher<lightnav_msgs::msg::ObstacleStatus>(
        "/lightnav/obstacle_status", rclcpp::QoS(10).reliable());
    grid_publisher_ = helper_->create_publisher<nav_msgs::msg::OccupancyGrid>(
        "/lightnav/local_grid", rclcpp::QoS(1).reliable());
    follow_subscription_ = helper_->create_subscription<lightnav_msgs::msg::FollowStatus>(
        "/lightnav/follow_status", rclcpp::QoS(10).reliable(),
        [this](lightnav_msgs::msg::FollowStatus::ConstSharedPtr message) { last_status_ = *message; });
    executor_.add_node(node_->get_node_base_interface());
    executor_.add_node(helper_);
    publishPath();
  }

  void publishPath() {
    lightnav_msgs::msg::PathWithId path;
    path.session_id = 1;
    path.path_id = 1;
    for (int index = 0; index <= 100; ++index) {
      geometry_msgs::msg::PoseStamped pose;
      pose.pose.position.x = 0.1 * index;
      pose.pose.orientation.w = 1.0;
      path.path.poses.push_back(pose);
    }
    path_publisher_->publish(path);
  }

  nav_msgs::msg::OccupancyGrid makeGrid(bool is_blocked) {
    nav_msgs::msg::OccupancyGrid grid;
    grid.info.width = 80;
    grid.info.height = 80;
    grid.info.resolution = 0.1F;
    grid.info.origin.position.x = -2.0;
    grid.info.origin.position.y = -4.0;
    grid.data.assign(80 * 80, 0);
    if (is_blocked) {
      for (int row = 38; row < 42; ++row) {
        for (int column = 30; column < 34; ++column) {  // x 1.0..1.4, y -0.2..0.2
          grid.data[row * 80 + column] = 100;
        }
      }
    }
    return grid;
  }

  // Publishes pose, clear obstacle status, and optionally a grid every 20 ms.
  void spinFor(std::chrono::milliseconds duration, std::optional<bool> grid_blocked) {
    const auto end = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < end) {
      geometry_msgs::msg::PoseStamped pose;
      pose.pose.orientation.w = 1.0;
      pose_publisher_->publish(pose);
      lightnav_msgs::msg::ObstacleStatus status;
      status_publisher_->publish(status);
      if (grid_blocked.has_value()) {
        grid_publisher_->publish(makeGrid(*grid_blocked));
      }
      executor_.spin_some(std::chrono::milliseconds(20));
    }
  }

  std::shared_ptr<rclcpp::Node> helper_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Publisher<lightnav_msgs::msg::PathWithId>::SharedPtr path_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
  rclcpp::Publisher<lightnav_msgs::msg::ObstacleStatus>::SharedPtr status_publisher_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_publisher_;
  rclcpp::Subscription<lightnav_msgs::msg::FollowStatus>::SharedPtr follow_subscription_;
  std::optional<lightnav_msgs::msg::FollowStatus> last_status_;
};

}  // namespace

TEST_F(ControllerNodeAvoidWiring, BlockedGridSwitchesToAvoidAndGridLossFallsBackToFollow) {
  spinFor(std::chrono::milliseconds(400), false);
  ASSERT_TRUE(last_status_.has_value());
  EXPECT_EQ(last_status_->active_layer, lightnav_msgs::msg::FollowStatus::LAYER_FOLLOW);

  spinFor(std::chrono::milliseconds(400), true);
  EXPECT_EQ(last_status_->active_layer, lightnav_msgs::msg::FollowStatus::LAYER_AVOID);

  spinFor(std::chrono::milliseconds(600), std::nullopt);  // grid stops: > grid_timeout_s
  EXPECT_EQ(last_status_->active_layer, lightnav_msgs::msg::FollowStatus::LAYER_FOLLOW);
}
