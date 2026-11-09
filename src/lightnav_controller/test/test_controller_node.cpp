#include <gtest/gtest.h>

#include <chrono>
#include <limits>
#include <memory>
#include <optional>
#include <string>

#include "lightnav_controller/controller_node.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"


class ControllerNodeLifecycle : public ::testing::Test {
 protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void SetUp() override {
    node_ = std::make_shared<lightnav_controller::ControllerNode>(rclcpp::NodeOptions().parameter_overrides({
        rclcpp::Parameter("max_path_deviation_m", 2.5), rclcpp::Parameter("grid_frame", "base_footprint")}));
  }

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
  node_ = std::make_shared<lightnav_controller::ControllerNode>(rclcpp::NodeOptions().parameter_overrides({rclcpp::Parameter("max_path_deviation_m", 2.5), rclcpp::Parameter("v_max", -1.0)}));
  EXPECT_EQ(node_->configure().label(), "unconfigured");
}

TEST_F(ControllerNodeLifecycle, ConfigureFailsWithoutMaxPathDeviation) {
  node_ = std::make_shared<lightnav_controller::ControllerNode>();
  EXPECT_EQ(node_->configure().label(), "unconfigured");
}

TEST_F(ControllerNodeLifecycle, ConfigureSucceedsWithMaxPathDeviation) {
  EXPECT_EQ(node_->configure().label(), "inactive");
}

TEST_F(ControllerNodeLifecycle, ConfigureFailsWithInvalidMaxPathDeviation) {
  for (const double max_path_deviation_m :
       {0.0, -1.0, std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity()}) {
    SCOPED_TRACE(max_path_deviation_m);
    node_ = std::make_shared<lightnav_controller::ControllerNode>(rclcpp::NodeOptions().parameter_overrides({rclcpp::Parameter("max_path_deviation_m", max_path_deviation_m)}));
    EXPECT_EQ(node_->configure().label(), "unconfigured");
  }
}

TEST_F(ControllerNodeLifecycle, ConfigureFailsWithEmptyGridFrame) {
  node_ = std::make_shared<lightnav_controller::ControllerNode>(rclcpp::NodeOptions().parameter_overrides({rclcpp::Parameter("max_path_deviation_m", 2.5), rclcpp::Parameter("grid_frame", "")}));
  EXPECT_EQ(node_->configure().label(), "unconfigured");
}

namespace {

// Drives the real node over topics inside one process (domain taken from the env).
class ControllerNodeAvoidWiring : public ControllerNodeLifecycle {
 protected:
  void SetUp() override {
    ControllerNodeLifecycle::SetUp();
    node_->set_parameter(rclcpp::Parameter("grid_timeout_s", 0.3));
    node_ = std::make_shared<lightnav_controller::ControllerNode>(rclcpp::NodeOptions().parameter_overrides({rclcpp::Parameter("max_path_deviation_m", 2.5), rclcpp::Parameter("grid_timeout_s", 0.3), rclcpp::Parameter("grid_frame", configured_grid_frame_)}));
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
    grid.header.frame_id = frame_id_;
    grid.info.origin.orientation.w = 1.0;
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

  std::string configured_grid_frame_{"base_link"};
  std::string frame_id_{"base_link"};
  std::shared_ptr<rclcpp::Node> helper_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Publisher<lightnav_msgs::msg::PathWithId>::SharedPtr path_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
  rclcpp::Publisher<lightnav_msgs::msg::ObstacleStatus>::SharedPtr status_publisher_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_publisher_;
  rclcpp::Subscription<lightnav_msgs::msg::FollowStatus>::SharedPtr follow_subscription_;
  std::optional<lightnav_msgs::msg::FollowStatus> last_status_;
};

// The robot configuration (docker/config/robot.yaml) sets grid_frame to the frame obstacle_monitor stamps.
class ControllerNodeAvoidWiringInFootprintFrame : public ControllerNodeAvoidWiring {
 protected:
  void SetUp() override {
    configured_grid_frame_ = "base_footprint";
    frame_id_ = "base_footprint";
    ControllerNodeAvoidWiring::SetUp();
  }
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

TEST_F(ControllerNodeAvoidWiring, GridWithWrongFrameIsTreatedAsMissing) {
  frame_id_ = "map";
  spinFor(std::chrono::milliseconds(500), true);
  ASSERT_TRUE(last_status_.has_value());
  EXPECT_NE(last_status_->active_layer, lightnav_msgs::msg::FollowStatus::LAYER_AVOID);
}

TEST_F(ControllerNodeAvoidWiringInFootprintFrame, GridInConfiguredFrameSwitchesToAvoid) {
  spinFor(std::chrono::milliseconds(500), true);
  ASSERT_TRUE(last_status_.has_value());
  EXPECT_EQ(last_status_->active_layer, lightnav_msgs::msg::FollowStatus::LAYER_AVOID);
}

TEST_F(ControllerNodeAvoidWiringInFootprintFrame, GridInDefaultFrameIsDroppedWhenAnotherFrameIsConfigured) {
  frame_id_ = "base_link";
  spinFor(std::chrono::milliseconds(500), true);
  ASSERT_TRUE(last_status_.has_value());
  EXPECT_NE(last_status_->active_layer, lightnav_msgs::msg::FollowStatus::LAYER_AVOID);
}

TEST_F(ControllerNodeLifecycle, RuntimeClassesAndStartupBaseline) {
  configureAndActivate();
  for (double value : {0.8, 0.4}) EXPECT_TRUE(node_->set_parameter(rclcpp::Parameter("v_max", value)).successful);
  for (double value : {0.05, 0.1}) EXPECT_TRUE(node_->set_parameter(rclcpp::Parameter("arc_step_m", value)).successful);
  for (double value : {0.2, std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN()}) {
    const auto result = node_->set_parameter(rclcpp::Parameter("arc_step_m", value));
    EXPECT_FALSE(result.successful); EXPECT_FALSE(result.reason.empty());
  }
  EXPECT_FALSE(node_->set_parameter(rclcpp::Parameter("use_sim_time", true)).successful);
  { const auto result = node_->set_parameter(rclcpp::Parameter("grid_frame", "map")); EXPECT_FALSE(result.successful); EXPECT_FALSE(result.reason.empty()); }
  node_->deactivate(); node_->cleanup();
  EXPECT_FALSE(node_->set_parameter(rclcpp::Parameter("arc_step_m", 0.2)).successful);
  EXPECT_EQ(node_->configure().label(), "inactive");
}

TEST_F(ControllerNodeLifecycle, RejectsInvalidCombinedConstraints) {
  EXPECT_FALSE(node_->set_parameter(rclcpp::Parameter("v_slow", 0.9)).successful);
  EXPECT_FALSE(node_->set_parameter(rclcpp::Parameter("avoid_trigger_m", 3.0)).successful);
  auto result = node_->set_parameters_atomically({rclcpp::Parameter("v_max", 0.9), rclcpp::Parameter("v_slow", 0.8)});
  EXPECT_TRUE(result.successful);
}

TEST_F(ControllerNodeLifecycle, NonfiniteStartupBaselineCannotBeChanged) {
  node_ = std::make_shared<lightnav_controller::ControllerNode>();
  const auto result = node_->set_parameter(rclcpp::Parameter("max_path_deviation_m", 2.5));
  EXPECT_FALSE(result.successful); EXPECT_FALSE(result.reason.empty());
}

TEST_F(ControllerNodeLifecycle, TuningParametersAcceptBothDirections) {
  configureAndActivate();
  for (const auto& name : {"lookahead_m", "search_window_m", "v_max", "v_slow", "w_max",
                           "w_rotate", "v_avoid", "k_end", "goal_tolerance_m",
                           "rotate_in_place_heading_rad", "w_path", "w_heading", "w_smooth",
                           "stuck_timeout_s"}) {
    SCOPED_TRACE(name);
    const double initial = node_->get_parameter(name).as_double();
    EXPECT_TRUE(node_->set_parameter(rclcpp::Parameter(name, initial * 1.1)).successful);
    EXPECT_TRUE(node_->set_parameter(rclcpp::Parameter(name, initial * 0.9)).successful);
    EXPECT_TRUE(node_->set_parameter(rclcpp::Parameter(name, initial)).successful);
  }
  const auto count = node_->get_parameter("arc_candidate_count").as_int();
  EXPECT_TRUE(node_->set_parameter(rclcpp::Parameter("arc_candidate_count", count + 1)).successful);
  EXPECT_TRUE(node_->set_parameter(rclcpp::Parameter("arc_candidate_count", count - 1)).successful);
  EXPECT_FALSE(node_->set_parameter(rclcpp::Parameter("arc_candidate_count", 0)).successful);
}

TEST_F(ControllerNodeLifecycle, EverySafetyParameterEnforcesStartupBaseline) {
  configureAndActivate();
  for (const auto& entry : std::vector<std::pair<std::string, bool>>{
           {"v_min", false}, {"max_path_deviation_m", false}, {"pose_timeout_s", false},
           {"obstacle_status_timeout_s", false}, {"grid_timeout_s", false},
           {"arc_step_m", false}, {"rejoin_distance_m", false}, {"avoid_trigger_m", true},
           {"avoid_release_m", true}, {"arc_length_m", true}, {"avoid_release_hold_s", true}}) {
    SCOPED_TRACE(entry.first);
    const double initial = node_->get_parameter(entry.first).as_double();
    const double conservative = initial * (entry.second ? 1.01 : 0.99);
    const double unsafe = initial * (entry.second ? 0.99 : 1.01);
    ASSERT_TRUE(node_->set_parameter(rclcpp::Parameter(entry.first, conservative)).successful);
    for (double invalid : {unsafe, std::numeric_limits<double>::quiet_NaN(),
                           std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity()}) {
      const auto result = node_->set_parameter(rclcpp::Parameter(entry.first, invalid));
      EXPECT_FALSE(result.successful);
      EXPECT_FALSE(result.reason.empty());
      EXPECT_DOUBLE_EQ(node_->get_parameter(entry.first).as_double(), conservative);
    }
    EXPECT_TRUE(node_->set_parameter(rclcpp::Parameter(entry.first, initial)).successful);
  }
}

TEST_F(ControllerNodeLifecycle, ReadOnlyAndInvalidValuesAreRejected) {
  for (int state = 0; state < 3; ++state) {
    if (state == 1) ASSERT_EQ(node_->configure().label(), "inactive");
    if (state == 2) ASSERT_EQ(node_->activate().label(), "active");
    const auto result = node_->set_parameter(rclcpp::Parameter("use_sim_time", true));
    EXPECT_FALSE(result.successful);
    EXPECT_FALSE(result.reason.empty());
  }
  { const auto result = node_->set_parameter(rclcpp::Parameter("control_hz", 30.0)); EXPECT_FALSE(result.successful); EXPECT_FALSE(result.reason.empty()); }
  { const auto result = node_->set_parameter(rclcpp::Parameter("grid_frame", "map")); EXPECT_FALSE(result.successful); EXPECT_FALSE(result.reason.empty()); }
  for (double invalid : {0.0, -1.0, std::numeric_limits<double>::quiet_NaN(),
                         std::numeric_limits<double>::infinity()}) {
    const auto old = node_->get_parameter("lookahead_m").as_double();
    const auto result = node_->set_parameter(rclcpp::Parameter("lookahead_m", invalid));
    EXPECT_FALSE(result.successful);
    EXPECT_FALSE(result.reason.empty());
    EXPECT_DOUBLE_EQ(node_->get_parameter("lookahead_m").as_double(), old);
  }
}

TEST_F(ControllerNodeLifecycle, ConfigureRejectsInconsistentOverrides) {
  for (const auto& invalid : {rclcpp::Parameter("v_slow", 1.0),
                              rclcpp::Parameter("avoid_trigger_m", 10.0)}) {
    node_ = std::make_shared<lightnav_controller::ControllerNode>(rclcpp::NodeOptions().parameter_overrides(
        {rclcpp::Parameter("max_path_deviation_m", 2.5), invalid}));
    EXPECT_EQ(node_->configure().label(), "unconfigured");
  }
}

TEST_F(ControllerNodeAvoidWiring, RuntimeChangesPreserveAvoidanceAndPath) {
  spinFor(std::chrono::milliseconds(500), true);
  ASSERT_TRUE(last_status_.has_value());
  ASSERT_EQ(last_status_->active_layer, lightnav_msgs::msg::FollowStatus::LAYER_AVOID);
  ASSERT_TRUE(node_->set_parameter(rclcpp::Parameter("w_path", 2.0)).successful);
  ASSERT_TRUE(node_->set_parameter(rclcpp::Parameter("arc_step_m", 0.05)).successful);
  spinFor(std::chrono::milliseconds(200), true);
  EXPECT_EQ(last_status_->active_layer, lightnav_msgs::msg::FollowStatus::LAYER_AVOID);
}

TEST_F(ControllerNodeAvoidWiring, ParameterServiceValidatesEachValueInOrder) {
  auto client = std::make_shared<rclcpp::AsyncParametersClient>(helper_, "/controller_node");
  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(5)));
  auto future = client->set_parameters({rclcpp::Parameter("v_max", 0.9),
                                        rclcpp::Parameter("v_slow", 0.8),
                                        rclcpp::Parameter("arc_step_m", 1.0)});
  ASSERT_EQ(executor_.spin_until_future_complete(future, std::chrono::seconds(5)),
            rclcpp::FutureReturnCode::SUCCESS);
  const auto results = future.get();
  ASSERT_EQ(results.size(), 3u);
  EXPECT_TRUE(results[0].successful);
  EXPECT_TRUE(results[1].successful);
  EXPECT_FALSE(results[2].successful);
  EXPECT_FALSE(results[2].reason.empty());
  EXPECT_DOUBLE_EQ(node_->get_parameter("v_max").as_double(), 0.9);
  EXPECT_DOUBLE_EQ(node_->get_parameter("v_slow").as_double(), 0.8);
}
