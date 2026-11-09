#include <gtest/gtest.h>

#include <chrono>
#include <functional>
#include <limits>
#include <memory>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <lightnav_msgs/msg/obstacle_status.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

#include "lightnav_obstacle_monitor/obstacle_monitor_node.hpp"

namespace lightnav
{
namespace
{
using std::chrono::milliseconds;
using ObstacleStatus = lightnav_msgs::msg::ObstacleStatus;
constexpr auto kWaitLimit = std::chrono::seconds(10);
const char kSensorFrame[] = "test_sensor";

// Runs the node under test and a helper node in one executor.
class ObstacleMonitorNodeTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  void SetUp() override
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides(
      {{"points_timeout_s", 0.3}, {"clear_hold_s", 0.2}, {"watchdog_hz", 20.0}});
    monitor_ = std::make_shared<ObstacleMonitorNode>(options);
    helper_ = std::make_shared<rclcpp::Node>("obstacle_monitor_test_helper");
    pointsPublisher_ = helper_->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/velodyne_points", rclcpp::QoS(5).reliable().durability_volatile());
    statusSubscription_ = helper_->create_subscription<ObstacleStatus>(
      "/lightnav/obstacle_status", rclcpp::QoS(10).reliable(),
      [this](ObstacleStatus::SharedPtr status) {
        lastStatus_ = *status;
        ++statusCount_;
      });
    gridSubscription_ = helper_->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/lightnav/local_grid", rclcpp::QoS(1).reliable(),
      [this](nav_msgs::msg::OccupancyGrid::SharedPtr) {++gridCount_;});
    executor_.add_node(monitor_->get_node_base_interface());
    executor_.add_node(helper_);
  }

  void TearDown() override
  {
    executor_.remove_node(helper_);
    executor_.remove_node(monitor_->get_node_base_interface());
  }

  void spinFor(milliseconds duration)
  {
    const auto end = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < end) {
      executor_.spin_some(milliseconds(5));
    }
  }

  // Keeps publishing clouds (when requested) until the condition holds.
  bool waitUntil(const std::function<bool()> & condition, bool shouldPublishCloud)
  {
    const auto end = std::chrono::steady_clock::now() + kWaitLimit;
    while (std::chrono::steady_clock::now() < end) {
      if (shouldPublishCloud) {
        pointsPublisher_->publish(makeObstacleCloud());
      }
      spinFor(milliseconds(50));
      if (condition()) {
        return true;
      }
    }
    return false;
  }

  void bringUp()
  {
    ASSERT_EQ(
      monitor_->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    ASSERT_EQ(monitor_->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  }

  void publishIdentityTransform()
  {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = helper_->now();
    transform.header.frame_id = "base_link";
    transform.child_frame_id = kSensorFrame;
    transform.transform.rotation.w = 1.0;
    broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(helper_);
    broadcaster_->sendTransform(transform);
  }

  // Three vertical columns 0.5 m ahead of the robot: inside the stop zone.
  sensor_msgs::msg::PointCloud2 makeObstacleCloud()
  {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.frame_id = kSensorFrame;
    cloud.header.stamp = helper_->now();
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(6);
    sensor_msgs::PointCloud2Iterator<float> iteratorX(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iteratorY(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iteratorZ(cloud, "z");
    for (const float columnY : {-0.2F, 0.0F, 0.2F}) {
      for (const float height : {0.0F, 0.5F}) {
        *iteratorX = 0.55F;
        *iteratorY = columnY;
        *iteratorZ = height;
        ++iteratorX;
        ++iteratorY;
        ++iteratorZ;
      }
    }
    return cloud;
  }

  bool isStale(const std::string & sourceName) const
  {
    for (const auto & name : lastStatus_.stale_sources) {
      if (name == sourceName) {
        return true;
      }
    }
    return false;
  }

  std::shared_ptr<ObstacleMonitorNode> monitor_;
  std::shared_ptr<rclcpp::Node> helper_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointsPublisher_;
  rclcpp::Subscription<ObstacleStatus>::SharedPtr statusSubscription_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr gridSubscription_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_;
  ObstacleStatus lastStatus_;
  int statusCount_ = 0;
  int gridCount_ = 0;
};
}  // namespace

TEST_F(ObstacleMonitorNodeTest, StructuralParametersAreReadOnly)
{
  const auto result = monitor_->set_parameter(rclcpp::Parameter("step_neighbor_radius_m", 0.1));
  EXPECT_FALSE(result.successful);
  EXPECT_FALSE(result.reason.empty());
  EXPECT_FALSE(monitor_->set_parameter(rclcpp::Parameter("use_sim_time", true)).successful);
}

TEST_F(ObstacleMonitorNodeTest, SafetyValuesRespectStartupAndConstraints)
{
  bringUp();
  EXPECT_TRUE(monitor_->set_parameter(rclcpp::Parameter("stop_zone_x_m", 1.2)).successful);
  EXPECT_TRUE(monitor_->set_parameter(rclcpp::Parameter("stop_zone_x_m", 1.0)).successful);
  EXPECT_FALSE(monitor_->set_parameter(rclcpp::Parameter("stop_zone_x_m", 0.9)).successful);
  EXPECT_FALSE(monitor_->set_parameter(rclcpp::Parameter("stop_zone_x_m", 3.0)).successful);
  EXPECT_TRUE(monitor_->set_parameter(rclcpp::Parameter("min_cell_count", 1)).successful);
  EXPECT_FALSE(monitor_->set_parameter(rclcpp::Parameter("min_cell_count", 3)).successful);
  for (double value : {std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity()}) {
    const auto result = monitor_->set_parameter(rclcpp::Parameter("clear_hold_s", value));
    EXPECT_FALSE(result.successful);
    EXPECT_FALSE(result.reason.empty());
  }
  EXPECT_EQ(monitor_->get_parameter("stop_zone_x_m").as_double(), 1.0);
  monitor_->deactivate();
  monitor_->cleanup();
  EXPECT_FALSE(monitor_->set_parameter(rclcpp::Parameter("stop_zone_x_m", 0.9)).successful);
  EXPECT_EQ(monitor_->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
}

TEST_F(ObstacleMonitorNodeTest, NonFiniteStartupAndWrongTypesAreRejected)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({{"clear_hold_s", std::numeric_limits<double>::quiet_NaN()}});
  auto node = std::make_shared<ObstacleMonitorNode>(options);
  const auto invalidBaseline = node->set_parameter(rclcpp::Parameter("clear_hold_s", 1.0));
  EXPECT_FALSE(invalidBaseline.successful);
  EXPECT_FALSE(invalidBaseline.reason.empty());
  EXPECT_EQ(node->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);

  const auto wrongType = monitor_->set_parameter(rclcpp::Parameter("min_cell_count", "invalid"));
  EXPECT_FALSE(wrongType.successful);
  EXPECT_FALSE(wrongType.reason.empty());
}

TEST_F(ObstacleMonitorNodeTest, AtomicChangesApplyTogetherAndRejectTogether)
{
  bringUp();
  const auto accepted = monitor_->set_parameters_atomically({
    rclcpp::Parameter("stop_zone_x_m", 1.2), rclcpp::Parameter("slow_zone_x_m", 2.6)});
  EXPECT_TRUE(accepted.successful);
  EXPECT_DOUBLE_EQ(monitor_->get_parameter("stop_zone_x_m").as_double(), 1.2);
  EXPECT_DOUBLE_EQ(monitor_->get_parameter("slow_zone_x_m").as_double(), 2.6);

  const auto rejected = monitor_->set_parameters_atomically({
    rclcpp::Parameter("stop_zone_x_m", 1.3), rclcpp::Parameter("slow_zone_x_m", 2.0)});
  EXPECT_FALSE(rejected.successful);
  EXPECT_FALSE(rejected.reason.empty());
  EXPECT_DOUBLE_EQ(monitor_->get_parameter("stop_zone_x_m").as_double(), 1.2);
  EXPECT_DOUBLE_EQ(monitor_->get_parameter("slow_zone_x_m").as_double(), 2.6);
}

TEST_F(ObstacleMonitorNodeTest, ParameterServiceValidatesEachValueSeparately)
{
  bringUp();
  auto client = std::make_shared<rclcpp::AsyncParametersClient>(helper_, "obstacle_monitor");
  ASSERT_TRUE(waitUntil([&client] {return client->service_is_ready();}, false));
  auto future = client->set_parameters({
    rclcpp::Parameter("stop_zone_x_m", 1.2), rclcpp::Parameter("slow_zone_x_m", 2.0)});
  ASSERT_TRUE(waitUntil([&future] {
    return future.wait_for(milliseconds(0)) == std::future_status::ready;
  }, false));
  const auto results = future.get();
  ASSERT_EQ(results.size(), 2u);
  EXPECT_TRUE(results[0].successful);
  EXPECT_FALSE(results[1].successful);
  EXPECT_FALSE(results[1].reason.empty());
  EXPECT_DOUBLE_EQ(monitor_->get_parameter("stop_zone_x_m").as_double(), 1.2);
  EXPECT_DOUBLE_EQ(monitor_->get_parameter("slow_zone_x_m").as_double(), 2.5);
}

TEST_F(ObstacleMonitorNodeTest, InvalidStartupZoneFailsConfigure)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({{"stop_zone_x_m", 4.0}});
  auto node = std::make_shared<ObstacleMonitorNode>(options);
  EXPECT_EQ(node->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
}

TEST_F(ObstacleMonitorNodeTest, UpdatingThresholdPreservesOccupiedState)
{
  bringUp();
  publishIdentityTransform();
  ASSERT_TRUE(waitUntil([this] {return lastStatus_.is_stop_zone_occupied;}, true));
  ASSERT_TRUE(monitor_->set_parameter(rclcpp::Parameter("clear_hold_s", 1.0)).successful);
  ASSERT_TRUE(waitUntil([this] {return !lastStatus_.is_input_stale;}, true));
  EXPECT_TRUE(lastStatus_.is_stop_zone_occupied);
}

TEST_F(ObstacleMonitorNodeTest, DefaultPointsQoSReliabilityAcceptsReliablePublisher)
{
  EXPECT_EQ(monitor_->get_parameter("points_qos_reliability").as_string(), "reliable");
  bringUp();
  publishIdentityTransform();
  ASSERT_TRUE(waitUntil([this] {return lastStatus_.is_stop_zone_occupied;}, true));
  EXPECT_FALSE(lastStatus_.is_input_stale);
}

TEST_F(ObstacleMonitorNodeTest, InvalidPointsQoSReliabilityIsRejected)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({{"points_qos_reliability", "unreliable"}});
  auto node = std::make_shared<ObstacleMonitorNode>(options);
  EXPECT_EQ(node->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
}

TEST_F(ObstacleMonitorNodeTest, WatchdogPublishesStaleStatusWithoutGridWhenNoInput)
{
  bringUp();
  ASSERT_TRUE(waitUntil([this] {return statusCount_ >= 3;}, false));
  EXPECT_TRUE(lastStatus_.is_input_stale);
  EXPECT_TRUE(isStale("points"));
  EXPECT_EQ(gridCount_, 0);
}

TEST_F(ObstacleMonitorNodeTest, MissingTransformIsReportedAsTfStale)
{
  bringUp();
  ASSERT_TRUE(waitUntil([this] {return isStale(std::string("tf:") + kSensorFrame);}, true));
  EXPECT_TRUE(lastStatus_.is_input_stale);
  EXPECT_EQ(gridCount_, 0);
}

TEST_F(ObstacleMonitorNodeTest, ObstacleInStopZoneIsReportedThenInputLossBecomesStale)
{
  bringUp();
  publishIdentityTransform();
  ASSERT_TRUE(waitUntil([this] {return lastStatus_.is_stop_zone_occupied;}, true));
  EXPECT_FALSE(lastStatus_.is_input_stale);
  EXPECT_GE(lastStatus_.stop_zone_cell_count, 2U);
  EXPECT_GT(gridCount_, 0);

  // Stop publishing: status turns stale and the grid stops, so a grid timeout can fire.
  ASSERT_TRUE(waitUntil([this] {return lastStatus_.is_input_stale;}, false));
  EXPECT_TRUE(isStale("points"));
  spinFor(milliseconds(100));
  const int gridCountWhenStale = gridCount_;
  spinFor(milliseconds(500));
  EXPECT_EQ(gridCount_, gridCountWhenStale);
}

TEST_F(ObstacleMonitorNodeTest, InvalidCloudIsDroppedAndBecomesStale)
{
  bringUp();
  publishIdentityTransform();
  ASSERT_TRUE(waitUntil([this] {return lastStatus_.is_stop_zone_occupied;}, true));
  const auto end = std::chrono::steady_clock::now() + kWaitLimit;
  bool isStaleNow = false;
  while (!isStaleNow && std::chrono::steady_clock::now() < end) {
    auto brokenCloud = makeObstacleCloud();
    brokenCloud.point_step = 0;
    pointsPublisher_->publish(brokenCloud);
    spinFor(milliseconds(50));
    isStaleNow = lastStatus_.is_input_stale;
  }
  EXPECT_TRUE(isStaleNow);
}

TEST_F(ObstacleMonitorNodeTest, ShutdownFromActiveStopsPublishing)
{
  bringUp();
  ASSERT_TRUE(waitUntil([this] {return statusCount_ >= 3;}, false));
  EXPECT_EQ(
    monitor_->shutdown().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED);
  spinFor(milliseconds(100));  // drain messages already in flight
  const int statusCountAfterShutdown = statusCount_;
  spinFor(milliseconds(500));
  EXPECT_EQ(statusCount_, statusCountAfterShutdown);
}

}  // namespace lightnav
