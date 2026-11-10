#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cstdarg>
#include <cstdio>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <lightnav_msgs/msg/obstacle_status.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rcutils/logging.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include "lightnav_obstacle_monitor/obstacle_monitor_node.hpp"

namespace lightnav
{
namespace
{
using std::chrono::milliseconds;
using ObstacleStatus = lightnav_msgs::msg::ObstacleStatus;
constexpr auto kWaitLimit = std::chrono::seconds(10);
const char kSensorFrame[] = "test_sensor";

class LogCapture
{
public:
  struct Entry {int severity; std::string message;};
  LogCapture() : previous_(rcutils_logging_get_output_handler())
  {
    current_ = this;
    rcutils_logging_set_output_handler(&capture);
  }
  ~LogCapture() {rcutils_logging_set_output_handler(previous_); current_ = nullptr;}
  bool contains(int severity, const std::string & message) const
  {
    for (const auto & entry : entries_) {
      if (entry.severity == severity && entry.message.find(message) != std::string::npos) {return true;}
    }
    return false;
  }
private:
  static void capture(const rcutils_log_location_t *, int severity, const char *,
    rcutils_time_point_value_t, const char * format, va_list * arguments)
  {
    char message[2048];
    vsnprintf(message, sizeof(message), format, *arguments);
    if (current_) {current_->entries_.push_back({severity, message});}
  }
  static LogCapture * current_;
  rcutils_logging_output_handler_t previous_;
  std::vector<Entry> entries_;
};
LogCapture * LogCapture::current_ = nullptr;

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
      [this](nav_msgs::msg::OccupancyGrid::SharedPtr grid) {lastGrid_ = *grid; ++gridCount_;});
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
    transform.header.frame_id = "odom";
    transform.child_frame_id = "base_link";
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
  nav_msgs::msg::OccupancyGrid lastGrid_;
  int statusCount_ = 0;
  int gridCount_ = 0;
};
}  // namespace

TEST_F(ObstacleMonitorNodeTest, StructuralParametersAreReadOnly)
{
  const auto result = monitor_->set_parameter(rclcpp::Parameter("step_neighbor_radius_m", 0.1));
  EXPECT_FALSE(result.successful);
  EXPECT_NE(result.reason.find("read-only"), std::string::npos);
  EXPECT_FALSE(monitor_->set_parameter(rclcpp::Parameter("use_sim_time", true)).successful);
}

TEST_F(ObstacleMonitorNodeTest, MemoryParametersRespectStartupAndAreApplied)
{
  bringUp();
  for (const auto & name : {"blind_radius_m", "memory_max_age_s"}) {
    const double initial = monitor_->get_parameter(name).as_double();
    EXPECT_TRUE(monitor_->set_parameter(rclcpp::Parameter(name, initial + 1.0)).successful);
    EXPECT_FALSE(monitor_->set_parameter(rclcpp::Parameter(name, initial - 0.1)).successful);
    for (double invalid : {std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity()}) {
      EXPECT_FALSE(monitor_->set_parameter(rclcpp::Parameter(name, invalid)).successful);
    }
    EXPECT_DOUBLE_EQ(monitor_->get_parameter(name).as_double(), initial + 1.0);
    EXPECT_TRUE(monitor_->set_parameter(rclcpp::Parameter(name, initial)).successful);
  }
  for (const auto & name : {"base_frame", "memory_frame"}) {
    EXPECT_TRUE(monitor_->describe_parameter(name).read_only);
    EXPECT_FALSE(monitor_->set_parameter(rclcpp::Parameter(name, "other_frame")).successful);
  }
  for (const auto & name : {"memory_max_range_m", "tf_max_age_s", "memory_reset_report_s",
      "memory_jump_distance_m", "memory_jump_angle_rad"}) {
    EXPECT_TRUE(monitor_->describe_parameter(name).read_only);
    EXPECT_FALSE(monitor_->set_parameter(rclcpp::Parameter(name, 2.0)).successful);
  }
}

TEST_F(ObstacleMonitorNodeTest, InvalidMemoryStartupValuesFailConfigure)
{
  for (const auto & name : {"blind_radius_m", "memory_max_age_s"}) {
    for (double invalid : {-1.0, std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity()}) {
      rclcpp::NodeOptions options;
      options.parameter_overrides({rclcpp::Parameter(name, invalid)});
      auto node = std::make_shared<ObstacleMonitorNode>(options);
      EXPECT_EQ(node->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
    }
  }
  rclcpp::NodeOptions options;
  options.parameter_overrides({{"memory_max_age_s", 0.0}});
  auto node = std::make_shared<ObstacleMonitorNode>(options);
  EXPECT_EQ(node->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
}

TEST_F(ObstacleMonitorNodeTest, InvalidStepNeighborOverridesFailConfigure)
{
  for (double value : {-0.1, std::numeric_limits<double>::quiet_NaN()}) {
    // Arrange
    rclcpp::NodeOptions options;
    options.parameter_overrides({{"step_neighbor_radius_m", value}});
    auto node = std::make_shared<ObstacleMonitorNode>(options);
    // Act
    const auto state = node->configure().id();
    // Assert
    EXPECT_EQ(state, lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
  }
}

TEST_F(ObstacleMonitorNodeTest, LogsAcceptedAndRejectedChanges)
{
  // Arrange
  bringUp();
  LogCapture logs;
  // Act
  const auto accepted = monitor_->set_parameter(rclcpp::Parameter("stop_zone_x_m", 1.2));
  const auto reversed = monitor_->set_parameter(rclcpp::Parameter("stop_zone_x_m", 0.9));
  const auto nan = monitor_->set_parameter(rclcpp::Parameter(
    "clear_hold_s", std::numeric_limits<double>::quiet_NaN()));
  const auto readOnly = monitor_->set_parameter(rclcpp::Parameter("use_sim_time", true));
  // Assert
  EXPECT_TRUE(accepted.successful);
  EXPECT_FALSE(reversed.successful);
  EXPECT_FALSE(nan.successful);
  EXPECT_FALSE(readOnly.successful);
  EXPECT_NE(reversed.reason.find("baseline="), std::string::npos);
  EXPECT_NE(reversed.reason.find("requested="), std::string::npos);
  EXPECT_TRUE(logs.contains(RCUTILS_LOG_SEVERITY_INFO, "parameter changed: stop_zone_x_m"));
  EXPECT_TRUE(logs.contains(RCUTILS_LOG_SEVERITY_WARN, reversed.reason));
  EXPECT_TRUE(logs.contains(RCUTILS_LOG_SEVERITY_WARN, nan.reason));
  EXPECT_TRUE(logs.contains(RCUTILS_LOG_SEVERITY_WARN, readOnly.reason));
}

TEST_F(ObstacleMonitorNodeTest, UseSimTimeIsRejectedInEveryLifecycleState)
{
  // Arrange and act / assert in each state
  EXPECT_FALSE(monitor_->set_parameter(rclcpp::Parameter("use_sim_time", true)).successful);
  ASSERT_EQ(monitor_->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  EXPECT_FALSE(monitor_->set_parameter(rclcpp::Parameter("use_sim_time", true)).successful);
  ASSERT_EQ(monitor_->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
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
  // Arrange: observed ground can clear the remembered obstacle in this test.
  executor_.remove_node(monitor_->get_node_base_interface());
  rclcpp::NodeOptions options;
  options.parameter_overrides({{"points_timeout_s", 0.3}, {"clear_hold_s", 0.2},
    {"watchdog_hz", 20.0}, {"blind_radius_m", 0.0}});
  monitor_ = std::make_shared<ObstacleMonitorNode>(options);
  executor_.add_node(monitor_->get_node_base_interface());
  bringUp();
  publishIdentityTransform();
  ASSERT_TRUE(waitUntil([this] {return lastStatus_.is_stop_zone_occupied;}, true));
  const int previousCount = statusCount_;
  // Act
  ASSERT_TRUE(monitor_->set_parameter(rclcpp::Parameter("clear_hold_s", 1.0)).successful);
  // Assert: the occupied state survives the update.
  EXPECT_DOUBLE_EQ(monitor_->get_parameter("clear_hold_s").as_double(), 1.0);
  ASSERT_TRUE(waitUntil([this, previousCount] {return statusCount_ > previousCount;}, false));
  EXPECT_TRUE(lastStatus_.is_stop_zone_occupied);

  // Continue sending valid empty clouds so the hysteresis advances without stale input.
  auto emptyCloud = makeObstacleCloud();
  sensor_msgs::PointCloud2Modifier modifier(emptyCloud);
  modifier.resize(0);
  const auto start = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - start < milliseconds(500)) {
    emptyCloud.header.stamp = helper_->now();
    pointsPublisher_->publish(emptyCloud);
    spinFor(milliseconds(50));
  }
  EXPECT_FALSE(lastStatus_.is_input_stale);
  EXPECT_TRUE(lastStatus_.is_stop_zone_occupied);
  // Ground observations clear the memory, then the updated hold time expires.
  auto groundCloud = makeObstacleCloud();
  sensor_msgs::PointCloud2Iterator<float> z(groundCloud, "z");
  for (int index = 0; index < 6; ++index, ++z) {*z = 0.0F;}
  const auto groundStart = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - groundStart < milliseconds(500)) {
    groundCloud.header.stamp = helper_->now();
    pointsPublisher_->publish(groundCloud);
    spinFor(milliseconds(50));
  }
  EXPECT_TRUE(lastStatus_.is_stop_zone_occupied);
  ASSERT_TRUE(waitUntil([this, &groundCloud] {
    groundCloud.header.stamp = helper_->now();
    pointsPublisher_->publish(groundCloud);
    return !lastStatus_.is_stop_zone_occupied;
  }, false));
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

TEST_F(ObstacleMonitorNodeTest, MissingMemoryTransformIsReportedAsStale)
{
  bringUp();
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = helper_->now();
  transform.header.frame_id = "base_link";
  transform.child_frame_id = kSensorFrame;
  transform.transform.rotation.w = 1.0;
  broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(helper_);
  broadcaster_->sendTransform(transform);
  ASSERT_TRUE(waitUntil([this] {return isStale("tf:odom");}, true));
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


TEST_F(ObstacleMonitorNodeTest, BlindMemoryKeepsStopAndGridPastAge)
{
  executor_.remove_node(monitor_->get_node_base_interface());
  rclcpp::NodeOptions options;
  options.parameter_overrides({{"points_timeout_s", 0.3}, {"clear_hold_s", 0.2},
    {"watchdog_hz", 20.0}, {"memory_max_age_s", 0.1}});
  monitor_ = std::make_shared<ObstacleMonitorNode>(options);
  executor_.add_node(monitor_->get_node_base_interface());
  bringUp();
  publishIdentityTransform();
  geometry_msgs::msg::TransformStamped approach;
  approach.header.stamp = helper_->now();
  approach.header.frame_id = "odom";
  approach.child_frame_id = "base_link";
  approach.transform.rotation.w = 1.0;
  approach.transform.translation.x = -0.4;
  broadcaster_->sendTransform(approach);
  spinFor(milliseconds(100));
  const auto seenEnd = std::chrono::steady_clock::now() + milliseconds(500);
  while (std::chrono::steady_clock::now() < seenEnd) {
    auto cloud = makeObstacleCloud();
    sensor_msgs::PointCloud2Iterator<float> x(cloud, "x");
    for (int index = 0; index < 6; ++index, ++x) {*x = 0.95F;}
    pointsPublisher_->publish(cloud);
    spinFor(milliseconds(50));
  }
  approach.header.stamp = helper_->now();
  approach.transform.translation.x = 0.0;
  broadcaster_->sendTransform(approach);
  spinFor(milliseconds(100));
  const auto end = std::chrono::steady_clock::now() + milliseconds(500);
  while (std::chrono::steady_clock::now() < end) {
    auto cloud = makeObstacleCloud();
    sensor_msgs::PointCloud2Iterator<float> x(cloud, "x"), z(cloud, "z");
    for (int index = 0; index < 6; ++index, ++x, ++z) {*x = 1.5F; *z = 0.0F;}
    pointsPublisher_->publish(cloud);
    spinFor(milliseconds(50));
  }
  EXPECT_FALSE(lastStatus_.is_input_stale);
  EXPECT_TRUE(lastStatus_.is_stop_zone_occupied);
  EXPECT_GE(lastStatus_.stop_zone_cell_count, 2U);
  EXPECT_TRUE(std::any_of(lastGrid_.data.begin(), lastGrid_.data.end(), [](auto value) {return value > 0;}));
  // Move back to make the remembered area observable, then observe ground there.
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = helper_->now();
  transform.header.frame_id = "odom";
  transform.child_frame_id = "base_link";
  transform.transform.rotation.w = 1.0;
  transform.transform.translation.x = -0.7;
  broadcaster_->sendTransform(transform);
  spinFor(milliseconds(100));
  const auto clearEnd = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (std::chrono::steady_clock::now() < clearEnd) {
    auto cloud = makeObstacleCloud();
    sensor_msgs::PointCloud2Iterator<float> x(cloud, "x"), z(cloud, "z");
    for (int index = 0; index < 6; ++index, ++x, ++z) {*x = 1.25F; *z = 0.0F;}
    pointsPublisher_->publish(cloud);
    spinFor(milliseconds(50));
  }
  EXPECT_FALSE(lastStatus_.is_input_stale);
  EXPECT_FALSE(lastStatus_.is_stop_zone_occupied);
  EXPECT_TRUE(std::none_of(lastGrid_.data.begin(), lastGrid_.data.end(), [](auto value) {return value > 0;}));
}

TEST_F(ObstacleMonitorNodeTest, SlightlyOlderDynamicTransformWorksButOldTransformIsStale)
{
  bringUp();
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = helper_->now();
  transform.header.frame_id = "base_link";
  transform.child_frame_id = kSensorFrame;
  transform.transform.rotation.w = 1.0;
  broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(helper_);
  broadcaster_->sendTransform(transform);
  tf2_ros::TransformBroadcaster dynamicBroadcaster(helper_);
  transform.header.frame_id = "odom";
  transform.child_frame_id = "base_link";
  transform.header.stamp = helper_->now() - rclcpp::Duration::from_seconds(0.05);
  dynamicBroadcaster.sendTransform(transform);
  spinFor(milliseconds(100));
  ASSERT_TRUE(waitUntil([this] {return lastStatus_.is_stop_zone_occupied &&
    !lastStatus_.is_input_stale && gridCount_ > 0;}, true));
  // Keep sending fresh clouds while the dynamic transform becomes too old.
  ASSERT_TRUE(waitUntil([this] {return isStale("tf:odom");}, true));
  EXPECT_TRUE(lastStatus_.is_input_stale);
}

TEST_F(ObstacleMonitorNodeTest, OldCloudReportsMemoryTfStale)
{
  bringUp();
  publishIdentityTransform();
  ASSERT_TRUE(waitUntil([this] {return lastStatus_.is_stop_zone_occupied;}, true));
  auto cloud = makeObstacleCloud();
  cloud.header.stamp = helper_->now() - rclcpp::Duration::from_seconds(2.0);
  pointsPublisher_->publish(cloud);
  ASSERT_TRUE(waitUntil([this] {return isStale("tf:odom");}, false));
  EXPECT_TRUE(lastStatus_.is_input_stale);
}

TEST_F(ObstacleMonitorNodeTest, TransformJumpIsReported)
{
  bringUp();
  publishIdentityTransform();
  ASSERT_TRUE(waitUntil([this] {return lastStatus_.is_stop_zone_occupied;}, true));
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = helper_->now();
  transform.header.frame_id = "odom";
  transform.child_frame_id = "base_link";
  transform.transform.rotation.w = 1.0;
  transform.transform.translation.x = 2.0;
  broadcaster_->sendTransform(transform);
  spinFor(milliseconds(100));
  ASSERT_TRUE(waitUntil([this] {return isStale("obstacle_memory_reset");}, true));
  EXPECT_TRUE(lastStatus_.is_input_stale);
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
