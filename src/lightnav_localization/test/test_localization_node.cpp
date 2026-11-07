#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <future>
#include <limits>
#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

#include "lightnav_localization/localization_node.hpp"
#include "lightnav_localization/map_metadata.hpp"
#include "lightnav_localization/pcd_io.hpp"
#include "synthetic_world.hpp"

using namespace lightnav_localization;
using namespace lightnav_localization::testing;
using namespace std::chrono_literals;
using LocalizationStatus = lightnav_msgs::msg::LocalizationStatus;
using SetInitialPose = lightnav_msgs::srv::SetInitialPose;

namespace {

constexpr double kMetersPerDegreeLatitude = 111320.0;
const GpsOrigin kOrigin{36.0, 140.0, 0.0};
const Eigen::Vector3d kTruthPosition(5.0, 0.0, 1.0);
constexpr double kTruthYaw = 0.7;
constexpr int kInputWheel = 1;
constexpr int kInputScans = 2;
constexpr int kInputGps = 4;

geometry_msgs::msg::Quaternion quaternion_from_yaw(double yaw) {
  geometry_msgs::msg::Quaternion quaternion;
  quaternion.z = std::sin(0.5 * yaw);
  quaternion.w = std::cos(0.5 * yaw);
  return quaternion;
}

double yaw_from_quaternion(const geometry_msgs::msg::Quaternion& quaternion) {
  return std::atan2(2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
                    1.0 - 2.0 * (quaternion.y * quaternion.y +
                                 quaternion.z * quaternion.z));
}

}  // namespace

class LocalizationNodeTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void SetUp() override {
    const auto unique = std::to_string(
        std::chrono::steady_clock::now().time_since_epoch().count());
    temp_directory_ =
        std::filesystem::temp_directory_path() / ("lightnav_localization_test_" + unique);
    std::filesystem::create_directories(temp_directory_);
    map_file_ = temp_directory_ / "map.pcd";
    map_yaml_ = temp_directory_ / "map.yaml";

    world_ = std::make_unique<SyntheticWorld>(
        std::vector<Eigen::Vector2d>{Eigen::Vector2d(0.0, 0.0),
                                     Eigen::Vector2d(30.0, 0.0)},
        true);
    const PointList map_points = world_->sample_map(0.2);
    write_pcd(map_file_.string(), map_points, PcdEncoding::kBinary);
    MapMetadata metadata;
    metadata.has_gps_origin = true;
    metadata.origin_latitude_deg = kOrigin.latitude_deg;
    metadata.origin_longitude_deg = kOrigin.longitude_deg;
    metadata.origin_altitude_m = kOrigin.altitude_m;
    write_map_metadata(map_yaml_.string(), metadata);
  }

  void TearDown() override {
    close_node();
    std::error_code error;
    std::filesystem::remove_all(temp_directory_, error);
  }

  rclcpp::NodeOptions make_options(const std::string& map_file,
                                   const std::string& map_yaml,
                                   const std::string& mode = "localization") const {
    rclcpp::NodeOptions options;
    options.parameter_overrides(
        {rclcpp::Parameter("map_file", map_file),
         rclcpp::Parameter("map_yaml", map_yaml),
         rclcpp::Parameter("mode", mode),
         rclcpp::Parameter("status_rate_hz", 10.0)});
    return options;
  }

  void create_node(const rclcpp::NodeOptions& options) {
    node_ = std::make_shared<LocalizationNode>(options);
    helper_ = std::make_shared<rclcpp::Node>("localization_node_test_helper");
    wheel_publisher_ = helper_->create_publisher<nav_msgs::msg::Odometry>(
        "/wheel/odom", rclcpp::QoS(50).reliable());
    points_publisher_ = helper_->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/velodyne_points", rclcpp::QoS(5).reliable());
    gps_publisher_ = helper_->create_publisher<sensor_msgs::msg::NavSatFix>(
        "/gps/fix", rclcpp::QoS(10).reliable());
    odom_subscription_ = helper_->create_subscription<nav_msgs::msg::Odometry>(
        "/lightnav/localization/odom", rclcpp::QoS(10).reliable(),
        [this](nav_msgs::msg::Odometry::SharedPtr message) {
          const int64_t stamp_nanoseconds =
              rclcpp::Time(message->header.stamp).nanoseconds();
          if (published_wheel_stamps_.find(stamp_nanoseconds) ==
              published_wheel_stamps_.end()) {
            all_odom_stamps_matched_ = false;
          }
          last_odom_ = *message;
          ++odom_count_;
        });
    status_subscription_ = helper_->create_subscription<LocalizationStatus>(
        "/lightnav/localization/status",
        rclcpp::QoS(1).reliable().transient_local(),
        [this](LocalizationStatus::SharedPtr message) {
          last_status_ = *message;
          ++status_count_;
        });
    set_initial_pose_client_ =
        helper_->create_client<SetInitialPose>("/localization_node/set_initial_pose");
    static_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(helper_);
    executor_.add_node(node_->get_node_base_interface());
    executor_.add_node(helper_);
  }

  void close_node() {
    if (node_) {
      executor_.remove_node(node_->get_node_base_interface());
    }
    if (helper_) {
      executor_.remove_node(helper_);
    }
    executor_.cancel();
    static_broadcaster_.reset();
    set_initial_pose_client_.reset();
    status_subscription_.reset();
    odom_subscription_.reset();
    gps_publisher_.reset();
    points_publisher_.reset();
    wheel_publisher_.reset();
    helper_.reset();
    node_.reset();
    odom_count_ = 0;
    status_count_ = 0;
  }

  void send_static_transform(const std::string& parent_frame,
                             const std::string& child_frame) {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = helper_->now();
    transform.header.frame_id = parent_frame;
    transform.child_frame_id = child_frame;
    transform.transform.rotation.w = 1.0;
    static_broadcaster_->sendTransform(transform);
  }

  void bring_up() {
    ASSERT_EQ(node_->configure().id(),
              lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    send_static_transform("base_footprint", "velodyne");
    send_static_transform("base_footprint", "gnss");
    ASSERT_EQ(node_->activate().id(),
              lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
    spin_for(200ms);
  }

  void spin_for(std::chrono::milliseconds duration) {
    const auto end = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < end) {
      executor_.spin_some(5ms);
    }
  }

  void publish_wheel(const rclcpp::Time& stamp) {
    nav_msgs::msg::Odometry message;
    message.header.stamp = stamp;
    message.header.frame_id = "odom";
    message.child_frame_id = "base_footprint";
    message.pose.pose.position.x = kTruthPosition.x();
    message.pose.pose.position.y = kTruthPosition.y();
    message.pose.pose.position.z = kTruthPosition.z();
    message.pose.pose.orientation = quaternion_from_yaw(kTruthYaw);
    message.pose.covariance[0] = 1e-4;
    message.pose.covariance[7] = 1e-4;
    message.pose.covariance[35] = 1e-4;
    published_wheel_stamps_.insert(rclcpp::Time(stamp).nanoseconds());
    wheel_publisher_->publish(message);
  }

  void publish_scan(const rclcpp::Time& stamp) {
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() = kTruthPosition;
    pose.linear() =
        Eigen::AngleAxisd(kTruthYaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    const std::vector<TimedPoint> points = world_->synthesize_scan(
        [&pose](double /*time_sec*/) { return pose; }, stamp.seconds() - 0.1, 0.1, true,
        360, 8);
    constexpr std::uint32_t kWidth = 32;
    const std::size_t height = std::max<std::size_t>(1, points.size() / kWidth);
    constexpr std::uint32_t kPointStep = 16;
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp = stamp;
    cloud.header.frame_id = "velodyne";
    cloud.height = static_cast<std::uint32_t>(height);
    cloud.width = kWidth;
    cloud.is_dense = false;
    cloud.is_bigendian = false;
    cloud.point_step = kPointStep;
    cloud.row_step = kPointStep * cloud.width;
    for (const char* name : {"x", "y", "z", "time"}) {
      sensor_msgs::msg::PointField field;
      field.name = name;
      field.offset = static_cast<std::uint32_t>(cloud.fields.size()) * 4U;
      field.datatype = sensor_msgs::msg::PointField::FLOAT32;
      field.count = 1;
      cloud.fields.push_back(field);
    }
    cloud.data.assign(static_cast<std::size_t>(cloud.row_step) * cloud.height, 0);
    float* data = reinterpret_cast<float*>(cloud.data.data());
    std::size_t point_index = 0;
    for (std::size_t row = 0; row < height; ++row) {
      for (std::uint32_t column = 0; column < kWidth; ++column, ++point_index) {
        float* point = data + (row * kWidth + column) * 4;
        if (point_index >= points.size()) continue;
        point[0] = points[point_index].position.x();
        point[1] = points[point_index].position.y();
        point[2] = points[point_index].position.z();
        point[3] = points[point_index].time_offset_sec;
        // Organized clouds contain NaN for missing returns.
        if (point_index % 97 == 0) {
          point[0] = std::numeric_limits<float>::quiet_NaN();
          point[1] = std::numeric_limits<float>::quiet_NaN();
          point[2] = std::numeric_limits<float>::quiet_NaN();
        }
      }
    }
    points_publisher_->publish(cloud);
  }

  void publish_gps(const rclcpp::Time& stamp) {
    const Eigen::Vector2d antenna_position =
        kTruthPosition.head<2>() + Eigen::Vector2d(0.4, 0.2);
    sensor_msgs::msg::NavSatFix message;
    message.header.stamp = stamp;
    message.header.frame_id = "gnss";
    message.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
    message.latitude = kOrigin.latitude_deg +
                       antenna_position.y() / kMetersPerDegreeLatitude;
    message.longitude = kOrigin.longitude_deg +
                        antenna_position.x() /
                            (kMetersPerDegreeLatitude *
                             std::cos(kOrigin.latitude_deg * M_PI / 180.0));
    message.altitude = kTruthPosition.z();
    message.position_covariance_type =
        sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_KNOWN;
    message.position_covariance[0] = 1.0;
    message.position_covariance[4] = 1.0;
    message.position_covariance[8] = 1.0;
    gps_publisher_->publish(message);
  }

  void run_for(std::chrono::milliseconds duration, int inputs) {
    const auto end = std::chrono::steady_clock::now() + duration;
    auto next_wheel = std::chrono::steady_clock::now();
    auto next_scan = std::chrono::steady_clock::now();
    auto next_gps = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() < end) {
      const auto now = std::chrono::steady_clock::now();
      const rclcpp::Time stamp = node_->get_clock()->now();
      if ((inputs & kInputWheel) != 0 && now >= next_wheel) {
        publish_wheel(stamp);
        while (next_wheel <= now) next_wheel += 20ms;
      }
      if ((inputs & kInputScans) != 0 && now >= next_scan) {
        publish_scan(stamp);
        while (next_scan <= now) next_scan += 100ms;
      }
      if ((inputs & kInputGps) != 0 && now >= next_gps) {
        publish_gps(stamp);
        while (next_gps <= now) next_gps += 1000ms;
      }
      executor_.spin_some(5ms);
    }
  }

  bool run_until(const std::function<bool()>& condition, std::chrono::milliseconds timeout,
                 int inputs) {
    const auto end = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < end) {
      if (condition()) return true;
      run_for(50ms, inputs);
    }
    return condition();
  }

  std::filesystem::path temp_directory_;
  std::filesystem::path map_file_;
  std::filesystem::path map_yaml_;
  std::unique_ptr<SyntheticWorld> world_;
  std::shared_ptr<LocalizationNode> node_;
  std::shared_ptr<rclcpp::Node> helper_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr wheel_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Subscription<LocalizationStatus>::SharedPtr status_subscription_;
  rclcpp::Client<SetInitialPose>::SharedPtr set_initial_pose_client_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
  nav_msgs::msg::Odometry last_odom_;
  LocalizationStatus last_status_;
  std::set<int64_t> published_wheel_stamps_;
  int odom_count_ = 0;
  int status_count_ = 0;
  bool all_odom_stamps_matched_ = true;
};

TEST_F(LocalizationNodeTest, ConfigureFailsWithoutMapFile) {
  create_node(make_options((temp_directory_ / "missing.pcd").string(),
                           map_yaml_.string()));
  EXPECT_EQ(node_->configure().id(),
            lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
}

TEST_F(LocalizationNodeTest, ConfigureFailsInMappingMode) {
  create_node(make_options(map_file_.string(), map_yaml_.string(), "mapping"));
  EXPECT_EQ(node_->configure().id(),
            lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
}

TEST_F(LocalizationNodeTest, StaysInitializingWithoutGpsOrService) {
  create_node(make_options(map_file_.string(), map_yaml_.string()));
  bring_up();
  run_for(2000ms, kInputWheel | kInputScans);
  ASSERT_GT(status_count_, 0);
  EXPECT_EQ(last_status_.state, LocalizationStatus::INITIALIZING);
  EXPECT_EQ(odom_count_, 0);
}

TEST_F(LocalizationNodeTest, GpsWithUnknownYawTracksAndWheelStopGoesStale) {
  create_node(make_options(map_file_.string(), map_yaml_.string()));
  bring_up();
  const auto test_start = std::chrono::steady_clock::now();
  const bool is_tracking = run_until(
      [this]() {
        if (last_status_.state != LocalizationStatus::TRACKING || odom_count_ == 0) {
          return false;
        }
        const Eigen::Vector2d error =
            Eigen::Vector2d(last_odom_.pose.pose.position.x,
                            last_odom_.pose.pose.position.y) -
            kTruthPosition.head<2>();
        return error.norm() < 0.1;
      },
      10000ms, kInputWheel | kInputScans | kInputGps);
  if (!is_tracking) {
    std::printf(
        "[ DIAG ] state=%u gps_status=%u inlier=%.3f lidar_inlier=%.3f "
        "seconds_since_match=%.3f std=(%.3f,%.3f,%.3f) odom_count=%d error=%s\n",
        static_cast<unsigned>(last_status_.state),
        static_cast<unsigned>(last_status_.gps_status), last_status_.last_inlier_ratio,
        last_status_.last_lidar_odom_inlier_ratio, last_status_.seconds_since_last_match,
        last_status_.position_std_m[0], last_status_.position_std_m[1],
        last_status_.position_std_m[2], odom_count_, last_status_.last_error.c_str());
  }
  ASSERT_TRUE(is_tracking) << "last_error: " << last_status_.last_error;
  ASSERT_GT(odom_count_, 0);
  ASSERT_GT(published_wheel_stamps_.size(), 0U);

  // N: frame ids, covariance and stamp contract.
  EXPECT_EQ(last_odom_.header.frame_id, "map");
  EXPECT_EQ(last_odom_.child_frame_id, "base_footprint");
  for (const std::size_t index : {0U, 7U, 14U, 21U, 28U, 35U}) {
    EXPECT_GT(last_odom_.pose.covariance[index], 0.0) << "covariance index " << index;
  }
  // lightnav_pose_adapter default max_position_variance_m2 = 4.0.
  EXPECT_LE(last_odom_.pose.covariance[0], 4.0);
  EXPECT_LE(last_odom_.pose.covariance[7], 4.0);
  EXPECT_NE(published_wheel_stamps_.find(rclcpp::Time(last_odom_.header.stamp).nanoseconds()),
            published_wheel_stamps_.end());
  EXPECT_TRUE(all_odom_stamps_matched_);

  const Eigen::Vector2d position_error =
      Eigen::Vector2d(last_odom_.pose.pose.position.x,
                      last_odom_.pose.pose.position.y) -
      kTruthPosition.head<2>();
  EXPECT_LT(position_error.norm(), 0.2);
  const double yaw_error =
      std::abs(yaw_from_quaternion(last_odom_.pose.pose.orientation) - kTruthYaw);
  EXPECT_LT(yaw_error, 0.1);
  std::printf(
      "[ J ] time_to_tracking_with_error=%.3f s position_error=%.4f m yaw_error=%.4f rad "
      "inlier_ratio=%.3f\n",
      std::chrono::duration<double>(std::chrono::steady_clock::now() - test_start).count(),
      position_error.norm(), yaw_error, last_status_.last_inlier_ratio);
  std::printf(
      "[ N ] frame_id=%s child_frame_id=%s covariance_diagonal=(%.4f,%.4f,%.4f) "
      "stamp_matched_input=1\n",
      last_odom_.header.frame_id.c_str(), last_odom_.child_frame_id.c_str(),
      last_odom_.pose.covariance[0], last_odom_.pose.covariance[7],
      last_odom_.pose.covariance[35]);

  // G: no wheel samples -> no odom, status INPUT_STALE within the timeout + 0.1 s.
  const auto wheel_stopped_at = std::chrono::steady_clock::now();
  const auto count_when_wheel_stopped = odom_count_;
  const bool is_stale = run_until(
      [this]() { return last_status_.state == LocalizationStatus::INPUT_STALE; },
      1000ms, kInputScans | kInputGps);
  const double stale_after_s =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - wheel_stopped_at)
          .count();
  EXPECT_TRUE(is_stale) << "last state: " << static_cast<int>(last_status_.state)
                        << " last_error: " << last_status_.last_error;
  EXPECT_LE(stale_after_s, 0.3);  // wheel_timeout_s (0.2) + 0.1 s
  const auto count_after_timeout = odom_count_;
  run_for(100ms, kInputScans | kInputGps);
  EXPECT_EQ(odom_count_, count_after_timeout);
  EXPECT_GE(count_after_timeout, count_when_wheel_stopped);
  std::printf("[ G ] stale_after_wheel_stop=%.3f s odom_stopped=1\n", stale_after_s);
}

TEST_F(LocalizationNodeTest, SetInitialPoseWithYawTracks) {
  create_node(make_options(map_file_.string(), map_yaml_.string()));
  bring_up();
  run_for(600ms, kInputWheel | kInputScans);
  const auto service_start = std::chrono::steady_clock::now();

  auto request = std::make_shared<SetInitialPose::Request>();
  request->pose.header.frame_id = "map";
  request->pose.pose.pose.position.x = kTruthPosition.x();
  request->pose.pose.pose.position.y = kTruthPosition.y();
  request->pose.pose.pose.position.z = kTruthPosition.z();
  request->pose.pose.pose.orientation = quaternion_from_yaw(kTruthYaw);
  request->pose.pose.covariance[35] = 0.0;  // yaw is specified
  auto future = set_initial_pose_client_->async_send_request(request);
  ASSERT_TRUE(run_until(
      [&future]() { return future.wait_for(0ms) == std::future_status::ready; }, 3000ms,
      kInputWheel | kInputScans));
  const std::shared_ptr<SetInitialPose::Response> response = future.get();
  ASSERT_EQ(response->accepted, true) << response->message;

  const bool is_tracking = run_until(
      [this]() { return last_status_.state == LocalizationStatus::TRACKING; },
      10000ms, kInputWheel | kInputScans);
  ASSERT_TRUE(is_tracking) << "last_error: " << last_status_.last_error;
  ASSERT_GT(odom_count_, 0);
  const Eigen::Vector2d position_error =
      Eigen::Vector2d(last_odom_.pose.pose.position.x,
                      last_odom_.pose.pose.position.y) -
      kTruthPosition.head<2>();
  EXPECT_LT(position_error.norm(), 0.2);
  std::printf("[ SET_INITIAL_POSE ] time_to_tracking=%.3f s position_error=%.4f m\n",
              std::chrono::duration<double>(std::chrono::steady_clock::now() - service_start)
                  .count(),
              position_error.norm());
}
