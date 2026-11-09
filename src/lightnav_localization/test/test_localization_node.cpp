#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <map>
#include <future>
#include <fstream>
#include <limits>
#include <memory>
#include <optional>
#include <random>
#include <set>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
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
const Eigen::Vector3d kGpsLeverArm(-0.2, 0.0, 0.7);
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

struct TrajectoryRow {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double yaw = 0.0;
};

// Rows of trajectory.csv (stamp_sec,x,y,z,roll,pitch,yaw,position_std_m) without the header.
std::vector<TrajectoryRow> read_trajectory_rows(const std::filesystem::path& file_path) {
  std::ifstream stream(file_path);
  std::string line;
  std::getline(stream, line);  // header
  std::vector<TrajectoryRow> rows;
  while (std::getline(stream, line)) {
    if (line.empty()) continue;
    std::istringstream fields(line);
    std::vector<double> values;
    std::string field;
    while (std::getline(fields, field, ',')) values.push_back(std::stod(field));
    constexpr std::size_t kYawColumn = 6;
    if (values.size() <= kYawColumn) continue;
    rows.push_back({values[1], values[2], values[3], values[kYawColumn]});
  }
  return rows;
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
    no_origin_yaml_ = temp_directory_ / "map_no_origin.yaml";

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
    MapMetadata no_origin_metadata;
    no_origin_metadata.has_gps_origin = false;
    write_map_metadata(no_origin_yaml_.string(), no_origin_metadata);
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
          observed_states_.insert(message->state);
          ++status_count_;
        });
    diagnostics_subscription_ = helper_->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
        "/diagnostics", rclcpp::QoS(10).reliable(),
        [this](diagnostic_msgs::msg::DiagnosticArray::SharedPtr message) {
          std::set<std::string> names;
          for (const auto& status : message->status) {
            names.insert(status.name);
            diagnostics_levels_[status.name] = status.level;
            if (status.name == "localization_node: performance") {
              std::map<std::string, std::string> sample;
              for (const auto& entry : status.values) sample[entry.key] = entry.value;
              if (sample.count("scans_processed") && std::stoi(sample.at("scans_processed")) > 0) {
                saw_scan_timing_ = std::stod(sample.at("scan_time_mean_ms")) > 0.0 &&
                    std::stod(sample.at("scan_time_max_ms")) >= std::stod(sample.at("scan_time_mean_ms"));
              }
            }
            for (const auto& entry : status.values) diagnostics_values_[status.name][entry.key] = entry.value;
            if (status.name != "localization_node: GNSS") continue;
            last_gnss_diagnostics_level_ = status.level;
            for (const auto& entry : status.values) last_gnss_diagnostics_[entry.key] = entry.value;
          }
          if (names.count("localization_node: GNSS") && names.count("localization_node: state") &&
              names.count("localization_node: map match") && names.count("localization_node: errors") &&
              names.count("localization_node: performance")) ++complete_diagnostics_count_;
        });
    set_initial_pose_client_ =
        helper_->create_client<SetInitialPose>("/localization_node/set_initial_pose");
    save_map_client_ =
        helper_->create_client<std_srvs::srv::Trigger>("/localization_node/save_map");
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
    save_map_client_.reset();
    status_subscription_.reset();
    map_cloud_subscription_.reset();
    path_subscription_.reset();
    last_map_cloud_.reset();
    last_path_.reset();
    diagnostics_subscription_.reset();
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
                             const std::string& child_frame,
                             const Eigen::Vector3d& translation = Eigen::Vector3d::Zero()) {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = helper_->now();
    transform.header.frame_id = parent_frame;
    transform.child_frame_id = child_frame;
    transform.transform.translation.x = translation.x();
    transform.transform.translation.y = translation.y();
    transform.transform.translation.z = translation.z();
    transform.transform.rotation.w = 1.0;
    static_broadcaster_->sendTransform(transform);
  }

  void bring_up(bool with_gps_lever_arm = true) {
    ASSERT_EQ(node_->configure().id(),
              lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    send_static_transform("base_footprint", "velodyne");
    send_static_transform("base_footprint", "gnss",
                          with_gps_lever_arm ? kGpsLeverArm
                                             : Eigen::Vector3d::Zero());
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
    Eigen::Isometry3d base_pose = Eigen::Isometry3d::Identity();
    base_pose.translation() = kTruthPosition;
    base_pose.linear() =
        Eigen::AngleAxisd(kTruthYaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    const Eigen::Vector3d antenna_position = base_pose * kGpsLeverArm;
    double noise_east = gps_normal_(gps_random_);
    double noise_north = gps_normal_(gps_random_);
    double noise_up = gps_normal_(gps_random_);
    if (gps_fix_count_ == 0) {
      // The first fix is a 1-2 m error sample; the initial alignment must still recover.
      noise_east += 1.2;
      noise_north -= 0.6;
      noise_up += 0.5;
    }
    ++gps_fix_count_;
    const double east = antenna_position.x() + noise_east;
    const double north = antenna_position.y() + noise_north;

    sensor_msgs::msg::NavSatFix message;
    message.header.stamp = stamp;
    message.header.frame_id = "gnss";
    message.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
    message.latitude =
        kOrigin.latitude_deg + north / kMetersPerDegreeLatitude;
    message.longitude = kOrigin.longitude_deg +
                        east / (kMetersPerDegreeLatitude *
                                std::cos(kOrigin.latitude_deg * M_PI / 180.0));
    message.altitude = kOrigin.altitude_m + antenna_position.z() + noise_up;
    message.position_covariance_type =
        sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_KNOWN;
    message.position_covariance[0] = 1.0;
    message.position_covariance[4] = 1.0;
    message.position_covariance[8] = 1.0;
    gps_publisher_->publish(message);
  }

  void publish_wheel_at(const rclcpp::Time& stamp, const Eigen::Isometry3d& pose) {
    nav_msgs::msg::Odometry message;
    message.header.stamp = stamp;
    message.header.frame_id = "odom";
    message.child_frame_id = "base_footprint";
    message.pose.pose.position.x = pose.translation().x();
    message.pose.pose.position.y = pose.translation().y();
    message.pose.pose.position.z = pose.translation().z();
    const Eigen::Quaterniond orientation(pose.rotation());
    message.pose.pose.orientation.x = orientation.x();
    message.pose.pose.orientation.y = orientation.y();
    message.pose.pose.orientation.z = orientation.z();
    message.pose.pose.orientation.w = orientation.w();
    message.pose.covariance[0] = 1e-4;
    message.pose.covariance[7] = 1e-4;
    message.pose.covariance[35] = 1e-4;
    published_wheel_stamps_.insert(rclcpp::Time(stamp).nanoseconds());
    wheel_publisher_->publish(message);
  }

  void publish_scan_at(const rclcpp::Time& stamp, const Eigen::Isometry3d& pose) {
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
      }
    }
    points_publisher_->publish(cloud);
  }

  void publish_gps_at(const rclcpp::Time& stamp, const Eigen::Isometry3d& pose) {
    const Eigen::Vector3d antenna_position = pose * kGpsLeverArm;
    const double east = antenna_position.x() + 0.3 * gps_normal_(gps_random_);
    const double north = antenna_position.y() + 0.3 * gps_normal_(gps_random_);
    sensor_msgs::msg::NavSatFix message;
    message.header.stamp = stamp;
    message.header.frame_id = "gnss";
    message.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
    message.latitude = kOrigin.latitude_deg + north / kMetersPerDegreeLatitude;
    message.longitude = kOrigin.longitude_deg +
                        east / (kMetersPerDegreeLatitude *
                                std::cos(kOrigin.latitude_deg * M_PI / 180.0));
    message.altitude = kOrigin.altitude_m + antenna_position.z();
    message.position_covariance_type =
        sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_KNOWN;
    message.position_covariance[0] = gps_variance_m2_;
    message.position_covariance[4] = gps_variance_m2_;
    message.position_covariance[8] = gps_variance_m2_;
    gps_publisher_->publish(message);
  }

  Eigen::Isometry3d drive_mapping(
      const std::function<Eigen::Isometry3d(double)>& pose_at,
      std::chrono::milliseconds duration) {
    const auto start = std::chrono::steady_clock::now();
    auto next_wheel = start;
    auto next_scan = start;
    auto next_gps = start;
    Eigen::Isometry3d last_pose = Eigen::Isometry3d::Identity();
    while (std::chrono::steady_clock::now() - start < duration) {
      const auto now = std::chrono::steady_clock::now();
      const double elapsed = std::chrono::duration<double>(now - start).count();
      last_pose = pose_at(elapsed);
      const rclcpp::Time stamp = node_->get_clock()->now();
      if (now >= next_wheel) {
        publish_wheel_at(stamp, last_pose);
        while (next_wheel <= now) next_wheel += 20ms;
      }
      if (now >= next_scan) {
        publish_scan_at(stamp, last_pose);
        while (next_scan <= now) next_scan += 100ms;
      }
      if (now >= next_gps) {
        publish_gps_at(stamp, last_pose);
        while (next_gps <= now) next_gps += 200ms;
      }
      executor_.spin_some(5ms);
    }
    return last_pose;
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

  rclcpp::NodeOptions make_gnss_options(int min_status, double min_interval_s) const {
    rclcpp::NodeOptions options = make_options(map_file_.string(), map_yaml_.string());
    options.append_parameter_override("gps_min_status", min_status);
    options.append_parameter_override("gps_min_interval_s", min_interval_s);
    return options;
  }

  // A fix at the truth antenna position moved by the offset, as the RTK receiver publishes it:
  // DIAGONAL_KNOWN, which the node ignores in favour of the status table (decision D2).
  sensor_msgs::msg::NavSatFix make_gnss_fix(const rclcpp::Time& stamp, int8_t status,
                                            double east_offset_m) const {
    Eigen::Isometry3d base_pose = Eigen::Isometry3d::Identity();
    base_pose.translation() = kTruthPosition;
    base_pose.linear() =
        Eigen::AngleAxisd(kTruthYaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    const Eigen::Vector3d antenna_position = base_pose * kGpsLeverArm;
    const double east = antenna_position.x() + east_offset_m;
    sensor_msgs::msg::NavSatFix message;
    message.header.stamp = stamp;
    message.header.frame_id = "gnss";
    message.status.status = status;
    message.latitude = kOrigin.latitude_deg + antenna_position.y() / kMetersPerDegreeLatitude;
    message.longitude = kOrigin.longitude_deg +
                        east / (kMetersPerDegreeLatitude *
                                std::cos(kOrigin.latitude_deg * M_PI / 180.0));
    message.altitude = kOrigin.altitude_m + antenna_position.z();
    message.position_covariance_type =
        sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
    message.position_covariance[0] = 0.0004;
    message.position_covariance[4] = 0.0004;
    message.position_covariance[8] = 0.0016;
    return message;
  }

  // Wheel and scans at the truth pose for `duration`, plus one fix every `fix_period`.
  void run_with_fixes(std::chrono::milliseconds duration, std::chrono::milliseconds fix_period,
                      int8_t status, double east_offset_m) {
    const auto end = std::chrono::steady_clock::now() + duration;
    auto next_wheel = std::chrono::steady_clock::now();
    auto next_scan = next_wheel;
    auto next_fix = next_wheel;
    while (std::chrono::steady_clock::now() < end) {
      const auto now = std::chrono::steady_clock::now();
      const rclcpp::Time stamp = node_->get_clock()->now();
      if (now >= next_wheel) {
        publish_wheel(stamp);
        while (next_wheel <= now) next_wheel += 20ms;
      }
      if (now >= next_scan) {
        publish_scan(stamp);
        while (next_scan <= now) next_scan += 100ms;
      }
      if (now >= next_fix) {
        gps_publisher_->publish(make_gnss_fix(stamp, status, east_offset_m));
        ++published_fix_count_;
        while (next_fix <= now) next_fix += fix_period;
      }
      executor_.spin_some(5ms);
    }
  }

  // set_initial_pose at the truth, then TRACKING with odom flowing.
  void start_tracking_at_truth() {
    run_for(600ms, kInputWheel | kInputScans);
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
    ASSERT_TRUE(future.get()->accepted);
    ASSERT_TRUE(run_until(
        [this]() { return last_status_.state == LocalizationStatus::TRACKING && odom_count_ > 0; },
        10000ms, kInputWheel | kInputScans));
  }

  // The value of a key of the latest GNSS /diagnostics (published at 1 Hz); -1 while absent.
  int gnss_diagnostics_value(const std::string& key) const {
    const auto found = last_gnss_diagnostics_.find(key);
    return found == last_gnss_diagnostics_.end() ? -1 : std::stoi(found->second);
  }

  bool wait_for_gnss_diagnostics(const std::function<bool()>& condition,
                                 std::chrono::milliseconds timeout = 3000ms) {
    return run_until(condition, timeout, kInputWheel | kInputScans);
  }

  std::shared_ptr<std_srvs::srv::Trigger::Response> call_save_map() {
    auto future =
        save_map_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    const auto end = std::chrono::steady_clock::now() + 5000ms;
    while (future.wait_for(0ms) != std::future_status::ready &&
           std::chrono::steady_clock::now() < end) {
      executor_.spin_some(10ms);
    }
    if (future.wait_for(0ms) != std::future_status::ready) return nullptr;
    return future.get();
  }

  rclcpp::NodeOptions make_mapping_options(const std::filesystem::path& output_directory) const {
    rclcpp::NodeOptions options;
    options.parameter_overrides(
        {rclcpp::Parameter("mode", "mapping"),
         rclcpp::Parameter("map_output_dir", output_directory.string()),
         rclcpp::Parameter("init_travel_m", 0.5),
         rclcpp::Parameter("keyframe_time_s", 0.5),
         rclcpp::Parameter("keyframe_distance_m", 0.5),
         rclcpp::Parameter("status_rate_hz", 10.0)});
    return options;
  }

  // Latched subscriptions to the live preview topics (the same QoS Foxglove uses).
  void subscribe_visualization() {
    const rclcpp::QoS latched_qos = rclcpp::QoS(1).reliable().transient_local();
    map_cloud_subscription_ = helper_->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/lightnav/localization/map_cloud", latched_qos,
        [this](sensor_msgs::msg::PointCloud2::SharedPtr message) { last_map_cloud_ = *message; });
    path_subscription_ = helper_->create_subscription<nav_msgs::msg::Path>(
        "/lightnav/localization/path", latched_qos,
        [this](nav_msgs::msg::Path::SharedPtr message) { last_path_ = *message; });
  }

  // Mapping without GPS: stand still until INITIALIZING ends, then drive 1.6 m straight along +x.
  void drive_mapping_without_gps_along_x() {
    Eigen::Isometry3d start_pose = Eigen::Isometry3d::Identity();
    start_pose.translation() = Eigen::Vector3d(5.0, 3.0, kSensorHeightM);
    const auto pose_after = [&start_pose](double distance_m) {
      Eigen::Isometry3d pose = start_pose;
      pose.translation().x() += distance_m;
      return pose;
    };
    drive_mapping([&start_pose](double /*elapsed*/) { return start_pose; }, 1500ms);
    drive_mapping([&pose_after](double elapsed) { return pose_after(0.8 * elapsed); }, 2000ms);
  }

  rclcpp::NodeOptions make_indoor_mapping_options(const std::filesystem::path& output_directory,
                                                  bool is_visualization_enabled) const {
    rclcpp::NodeOptions options = make_mapping_options(output_directory);
    std::vector<rclcpp::Parameter> overrides = options.parameter_overrides();
    overrides.emplace_back("gps_topic", "/lightnav/gnss_disabled");
    overrides.emplace_back("allow_mapping_without_gps", true);
    overrides.emplace_back("gps_timeout_s", 1.0);
    overrides.emplace_back("visualization_enabled", is_visualization_enabled);
    overrides.emplace_back("visualization_rate_hz", 5.0);
    options.parameter_overrides(overrides);
    return options;
  }

  // Mapping for 2 s, moving along +x. Returns the last pose sent.
  Eigen::Isometry3d drive_mapping_forward() {
    return drive_mapping(
        [](double elapsed) {
          Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
          pose.translation() = Eigen::Vector3d(2.0 + 2.0 * elapsed, 0.0, 1.0);
          return pose;
        },
        2000ms);
  }

  // Contents of map.pcd, trajectory.csv and map.yaml in this order, with their modification times.
  std::vector<std::pair<std::string, std::filesystem::file_time_type>> snapshot_map_files(
      const std::filesystem::path& directory) const {
    std::vector<std::pair<std::string, std::filesystem::file_time_type>> files;
    for (const char* name : {"map.pcd", "trajectory.csv", "map.yaml"}) {
      std::ifstream stream(directory / name, std::ios::binary);
      std::stringstream buffer;
      buffer << stream.rdbuf();
      files.emplace_back(buffer.str(), std::filesystem::last_write_time(directory / name));
    }
    return files;
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_cloud_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscription_;
  std::optional<sensor_msgs::msg::PointCloud2> last_map_cloud_;
  std::optional<nav_msgs::msg::Path> last_path_;

  std::filesystem::path temp_directory_;
  std::filesystem::path map_file_;
  std::filesystem::path map_yaml_;
  std::filesystem::path no_origin_yaml_;
  std::unique_ptr<SyntheticWorld> world_;
  std::shared_ptr<LocalizationNode> node_;
  std::shared_ptr<rclcpp::Node> helper_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr wheel_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Subscription<LocalizationStatus>::SharedPtr status_subscription_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_subscription_;
  rclcpp::Client<SetInitialPose>::SharedPtr set_initial_pose_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr save_map_client_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
  nav_msgs::msg::Odometry last_odom_;
  LocalizationStatus last_status_;
  std::map<std::string, std::string> last_gnss_diagnostics_;
  uint8_t last_gnss_diagnostics_level_ = 255;
  int published_fix_count_ = 0;
  std::set<int64_t> published_wheel_stamps_;
  // Every state published on /lightnav/localization/status since the last clear().
  std::set<uint8_t> observed_states_;
  // Position variance of the fixes sent by publish_gps_at (sigma 0.6 m by default).
  double gps_variance_m2_ = 0.36;
  int odom_count_ = 0;
  int status_count_ = 0;
  int complete_diagnostics_count_ = 0;
  bool saw_scan_timing_ = false;
  std::map<std::string, std::map<std::string, std::string>> diagnostics_values_;
  std::map<std::string, uint8_t> diagnostics_levels_;
  bool all_odom_stamps_matched_ = true;
  std::mt19937 gps_random_{20261106};
  std::normal_distribution<double> gps_normal_{0.0, 1.0};
  int gps_fix_count_ = 0;
};

TEST_F(LocalizationNodeTest, PublishesAllDiagnosticsTogether) {
  create_node(make_options(map_file_.string(), map_yaml_.string()));
  bring_up();
  start_tracking_at_truth();
  run_for(5000ms, kInputWheel | kInputScans);
  EXPECT_GE(complete_diagnostics_count_, 3);
  EXPECT_TRUE(saw_scan_timing_);
  EXPECT_EQ(last_gnss_diagnostics_level_, diagnostic_msgs::msg::DiagnosticStatus::OK);
  const auto& performance = diagnostics_values_.at("localization_node: performance");
  EXPECT_GE(std::stod(performance.at("cpu_percent")), 0.0);
#ifdef __linux__
  EXPECT_GT(std::stod(performance.at("memory_rss_mb")), 0.0);
#else
  EXPECT_EQ(performance.at("memory_rss_mb"), "unavailable");
#endif
}

TEST_F(LocalizationNodeTest, MapMatchDiagnosticsCountEachRejectionReason) {
  create_node(make_options(map_file_.string(), map_yaml_.string()));
  bring_up();
  start_tracking_at_truth();
  run_for(1200ms, kInputWheel | kInputScans);
  const auto initial_match = diagnostics_values_.at("localization_node: map match");
  int attempt = 0;
  node_->set_map_match_hook_for_test([&attempt](MatchResult& result) {
    ++attempt;
    if (attempt > 4) {
      result.converged = true;
      result.inlier_ratio = 1.0;
      result.target_to_source.translation() = kTruthPosition;
      result.target_to_source.linear() =
          Eigen::AngleAxisd(kTruthYaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
      return;
    }
    if (attempt == 1) result.converged = false;
    if (attempt == 2) {
      result.converged = true;
      result.inlier_ratio = 0.0;
    }
    if (attempt == 3) {
      result.converged = true;
      result.inlier_ratio = 1.0;
      result.target_to_source.translation().x() += 100.0;
    }
    if (attempt == 4) {
      result.converged = true;
      result.inlier_ratio = 1.0;
      result.target_to_source.linear() =
          result.target_to_source.linear() *
          Eigen::AngleAxisd(1.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    }
  });
  run_for(5000ms, kInputWheel | kInputScans);
  ASSERT_GE(attempt, 4);
  const auto& match = diagnostics_values_.at("localization_node: map match");
  EXPECT_EQ(std::stoi(match.at("rejected_not_converged_total")) -
                std::stoi(initial_match.at("rejected_not_converged_total")),
            1);
  EXPECT_EQ(std::stoi(match.at("rejected_inlier_total")) -
                std::stoi(initial_match.at("rejected_inlier_total")),
            1);
  EXPECT_EQ(std::stoi(match.at("rejected_translation_gate_total")) -
                std::stoi(initial_match.at("rejected_translation_gate_total")),
            1);
  EXPECT_EQ(std::stoi(match.at("rejected_rotation_gate_total")) -
                std::stoi(initial_match.at("rejected_rotation_gate_total")),
            1);
}

TEST_F(LocalizationNodeTest, StateDiagnosticsWarnOnOdomOnlyAndErrorOnLost) {
  auto options = make_options(map_file_.string(), map_yaml_.string());
  options.append_parameter_override("tracking_window_s", 0.5);
  options.append_parameter_override("gps_timeout_s", 0.1);
  options.append_parameter_override("odom_only_max_s", 3.0);
  create_node(options);
  bring_up();
  start_tracking_at_truth();
  node_->set_map_match_hook_for_test([](MatchResult& result) { result.converged = false; });
  bool observed_odom_only = false;
  bool observed_lost = false;
  for (int index = 0; index < 35 && !observed_lost; ++index) {
    run_for(200ms, kInputWheel | kInputScans);
    const auto found = diagnostics_values_.find("localization_node: state");
    if (found == diagnostics_values_.end()) continue;
    if (found->second.at("state") == "ODOM_ONLY") {
      EXPECT_EQ(diagnostics_levels_.at("localization_node: state"),
                diagnostic_msgs::msg::DiagnosticStatus::WARN);
      observed_odom_only = true;
    }
    if (found->second.at("state") == "LOST") {
      EXPECT_EQ(diagnostics_levels_.at("localization_node: state"),
                diagnostic_msgs::msg::DiagnosticStatus::ERROR);
      observed_lost = true;
    }
  }
  EXPECT_TRUE(observed_odom_only);
  EXPECT_TRUE(observed_lost);
}

TEST_F(LocalizationNodeTest, MappingOdomOnlyStateIsOk) {
  rclcpp::NodeOptions options;
  options.parameter_overrides(
      {rclcpp::Parameter("mode", "mapping"),
       rclcpp::Parameter("map_output_dir", (temp_directory_ / "mapping_diagnostics").string()),
       rclcpp::Parameter("allow_mapping_without_gps", true),
       rclcpp::Parameter("gps_timeout_s", 0.5)});
  create_node(options);
  bring_up();
  run_for(4000ms, kInputWheel | kInputScans);
  ASSERT_EQ(diagnostics_values_.at("localization_node: state").at("state"), "ODOM_ONLY");
  EXPECT_EQ(diagnostics_levels_.at("localization_node: state"),
            diagnostic_msgs::msg::DiagnosticStatus::OK);
}

TEST_F(LocalizationNodeTest, OptimizationFailureExpiresFromErrorDiagnostics) {
  auto options = make_options(map_file_.string(), map_yaml_.string());
  options.append_parameter_override("last_error_hold_s", 2.0);
  create_node(options);
  bring_up();
  start_tracking_at_truth();
  node_->inject_optimization_failure_for_test();
  bool observed_warning = false;
  bool observed_clear = false;
  for (int index = 0; index < 50 && !observed_clear; ++index) {
    run_for(200ms, kInputWheel | kInputScans);
    const auto found = diagnostics_values_.find("localization_node: errors");
    if (found == diagnostics_values_.end() || found->second.at("error_events_total") != "1")
      continue;
    if (diagnostics_levels_.at("localization_node: errors") ==
        diagnostic_msgs::msg::DiagnosticStatus::WARN)
      observed_warning = true;
    if (observed_warning &&
        diagnostics_levels_.at("localization_node: errors") ==
            diagnostic_msgs::msg::DiagnosticStatus::OK &&
        found->second.at("last_error").empty())
      observed_clear = true;
  }
  EXPECT_TRUE(observed_warning);
  EXPECT_TRUE(observed_clear);
}

TEST_F(LocalizationNodeTest, ConfigureFailsWithoutMapFile) {
  create_node(make_options((temp_directory_ / "missing.pcd").string(),
                           map_yaml_.string()));
  EXPECT_EQ(node_->configure().id(),
            lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
}

TEST_F(LocalizationNodeTest, MappingModeSavesMapOnTrigger) {
  const std::filesystem::path output_directory = temp_directory_ / "mapping_output";
  rclcpp::NodeOptions options;
  options.parameter_overrides(
      {rclcpp::Parameter("mode", "mapping"),
       rclcpp::Parameter("map_output_dir", output_directory.string()),
       rclcpp::Parameter("init_travel_m", 0.5),
       rclcpp::Parameter("keyframe_time_s", 0.5),
       rclcpp::Parameter("keyframe_distance_m", 0.5),
       rclcpp::Parameter("status_rate_hz", 10.0)});
  create_node(options);
  bring_up();

  const auto start = std::chrono::steady_clock::now();
  auto next_wheel = start;
  auto next_scan = start;
  auto next_gps = start;
  while (std::chrono::steady_clock::now() - start < std::chrono::milliseconds(3000)) {
    const auto now = std::chrono::steady_clock::now();
    const double elapsed =
        std::chrono::duration<double>(now - start).count();
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() = Eigen::Vector3d(2.0 + 2.0 * elapsed, 0.0, 1.0);
    const rclcpp::Time stamp = node_->get_clock()->now();
    if (now >= next_wheel) {
      publish_wheel_at(stamp, pose);
      while (next_wheel <= now) next_wheel += 20ms;
    }
    if (now >= next_scan) {
      publish_scan_at(stamp, pose);
      while (next_scan <= now) next_scan += 100ms;
    }
    if (now >= next_gps) {
      publish_gps_at(stamp, pose);
      while (next_gps <= now) next_gps += 200ms;
    }
    executor_.spin_some(5ms);
  }
  ASSERT_GT(status_count_, 0);
  EXPECT_EQ(last_status_.mode, LocalizationStatus::MODE_MAPPING);
  ASSERT_GT(odom_count_, 0) << "last_error: " << last_status_.last_error;

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto future = save_map_client_->async_send_request(request);
  const auto end = std::chrono::steady_clock::now() + 5000ms;
  while (future.wait_for(0ms) != std::future_status::ready &&
         std::chrono::steady_clock::now() < end) {
    executor_.spin_some(10ms);
  }
  ASSERT_EQ(future.wait_for(0ms), std::future_status::ready);
  const std::shared_ptr<std_srvs::srv::Trigger::Response> response = future.get();
  ASSERT_TRUE(response->success) << response->message;

  ASSERT_TRUE(std::filesystem::exists(output_directory / "map.pcd"));
  ASSERT_TRUE(std::filesystem::exists(output_directory / "map.yaml"));
  ASSERT_TRUE(std::filesystem::exists(output_directory / "trajectory.csv"));
  const PointList map_points = read_pcd((output_directory / "map.pcd").string());
  const MapMetadata metadata =
      read_map_metadata((output_directory / "map.yaml").string());
  EXPECT_FALSE(map_points.empty());
  EXPECT_GT(metadata.keyframe_count, 0);
  EXPECT_TRUE(metadata.batch_optimized);
  EXPECT_NEAR(metadata.voxel_size_m, 0.2, 1e-9);
  std::ifstream csv((output_directory / "trajectory.csv").string());
  ASSERT_TRUE(csv);
  std::string line;
  ASSERT_TRUE(std::getline(csv, line));
  EXPECT_EQ(line, "stamp_sec,x,y,z,roll,pitch,yaw,position_std_m");
  int row_count = 0;
  while (std::getline(csv, line)) {
    if (!line.empty()) ++row_count;
  }
  EXPECT_EQ(row_count, metadata.keyframe_count);
  std::printf("[ K_NODE ] keyframes=%d map_points=%zu message=%s\n",
              metadata.keyframe_count, map_points.size(), response->message.c_str());
}

TEST_F(LocalizationNodeTest, MappingSavesMapOnShutdown) {
  const std::filesystem::path output_directory = temp_directory_ / "mapping_shutdown";
  rclcpp::NodeOptions options;
  options.parameter_overrides(
      {rclcpp::Parameter("mode", "mapping"),
       rclcpp::Parameter("map_output_dir", output_directory.string()),
       rclcpp::Parameter("init_travel_m", 0.5),
       rclcpp::Parameter("keyframe_time_s", 0.5),
       rclcpp::Parameter("keyframe_distance_m", 0.5),
       rclcpp::Parameter("status_rate_hz", 10.0)});
  create_node(options);
  bring_up();
  drive_mapping(
      [](double elapsed) {
        Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
        pose.translation() = Eigen::Vector3d(2.0 + 2.0 * elapsed, 0.0, 1.0);
        return pose;
      },
      2000ms);
  ASSERT_GT(odom_count_, 0) << last_status_.last_error;
  ASSERT_EQ(node_->shutdown().id(),
            lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED);
  ASSERT_TRUE(std::filesystem::exists(output_directory / "map.pcd"));
  ASSERT_TRUE(std::filesystem::exists(output_directory / "map.yaml"));
  ASSERT_TRUE(std::filesystem::exists(output_directory / "trajectory.csv"));
  const MapMetadata metadata =
      read_map_metadata((output_directory / "map.yaml").string());
  EXPECT_GT(metadata.keyframe_count, 0);
  EXPECT_TRUE(metadata.batch_optimized);
  std::printf("[ MAPPING_SHUTDOWN ] keyframes=%d saved=1\n", metadata.keyframe_count);
}

// Issue #43 D7 (2): after a successful save_map the recording stops, so the stop-time save is skipped
// even though stationary scans keep arriving (a keyframe is added every keyframe_time_s otherwise).
TEST_F(LocalizationNodeTest, MappingDoesNotSaveOnDeactivateAfterSaveMap) {
  const std::filesystem::path output_directory = temp_directory_ / "mapping_after_save_map";
  create_node(make_mapping_options(output_directory));
  bring_up();
  const Eigen::Isometry3d last_pose = drive_mapping_forward();
  ASSERT_GT(odom_count_, 0) << last_status_.last_error;

  const auto response = call_save_map();
  ASSERT_NE(response, nullptr);
  ASSERT_TRUE(response->success) << response->message;
  const auto saved = snapshot_map_files(output_directory);

  // Stationary scans for 3 s (six times keyframe_time_s).
  drive_mapping([&last_pose](double /*elapsed*/) { return last_pose; }, 3000ms);
  EXPECT_NE(last_status_.last_error.find("mapping recording stopped"), std::string::npos)
      << last_status_.last_error;
  ASSERT_EQ(node_->deactivate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  const auto after_deactivate = snapshot_map_files(output_directory);
  ASSERT_EQ(saved.size(), after_deactivate.size());
  for (std::size_t index = 0; index < saved.size(); ++index) {
    EXPECT_EQ(saved[index].first, after_deactivate[index].first) << "file " << index;
    EXPECT_EQ(saved[index].second, after_deactivate[index].second) << "file " << index;
  }
}

TEST_F(LocalizationNodeTest, MappingSavesOnceOnDeactivateWithoutSaveMap) {
  const std::filesystem::path output_directory = temp_directory_ / "mapping_deactivate";
  create_node(make_mapping_options(output_directory));
  bring_up();
  const Eigen::Isometry3d last_pose = drive_mapping_forward();
  ASSERT_GT(odom_count_, 0) << last_status_.last_error;
  drive_mapping([&last_pose](double /*elapsed*/) { return last_pose; }, 1000ms);
  ASSERT_FALSE(std::filesystem::exists(output_directory / "map.yaml"));

  ASSERT_EQ(node_->deactivate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  ASSERT_TRUE(std::filesystem::exists(output_directory / "map.pcd"));
  ASSERT_TRUE(std::filesystem::exists(output_directory / "trajectory.csv"));
  const MapMetadata metadata = read_map_metadata((output_directory / "map.yaml").string());
  EXPECT_GT(metadata.keyframe_count, 0);
  EXPECT_TRUE(metadata.batch_optimized);
}

TEST_F(LocalizationNodeTest, StaysInitializingWithoutGpsOrService) {
  create_node(make_options(map_file_.string(), map_yaml_.string()));
  bring_up();
  run_for(2000ms, kInputWheel | kInputScans);
  ASSERT_GT(status_count_, 0);
  EXPECT_EQ(last_status_.state, LocalizationStatus::INITIALIZING);
  EXPECT_EQ(odom_count_, 0);
}

TEST_F(LocalizationNodeTest, ReportsAMapWithoutGpsOriginInLastError) {
  create_node(make_options(map_file_.string(), no_origin_yaml_.string()));
  bring_up();
  ASSERT_TRUE(run_until([this]() { return status_count_ > 0; }, 2000ms, kInputWheel | kInputScans));
  EXPECT_EQ(last_status_.state, LocalizationStatus::INITIALIZING);
  EXPECT_EQ(odom_count_, 0);
  EXPECT_NE(last_status_.last_error.find("GPS origin"), std::string::npos)
      << last_status_.last_error;
  std::printf("[ NO_ORIGIN ] last_error=%s\n", last_status_.last_error.c_str());
}

TEST_F(LocalizationNodeTest, MissingGpsOriginNoteKeepsErrorsLevelOk) {
  create_node(make_options(map_file_.string(), no_origin_yaml_.string()));
  bring_up();
  ASSERT_TRUE(run_until(
      [this]() {
        return diagnostics_values_.count("localization_node: errors") > 0;
      },
      3000ms, kInputWheel | kInputScans));
  const auto &errors = diagnostics_values_.at("localization_node: errors");
  EXPECT_EQ(errors.at("last_error"), "map has no GPS origin: GPS disabled");
  EXPECT_EQ(errors.at("error_events_total"), "0");
  EXPECT_EQ(diagnostics_levels_.at("localization_node: errors"),
            diagnostic_msgs::msg::DiagnosticStatus::OK);
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
  EXPECT_TRUE(std::isfinite(last_status_.seconds_since_last_match));
  EXPECT_GE(last_status_.seconds_since_last_match, 0.0F);
  EXPECT_LT(last_status_.seconds_since_last_match, 2.0F);
  EXPECT_GT(last_status_.last_inlier_ratio, 0.5F);
  std::printf(
      "[ J ] time_to_tracking_with_error=%.3f s position_error=%.4f m yaw_error=%.4f rad "
      "inlier_ratio=%.3f seconds_since_last_match=%.3f\n",
      std::chrono::duration<double>(std::chrono::steady_clock::now() - test_start).count(),
      position_error.norm(), yaw_error, last_status_.last_inlier_ratio,
      last_status_.seconds_since_last_match);
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
  const Eigen::Vector3d requested_position =
      kTruthPosition + Eigen::Vector3d(0.3, 0.0, 0.0);
  const double requested_yaw = kTruthYaw + 0.05;
  request->pose.pose.pose.position.x = requested_position.x();
  request->pose.pose.pose.position.y = requested_position.y();
  request->pose.pose.pose.position.z = requested_position.z();
  request->pose.pose.pose.orientation = quaternion_from_yaw(requested_yaw);
  request->pose.pose.covariance[35] = 0.0;  // yaw is specified
  auto future = set_initial_pose_client_->async_send_request(request);
  ASSERT_TRUE(run_until(
      [&future]() { return future.wait_for(0ms) == std::future_status::ready; }, 3000ms,
      kInputWheel | kInputScans));
  const std::shared_ptr<SetInitialPose::Response> response = future.get();
  ASSERT_EQ(response->accepted, true) << response->message;

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

TEST_F(LocalizationNodeTest, ConfigureFailsWithInvalidGnssInputParameters) {
  create_node(make_gnss_options(3, 0.0));
  EXPECT_EQ(node_->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
  close_node();
  create_node(make_gnss_options(-1, -0.5));
  EXPECT_EQ(node_->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
}

TEST_F(LocalizationNodeTest, ConfigureRejectsInvalidMapMatcherParameters) {
  for (const auto& parameter : {rclcpp::Parameter("map_match_method", "invalid"),
                                rclcpp::Parameter("map_vgicp_voxel_m", 0.0),
                                rclcpp::Parameter("map_vgicp_coarse_voxel_m", -1.0),
                                rclcpp::Parameter("map_vgicp_coarse_correspondence_m", 0.0),
                                rclcpp::Parameter("map_max_correspondence_distance_m", 0.0)}) {
    auto options = make_gnss_options(-1, 0.0);
    options.append_parameter_override(parameter.get_name(), parameter.get_parameter_value());
    create_node(options);
    EXPECT_EQ(node_->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
    close_node();
  }
}

TEST_F(LocalizationNodeTest, WheelSlipDeclaredDefaultsAreDisabled) {
  create_node(make_options(map_file_.string(), map_yaml_.string()));
  EXPECT_DOUBLE_EQ(node_->get_parameter("lidar_odom_slip_gate_yaw_rad").as_double(), 0.0);
  EXPECT_DOUBLE_EQ(node_->get_parameter("wheel_yaw_slip_threshold_rad").as_double(), 0.0);
}

TEST_F(LocalizationNodeTest, ConfigureFailsWithInvalidWheelSlipParameters) {
  const std::vector<rclcpp::Parameter> invalid_parameters = {
      rclcpp::Parameter("lidar_odom_slip_wheel_rate_window_s", 0.0),
      rclcpp::Parameter("lidar_odom_slip_wheel_rate_window_s", std::numeric_limits<double>::infinity()),
      rclcpp::Parameter("lidar_odom_slip_wheel_rate_window_s", std::numeric_limits<double>::quiet_NaN()),
      rclcpp::Parameter("wheel_yaw_slip_threshold_rad", -1.0),
      rclcpp::Parameter("wheel_yaw_slip_threshold_rad", std::numeric_limits<double>::infinity()),
      rclcpp::Parameter("wheel_yaw_slip_threshold_rad", std::numeric_limits<double>::quiet_NaN()),
      rclcpp::Parameter("wheel_robust_kernel", "unknown"),
      rclcpp::Parameter("wheel_robust_k", 0.0),
      rclcpp::Parameter("wheel_robust_k", -1.0),
      rclcpp::Parameter("wheel_robust_k", std::numeric_limits<double>::infinity()),
      rclcpp::Parameter("wheel_robust_k", std::numeric_limits<double>::quiet_NaN()),
  };
  for (const auto& parameter : invalid_parameters) {
    SCOPED_TRACE(parameter.get_name());
    rclcpp::NodeOptions options = make_options(map_file_.string(), map_yaml_.string());

    options.append_parameter_override(parameter.get_name(), parameter.get_parameter_value());
    create_node(options);
    EXPECT_EQ(node_->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
    close_node();
  }
}

TEST_F(LocalizationNodeTest, DiagnosticsCountAcceptedGateAndTimeGapFixes) {
  create_node(make_gnss_options(-1, 0.0));
  bring_up();
  start_tracking_at_truth();

  // The Localizer drops a fix older than its 3 s wheel buffer without counting it, so the fix must
  // lie inside the buffer and still be more than gps_keyframe_max_gap_s (1.2 s) before the first
  // keyframe, which was added when tracking started a moment ago.
  const rclcpp::Time stale = node_->get_clock()->now() - rclcpp::Duration::from_seconds(2.5);
  gps_publisher_->publish(make_gnss_fix(stale, 1, 0.0));
  ASSERT_TRUE(wait_for_gnss_diagnostics(
      [this]() { return gnss_diagnostics_value("time_gap_rejected_total") >= 1; }))
      << "time_gap_rejected_total=" << gnss_diagnostics_value("time_gap_rejected_total");
  EXPECT_EQ(gnss_diagnostics_value("accepted_total"), 0);
  EXPECT_EQ(gnss_diagnostics_value("gate_rejected_total"), 0);

  gps_publisher_->publish(make_gnss_fix(node_->get_clock()->now(), 2, 0.0));
  ASSERT_TRUE(wait_for_gnss_diagnostics([this]() { return gnss_diagnostics_value("accepted_total") >= 1; }))
      << "accepted_total=" << gnss_diagnostics_value("accepted_total");
  EXPECT_EQ(gnss_diagnostics_value("gate_rejected_total"), 0);
  EXPECT_EQ(gnss_diagnostics_value("gps_status"), 2);
  EXPECT_TRUE(wait_for_gnss_diagnostics([this]() { return last_status_.gps_status == 2; }));

  const int accepted_before = gnss_diagnostics_value("accepted_total");
  gps_publisher_->publish(make_gnss_fix(node_->get_clock()->now(), 1, 30.0));  // 30 m off: gate
  ASSERT_TRUE(wait_for_gnss_diagnostics(
      [this]() { return gnss_diagnostics_value("gate_rejected_total") >= 1; }))
      << "gate_rejected_total=" << gnss_diagnostics_value("gate_rejected_total");
  EXPECT_EQ(gnss_diagnostics_value("accepted_total"), accepted_before);
  EXPECT_GE(gnss_diagnostics_value("conflict_window_rejects"), 1);
  EXPECT_EQ(gnss_diagnostics_value("time_gap_rejected_total"), 1);
  EXPECT_EQ(gnss_diagnostics_value("gps_status"), 1);
  std::printf("[ GNSS_DIAG ] accepted=%d gate=%d time_gap=%d window=%d\n",
              gnss_diagnostics_value("accepted_total"), gnss_diagnostics_value("gate_rejected_total"),
              gnss_diagnostics_value("time_gap_rejected_total"),
              gnss_diagnostics_value("conflict_window_rejects"));
}

TEST_F(LocalizationNodeTest, GpsMinIntervalLimitsFixesReachingTheLocalizer) {
  create_node(make_gnss_options(-1, 1.0));
  bring_up();
  start_tracking_at_truth();
  ASSERT_TRUE(wait_for_gnss_diagnostics([this]() { return gnss_diagnostics_value("accepted_total") >= 0; }));

  // 5 Hz of fixes the gate rejects: the rejections still happen at most once per second.
  const int gate_before = gnss_diagnostics_value("gate_rejected_total");
  const int published_before = published_fix_count_;
  run_with_fixes(3000ms, 200ms, 1, 30.0);
  ASSERT_TRUE(wait_for_gnss_diagnostics([this, gate_before]() {
    return gnss_diagnostics_value("gate_rejected_total") > gate_before;
  }));
  run_for(1200ms, kInputWheel | kInputScans);  // let the last diagnostics through
  const int published_fixes = published_fix_count_ - published_before;
  const int rejected = gnss_diagnostics_value("gate_rejected_total") - gate_before;
  EXPECT_GE(published_fixes, 12);
  EXPECT_GE(rejected, 2);
  EXPECT_LE(rejected, 4) << "published " << published_fixes << " fixes in 3 s";

  // The same for accepted fixes: one factor per second, not five.
  const int accepted_before = gnss_diagnostics_value("accepted_total");
  run_with_fixes(3000ms, 200ms, 1, 0.0);
  run_for(1200ms, kInputWheel | kInputScans);
  const int accepted = gnss_diagnostics_value("accepted_total") - accepted_before;
  EXPECT_GE(accepted, 2);
  EXPECT_LE(accepted, 4);
  std::printf("[ GNSS_THIN ] gate_rejected=%d accepted=%d of %d fixes\n", rejected, accepted,
              published_fixes);
}

TEST_F(LocalizationNodeTest, GpsMinStatusKeepsSinglePointOutOfEverythingButTheStatus) {
  create_node(make_gnss_options(1, 0.0));
  bring_up();

  // Status 0 fixes (the single-point solution) must not initialise the pose, even with wheel and scans.
  run_with_fixes(2500ms, 200ms, 0, 0.0);
  EXPECT_EQ(last_status_.state, LocalizationStatus::INITIALIZING);
  EXPECT_EQ(odom_count_, 0);
  EXPECT_EQ(last_status_.gps_status, 0) << "the dropped fix is still reported";
  EXPECT_FLOAT_EQ(last_status_.gps_std_m, 2.0F);

  start_tracking_at_truth();
  run_with_fixes(1500ms, 200ms, 0, 30.0);  // far and single point: neither a factor nor a rejection
  ASSERT_TRUE(wait_for_gnss_diagnostics([this]() { return gnss_diagnostics_value("gps_status") == 0; }));
  EXPECT_EQ(gnss_diagnostics_value("accepted_total"), 0);
  EXPECT_EQ(gnss_diagnostics_value("gate_rejected_total"), 0);
  EXPECT_EQ(gnss_diagnostics_value("time_gap_rejected_total"), 0);
  EXPECT_EQ(gnss_diagnostics_value("conflict_window_rejects"), 0);

  // A Float fix goes through, so the zeros above are not a dead GNSS path.
  gps_publisher_->publish(make_gnss_fix(node_->get_clock()->now(), 1, 0.0));
  EXPECT_TRUE(wait_for_gnss_diagnostics(
      [this]() { return gnss_diagnostics_value("accepted_total") >= 1; }));
  EXPECT_EQ(gnss_diagnostics_value("gps_status"), 1);
}

// Issue #43 section 12 (indoors, no GNSS), acceptance (a). gps_topic points at a topic nobody publishes, as
// docker/config/localization_indoor.yaml does: a fix on /gps/fix (sigma 1.0 m passes init_gps_max_std_m) would
// otherwise become the map origin. The map origin and +x are the pose of the vehicle when mapping starts.
TEST_F(LocalizationNodeTest, MappingWithoutGpsStartsAtTheStartPoseAndDrivesAlongPlusX) {
  constexpr double kStartX = 5.0;
  constexpr double kStartY = 3.0;
  constexpr double kStartYaw = 0.7;
  constexpr double kForwardSpeedMps = 0.8;
  constexpr double kForwardSeconds = 2.0;
  constexpr double kForwardDistanceM = kForwardSpeedMps * kForwardSeconds;
  constexpr double kMaxStartPositionErrorM = 0.05;
  constexpr double kMaxStartYawErrorRad = 0.02;
  constexpr double kMaxLateralRatio = 0.1;
  constexpr double kMaxHeightM = 0.1;

  // The corridor around y = 3 holds the start pose (the fixture corridor is around y = 0).
  world_ = std::make_unique<SyntheticWorld>(
      std::vector<Eigen::Vector2d>{Eigen::Vector2d(0.0, kStartY), Eigen::Vector2d(30.0, kStartY)},
      true);
  const std::filesystem::path output_directory = temp_directory_ / "mapping_without_gps";
  rclcpp::NodeOptions options = make_mapping_options(output_directory);
  std::vector<rclcpp::Parameter> overrides = options.parameter_overrides();
  overrides.emplace_back("gps_topic", "/lightnav/gnss_disabled");
  overrides.emplace_back("allow_mapping_without_gps", true);
  overrides.emplace_back("gps_timeout_s", 1.0);
  options.parameter_overrides(overrides);
  create_node(options);
  gps_variance_m2_ = 1.0;
  bring_up();

  Eigen::Isometry3d start_pose = Eigen::Isometry3d::Identity();
  start_pose.translation() = Eigen::Vector3d(kStartX, kStartY, kSensorHeightM);
  start_pose.linear() =
      Eigen::AngleAxisd(kStartYaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const Eigen::Vector3d heading(std::cos(kStartYaw), std::sin(kStartYaw), 0.0);
  const auto pose_after = [&start_pose, &heading](double distance_m) {
    Eigen::Isometry3d pose = start_pose;
    pose.translation() += distance_m * heading;
    return pose;
  };

  // Stand still until the node leaves INITIALIZING (gps_timeout_s), drive straight ahead, stop.
  drive_mapping([&start_pose](double /*elapsed*/) { return start_pose; }, 1500ms);
  drive_mapping(
      [&pose_after](double elapsed) { return pose_after(kForwardSpeedMps * elapsed); },
      std::chrono::milliseconds(static_cast<int>(kForwardSeconds * 1000.0)));
  drive_mapping([&pose_after](double /*elapsed*/) { return pose_after(kForwardDistanceM); },
                1000ms);
  ASSERT_GT(odom_count_, 0) << "last_error: " << last_status_.last_error;
  EXPECT_EQ(observed_states_.count(LocalizationStatus::LOST), 0U);

  const auto response = call_save_map();
  ASSERT_NE(response, nullptr);
  ASSERT_TRUE(response->success) << response->message;
  const MapMetadata metadata = read_map_metadata((output_directory / "map.yaml").string());
  EXPECT_FALSE(metadata.has_gps_origin);

  const std::vector<TrajectoryRow> rows = read_trajectory_rows(output_directory / "trajectory.csv");
  ASSERT_GE(rows.size(), 2U);
  const TrajectoryRow& first = rows.front();
  const TrajectoryRow& last = rows.back();
  EXPECT_LT(std::hypot(first.x, first.y), kMaxStartPositionErrorM);
  EXPECT_LT(std::abs(first.yaw), kMaxStartYawErrorRad);
  EXPECT_GT(last.x, 0.75 * kForwardDistanceM);
  EXPECT_LT(last.x, 1.25 * kForwardDistanceM);
  EXPECT_LT(std::abs(last.y), kMaxLateralRatio * last.x);
  for (const TrajectoryRow& row : rows) {
    EXPECT_LT(std::abs(row.z), kMaxHeightM);
  }
  std::printf("[ INDOOR_MAPPING ] keyframes=%zu first=(%.3f,%.3f,yaw %.3f) last=(%.3f,%.3f,z %.3f)\n",
              rows.size(), first.x, first.y, first.yaw, last.x, last.y, last.z);
}

// Live preview for Foxglove (Issue #43 replay tooling): the map cloud and the path are latched topics.
TEST_F(LocalizationNodeTest, MappingPublishesTheLivePreviewMapCloudAndPath) {
  constexpr double kMinDrivenDistanceM = 0.5;
  world_ = std::make_unique<SyntheticWorld>(
      std::vector<Eigen::Vector2d>{Eigen::Vector2d(0.0, 3.0), Eigen::Vector2d(30.0, 3.0)}, true);
  create_node(make_indoor_mapping_options(temp_directory_ / "preview", true));
  subscribe_visualization();
  gps_variance_m2_ = 1.0;
  bring_up();

  drive_mapping_without_gps_along_x();
  spin_for(500ms);

  ASSERT_TRUE(last_map_cloud_.has_value());
  EXPECT_EQ(last_map_cloud_->header.frame_id, "map");
  EXPECT_GT(last_map_cloud_->width, 100U);
  ASSERT_TRUE(last_path_.has_value());
  EXPECT_EQ(last_path_->header.frame_id, "map");
  ASSERT_GE(last_path_->poses.size(), 2U);
  const double driven_x =
      last_path_->poses.back().pose.position.x - last_path_->poses.front().pose.position.x;
  EXPECT_GT(driven_x, kMinDrivenDistanceM);
}

TEST_F(LocalizationNodeTest, VisualizationEnabledFalsePublishesNeitherMapCloudNorPath) {
  create_node(make_indoor_mapping_options(temp_directory_ / "preview_off", false));
  bring_up();

  drive_mapping_without_gps_along_x();

  EXPECT_EQ(helper_->count_publishers("/lightnav/localization/map_cloud"), 0U);
  EXPECT_EQ(helper_->count_publishers("/lightnav/localization/path"), 0U);
}

TEST_F(LocalizationNodeTest, LocalizationModePublishesThePriorMapCloudOnActivate) {
  create_node(make_options(map_file_.string(), map_yaml_.string()));
  subscribe_visualization();

  bring_up();

  ASSERT_TRUE(last_map_cloud_.has_value());
  EXPECT_EQ(last_map_cloud_->header.frame_id, "map");
  EXPECT_GT(last_map_cloud_->width, 100U);
}

// Acceptance (b): a map without a GPS origin plus set_initial_pose (with yaw) keeps TRACKING with only the
// wheel and the point cloud, for longer than gps_timeout_s (3 s) + tracking_window_s (5 s).
TEST_F(LocalizationNodeTest, TracksWithoutGpsOnAMapWithoutGpsOrigin) {
  constexpr auto kHold = 10000ms;
  create_node(make_options(map_file_.string(), no_origin_yaml_.string()));
  bring_up();
  run_for(600ms, kInputWheel | kInputScans);

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
  ASSERT_TRUE(response->accepted) << response->message;
  ASSERT_TRUE(run_until(
      [this]() { return last_status_.state == LocalizationStatus::TRACKING && odom_count_ > 0; },
      10000ms, kInputWheel | kInputScans))
      << "last_error: " << last_status_.last_error;

  observed_states_.clear();
  run_for(kHold, kInputWheel | kInputScans);
  EXPECT_EQ(observed_states_, std::set<uint8_t>{LocalizationStatus::TRACKING})
      << "last_error: " << last_status_.last_error;
  const Eigen::Vector2d position_error =
      Eigen::Vector2d(last_odom_.pose.pose.position.x, last_odom_.pose.pose.position.y) -
      kTruthPosition.head<2>();
  EXPECT_LT(position_error.norm(), 0.2);
}
