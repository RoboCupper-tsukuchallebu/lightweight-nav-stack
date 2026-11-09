#include "lightnav_localization/localization_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <ctime>
#include <exception>
#include <fstream>
#include <sys/resource.h>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <tf2/time.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include "lightnav_localization/deskew.hpp"
#include "lightnav_localization/downsample.hpp"
#include "lightnav_localization/mapping_initialization.hpp"
#include "lightnav_localization/point_cloud_extraction.hpp"

namespace lightnav_localization {
namespace {

constexpr double kNeverSec = -1e9;
constexpr double kValidTimestampFloorSec = -1e8;
constexpr double kMicrosecondsPerSecond = 1e6;
constexpr double kWheelBufferSec = 3.0;
constexpr double kDefaultWatchdogPeriodSec = 0.02;
constexpr double kDiagnosticsPeriodSec = 1.0;
constexpr double kScanMaximumWarnMs = 100.0;
constexpr double kScanMeanErrorMs = 150.0;
constexpr double kKilobytesPerMegabyte = 1024.0;
constexpr int kLowestNavSatStatus = -1;   // STATUS_NO_FIX
constexpr int kHighestNavSatStatus = 2;   // STATUS_GBAS_FIX
constexpr std::size_t kMaxPendingScans = 20;

using DiagnosticStatus = diagnostic_msgs::msg::DiagnosticStatus;

diagnostic_msgs::msg::KeyValue make_diagnostic_entry(const std::string& key,
                                                     const std::string& value) {
  diagnostic_msgs::msg::KeyValue entry;
  entry.key = key;
  entry.value = value;
  return entry;
}

void add_diagnostic_value(DiagnosticStatus& status, const std::string& key,
                          const std::string& value) {
  status.values.push_back(make_diagnostic_entry(key, value));
}

std::string localization_state_name(LocalizationState state) {
  switch (state) {
    case LocalizationState::kInitializing:
      return "INITIALIZING";
    case LocalizationState::kTracking:
      return "TRACKING";
    case LocalizationState::kGpsOnly:
      return "GPS_ONLY";
    case LocalizationState::kOdomOnly:
      return "ODOM_ONLY";
    case LocalizationState::kLost:
      return "LOST";
    case LocalizationState::kInputStale:
      return "INPUT_STALE";
  }
  return "";
}

std::optional<double> read_memory_rss_mb() {
#ifdef __linux__
  std::ifstream proc_status("/proc/self/status");
  std::string line;
  while (std::getline(proc_status, line)) {
    if (line.rfind("VmRSS:", 0) != 0) continue;
    try {
      return std::stod(line.substr(6)) / kKilobytesPerMegabyte;
    } catch (const std::exception&) {
      return std::nullopt;
    }
  }
#endif
  return std::nullopt;
}

std::string utc_now_iso8601() {
  const std::time_t now = std::chrono::system_clock::to_time_t(
      std::chrono::system_clock::now());
  std::tm broken_down{};
  gmtime_r(&now, &broken_down);
  char buffer[32] = {};
  std::strftime(buffer, sizeof(buffer), "%Y-%m-%dT%H:%M:%SZ", &broken_down);
  return std::string(buffer);
}

Eigen::Isometry3d invert_isometry(const Eigen::Isometry3d& transform) {
  Eigen::Isometry3d inverse = Eigen::Isometry3d::Identity();
  inverse.linear() = transform.linear().transpose();
  inverse.translation() = -(inverse.linear() * transform.translation());
  return inverse;
}

bool is_finite_isometry(const Eigen::Isometry3d& transform) {
  return transform.matrix().allFinite();
}

bool pose_message_to_isometry(const geometry_msgs::msg::Pose& pose,
                              Eigen::Isometry3d& transform) {
  const auto& position = pose.position;
  const auto& orientation = pose.orientation;
  if (!std::isfinite(position.x) || !std::isfinite(position.y) ||
      !std::isfinite(position.z) || !std::isfinite(orientation.x) ||
      !std::isfinite(orientation.y) || !std::isfinite(orientation.z) ||
      !std::isfinite(orientation.w)) {
    return false;
  }
  const double norm = std::sqrt(orientation.x * orientation.x + orientation.y * orientation.y +
                                orientation.z * orientation.z + orientation.w * orientation.w);
  if (!(norm > 1e-9)) return false;
  Eigen::Quaterniond quaternion(orientation.w / norm, orientation.x / norm, orientation.y / norm,
                                orientation.z / norm);
  transform = Eigen::Isometry3d::Identity();
  transform.linear() = quaternion.toRotationMatrix();
  transform.translation() = Eigen::Vector3d(position.x, position.y, position.z);
  return is_finite_isometry(transform);
}

double yaw_of(const Eigen::Isometry3d& transform) {
  return std::atan2(transform.rotation()(1, 0), transform.rotation()(0, 0));
}

uint8_t state_to_message(LocalizationState state) {
  using Status = lightnav_msgs::msg::LocalizationStatus;
  switch (state) {
    case LocalizationState::kInitializing: return Status::INITIALIZING;
    case LocalizationState::kTracking: return Status::TRACKING;
    case LocalizationState::kGpsOnly: return Status::GPS_ONLY;
    case LocalizationState::kOdomOnly: return Status::ODOM_ONLY;
    case LocalizationState::kLost: return Status::LOST;
    case LocalizationState::kInputStale: return Status::INPUT_STALE;
  }
  return Status::INITIALIZING;
}

int field_datatype(const sensor_msgs::msg::PointCloud2& cloud, const std::string& name) {
  for (const auto& field : cloud.fields) {
    if (field.name == name) return static_cast<int>(field.datatype);
  }
  return -1;
}

int field_offset(const sensor_msgs::msg::PointCloud2& cloud, const std::string& name) {
  for (const auto& field : cloud.fields) {
    if (field.name == name) return static_cast<int>(field.offset);
  }
  return -1;
}

PointCloudFieldLayout make_layout(const sensor_msgs::msg::PointCloud2& cloud) {
  PointCloudFieldLayout layout;
  layout.width = cloud.width;
  layout.height = cloud.height;
  layout.point_step = cloud.point_step;
  layout.row_step = cloud.row_step;
  layout.is_bigendian = cloud.is_bigendian != 0;
  layout.x_offset = field_offset(cloud, "x");
  layout.y_offset = field_offset(cloud, "y");
  layout.z_offset = field_offset(cloud, "z");
  layout.x_datatype = field_datatype(cloud, "x");
  layout.y_datatype = field_datatype(cloud, "y");
  layout.z_datatype = field_datatype(cloud, "z");
  layout.time_offset = field_offset(cloud, "time");
  layout.time_datatype = field_datatype(cloud, "time");
  return layout;
}

sensor_msgs::msg::PointCloud2 make_xyz_cloud(const PointList& points, const std::string& frame_id,
                                             const rclcpp::Time& stamp) {
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.stamp = stamp;
  cloud.header.frame_id = frame_id;
  cloud.height = 1;
  cloud.width = static_cast<std::uint32_t>(points.size());
  cloud.is_dense = true;
  cloud.is_bigendian = false;
  cloud.point_step = 3 * static_cast<std::uint32_t>(sizeof(float));
  cloud.row_step = cloud.point_step * cloud.width;
  for (const char* name : {"x", "y", "z"}) {
    sensor_msgs::msg::PointField field;
    field.name = name;
    field.offset = static_cast<std::uint32_t>(cloud.fields.size()) *
                   static_cast<std::uint32_t>(sizeof(float));
    field.datatype = sensor_msgs::msg::PointField::FLOAT32;
    field.count = 1;
    cloud.fields.push_back(field);
  }
  cloud.data.resize(static_cast<std::size_t>(cloud.row_step));
  std::memcpy(cloud.data.data(), points.data(), cloud.data.size());
  return cloud;
}

}  // namespace

LocalizationNode::LocalizationNode(const rclcpp::NodeOptions& options)
    : rclcpp_lifecycle::LifecycleNode("localization_node", options) {
  declare_parameters();
}

LocalizationNode::~LocalizationNode() = default;

void LocalizationNode::declare_parameters() {
  declare_parameter<std::string>("mode", "localization");
  declare_parameter<std::string>("map_file", "");
  declare_parameter<std::string>("map_yaml", "");
  declare_parameter<std::string>("map_frame", "map");
  declare_parameter<std::string>("odom_frame", "odom");
  declare_parameter<std::string>("base_frame", "base_footprint");
  declare_parameter<std::string>("gps_frame", "gnss");
  declare_parameter<std::string>("wheel_odom_topic", "/wheel/odom");
  declare_parameter<std::string>("points_topic", "/velodyne_points");
  declare_parameter<std::string>("gps_topic", "/gps/fix");
  declare_parameter<bool>("publish_tf", true);
  declare_parameter<double>("status_rate_hz", 2.0);
  declare_parameter<double>("diagnostics_cpu_warn_percent", 200.0);
  declare_parameter<double>("diagnostics_memory_warn_mb", 2048.0);
  declare_parameter<double>("last_error_hold_s", 10.0);
  declare_parameter<int>("num_threads", 4);
  // Live preview topics /lightnav/localization/map_cloud and /path (Foxglove / RViz). Set false to publish nothing.
  declare_parameter<bool>("visualization_enabled", true);
  declare_parameter<double>("visualization_rate_hz", 0.5);
  declare_parameter<double>("visualization_voxel_m", 0.2);
  declare_parameter<int>("visualization_max_points", 1000000);
  declare_parameter<double>("path_min_step_m", 0.1);
  declare_parameter<int>("path_max_poses", 20000);
  declare_parameter<double>("init_gps_max_std_m", 3.0);
  // GNSS input contract (Issue #43 section 4.3): drop fixes below this NavSatStatus (-1 keeps all) and
  // thin the rest to one per gps_min_interval_s (0 keeps all) before they reach the Localizer.
  declare_parameter<int>("gps_min_status", kLowestNavSatStatus);
  declare_parameter<double>("gps_min_interval_s", 0.0);
  declare_parameter<int>("init_scan_count", 3);
  declare_parameter<double>("init_yaw_step_deg", 15.0);
  declare_parameter<double>("init_coarse_corr_m", 2.0);
  declare_parameter<double>("init_fine_corr_m", 0.5);
  declare_parameter<double>("init_inlier_ratio_min", 0.6);
  declare_parameter<std::string>("map_output_dir", "");
  declare_parameter<double>("map_voxel_size_m", 0.2);
  declare_parameter<double>("keyframe_cloud_voxel_m", 0.25);
  declare_parameter<int>("batch_max_iterations", 100);
  declare_parameter<double>("init_travel_m", 3.0);
  declare_parameter<bool>("allow_mapping_without_gps", false);

  declare_parameter<double>("wheel_timeout_s", 0.2);
  declare_parameter<double>("scan_timeout_s", 0.5);
  declare_parameter<double>("gps_timeout_s", 3.0);
  declare_parameter<double>("voxel_size_m", 0.5);
  declare_parameter<int>("submap_keyframes", 10);
  declare_parameter<double>("submap_voxel_size_m", 0.25);
  declare_parameter<int>("lidar_odom_max_iterations", 20);
  declare_parameter<double>("lidar_odom_inlier_ratio_min", 0.6);
  declare_parameter<double>("lidar_odom_gate_translation_m", 0.3);
  declare_parameter<double>("lidar_odom_gate_translation_ratio", 0.2);
  declare_parameter<double>("lidar_odom_gate_yaw_rad", 0.1);
  declare_parameter<double>("lidar_odom_covariance_scale", 1.0);
  declare_parameter<double>("lidar_odom_sigma_floor_m", 0.02);
  declare_parameter<double>("lidar_odom_sigma_floor_rad", 0.005);
  declare_parameter<double>("wheel_sigma_translation_per_m", 0.02);
  declare_parameter<double>("wheel_sigma_yaw_per_rad", 0.1);
  declare_parameter<double>("wheel_sigma_yaw_per_m", 0.01);
  declare_parameter<double>("planar_sigma_z_per_m", 0.05);
  declare_parameter<double>("planar_sigma_tilt_rad", 0.05);
  declare_parameter<double>("map_submap_rebuild_distance_m", 50.0);
  declare_parameter<double>("keyframe_distance_m", 1.0);
  declare_parameter<double>("keyframe_time_s", 1.0);
  declare_parameter<double>("match_rate_hz", 2.0);
  declare_parameter<std::string>("map_match_method", "gicp");
  declare_parameter<double>("map_vgicp_voxel_m", 0.5);
  declare_parameter<bool>("map_vgicp_two_stage", false);
  declare_parameter<double>("map_vgicp_coarse_voxel_m", 2.0);
  declare_parameter<double>("map_vgicp_coarse_correspondence_m", 2.0);
  declare_parameter<double>("map_max_correspondence_distance_m", 1.0);
  declare_parameter<int>("map_max_iterations", 30);
  declare_parameter<double>("map_inlier_ratio_min", 0.5);
  declare_parameter<double>("map_gate_translation_m", 1.0);
  declare_parameter<double>("map_gate_rotation_rad", 0.2);
  declare_parameter<double>("map_covariance_scale", 1.0);
  declare_parameter<double>("map_sigma_floor_m", 0.05);
  declare_parameter<double>("map_sigma_floor_rad", 0.01);
  declare_parameter<double>("degeneracy_ratio", 0.05);
  declare_parameter<double>("huber_k", 1.0);
  declare_parameter<std::vector<double>>("gps_std_m_by_status",
                                         std::vector<double>{2.0, 1.0, 0.3});
  declare_parameter<double>("gps_max_hdop", 3.0);
  declare_parameter<double>("gps_gate_mahalanobis", 3.0);
  declare_parameter<double>("gps_trusted_std_m", 0.3);
  declare_parameter<double>("gps_absolute_gate_m", 2.0);
  declare_parameter<int>("gps_conflict_window", 10);
  declare_parameter<double>("gps_conflict_ratio", 0.5);
  declare_parameter<double>("gps_altitude_sigma_scale", 3.0);
  declare_parameter<double>("gps_altitude_sigma_floor_m", 1.0);
  declare_parameter<double>("gps_keyframe_max_gap_s", 1.2);
  declare_parameter<double>("window_s", 30.0);
  declare_parameter<double>("correction_slew_mps", 0.2);
  declare_parameter<double>("correction_slew_rps", 0.1);
  declare_parameter<double>("large_correction_threshold_m", 1.0);
  declare_parameter<double>("tracking_window_s", 5.0);
  declare_parameter<double>("lost_position_std_m", 2.0);
  declare_parameter<double>("odom_only_max_distance_m", 50.0);
  declare_parameter<double>("odom_only_max_s", 60.0);
  declare_parameter<double>("drift_rate", 0.02);
}

bool LocalizationNode::read_parameters(std::string& error) {
  mode_ = get_parameter("mode").as_string();
  map_file_ = get_parameter("map_file").as_string();
  map_yaml_ = get_parameter("map_yaml").as_string();
  map_frame_ = get_parameter("map_frame").as_string();
  odom_frame_ = get_parameter("odom_frame").as_string();
  base_frame_ = get_parameter("base_frame").as_string();
  gps_frame_ = get_parameter("gps_frame").as_string();
  wheel_odom_topic_ = get_parameter("wheel_odom_topic").as_string();
  points_topic_ = get_parameter("points_topic").as_string();
  gps_topic_ = get_parameter("gps_topic").as_string();
  publish_tf_ = get_parameter("publish_tf").as_bool();
  status_rate_hz_ = get_parameter("status_rate_hz").as_double();
  diagnostics_cpu_warn_percent_ = get_parameter("diagnostics_cpu_warn_percent").as_double();
  diagnostics_memory_warn_mb_ = get_parameter("diagnostics_memory_warn_mb").as_double();
  const double last_error_hold_s = get_parameter("last_error_hold_s").as_double();
  num_threads_ = static_cast<int>(get_parameter("num_threads").as_int());
  visualization_enabled_ = get_parameter("visualization_enabled").as_bool();
  visualization_rate_hz_ = get_parameter("visualization_rate_hz").as_double();
  visualization_voxel_m_ = get_parameter("visualization_voxel_m").as_double();
  visualization_max_points_ = static_cast<int>(get_parameter("visualization_max_points").as_int());
  path_min_step_m_ = get_parameter("path_min_step_m").as_double();
  path_max_poses_ = static_cast<int>(get_parameter("path_max_poses").as_int());
  init_gps_max_std_m_ = get_parameter("init_gps_max_std_m").as_double();
  gps_min_status_ = static_cast<int>(get_parameter("gps_min_status").as_int());
  gps_min_interval_s_ = get_parameter("gps_min_interval_s").as_double();
  alignment_parameters_.scan_count = static_cast<int>(get_parameter("init_scan_count").as_int());
  alignment_parameters_.yaw_step_deg = get_parameter("init_yaw_step_deg").as_double();
  alignment_parameters_.coarse_correspondence_m = get_parameter("init_coarse_corr_m").as_double();
  alignment_parameters_.fine_correspondence_m = get_parameter("init_fine_corr_m").as_double();
  alignment_parameters_.inlier_ratio_min = get_parameter("init_inlier_ratio_min").as_double();
  map_output_dir_ = get_parameter("map_output_dir").as_string();
  map_voxel_size_m_ = get_parameter("map_voxel_size_m").as_double();
  keyframe_cloud_voxel_m_ = get_parameter("keyframe_cloud_voxel_m").as_double();
  batch_max_iterations_ = static_cast<int>(get_parameter("batch_max_iterations").as_int());
  init_travel_m_ = get_parameter("init_travel_m").as_double();
  allow_mapping_without_gps_ = get_parameter("allow_mapping_without_gps").as_bool();

  if (map_frame_.empty() || odom_frame_.empty() || base_frame_.empty() || gps_frame_.empty() ||
      wheel_odom_topic_.empty() || points_topic_.empty() || gps_topic_.empty()) {
    error = "frame names and input topics must not be empty";
    return false;
  }
  if (!(status_rate_hz_ > 0.0) || !std::isfinite(status_rate_hz_)) {
    error = "status_rate_hz must be positive and finite";
    return false;
  }
  if (!std::isfinite(diagnostics_cpu_warn_percent_) || diagnostics_cpu_warn_percent_ < 0.0 ||
      !std::isfinite(diagnostics_memory_warn_mb_) || diagnostics_memory_warn_mb_ < 0.0 ||
      !std::isfinite(last_error_hold_s) || last_error_hold_s < 0.0) {
    error = "diagnostics thresholds and last_error_hold_s must be finite and nonnegative";
    return false;
  }
  if (visualization_enabled_ &&
      (!(visualization_rate_hz_ > 0.0) || !std::isfinite(visualization_rate_hz_) ||
       !(visualization_voxel_m_ > 0.0) || !std::isfinite(visualization_voxel_m_) ||
       visualization_max_points_ < 1 || !(path_min_step_m_ >= 0.0) ||
       !std::isfinite(path_min_step_m_) || path_max_poses_ < 1)) {
    error = "visualization_rate_hz, visualization_voxel_m, visualization_max_points and "
            "path_max_poses must be positive, and path_min_step_m must not be negative";
    return false;
  }
  if (alignment_parameters_.scan_count < 1 || !(alignment_parameters_.yaw_step_deg > 0.0) ||
      !(alignment_parameters_.coarse_correspondence_m > 0.0) ||
      !(alignment_parameters_.fine_correspondence_m > 0.0) ||
      !(alignment_parameters_.inlier_ratio_min > 0.0) || !(init_gps_max_std_m_ > 0.0)) {
    error = "initial alignment parameters must be positive";
    return false;
  }
  if (gps_min_status_ < kLowestNavSatStatus || gps_min_status_ > kHighestNavSatStatus) {
    error = "gps_min_status must be between -1 (NO_FIX) and 2 (GBAS_FIX)";
    return false;
  }
  if (!(gps_min_interval_s_ >= 0.0) || !std::isfinite(gps_min_interval_s_)) {
    error = "gps_min_interval_s must be non-negative and finite";
    return false;
  }
  const double match_rate_hz = get_parameter("match_rate_hz").as_double();
  if (!(match_rate_hz > 0.0) || !std::isfinite(match_rate_hz)) {
    error = "match_rate_hz must be positive and finite";
    return false;
  }
  const std::vector<double> gps_std_by_status =
      get_parameter("gps_std_m_by_status").as_double_array();
  if (gps_std_by_status.size() != 3) {
    error = "gps_std_m_by_status must have three values (FIX, SBAS_FIX, GBAS_FIX)";
    return false;
  }
  for (const double std_m : gps_std_by_status) {
    if (!(std_m > 0.0) || !std::isfinite(std_m)) {
      error = "gps_std_m_by_status values must be positive and finite";
      return false;
    }
  }

  localizer_parameters_ = LocalizerParameters{};
  localizer_parameters_.last_error_hold_s = last_error_hold_s;
  localizer_parameters_.graph.window_s = get_parameter("window_s").as_double();
  localizer_parameters_.graph.huber_k = get_parameter("huber_k").as_double();
  localizer_parameters_.graph.wheel_sigma_translation_per_m =
      get_parameter("wheel_sigma_translation_per_m").as_double();
  localizer_parameters_.graph.wheel_sigma_yaw_per_rad =
      get_parameter("wheel_sigma_yaw_per_rad").as_double();
  localizer_parameters_.graph.wheel_sigma_yaw_per_m =
      get_parameter("wheel_sigma_yaw_per_m").as_double();
  localizer_parameters_.graph.planar_sigma_z_per_m =
      get_parameter("planar_sigma_z_per_m").as_double();
  localizer_parameters_.graph.planar_sigma_tilt_rad =
      get_parameter("planar_sigma_tilt_rad").as_double();

  localizer_parameters_.lidar_odometry.submap_keyframes =
      static_cast<int>(get_parameter("submap_keyframes").as_int());
  localizer_parameters_.lidar_odometry.submap_voxel_size_m =
      get_parameter("submap_voxel_size_m").as_double();
  localizer_parameters_.lidar_odometry.max_iterations =
      static_cast<int>(get_parameter("lidar_odom_max_iterations").as_int());
  localizer_parameters_.lidar_odometry.inlier_ratio_min =
      get_parameter("lidar_odom_inlier_ratio_min").as_double();
  localizer_parameters_.lidar_odometry.gate_translation_m =
      get_parameter("lidar_odom_gate_translation_m").as_double();
  localizer_parameters_.lidar_odometry.gate_translation_ratio =
      get_parameter("lidar_odom_gate_translation_ratio").as_double();
  localizer_parameters_.lidar_odometry.gate_yaw_rad =
      get_parameter("lidar_odom_gate_yaw_rad").as_double();
  localizer_parameters_.lidar_odometry_information.covariance_scale =
      get_parameter("lidar_odom_covariance_scale").as_double();
  localizer_parameters_.lidar_odometry_information.sigma_floor_m =
      get_parameter("lidar_odom_sigma_floor_m").as_double();
  localizer_parameters_.lidar_odometry_information.sigma_floor_rad =
      get_parameter("lidar_odom_sigma_floor_rad").as_double();
  localizer_parameters_.lidar_odometry_information.degeneracy_ratio =
      get_parameter("degeneracy_ratio").as_double();

  localizer_parameters_.corrector.slew_mps = get_parameter("correction_slew_mps").as_double();
  localizer_parameters_.corrector.slew_rps = get_parameter("correction_slew_rps").as_double();
  localizer_parameters_.corrector.large_correction_threshold_m =
      get_parameter("large_correction_threshold_m").as_double();

  localizer_parameters_.state.wheel_timeout_s = get_parameter("wheel_timeout_s").as_double();
  localizer_parameters_.state.scan_timeout_s = get_parameter("scan_timeout_s").as_double();
  localizer_parameters_.state.gps_timeout_s = get_parameter("gps_timeout_s").as_double();
  localizer_parameters_.state.tracking_window_s = get_parameter("tracking_window_s").as_double();
  localizer_parameters_.state.lost_position_std_m =
      get_parameter("lost_position_std_m").as_double();
  localizer_parameters_.state.odom_only_max_distance_m =
      get_parameter("odom_only_max_distance_m").as_double();
  localizer_parameters_.state.odom_only_max_s = get_parameter("odom_only_max_s").as_double();

  localizer_parameters_.gps.std_fix_m = gps_std_by_status[0];
  localizer_parameters_.gps.std_sbas_m = gps_std_by_status[1];
  localizer_parameters_.gps.std_gbas_m = gps_std_by_status[2];
  localizer_parameters_.gps.max_hdop = get_parameter("gps_max_hdop").as_double();
  localizer_parameters_.gps.gate_mahalanobis =
      get_parameter("gps_gate_mahalanobis").as_double();
  localizer_parameters_.gps.gps_trusted_std_m = get_parameter("gps_trusted_std_m").as_double();
  localizer_parameters_.gps.gps_absolute_gate_m =
      get_parameter("gps_absolute_gate_m").as_double();
  localizer_parameters_.gps.altitude_sigma_scale =
      get_parameter("gps_altitude_sigma_scale").as_double();
  localizer_parameters_.gps.altitude_sigma_floor_m =
      get_parameter("gps_altitude_sigma_floor_m").as_double();
  localizer_parameters_.gps_conflict_window =
      static_cast<int>(get_parameter("gps_conflict_window").as_int());
  localizer_parameters_.gps_conflict_ratio = get_parameter("gps_conflict_ratio").as_double();
  localizer_parameters_.gps_keyframe_max_gap_s =
      get_parameter("gps_keyframe_max_gap_s").as_double();

  localizer_parameters_.map_matcher.method = get_parameter("map_match_method").as_string();
  localizer_parameters_.map_matcher.vgicp_voxel_m = get_parameter("map_vgicp_voxel_m").as_double();
  localizer_parameters_.map_matcher.vgicp_two_stage = get_parameter("map_vgicp_two_stage").as_bool();
  localizer_parameters_.map_matcher.vgicp_coarse_voxel_m = get_parameter("map_vgicp_coarse_voxel_m").as_double();
  localizer_parameters_.map_matcher.vgicp_coarse_correspondence_m = get_parameter("map_vgicp_coarse_correspondence_m").as_double();
  const auto& matcher = localizer_parameters_.map_matcher;
  if ((matcher.method != "gicp" && matcher.method != "vgicp") ||
      !std::isfinite(matcher.vgicp_voxel_m) || matcher.vgicp_voxel_m <= 0.0 ||
      !std::isfinite(matcher.vgicp_coarse_voxel_m) || matcher.vgicp_coarse_voxel_m <= 0.0 ||
      !std::isfinite(matcher.vgicp_coarse_correspondence_m) || matcher.vgicp_coarse_correspondence_m <= 0.0) {
    error = "invalid map matching method or VGICP parameters";
    return false;
  }
  localizer_parameters_.map_matcher.max_correspondence_distance_m =
      get_parameter("map_max_correspondence_distance_m").as_double();
  localizer_parameters_.map_matcher.max_iterations =
      static_cast<int>(get_parameter("map_max_iterations").as_int());
  if (!std::isfinite(matcher.max_correspondence_distance_m) || matcher.max_correspondence_distance_m <= 0.0) {
    error = "map_max_correspondence_distance_m must be positive and finite";
    return false;
  }
  localizer_parameters_.map_matcher.inlier_ratio_min =
      get_parameter("map_inlier_ratio_min").as_double();
  localizer_parameters_.map_matcher.gate_translation_m =
      get_parameter("map_gate_translation_m").as_double();
  localizer_parameters_.map_matcher.gate_rotation_rad =
      get_parameter("map_gate_rotation_rad").as_double();
  localizer_parameters_.map_information.covariance_scale =
      get_parameter("map_covariance_scale").as_double();
  localizer_parameters_.map_information.sigma_floor_m =
      get_parameter("map_sigma_floor_m").as_double();
  localizer_parameters_.map_information.sigma_floor_rad =
      get_parameter("map_sigma_floor_rad").as_double();
  localizer_parameters_.map_information.degeneracy_ratio =
      get_parameter("degeneracy_ratio").as_double();

  localizer_parameters_.voxel_size_m = get_parameter("voxel_size_m").as_double();
  localizer_parameters_.keyframe_distance_m = get_parameter("keyframe_distance_m").as_double();
  localizer_parameters_.keyframe_time_s = get_parameter("keyframe_time_s").as_double();
  localizer_parameters_.match_period_s = 1.0 / match_rate_hz;
  localizer_parameters_.map_submap_rebuild_distance_m =
      get_parameter("map_submap_rebuild_distance_m").as_double();
  localizer_parameters_.drift_rate = get_parameter("drift_rate").as_double();

  const bool positive_parameters =
      localizer_parameters_.graph.window_s > 0.0 &&
      localizer_parameters_.voxel_size_m > 0.0 &&
      localizer_parameters_.keyframe_distance_m > 0.0 &&
      localizer_parameters_.keyframe_time_s > 0.0 &&
      localizer_parameters_.lidar_odometry.submap_keyframes > 0 &&
      localizer_parameters_.lidar_odometry.submap_voxel_size_m > 0.0 &&
      localizer_parameters_.lidar_odometry.max_iterations > 0 &&
      localizer_parameters_.map_matcher.max_iterations > 0 &&
      localizer_parameters_.gps_conflict_window > 0 &&
      localizer_parameters_.state.wheel_timeout_s >= 0.0 &&
      localizer_parameters_.state.scan_timeout_s >= 0.0 &&
      localizer_parameters_.state.gps_timeout_s >= 0.0 &&
      std::isfinite(localizer_parameters_.corrector.slew_mps) &&
      localizer_parameters_.corrector.slew_mps >= 0.0 &&
      std::isfinite(localizer_parameters_.corrector.slew_rps) &&
      localizer_parameters_.corrector.slew_rps >= 0.0;
  if (!positive_parameters) {
    error = "localizer parameters must be positive (or non-negative timeouts)";
    return false;
  }
  if (!(localizer_parameters_.gps_conflict_ratio > 0.0) ||
      localizer_parameters_.gps_conflict_ratio > 1.0 ||
      !std::isfinite(localizer_parameters_.gps_conflict_ratio)) {
    error = "gps_conflict_ratio must be in (0, 1]";
    return false;
  }
  if (num_threads_ < 1) {
    error = "num_threads must be at least 1";
    return false;
  }
  if (is_mapping()) {
    if (map_output_dir_.empty()) {
      error = "map_output_dir is required in mapping mode";
      return false;
    }
    if (!(map_voxel_size_m_ > 0.0) || !std::isfinite(map_voxel_size_m_) ||
        !(keyframe_cloud_voxel_m_ > 0.0) || !std::isfinite(keyframe_cloud_voxel_m_) ||
        batch_max_iterations_ < 1 || !(init_travel_m_ > 0.0) || !std::isfinite(init_travel_m_)) {
      error = "mapping parameters (map_voxel_size_m, keyframe_cloud_voxel_m, "
              "batch_max_iterations, init_travel_m) must be positive";
      return false;
    }
  }
  return true;
}

bool LocalizationNode::load_map_data(std::string& error) {
  if (map_file_.empty()) {
    error = "map_file is required in localization mode";
    return false;
  }
  PointList loaded_points;
  try {
    loaded_points = read_pcd(map_file_);
  } catch (const std::exception& exception) {
    error = std::string("map_file cannot be read: ") + exception.what();
    return false;
  }
  if (loaded_points.empty()) {
    error = "map_file contains no points: " + map_file_;
    return false;
  }
  MapMetadata loaded_metadata;
  try {
    loaded_metadata = map_yaml_.empty() ? MapMetadata{} : read_map_metadata_or_no_origin(map_yaml_);
  } catch (const std::exception& exception) {
    error = std::string("map_yaml cannot be read: ") + exception.what();
    return false;
  }

  std::optional<GpsOrigin> loaded_origin;
  std::optional<GpsConverter> loaded_converter;
  if (loaded_metadata.has_gps_origin) {
    GpsOrigin origin;
    origin.latitude_deg = loaded_metadata.origin_latitude_deg;
    origin.longitude_deg = loaded_metadata.origin_longitude_deg;
    origin.altitude_m = loaded_metadata.origin_altitude_m;
    loaded_origin = origin;
    loaded_converter.emplace(origin, localizer_parameters_.gps);
  }

  map_points_ = std::move(loaded_points);
  map_metadata_ = loaded_metadata;
  gps_origin_ = loaded_origin;
  gps_converter_ = std::move(loaded_converter);
  map_has_no_gps_origin_ = !gps_origin_.has_value();
  return true;
}

void LocalizationNode::reset_runtime_state() {
  localizer_.reset();
  is_localizer_initialized_ = false;
  has_localizer_wheel_ = false;
  first_localizer_wheel_stamp_sec_ = 0.0;
  initial_position_.reset();
  initial_yaw_.reset();
  initial_position_from_gps_ = false;
  alignment_scans_.clear();
  alignment_target_.reset();
  alignment_target_center_ = Eigen::Vector3d::Zero();
  pending_scans_.clear();
  last_alignment_attempt_sec_ = kNeverSec;
  has_lidar_to_base_ = false;
  lidar_to_base_ = Eigen::Isometry3d::Identity();
  has_gps_antenna_to_base_ = false;
  gps_antenna_in_base_ = Eigen::Vector3d::Zero();
  map_has_no_gps_origin_ = false;
  mapping_has_gps_origin_ = false;
  mapping_interrupted_ = false;
  keyframe_count_at_save_map_.reset();
  mapping_recording_stopped_note_.clear();
  has_mapping_activation_time_ = false;
  mapping_activation_sec_ = 0.0;
  mapping_reference_odom_.reset();
  mapping_reference_enu_.reset();
  latest_mapping_gps_enu_.reset();
  has_wheel_odom_ = false;
  last_odom_to_base_ = Eigen::Isometry3d::Identity();
  alignment_odom_buffer_ = PoseBuffer();
  last_output_.reset();
  last_output_map_to_base_.reset();
  received_gps_status_ = 255;
  received_gps_std_m_ = 0.0F;
  last_passed_gps_stamp_sec_.reset();
  has_diagnostics_publish_time_ = false;
  has_cpu_sample_ = false;
  last_status_state_ = LocalizationState::kInitializing;
  has_status_publish_time_ = false;
  visualization_map_.reset();
  visualization_path_.reset();
  published_map_version_ = 0;
  published_path_version_ = 0;
  has_visualization_publish_time_ = false;
  node_error_.clear();
}

bool LocalizationNode::is_active() const {
  return get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
}

LocalizationNode::CallbackReturn LocalizationNode::on_configure(
    const rclcpp_lifecycle::State& /*state*/) {
  std::string error;
  if (!read_parameters(error)) {
    RCLCPP_ERROR(get_logger(), "invalid parameters: %s", error.c_str());
    return CallbackReturn::FAILURE;
  }
  if (mode_ != "localization" && mode_ != "mapping") {
    RCLCPP_ERROR(get_logger(), "unsupported mode: %s", mode_.c_str());
    return CallbackReturn::FAILURE;
  }
  reset_runtime_state();
  localizer_parameters_.mapping = is_mapping();
  localizer_parameters_.keyframe_cloud_voxel_m = keyframe_cloud_voxel_m_;
  if (is_mapping()) {
    map_points_.clear();
    map_metadata_ = MapMetadata{};
    gps_origin_.reset();
    gps_converter_.reset();
    map_has_no_gps_origin_ = false;
  } else if (!load_map_data(error)) {
    RCLCPP_ERROR(get_logger(), "%s", error.c_str());
    return CallbackReturn::FAILURE;
  }

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, false);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

  odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>(
      "/lightnav/localization/odom", rclcpp::QoS(10).reliable());
  status_publisher_ = create_publisher<lightnav_msgs::msg::LocalizationStatus>(
      "/lightnav/localization/status",
      rclcpp::QoS(1).reliable().transient_local());
  aligned_scan_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/lightnav/localization/aligned_scan", rclcpp::SensorDataQoS());
  diagnostics_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", rclcpp::QoS(10).reliable());

  if (visualization_enabled_) {
    // transient_local: a viewer that connects late (Foxglove) still gets the latest map and path.
    const rclcpp::QoS latched_qos = rclcpp::QoS(1).reliable().transient_local();
    map_cloud_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "/lightnav/localization/map_cloud", latched_qos);
    path_publisher_ = create_publisher<nav_msgs::msg::Path>("/lightnav/localization/path", latched_qos);
    visualization_map_ = std::make_unique<VoxelMapAccumulator>(
        visualization_voxel_m_, static_cast<std::size_t>(visualization_max_points_));
    visualization_path_ = std::make_unique<PathRecorder>(
        path_min_step_m_, static_cast<std::size_t>(path_max_poses_));
  }

  wheel_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      wheel_odom_topic_, rclcpp::QoS(50).reliable(),
      [this](nav_msgs::msg::Odometry::SharedPtr message) { handle_wheel_odom(message); });
  points_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      points_topic_, rclcpp::QoS(5).reliable(),
      [this](sensor_msgs::msg::PointCloud2::SharedPtr message) { handle_point_cloud(message); });
  gps_subscription_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      gps_topic_, rclcpp::QoS(10).reliable(),
      [this](sensor_msgs::msg::NavSatFix::SharedPtr message) { handle_nav_sat_fix(message); });

  set_initial_pose_service_ = create_service<lightnav_msgs::srv::SetInitialPose>(
      "~/set_initial_pose",
      [this](const std::shared_ptr<lightnav_msgs::srv::SetInitialPose::Request> request,
             std::shared_ptr<lightnav_msgs::srv::SetInitialPose::Response> response) {
        handle_set_initial_pose(request, response);
      });
  reload_map_service_ = create_service<std_srvs::srv::Trigger>(
      "~/reload_map",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
             std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        handle_reload_map(request, response);
      });
  save_map_service_ = create_service<std_srvs::srv::Trigger>(
      "~/save_map",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
             std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        handle_save_map(request, response);
      });

  if (is_mapping()) {
    RCLCPP_INFO(get_logger(), "configured mapping mode, output %s", map_output_dir_.c_str());
  } else {
    RCLCPP_INFO(get_logger(), "configured localization mode with %zu map points from %s",
                map_points_.size(), map_file_.c_str());
  }
  return CallbackReturn::SUCCESS;
}

LocalizationNode::CallbackReturn LocalizationNode::on_activate(
    const rclcpp_lifecycle::State& /*state*/) {
  odom_publisher_->on_activate();
  status_publisher_->on_activate();
  aligned_scan_publisher_->on_activate();
  diagnostics_publisher_->on_activate();
  if (map_cloud_publisher_) map_cloud_publisher_->on_activate();
  if (path_publisher_) path_publisher_->on_activate();
  has_status_publish_time_ = false;
  last_status_state_ = LocalizationState::kInitializing;
  mapping_activation_sec_ = get_clock()->now().seconds();
  // With use_sim_time the clock may still be 0 at activation. Start the gps_timeout_s wait
  // only once the clock has a real value.
  has_mapping_activation_time_ = mapping_activation_sec_ > 0.0;
  watchdog_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(kDefaultWatchdogPeriodSec)),
      [this]() { handle_watchdog(); });
  publish_status();
  publish_prior_map_cloud();
  RCLCPP_INFO(get_logger(), "localization node active in %s mode", mode_.c_str());
  return CallbackReturn::SUCCESS;
}

LocalizationNode::CallbackReturn LocalizationNode::on_deactivate(
    const rclcpp_lifecycle::State& /*state*/) {
  if (watchdog_timer_) {
    watchdog_timer_->cancel();
    watchdog_timer_.reset();
  }
  if (odom_publisher_) odom_publisher_->on_deactivate();
  if (status_publisher_) status_publisher_->on_deactivate();
  if (aligned_scan_publisher_) aligned_scan_publisher_->on_deactivate();
  if (diagnostics_publisher_) diagnostics_publisher_->on_deactivate();
  if (map_cloud_publisher_) map_cloud_publisher_->on_deactivate();
  if (path_publisher_) path_publisher_->on_deactivate();
  save_mapping_outputs_on_stop("deactivate");
  has_status_publish_time_ = false;
  return CallbackReturn::SUCCESS;
}

LocalizationNode::CallbackReturn LocalizationNode::on_cleanup(
    const rclcpp_lifecycle::State& /*state*/) {
  if (watchdog_timer_) {
    watchdog_timer_->cancel();
    watchdog_timer_.reset();
  }
  wheel_subscription_.reset();
  points_subscription_.reset();
  gps_subscription_.reset();
  set_initial_pose_service_.reset();
  reload_map_service_.reset();
  save_map_service_.reset();
  odom_publisher_.reset();
  status_publisher_.reset();
  aligned_scan_publisher_.reset();
  diagnostics_publisher_.reset();
  map_cloud_publisher_.reset();
  path_publisher_.reset();
  tf_listener_.reset();
  tf_buffer_.reset();
  tf_broadcaster_.reset();
  localizer_.reset();
  map_points_.clear();
  map_metadata_ = MapMetadata{};
  gps_origin_.reset();
  gps_converter_.reset();
  reset_runtime_state();
  return CallbackReturn::SUCCESS;
}

LocalizationNode::CallbackReturn LocalizationNode::on_shutdown(
    const rclcpp_lifecycle::State& state) {
  if (watchdog_timer_) {
    watchdog_timer_->cancel();
    watchdog_timer_.reset();
  }
  RCLCPP_INFO(get_logger(), "on_shutdown in %s mode", mode_.c_str());
  const bool was_active =
      state.id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
  if (was_active) save_mapping_outputs_on_stop("shutdown");
  odom_publisher_.reset();
  status_publisher_.reset();
  aligned_scan_publisher_.reset();
  diagnostics_publisher_.reset();
  map_cloud_publisher_.reset();
  path_publisher_.reset();
  wheel_subscription_.reset();
  points_subscription_.reset();
  gps_subscription_.reset();
  set_initial_pose_service_.reset();
  reload_map_service_.reset();
  save_map_service_.reset();
  tf_listener_.reset();
  tf_buffer_.reset();
  tf_broadcaster_.reset();
  localizer_.reset();
  return CallbackReturn::SUCCESS;
}

bool LocalizationNode::resolve_lidar_extrinsic(const sensor_msgs::msg::PointCloud2& cloud,
                                               std::string& error) {
  if (cloud.header.frame_id.empty()) {
    error = "point cloud frame_id is empty; cannot look up the LiDAR extrinsic";
    return false;
  }
  try {
    const geometry_msgs::msg::TransformStamped transform_stamped = tf_buffer_->lookupTransform(
        base_frame_, cloud.header.frame_id, tf2::TimePointZero);
    const Eigen::Isometry3d transform = tf2::transformToEigen(transform_stamped);
    if (!is_finite_isometry(transform)) {
      error = "LiDAR extrinsic from TF is not finite";
      return false;
    }
    lidar_to_base_ = transform;
    has_lidar_to_base_ = true;
    node_error_.clear();
    return true;
  } catch (const tf2::TransformException& exception) {
    error = std::string("waiting for TF ") + base_frame_ + " -> " + cloud.header.frame_id +
            ": " + exception.what();
    return false;
  }
}

bool LocalizationNode::resolve_gps_extrinsic(std::string& error) {
  try {
    const geometry_msgs::msg::TransformStamped transform_stamped =
        tf_buffer_->lookupTransform(base_frame_, gps_frame_, tf2::TimePointZero);
    const Eigen::Isometry3d transform = tf2::transformToEigen(transform_stamped);
    if (!is_finite_isometry(transform)) {
      error = "GPS antenna extrinsic from TF is not finite";
      return false;
    }
    gps_antenna_in_base_ = transform.translation();
    has_gps_antenna_to_base_ = true;
    node_error_.clear();
    return true;
  } catch (const tf2::TransformException& exception) {
    error = std::string("waiting for TF ") + base_frame_ + " -> " + gps_frame_ + ": " +
            exception.what();
    return false;
  }
}

GpsFix LocalizationNode::to_gps_fix(const sensor_msgs::msg::NavSatFix& message) const {
  GpsFix fix;
  fix.latitude_deg = message.latitude;
  fix.longitude_deg = message.longitude;
  fix.altitude_m = message.altitude;
  const int status = static_cast<int>(message.status.status);
  fix.status = status >= static_cast<int>(GpsFixStatus::kNoFix) &&
                       status <= static_cast<int>(GpsFixStatus::kGbasFix)
                   ? static_cast<GpsFixStatus>(status)
                   : GpsFixStatus::kNoFix;
  if (message.position_covariance_type ==
      sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_KNOWN) {
    const double horizontal_variance =
        std::max(message.position_covariance[0], message.position_covariance[4]);
    if (std::isfinite(horizontal_variance) && horizontal_variance > 0.0) {
      fix.has_known_covariance = true;
      fix.known_std_m = std::sqrt(horizontal_variance);
    }
  }
  fix.hdop = 0.0;
  return fix;
}

bool LocalizationNode::is_usable_initial_fix(const GpsFix& fix) const {
  if (!gps_converter_ || fix.status == GpsFixStatus::kNoFix) return false;
  if (!std::isfinite(fix.latitude_deg) || !std::isfinite(fix.longitude_deg) ||
      !std::isfinite(fix.altitude_m) || std::abs(fix.latitude_deg) > 90.0 ||
      std::abs(fix.longitude_deg) > 180.0) {
    return false;
  }
  if (fix.hdop > localizer_parameters_.gps.max_hdop) return false;
  return gps_converter_->horizontal_std_m(fix) <= init_gps_max_std_m_;
}

bool LocalizationNode::is_usable_mapping_fix(const GpsFix& fix) const {
  if (fix.status == GpsFixStatus::kNoFix) return false;
  if (!std::isfinite(fix.latitude_deg) || !std::isfinite(fix.longitude_deg) ||
      !std::isfinite(fix.altitude_m) || std::abs(fix.latitude_deg) > 90.0 ||
      std::abs(fix.longitude_deg) > 180.0) {
    return false;
  }
  if (fix.hdop > localizer_parameters_.gps.max_hdop) return false;
  double horizontal_std_m = 0.0;
  if (fix.has_known_covariance && std::isfinite(fix.known_std_m) && fix.known_std_m > 0.0) {
    horizontal_std_m = fix.known_std_m;
  } else if (fix.status == GpsFixStatus::kSbasFix) {
    horizontal_std_m = localizer_parameters_.gps.std_sbas_m;
  } else if (fix.status == GpsFixStatus::kGbasFix) {
    horizontal_std_m = localizer_parameters_.gps.std_gbas_m;
  } else {
    horizontal_std_m = localizer_parameters_.gps.std_fix_m;
  }
  return horizontal_std_m <= init_gps_max_std_m_;
}

void LocalizationNode::handle_wheel_odom(const nav_msgs::msg::Odometry::SharedPtr message) {
  if (!is_active()) return;
  Eigen::Isometry3d odom_to_base = Eigen::Isometry3d::Identity();
  if (!pose_message_to_isometry(message->pose.pose, odom_to_base)) return;

  const rclcpp::Time stamp(message->header.stamp, get_clock()->get_clock_type());
  last_wheel_stamp_ = stamp;
  last_odom_to_base_ = odom_to_base;
  has_wheel_odom_ = true;
  const double stamp_sec = stamp.seconds();
  alignment_odom_buffer_.add(stamp_sec, odom_to_base);
  alignment_odom_buffer_.prune_before(stamp_sec - kWheelBufferSec);

  if (!is_localizer_initialized_) {
    if (is_mapping()) {
      // With GPS the next fix carries the current position; starting between fixes would use a
      // stale fix as the initial pose. Without GPS the wheel sample is enough.
      if (!mapping_has_gps_origin_) maybe_start_mapping();
    } else {
      try_start_initialization();
    }
  }
  if (is_localizer_initialized_) {
    if (!has_localizer_wheel_) {
      has_localizer_wheel_ = true;
      first_localizer_wheel_stamp_sec_ = stamp_sec;
    }
    // Capture event time now; the next diagnostics publication may be up to a second away.
    localizer_->poll_state(get_clock()->now().seconds());
    const std::optional<LocalizerOutput> output =
        localizer_->on_wheel_odom(stamp_sec, odom_to_base);
    if (output) {
      last_output_ = *output;
      publish_output(*output, stamp, odom_to_base);
      if (output->state != last_status_state_) publish_status();
    }
  }
  handle_pending_scans();
}

void LocalizationNode::add_alignment_scan(const PointList& scan) {
  const std::size_t limit = static_cast<std::size_t>(
      std::max(1, alignment_parameters_.scan_count));
  alignment_scans_.push_back(scan);
  while (alignment_scans_.size() > limit) alignment_scans_.pop_front();
}

void LocalizationNode::maybe_align_initial_yaw() {
  if (is_localizer_initialized_ || !initial_position_ || initial_yaw_) return;
  if (alignment_scans_.size() < static_cast<std::size_t>(
                                    std::max(1, alignment_parameters_.scan_count))) {
    return;
  }
  const double now_sec = get_clock()->now().seconds();
  if (now_sec - last_alignment_attempt_sec_ < 0.5) return;
  last_alignment_attempt_sec_ = now_sec;

  PointList accumulated;
  std::size_t point_count = 0;
  for (const PointList& scan : alignment_scans_) point_count += scan.size();
  accumulated.reserve(point_count);
  for (const PointList& scan : alignment_scans_) {
    accumulated.insert(accumulated.end(), scan.begin(), scan.end());
  }
  if (accumulated.empty()) {
    node_error_ = "initial yaw alignment has no scan points";
    return;
  }
  const PointList downsampled =
      downsample_voxel(accumulated, localizer_parameters_.voxel_size_m);
  if (downsampled.size() < 10) {
    node_error_ = "initial yaw alignment has too few downsampled scan points";
    return;
  }

  // The map target only changes when the initial position moves; rebuilding it on every failed
  // attempt would block the single-threaded executor for no reason.
  if (!alignment_target_ ||
      (alignment_target_center_.head<2>() - initial_position_->head<2>()).norm() > 1.0) {
    const std::optional<PreparedTarget> target = build_initial_alignment_target(
        map_points_, *initial_position_, alignment_parameters_.search_radius_m);
    if (!target) {
      node_error_ = "initial yaw alignment has no usable map target near the initial position";
      return;
    }
    alignment_target_ = std::move(*target);
    alignment_target_center_ = *initial_position_;
  }

  const Eigen::Vector3d lever_arm_in_base =
      initial_position_from_gps_ ? gps_antenna_in_base_ : Eigen::Vector3d::Zero();
  const InitialAlignmentResult result =
      align_initial_yaw(*alignment_target_, downsampled, *initial_position_,
                        alignment_parameters_, lever_arm_in_base);
  if (!result.accepted) {
    node_error_ = "initial yaw alignment failed: " + result.error +
                  " (inlier ratio " + std::to_string(result.inlier_ratio) + ")";
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "%s", node_error_.c_str());
    return;
  }
  // The fine match returns the base pose, so use its translation too (a GPS reference is the
  // antenna position and its 3D error is larger than the map-match gate).
  initial_position_ = result.map_to_base.translation();
  initial_position_from_gps_ = false;
  initial_yaw_ = result.yaw_rad;
  alignment_target_.reset();
  node_error_.clear();
  try_start_initialization();
}

void LocalizationNode::handle_point_cloud(
    const sensor_msgs::msg::PointCloud2::SharedPtr message) {
  if (!is_active()) return;
  if (!has_lidar_to_base_) {
    std::string error;
    if (!resolve_lidar_extrinsic(*message, error)) {
      node_error_ = error;
      return;
    }
  }
  std::vector<TimedPoint> timed_points =
      extract_timed_points(message->data.data(), message->data.size(), make_layout(*message));
  if (timed_points.empty()) return;
  float earliest_offset = std::numeric_limits<float>::infinity();
  float latest_offset = -std::numeric_limits<float>::infinity();
  for (const TimedPoint& point : timed_points) {
    earliest_offset = std::min(earliest_offset, point.time_offset_sec);
    latest_offset = std::max(latest_offset, point.time_offset_sec);
  }

  PendingScan pending;
  pending.stamp = message->header.stamp;
  pending.stamp_sec =
      rclcpp::Time(message->header.stamp, get_clock()->get_clock_type()).seconds();
  pending.earliest_offset_sec = earliest_offset;
  pending.latest_offset_sec = latest_offset;
  pending.points = std::move(timed_points);
  pending_scans_.push_back(std::move(pending));
  while (pending_scans_.size() > kMaxPendingScans) pending_scans_.pop_front();
  handle_pending_scans();
}

// Deskews scans that the wheel buffer now covers. Scans are also queued while waiting for the
// initial pose/GPS; the alignment above needs the deskewed cloud but the wheel samples that cover
// a scan usually arrive after it.
void LocalizationNode::handle_pending_scans() {
  if (pending_scans_.empty() || !has_lidar_to_base_) return;
  std::deque<PendingScan> remaining;
  for (PendingScan& scan : pending_scans_) {
    if (!alignment_odom_buffer_.covers(scan.stamp_sec + scan.latest_offset_sec)) {
      remaining.push_back(std::move(scan));
      continue;
    }
    const std::optional<PointList> deskewed =
        deskew_scan(scan.points, scan.stamp_sec, alignment_odom_buffer_, lidar_to_base_);
    if (!deskewed) continue;  // older than the wheel buffer; it cannot become usable again

    if (!is_localizer_initialized_) {
      if (is_mapping()) {
        if (!mapping_has_gps_origin_) maybe_start_mapping();
        if (!is_localizer_initialized_) continue;
      } else {
        add_alignment_scan(*deskewed);
        if (initial_position_ && !initial_yaw_) maybe_align_initial_yaw();
        if (!is_localizer_initialized_) try_start_initialization();
        if (!is_localizer_initialized_) continue;  // consumed by the initial alignment
      }
    }
    if (!has_localizer_wheel_) {
      remaining.push_back(std::move(scan));
      continue;
    }
    if (scan.stamp_sec + scan.earliest_offset_sec <
        first_localizer_wheel_stamp_sec_ - 1e-6) {
      // Localizer's wheel buffer starts at its first wheel sample.
      continue;
    }
    // Capture event time now; the next diagnostics publication may be up to a second away.
    localizer_->poll_state(get_clock()->now().seconds());
    localizer_->on_scan(scan.stamp_sec, scan.points);
    if (last_output_map_to_base_) {
      publish_aligned_scan(*deskewed,
                           rclcpp::Time(scan.stamp, get_clock()->get_clock_type()));
      accumulate_visualization_scan(*deskewed);
    }
  }
  pending_scans_ = std::move(remaining);
  while (pending_scans_.size() > kMaxPendingScans) pending_scans_.pop_front();
}

void LocalizationNode::handle_nav_sat_fix(
    const sensor_msgs::msg::NavSatFix::SharedPtr message) {
  if (!is_active()) return;
  const GpsFix fix = to_gps_fix(*message);
  record_received_gps(fix, *message);
  // D8: a fix below gps_min_status (the single-point solution) is no GNSS observation at all. It
  // counts as neither a rejection nor a reference for the rate limit below.
  if (static_cast<int>(fix.status) < gps_min_status_) return;
  const rclcpp::Time stamp(message->header.stamp, get_clock()->get_clock_type());
  const double stamp_sec = stamp.seconds();

  if (is_mapping() && !is_localizer_initialized_) {
    if (!mapping_has_gps_origin_) {
      if (!is_usable_mapping_fix(fix)) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "waiting for a good GPS fix for the mapping origin");
        return;
      }
      GpsOrigin origin;
      origin.latitude_deg = fix.latitude_deg;
      origin.longitude_deg = fix.longitude_deg;
      origin.altitude_m = fix.altitude_m;
      gps_origin_ = origin;
      gps_converter_.emplace(origin, localizer_parameters_.gps);
      mapping_has_gps_origin_ = true;
      map_has_no_gps_origin_ = false;
      if (has_wheel_odom_) mapping_reference_odom_ = last_odom_to_base_;
      mapping_reference_enu_ = gps_converter_->to_enu(fix);
      latest_mapping_gps_enu_ = mapping_reference_enu_;
      node_error_.clear();
      RCLCPP_INFO(get_logger(), "mapping origin set at lat %.8f, lon %.8f, alt %.3f",
                  origin.latitude_deg, origin.longitude_deg, origin.altitude_m);
    } else if (gps_converter_) {
      if (!is_usable_mapping_fix(fix)) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "ignoring an unusable GPS fix while waiting for the mapping start");
        return;
      }
      latest_mapping_gps_enu_ = gps_converter_->to_enu(fix);
    }
    maybe_start_mapping();
    return;
  }
  if (!gps_origin_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "GPS fix received but the map has no GPS origin; ignoring");
    return;  // GPS disabled: the map has no origin.
  }
  if (!has_gps_antenna_to_base_) {
    std::string error;
    if (!resolve_gps_extrinsic(error)) {
      if (is_mapping()) {
        // Mapping must not depend on an antenna TF that bringup may not provide.
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "GPS antenna TF missing in mapping mode; using a zero lever arm: %s",
            error.c_str());
        gps_antenna_in_base_ = Eigen::Vector3d::Zero();
        has_gps_antenna_to_base_ = true;
      } else {
        node_error_ = error;
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "%s", error.c_str());
        return;
      }
    }
  }

  if (!is_localizer_initialized_) {
    if (!initial_position_ && is_usable_initial_fix(fix)) {
      initial_position_ = gps_converter_->to_enu(fix);
      initial_position_from_gps_ = true;
      alignment_target_.reset();
      node_error_.clear();
      maybe_align_initial_yaw();
    }
    try_start_initialization();
    return;
  }

  if (!is_gps_due(stamp_sec)) return;
  localizer_->on_gps(stamp_sec, fix);
}

void LocalizationNode::record_received_gps(const GpsFix& fix,
                                           const sensor_msgs::msg::NavSatFix& message) {
  received_gps_status_ = static_cast<uint8_t>(message.status.status);
  received_gps_std_m_ = static_cast<float>(localizer_parameters_.gps.horizontal_std_m(fix));
}

bool LocalizationNode::is_gps_due(double stamp_sec) {
  if (last_passed_gps_stamp_sec_) {
    const double elapsed_s = stamp_sec - *last_passed_gps_stamp_sec_;
    // A negative step means the clock restarted (bag loop, sim time reset): let it through,
    // otherwise every later fix would be thinned against a stamp from the future.
    if (elapsed_s >= 0.0 && elapsed_s < gps_min_interval_s_) return false;
  }
  last_passed_gps_stamp_sec_ = stamp_sec;
  return true;
}

bool LocalizationNode::try_start_initialization() {
  if (is_localizer_initialized_) return true;
  if (!initial_position_ || !initial_yaw_) return false;
  if (!has_lidar_to_base_) {
    node_error_ = "waiting for TF " + base_frame_ + " -> LiDAR frame before initializing";
    return false;
  }
  if (gps_origin_ && !has_gps_antenna_to_base_) {
    std::string error;
    if (!resolve_gps_extrinsic(error)) {
      if (!is_mapping()) {
        node_error_ = error;
        return false;
      }
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "GPS antenna TF missing in mapping mode; using a zero lever arm: %s",
          error.c_str());
      gps_antenna_in_base_ = Eigen::Vector3d::Zero();
      has_gps_antenna_to_base_ = true;
    }
  }

  localizer_parameters_.lidar_to_base = lidar_to_base_;
  localizer_parameters_.gps.antenna_in_base = gps_antenna_in_base_;
  Eigen::Vector3d position = *initial_position_;
  if (initial_position_from_gps_ && has_gps_antenna_to_base_) {
    // The fix is the antenna position; remove the lever arm once the yaw is known.
    position -= Eigen::AngleAxisd(*initial_yaw_, Eigen::Vector3d::UnitZ()) *
                gps_antenna_in_base_;
  }
  Eigen::Isometry3d map_to_base = Eigen::Isometry3d::Identity();
  map_to_base.translation() = position;
  map_to_base.linear() =
      Eigen::AngleAxisd(*initial_yaw_, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  if (!is_finite_isometry(map_to_base)) {
    node_error_ = "initial pose is not finite";
    return false;
  }

  if (!localizer_) {
    localizer_ =
        std::make_unique<Localizer>(localizer_parameters_, map_points_, gps_origin_);
  }
  localizer_->initialize(map_to_base);
  is_localizer_initialized_ = true;
  has_localizer_wheel_ = false;
  alignment_scans_.clear();
  alignment_target_.reset();
  last_output_.reset();
  last_output_map_to_base_.reset();
  node_error_.clear();
  RCLCPP_INFO(get_logger(), "initialized %s at (%.3f, %.3f, %.3f), yaw %.3f rad",
              mode_.c_str(), position.x(), position.y(), position.z(), *initial_yaw_);
  return true;
}

void LocalizationNode::maybe_start_mapping() {
  if (!is_mapping() || is_localizer_initialized_) return;
  if (!has_wheel_odom_) return;
  if (!initial_position_) {
    if (mapping_has_gps_origin_ && gps_converter_ && latest_mapping_gps_enu_) {
      if (!mapping_reference_odom_ || !mapping_reference_enu_) {
        mapping_reference_odom_ = last_odom_to_base_;
        mapping_reference_enu_ = *latest_mapping_gps_enu_;
        node_error_ = "mapping origin set; waiting for init_travel_m";
        return;
      }
      const Eigen::Isometry3d wheel_delta =
          invert_isometry(*mapping_reference_odom_) * last_odom_to_base_;
      const double travel_m = wheel_delta.translation().head<2>().norm();
      if (travel_m < init_travel_m_) {
        node_error_ = "mapping: waiting for init_travel_m (" + std::to_string(travel_m) +
                      " m of " + std::to_string(init_travel_m_) + " m)";
        return;
      }
      const Eigen::Vector3d gps_delta = *latest_mapping_gps_enu_ - *mapping_reference_enu_;
      if (gps_delta.head<2>().norm() < 0.5 * init_travel_m_) {
        node_error_ = "mapping: GPS displacement is too small to determine the initial yaw";
        return;
      }
      initial_yaw_ = mapping_initial_yaw(gps_delta, wheel_delta);
      initial_position_ = *latest_mapping_gps_enu_;
      initial_position_from_gps_ = true;
    } else if (!mapping_has_gps_origin_ && allow_mapping_without_gps_) {
      const double now_sec = get_clock()->now().seconds();
      if (!has_mapping_activation_time_) {
        if (!(now_sec > 0.0)) {
          node_error_ = "mapping: waiting for the clock before starting without GPS";
          return;
        }
        has_mapping_activation_time_ = true;
        mapping_activation_sec_ = now_sec;
      }
      const double waited_sec = now_sec - mapping_activation_sec_;
      if (waited_sec < localizer_parameters_.state.gps_timeout_s) {
        node_error_ = "mapping: waiting for gps_timeout_s before starting without GPS";
        return;
      }
      initial_position_ = Eigen::Vector3d::Zero();
      initial_yaw_ = 0.0;
      initial_position_from_gps_ = false;
      map_has_no_gps_origin_ = true;
    } else {
      node_error_ = "waiting for a GPS fix for the mapping origin";
      return;
    }
  }
  try_start_initialization();
}

void LocalizationNode::handle_set_initial_pose(
    const std::shared_ptr<lightnav_msgs::srv::SetInitialPose::Request> request,
    std::shared_ptr<lightnav_msgs::srv::SetInitialPose::Response> response) {
  if (!is_active()) {
    response->accepted = false;
    response->message = "localization node is not active";
    return;
  }
  if (is_mapping()) {
    response->accepted = false;
    response->message = "set_initial_pose is not available in mapping mode";
    return;
  }
  const auto& pose = request->pose.pose.pose;
  Eigen::Isometry3d requested = Eigen::Isometry3d::Identity();
  if (!pose_message_to_isometry(pose, requested)) {
    response->accepted = false;
    response->message = "pose contains non-finite values or an invalid quaternion";
    return;
  }
  const auto& orientation = pose.orientation;
  const bool is_identity_orientation =
      std::abs(orientation.x) < 1e-6 && std::abs(orientation.y) < 1e-6 &&
      std::abs(orientation.z) < 1e-6 && std::abs(orientation.w - 1.0) < 1e-6;
  const bool has_yaw =
      !(is_identity_orientation && request->pose.pose.covariance[35] < 0.0);

  initial_position_ = requested.translation();
  initial_position_from_gps_ = false;
  alignment_target_.reset();
  if (has_yaw) {
    initial_yaw_ = yaw_of(requested);
  } else {
    initial_yaw_.reset();
  }
  is_localizer_initialized_ = false;
  has_localizer_wheel_ = false;
  last_output_.reset();
  last_output_map_to_base_.reset();
  node_error_.clear();

  const bool is_started = try_start_initialization();
  if (!is_started && !has_yaw) maybe_align_initial_yaw();
  response->accepted = true;
  if (is_localizer_initialized_) {
    response->message = "initial pose applied";
  } else if (!has_yaw) {
    response->message = "initial position accepted; waiting for the initial yaw scan match";
  } else {
    response->message = "initial pose accepted; waiting for the required TF";
  }
  publish_status();
}

void LocalizationNode::handle_reload_map(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  if (!is_active()) {
    response->success = false;
    response->message = "localization node is not active";
    return;
  }
  if (is_mapping()) {
    response->success = false;
    response->message = "reload_map is not available in mapping mode";
    return;
  }
  std::string error;
  if (!load_map_data(error)) {
    response->success = false;
    response->message = error;
    return;
  }
  localizer_.reset();
  is_localizer_initialized_ = false;
  has_localizer_wheel_ = false;
  initial_position_.reset();
  initial_yaw_.reset();
  initial_position_from_gps_ = false;
  alignment_scans_.clear();
  last_output_.reset();
  last_output_map_to_base_.reset();
  // Reload starts a new localization session, so errors from the old map no longer apply.
  node_error_.clear();
  response->success = true;
  response->message = "map reloaded: " + std::to_string(map_points_.size()) + " points";
  if (visualization_path_) visualization_path_->clear();
  publish_prior_map_cloud();
  publish_status();
}

void LocalizationNode::handle_save_map(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  if (!is_active()) {
    response->success = false;
    response->message = "localization node is not active";
    return;
  }
  std::string message;
  response->success = save_mapping_outputs(message);
  response->message = message;
  if (response->success) stop_mapping_recording_after_save_map();
}

void LocalizationNode::stop_mapping_recording_after_save_map() {
  // The saved map is final: later keyframes would not be in it, and the stop-time save is skipped
  // as long as the keyframe count stays at this value.
  localizer_->interrupt_mapping();
  mapping_interrupted_ = true;
  keyframe_count_at_save_map_ = localizer_->mapping_keyframe_count();
  mapping_recording_stopped_note_ = "mapping recording stopped after save_map";
  RCLCPP_INFO(get_logger(), "%s; restart mapping to record more (the saved map is overwritten then)",
              mapping_recording_stopped_note_.c_str());
}

void LocalizationNode::save_mapping_outputs_on_stop(const char* phase) {
  if (!is_mapping() || !localizer_) return;
  if (keyframe_count_at_save_map_ &&
      localizer_->mapping_keyframe_count() == *keyframe_count_at_save_map_) {
    RCLCPP_INFO(get_logger(),
                "mapping map not saved on %s: save_map already saved it and no keyframe was added since",
                phase);
    return;
  }
  std::string message;
  if (save_mapping_outputs(message)) {
    RCLCPP_INFO(get_logger(), "mapping map saved on %s: %s", phase, message.c_str());
  } else {
    RCLCPP_WARN(get_logger(), "mapping map could not be saved on %s: %s", phase, message.c_str());
  }
}

bool LocalizationNode::save_mapping_outputs(std::string& message) {
  if (!is_mapping()) {
    message = "save_map is only available in mapping mode";
    return false;
  }
  if (!localizer_ || !is_localizer_initialized_) {
    message = "mapping has not started; no keyframes to save";
    return false;
  }
  MapOutputParameters parameters;
  parameters.map_output_dir = map_output_dir_;
  parameters.map_voxel_size_m = map_voxel_size_m_;
  parameters.batch_max_iterations = batch_max_iterations_;
  parameters.created = utc_now_iso8601();
  parameters.has_gps_origin = mapping_has_gps_origin_ && gps_origin_.has_value();
  if (parameters.has_gps_origin) parameters.gps_origin = *gps_origin_;
  MapOutputResult result;
  std::string error;
  if (!localizer_->save_map(parameters, result, error)) {
    message = "save_map failed: " + error;
    return false;
  }
  message = "saved " + std::to_string(result.keyframe_count) + " keyframes, " +
            std::to_string(result.map_point_count) + " map points to " + map_output_dir_;
  RCLCPP_INFO(get_logger(), "%s", message.c_str());
  return true;
}

void LocalizationNode::handle_watchdog() {
  if (!is_active()) return;
  const rclcpp::Time now = get_clock()->now();
  LocalizationState state = LocalizationState::kInitializing;
  if (localizer_ && is_localizer_initialized_) {
    state = localizer_->poll_state(now.seconds());
  }
  bool should_publish = !has_status_publish_time_ || state != last_status_state_;
  if (has_status_publish_time_) {
    const double elapsed = (now - last_status_publish_time_).seconds();
    should_publish = should_publish || elapsed >= 1.0 / std::max(1e-6, status_rate_hz_);
  }
  if (should_publish) publish_status();
  publish_diagnostics(now);
  publish_visualization(now);
}

DiagnosticStatus LocalizationNode::build_gnss_status(bool has_localizer) const {
  const auto count_text = [has_localizer](const auto& count_of_localizer) {
    return std::to_string(has_localizer ? count_of_localizer() : std::size_t{0});
  };
  const bool is_conflict = last_output_ && last_output_->is_gps_conflict;

  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "localization_node: GNSS";
  status.level = is_conflict ? diagnostic_msgs::msg::DiagnosticStatus::WARN
                             : diagnostic_msgs::msg::DiagnosticStatus::OK;
  status.message = is_conflict ? "GNSS conflicts with the estimate" : "OK";
  status.values.push_back(make_diagnostic_entry("gate_rejected_total", count_text([this] {
                                                  return localizer_->gps_gate_rejection_count();
                                                })));
  status.values.push_back(make_diagnostic_entry("time_gap_rejected_total", count_text([this] {
                                                  return localizer_->gps_time_gap_rejection_count();
                                                })));
  status.values.push_back(make_diagnostic_entry(
      "conflict_window_rejects",
      count_text([this] { return localizer_->gps_conflict_window_reject_count(); })));
  status.values.push_back(make_diagnostic_entry(
      "accepted_total", count_text([this] { return localizer_->accepted_gps_factor_count(); })));
  status.values.push_back(
      make_diagnostic_entry("gps_status", std::to_string(received_gps_status_)));
  status.values.push_back(make_diagnostic_entry("gps_std_m", std::to_string(received_gps_std_m_)));
  status.values.push_back(make_diagnostic_entry("is_gps_conflict", is_conflict ? "true" : "false"));

  return status;
}

DiagnosticStatus LocalizationNode::build_state_status(const rclcpp::Time& now, bool has_localizer) {
  DiagnosticStatus state_status;
  state_status.name = "localization_node: state";
  const LocalizationState state =
      has_localizer ? localizer_->poll_state(now.seconds()) : LocalizationState::kInitializing;
  const std::string state_name = localization_state_name(state);
  state_status.level = DiagnosticStatus::OK;
  if (state == LocalizationState::kLost || state == LocalizationState::kInputStale)
    state_status.level = DiagnosticStatus::ERROR;
  else if (state == LocalizationState::kInitializing ||
           (!is_mapping() && (state == LocalizationState::kOdomOnly ||
                              (state == LocalizationState::kGpsOnly && !map_file_.empty()))))
    state_status.level = DiagnosticStatus::WARN;
  state_status.message = state_name;
  const double last_match = has_localizer ? localizer_->last_map_match_stamp_sec() : kNeverSec;
  const double match_age =
      last_match > kValidTimestampFloorSec ? std::max(0.0, now.seconds() - last_match) : -1.0;
  const double odom_duration =
      has_localizer
          ? std::max(0.0, now.seconds() - localizer_->last_absolute_observation_stamp_sec())
          : 0.0;
  add_diagnostic_value(state_status, "state", state_name);
  add_diagnostic_value(state_status, "mode", is_mapping() ? "mapping" : "localization");
  double position_std = last_output_ ? last_output_->position_std_m : 0.0;
  if (has_localizer) {
    const auto deviations = localizer_->latest_position_and_yaw_std();
    if (deviations)
      position_std = std::max(position_std, std::max((*deviations)[0], (*deviations)[1]));
  }
  add_diagnostic_value(state_status, "position_std_m", std::to_string(position_std));
  add_diagnostic_value(state_status, "seconds_since_last_match", std::to_string(match_age));
  add_diagnostic_value(state_status, "odom_only_distance_m",
                       std::to_string(has_localizer ? localizer_->odom_only_distance_m() : 0.0));
  add_diagnostic_value(state_status, "odom_only_duration_s", std::to_string(odom_duration));
  add_diagnostic_value(state_status, "is_large_correction",
                       last_output_ && last_output_->is_large_correction ? "true" : "false");
  add_diagnostic_value(state_status, "pending_correction_m",
                       std::to_string(last_output_ ? last_output_->pending_correction_m : 0.0));
  return state_status;
}

DiagnosticStatus LocalizationNode::build_map_match_status(const rclcpp::Time& now,
                                                          bool has_localizer) const {
  const double last_match = has_localizer ? localizer_->last_map_match_stamp_sec() : kNeverSec;
  const double match_age =
      last_match > kValidTimestampFloorSec ? std::max(0.0, now.seconds() - last_match) : -1.0;
  DiagnosticStatus match_status;
  match_status.name = "localization_node: map match";
  match_status.level = DiagnosticStatus::OK;
  match_status.message = "no prior map";
  if (is_mapping() || map_points_.empty()) return match_status;
  match_status.message = "OK";
  const double inlier_ratio = has_localizer ? localizer_->last_inlier_ratio() : 0.0;
  if (match_age > localizer_parameters_.state.odom_only_max_s)
    match_status.level = DiagnosticStatus::ERROR;
  else if (match_age > localizer_parameters_.state.tracking_window_s ||
           inlier_ratio < localizer_parameters_.map_matcher.inlier_ratio_min)
    match_status.level = DiagnosticStatus::WARN;
  const MapMatchStatistics counts =
      has_localizer ? localizer_->map_match_statistics() : MapMatchStatistics{};
  add_diagnostic_value(match_status, "last_inlier_ratio", std::to_string(inlier_ratio));
  add_diagnostic_value(
      match_status, "last_lidar_odom_inlier_ratio",
      std::to_string(has_localizer ? localizer_->last_lidar_odom_inlier_ratio() : 0.0));
  add_diagnostic_value(match_status, "seconds_since_last_match", std::to_string(match_age));
  const auto eigenvalue =
      has_localizer ? localizer_->last_match_eigenvalue_ratio() : std::nullopt;
  add_diagnostic_value(match_status, "last_match_eigenvalue_ratio",
                       eigenvalue ? std::to_string(*eigenvalue) : "");
  add_diagnostic_value(match_status, "attempted_total", std::to_string(counts.attempted_total));
  add_diagnostic_value(match_status, "accepted_total", std::to_string(counts.accepted_total));
  add_diagnostic_value(match_status, "last_gate_scale", std::to_string(counts.last_gate_scale));
  add_diagnostic_value(match_status, "rejected_not_converged_total",
                       std::to_string(counts.rejected_not_converged_total));
  add_diagnostic_value(match_status, "rejected_inlier_total",
                       std::to_string(counts.rejected_inlier_total));
  add_diagnostic_value(match_status, "rejected_translation_gate_total",
                       std::to_string(counts.rejected_translation_gate_total));
  add_diagnostic_value(match_status, "rejected_rotation_gate_total",
                       std::to_string(counts.rejected_rotation_gate_total));
  return match_status;
}


DiagnosticStatus LocalizationNode::build_errors_status(bool has_localizer) const {
  DiagnosticStatus errors_status;
  errors_status.name = "localization_node: errors";
  errors_status.level = (!node_error_.empty() || (localizer_ && !localizer_->last_error().empty()))
                            ? DiagnosticStatus::WARN
                            : DiagnosticStatus::OK;
  errors_status.message = errors_status.level == DiagnosticStatus::OK ? "OK" : "error present";
  add_diagnostic_value(errors_status, "last_error", composed_last_error());
  add_diagnostic_value(errors_status, "error_events_total",
                       std::to_string(has_localizer ? localizer_->error_events_total() : 0));
  return errors_status;
}

double LocalizationNode::sample_cpu_percent() {
  const auto cpu_time = std::chrono::steady_clock::now();
  struct rusage usage{};
  double cpu_percent = 0.0;
  if (getrusage(RUSAGE_SELF, &usage) == 0) {
    const double usage_sec = static_cast<double>(usage.ru_utime.tv_sec + usage.ru_stime.tv_sec) +
                             static_cast<double>(usage.ru_utime.tv_usec + usage.ru_stime.tv_usec) /
                                 kMicrosecondsPerSecond;
    const double elapsed = std::chrono::duration<double>(cpu_time - last_cpu_time_).count();
    if (has_cpu_sample_ && elapsed > 0.0)
      cpu_percent = std::max(0.0, 100.0 * (usage_sec - last_cpu_usage_sec_) / elapsed);
    last_cpu_time_ = cpu_time;
    last_cpu_usage_sec_ = usage_sec;
    has_cpu_sample_ = true;
  }
  return cpu_percent;
}

DiagnosticStatus LocalizationNode::build_performance_status(bool has_localizer) {
  const ScanTimeStatistics scan =
      has_localizer ? localizer_->take_scan_time_statistics() : ScanTimeStatistics{};
  const double mean_ms = scan.count ? scan.total_ms / static_cast<double>(scan.count) : 0.0;
  const double cpu_percent = sample_cpu_percent();
  const std::optional<double> memory_mb = read_memory_rss_mb();
  DiagnosticStatus performance_status;
  performance_status.name = "localization_node: performance";
  performance_status.level =
      mean_ms > kScanMeanErrorMs
          ? DiagnosticStatus::ERROR
          : ((scan.maximum_ms > kScanMaximumWarnMs || cpu_percent > diagnostics_cpu_warn_percent_ ||
              (memory_mb && *memory_mb > diagnostics_memory_warn_mb_))
                 ? DiagnosticStatus::WARN
                 : DiagnosticStatus::OK);
  performance_status.message =
      performance_status.level == DiagnosticStatus::OK ? "OK" : "resource threshold exceeded";
  add_diagnostic_value(performance_status, "scans_processed", std::to_string(scan.count));
  add_diagnostic_value(performance_status, "scan_time_mean_ms", std::to_string(mean_ms));
  add_diagnostic_value(performance_status, "scan_time_max_ms", std::to_string(scan.maximum_ms));
  add_diagnostic_value(performance_status, "last_match_time_ms",
                       std::to_string(has_localizer ? localizer_->last_match_time_ms() : 0.0));
  add_diagnostic_value(
      performance_status, "last_submap_rebuild_time_ms",
      std::to_string(has_localizer ? localizer_->last_submap_rebuild_time_ms() : 0.0));
  add_diagnostic_value(performance_status, "cpu_percent", std::to_string(cpu_percent));
  add_diagnostic_value(performance_status, "memory_rss_mb",
                       memory_mb ? std::to_string(*memory_mb) : "unavailable");
  return performance_status;
}

void LocalizationNode::publish_diagnostics(const rclcpp::Time& now) {
  if (!diagnostics_publisher_ || !diagnostics_publisher_->is_activated()) return;
  if (has_diagnostics_publish_time_ &&
      (now - last_diagnostics_publish_time_).seconds() < kDiagnosticsPeriodSec)
    return;
  const bool has_localizer = localizer_ && is_localizer_initialized_;
  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = now;
  array.status.push_back(build_gnss_status(has_localizer));
  array.status.push_back(build_state_status(now, has_localizer));
  array.status.push_back(build_map_match_status(now, has_localizer));
  array.status.push_back(build_errors_status(has_localizer));
  array.status.push_back(build_performance_status(has_localizer));
  diagnostics_publisher_->publish(array);
  last_diagnostics_publish_time_ = now;
  has_diagnostics_publish_time_ = true;
}

std::string LocalizationNode::composed_last_error() const {
  std::string error_message = node_error_;
  const auto append = [&error_message](const std::string& part) {
    if (part.empty()) return;
    if (!error_message.empty()) error_message += "; ";
    error_message += part;
  };
  if (map_has_no_gps_origin_) append("map has no GPS origin: GPS disabled");
  append(mapping_recording_stopped_note_);
  if (localizer_) append(localizer_->last_error());
  return error_message;
}

void LocalizationNode::publish_status() {
  using Status = lightnav_msgs::msg::LocalizationStatus;
  if (!status_publisher_ || !status_publisher_->is_activated()) return;
  Status status;
  const rclcpp::Time now = get_clock()->now();
  status.header.stamp = now;
  status.header.frame_id = map_frame_;
  status.mode = is_mapping() ? Status::MODE_MAPPING : Status::MODE_LOCALIZATION;
  status.state = Status::INITIALIZING;
  status.gps_status = received_gps_status_;
  status.gps_std_m = received_gps_std_m_;
  status.seconds_since_last_match = -1.0F;
  status.position_std_m[0] = 0.0;
  status.position_std_m[1] = 0.0;
  status.position_std_m[2] = 0.0;

  if (localizer_ && is_localizer_initialized_) {
    const LocalizationState state = localizer_->poll_state(now.seconds());
    status.state = state_to_message(state);
    last_status_state_ = state;
    if (is_mapping() && state == LocalizationState::kLost && !mapping_interrupted_) {
      mapping_interrupted_ = true;
      node_error_ = "mapping interrupted: localization LOST; the partial map can be saved";
      if (localizer_) localizer_->interrupt_mapping();
      RCLCPP_ERROR(get_logger(), "%s", node_error_.c_str());
    }
    status.last_inlier_ratio = static_cast<float>(localizer_->last_inlier_ratio());
    status.last_lidar_odom_inlier_ratio =
        static_cast<float>(localizer_->last_lidar_odom_inlier_ratio());
    status.odom_only_distance_m =
        static_cast<float>(localizer_->odom_only_distance_m());
    status.last_scan_time_ms = static_cast<float>(localizer_->last_scan_time_ms());
    status.last_match_time_ms = static_cast<float>(localizer_->last_match_time_ms());
    const double last_match_stamp = localizer_->last_map_match_stamp_sec();
    status.seconds_since_last_match =
        last_match_stamp > -1e8
            ? static_cast<float>(std::max(0.0, now.seconds() - last_match_stamp))
            : -1.0F;
    const std::optional<Eigen::Vector3d> std_values =
        localizer_->latest_position_and_yaw_std();
    if (std_values) {
      const double drift_included_std =
          last_output_ ? last_output_->position_std_m : 0.0;
      status.position_std_m[0] = std::max((*std_values)[0], drift_included_std);
      status.position_std_m[1] = std::max((*std_values)[1], drift_included_std);
      status.position_std_m[2] = (*std_values)[2];
    } else {
      const double position_std = last_output_ ? last_output_->position_std_m : 0.0;
      status.position_std_m[0] = position_std;
      status.position_std_m[1] = position_std;
      status.position_std_m[2] = localizer_parameters_.initial_sigma_yaw_rad;
    }
    if (last_output_) {
      status.is_large_correction = last_output_->is_large_correction;
      status.is_gps_conflict = last_output_->is_gps_conflict;
      status.pending_correction_m =
          static_cast<float>(last_output_->pending_correction_m);
    }
    status.last_error = composed_last_error();
  } else {
    last_status_state_ = LocalizationState::kInitializing;
    status.last_error = composed_last_error();
  }

  status_publisher_->publish(status);
  last_status_publish_time_ = now;
  has_status_publish_time_ = true;
}

void LocalizationNode::publish_output(const LocalizerOutput& output,
                                      const rclcpp::Time& stamp,
                                      const Eigen::Isometry3d& odom_to_base) {
  nav_msgs::msg::Odometry odometry;
  odometry.header.stamp = stamp;
  odometry.header.frame_id = map_frame_;
  odometry.child_frame_id = base_frame_;
  odometry.pose.pose.position.x = output.map_to_base.translation().x();
  odometry.pose.pose.position.y = output.map_to_base.translation().y();
  odometry.pose.pose.position.z = output.map_to_base.translation().z();
  const Eigen::Quaterniond orientation(output.map_to_base.linear());
  odometry.pose.pose.orientation.x = orientation.x();
  odometry.pose.pose.orientation.y = orientation.y();
  odometry.pose.pose.orientation.z = orientation.z();
  odometry.pose.pose.orientation.w = orientation.w();
  odometry.twist.twist = geometry_msgs::msg::Twist();

  Eigen::Vector3d std_values(output.position_std_m, output.position_std_m,
                             localizer_parameters_.initial_sigma_yaw_rad);
  if (localizer_) {
    if (const auto latest = localizer_->latest_position_and_yaw_std()) std_values = *latest;
  }
  std_values[0] = std::max(std_values[0], output.position_std_m);
  std_values[1] = std::max(std_values[1], output.position_std_m);
  constexpr double kMinVariance = 1e-4;
  odometry.pose.covariance.fill(0.0);
  odometry.pose.covariance[0] = std::max(std_values[0] * std_values[0], kMinVariance);
  odometry.pose.covariance[7] = std::max(std_values[1] * std_values[1], kMinVariance);
  odometry.pose.covariance[14] = std::max(std_values[0] * std_values[0], kMinVariance);
  odometry.pose.covariance[21] = kMinVariance;
  odometry.pose.covariance[28] = kMinVariance;
  odometry.pose.covariance[35] = std::max(std_values[2] * std_values[2], kMinVariance);
  odometry.twist.covariance.fill(0.0);
  odometry.twist.covariance[0] = kMinVariance;
  odometry.twist.covariance[7] = kMinVariance;
  odometry.twist.covariance[14] = kMinVariance;
  odometry.twist.covariance[21] = kMinVariance;
  odometry.twist.covariance[28] = kMinVariance;
  odometry.twist.covariance[35] = kMinVariance;

  odom_publisher_->publish(odometry);
  last_output_map_to_base_ = output.map_to_base;
  last_output_stamp_ = stamp;
  last_output_ = output;
  if (visualization_path_) visualization_path_->add(stamp.seconds(), output.map_to_base);

  if (publish_tf_ && tf_broadcaster_) {
    const Eigen::Isometry3d map_to_odom =
        output.map_to_base * invert_isometry(odom_to_base);
    geometry_msgs::msg::TransformStamped transform = tf2::eigenToTransform(map_to_odom);
    transform.header.stamp = stamp;
    transform.header.frame_id = map_frame_;
    transform.child_frame_id = odom_frame_;
    tf_broadcaster_->sendTransform(transform);
  }
}

void LocalizationNode::publish_aligned_scan(const PointList& points_in_base,
                                            const rclcpp::Time& stamp) {
  if (!aligned_scan_publisher_ || !aligned_scan_publisher_->is_activated() ||
      !last_output_map_to_base_ || points_in_base.empty()) {
    return;
  }
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.stamp = stamp;
  cloud.header.frame_id = map_frame_;
  cloud.height = 1;
  cloud.width = static_cast<std::uint32_t>(points_in_base.size());
  cloud.is_dense = true;
  cloud.is_bigendian = false;
  cloud.point_step = 3 * static_cast<std::uint32_t>(sizeof(float));
  cloud.row_step = cloud.point_step * cloud.width;
  for (const char* name : {"x", "y", "z"}) {
    sensor_msgs::msg::PointField field;
    field.name = name;
    field.offset = static_cast<std::uint32_t>(cloud.fields.size()) *
                   static_cast<std::uint32_t>(sizeof(float));
    field.datatype = sensor_msgs::msg::PointField::FLOAT32;
    field.count = 1;
    cloud.fields.push_back(field);
  }
  cloud.data.resize(static_cast<std::size_t>(cloud.row_step));
  std::size_t cursor = 0;
  for (const Eigen::Vector3f& point : points_in_base) {
    const Eigen::Vector3d transformed =
        last_output_map_to_base_->linear() * point.cast<double>() +
        last_output_map_to_base_->translation();
    const float values[3] = {static_cast<float>(transformed.x()),
                             static_cast<float>(transformed.y()),
                             static_cast<float>(transformed.z())};
    std::memcpy(cloud.data.data() + cursor, values, sizeof(values));
    cursor += sizeof(values);
  }
  aligned_scan_publisher_->publish(cloud);
}

void LocalizationNode::accumulate_visualization_scan(const PointList& points_in_base) {
  // Localization mode shows the prior map instead, so scans are only accumulated while mapping.
  if (!visualization_map_ || !is_mapping() || !last_output_map_to_base_) return;
  PointList points_in_map;
  points_in_map.reserve(points_in_base.size());
  for (const Eigen::Vector3f& point : points_in_base) {
    points_in_map.push_back(
        (last_output_map_to_base_->linear() * point.cast<double>() +
         last_output_map_to_base_->translation())
            .cast<float>());
  }
  visualization_map_->add(points_in_map);
}

void LocalizationNode::publish_prior_map_cloud() {
  if (!map_cloud_publisher_ || !map_cloud_publisher_->is_activated() || is_mapping() ||
      map_points_.empty()) {
    return;
  }
  const PointList preview = downsample_voxel(map_points_, visualization_voxel_m_);
  map_cloud_publisher_->publish(make_xyz_cloud(preview, map_frame_, get_clock()->now()));
}

// Publishes the accumulated map and the path at visualization_rate_hz (node clock), and only while
// someone subscribes and the content changed: a 10^6 point cloud is not cheap to build.
void LocalizationNode::publish_visualization(const rclcpp::Time& now) {
  if (!visualization_map_ || !visualization_path_) return;
  if (has_visualization_publish_time_ &&
      (now - last_visualization_publish_time_).seconds() < 1.0 / visualization_rate_hz_) {
    return;
  }
  last_visualization_publish_time_ = now;
  has_visualization_publish_time_ = true;

  if (map_cloud_publisher_ && map_cloud_publisher_->is_activated() && is_mapping() &&
      map_cloud_publisher_->get_subscription_count() > 0 &&
      visualization_map_->version() != published_map_version_) {
    published_map_version_ = visualization_map_->version();
    map_cloud_publisher_->publish(make_xyz_cloud(visualization_map_->points(), map_frame_, now));
  }
  if (path_publisher_ && path_publisher_->is_activated() &&
      path_publisher_->get_subscription_count() > 0 &&
      visualization_path_->version() != published_path_version_) {
    published_path_version_ = visualization_path_->version();
    nav_msgs::msg::Path path;
    path.header.stamp = now;
    path.header.frame_id = map_frame_;
    for (const TimedPose& timed_pose : visualization_path_->poses()) {
      geometry_msgs::msg::PoseStamped pose_stamped;
      pose_stamped.header.frame_id = map_frame_;
      pose_stamped.header.stamp = rclcpp::Time(static_cast<int64_t>(timed_pose.stamp_sec * 1e9),
                                               get_clock()->get_clock_type());
      const Eigen::Vector3d& position = timed_pose.map_to_base.translation();
      pose_stamped.pose.position.x = position.x();
      pose_stamped.pose.position.y = position.y();
      pose_stamped.pose.position.z = position.z();
      const Eigen::Quaterniond orientation(timed_pose.map_to_base.linear());
      pose_stamped.pose.orientation.x = orientation.x();
      pose_stamped.pose.orientation.y = orientation.y();
      pose_stamped.pose.orientation.z = orientation.z();
      pose_stamped.pose.orientation.w = orientation.w();
      path.poses.push_back(pose_stamped);
    }
    path_publisher_->publish(path);
  }
}

}  // namespace lightnav_localization
