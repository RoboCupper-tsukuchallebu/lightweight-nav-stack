#pragma once

#include <Eigen/Geometry>
#include <builtin_interfaces/msg/time.hpp>
#include <cstdint>
#include <deque>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <lightnav_msgs/msg/localization_status.hpp>
#include <lightnav_msgs/srv/set_initial_pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include "lightnav_localization/gps_input.hpp"
#include "lightnav_localization/initial_alignment.hpp"
#include "lightnav_localization/localizer.hpp"
#include "lightnav_localization/map_builder.hpp"
#include "lightnav_localization/map_metadata.hpp"
#include "lightnav_localization/pcd_io.hpp"
#include "lightnav_localization/pose_buffer.hpp"

namespace lightnav_localization {

// Lifecycle node for localization mode (Issue #29). The ROS-free pipeline lives in Localizer; this
// class owns parameter/TF/message plumbing, the initial alignment and the status/TF publications.
class LocalizationNode : public rclcpp_lifecycle::LifecycleNode {
 public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  explicit LocalizationNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~LocalizationNode() override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State& state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State& state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State& state) override;

 private:
  void declare_parameters();
  bool read_parameters(std::string& error);
  bool load_map_data(std::string& error);
  void reset_runtime_state();
  bool is_active() const;
  bool is_mapping() const { return mode_ == "mapping"; }

  void handle_wheel_odom(const nav_msgs::msg::Odometry::SharedPtr message);
  void handle_point_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr message);
  void handle_nav_sat_fix(const sensor_msgs::msg::NavSatFix::SharedPtr message);
  void handle_watchdog();
  void handle_set_initial_pose(
      const std::shared_ptr<lightnav_msgs::srv::SetInitialPose::Request> request,
      std::shared_ptr<lightnav_msgs::srv::SetInitialPose::Response> response);
  void handle_reload_map(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                         std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void handle_save_map(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                       std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  bool save_mapping_outputs(std::string& message);
  // on_deactivate / on_shutdown: saves once, unless save_map already saved and no keyframe was added since.
  void stop_mapping_recording_after_save_map();
  void save_mapping_outputs_on_stop(const char* phase);

  bool resolve_lidar_extrinsic(const sensor_msgs::msg::PointCloud2& cloud, std::string& error);
  bool resolve_gps_extrinsic(std::string& error);
  bool try_start_initialization();
  void maybe_start_mapping();
  void maybe_align_initial_yaw();
  void add_alignment_scan(const PointList& scan);
  void handle_pending_scans();
  GpsFix to_gps_fix(const sensor_msgs::msg::NavSatFix& message) const;
  bool is_usable_initial_fix(const GpsFix& fix) const;
  bool is_usable_mapping_fix(const GpsFix& fix) const;
  // Remembers the status and sigma of the newest fix before gps_min_status can drop it.
  void record_received_gps(const GpsFix& fix, const sensor_msgs::msg::NavSatFix& message);
  // gps_min_interval_s: true when this fix may go to the Localizer (and then becomes the reference).
  bool is_gps_due(double stamp_sec);

  struct PendingScan {
    builtin_interfaces::msg::Time stamp;
    double stamp_sec = 0.0;
    float earliest_offset_sec = 0.0F;
    float latest_offset_sec = 0.0F;
    std::vector<TimedPoint> points;
  };

  void publish_output(const LocalizerOutput& output, const rclcpp::Time& stamp,
                      const Eigen::Isometry3d& odom_to_base);
  void publish_status();
  void publish_aligned_scan(const PointList& points_in_base, const rclcpp::Time& stamp);
  void publish_diagnostics(const rclcpp::Time& now);
  std::string composed_last_error() const;

  // Parameters.
  std::string mode_;
  std::string map_file_;
  std::string map_yaml_;
  std::string map_output_dir_;
  double map_voxel_size_m_ = 0.2;
  double keyframe_cloud_voxel_m_ = 0.25;
  int batch_max_iterations_ = 100;
  LoopClosureParameters loop_closure_parameters_;
  double init_travel_m_ = 3.0;
  bool allow_mapping_without_gps_ = false;
  std::string map_frame_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string gps_frame_;
  std::string wheel_odom_topic_;
  std::string points_topic_;
  std::string gps_topic_;
  bool publish_tf_ = true;
  double status_rate_hz_ = 2.0;
  double init_gps_max_std_m_ = 3.0;
  int gps_min_status_ = -1;  // NavSatStatus value; fixes below it never reach the Localizer
  double gps_min_interval_s_ = 0.0;
  int num_threads_ = 4;
  LocalizerParameters localizer_parameters_;
  InitialAlignmentParameters alignment_parameters_;

  // Map and GPS origin.
  PointList map_points_;
  MapMetadata map_metadata_;
  std::optional<GpsOrigin> gps_origin_;
  std::optional<GpsConverter> gps_converter_;
  bool map_has_no_gps_origin_ = false;

  // Localizer and initial alignment.
  std::unique_ptr<Localizer> localizer_;
  bool is_localizer_initialized_ = false;
  bool has_localizer_wheel_ = false;
  double first_localizer_wheel_stamp_sec_ = 0.0;
  std::optional<Eigen::Vector3d> initial_position_;
  std::optional<double> initial_yaw_;
  bool initial_position_from_gps_ = false;
  bool mapping_has_gps_origin_ = false;
  bool mapping_interrupted_ = false;
  // Keyframe count right after a successful save_map call (Issue #43 D7 (2)).
  std::optional<std::size_t> keyframe_count_at_save_map_;
  std::string mapping_recording_stopped_note_;
  bool has_mapping_activation_time_ = false;
  double mapping_activation_sec_ = 0.0;
  std::optional<Eigen::Isometry3d> mapping_reference_odom_;
  std::optional<Eigen::Vector3d> mapping_reference_enu_;
  std::optional<Eigen::Vector3d> latest_mapping_gps_enu_;
  std::deque<PointList> alignment_scans_;
  std::optional<PreparedTarget> alignment_target_;
  Eigen::Vector3d alignment_target_center_ = Eigen::Vector3d::Zero();
  double last_alignment_attempt_sec_ = -1e9;

  // Extrinsics read from TF after activation.
  bool has_lidar_to_base_ = false;
  Eigen::Isometry3d lidar_to_base_ = Eigen::Isometry3d::Identity();
  bool has_gps_antenna_to_base_ = false;
  Eigen::Vector3d gps_antenna_in_base_ = Eigen::Vector3d::Zero();

  // Wheel input and the node-side buffer used for initial alignment/debug deskew.
  bool has_wheel_odom_ = false;
  rclcpp::Time last_wheel_stamp_;
  Eigen::Isometry3d last_odom_to_base_ = Eigen::Isometry3d::Identity();
  PoseBuffer alignment_odom_buffer_;
  std::deque<PendingScan> pending_scans_;

  // Last published output (for status fields and the debug aligned scan).
  std::optional<LocalizerOutput> last_output_;
  std::optional<Eigen::Isometry3d> last_output_map_to_base_;
  rclcpp::Time last_output_stamp_;

  // Diagnostics.
  std::string node_error_;
  uint8_t received_gps_status_ = 255;  // NavSatStatus of the newest fix (-1 -> 255)
  float received_gps_std_m_ = 0.0F;
  std::optional<double> last_passed_gps_stamp_sec_;
  rclcpp::Time last_diagnostics_publish_time_;
  bool has_diagnostics_publish_time_ = false;
  LocalizationState last_status_state_ = LocalizationState::kInitializing;
  rclcpp::Time last_status_publish_time_;
  bool has_status_publish_time_ = false;

  // ROS entities.
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<lightnav_msgs::msg::LocalizationStatus>::SharedPtr
      status_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      aligned_scan_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
      diagnostics_publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr wheel_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr points_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_subscription_;
  rclcpp::Service<lightnav_msgs::srv::SetInitialPose>::SharedPtr set_initial_pose_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reload_map_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_map_service_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}  // namespace lightnav_localization
