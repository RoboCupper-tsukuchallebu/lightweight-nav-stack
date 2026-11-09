#pragma once

#include <Eigen/Geometry>

#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <lightnav_msgs/msg/obstacle_status.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "lightnav_obstacle_monitor/obstacle_grid.hpp"
#include "lightnav_obstacle_monitor/obstacle_memory.hpp"

namespace lightnav
{

class ObstacleMonitorNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  using SteadyClock = std::chrono::steady_clock;

  explicit ObstacleMonitorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;

private:
  // Latest occupied cells of one sensor, with the time of the last successful update.
  struct SensorState
  {
    std::vector<std::uint8_t> occupiedCells;
    std::optional<SteadyClock::time_point> lastUpdate;
    bool hasLastTransformFailed = false;
    std::string sensorFrame;
  };

  void handlePointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr cloud);
  void handleLaserScan(const sensor_msgs::msg::LaserScan::SharedPtr scan);
  void handleWatchdog();
  std::optional<Eigen::Isometry3d> lookupBaseFromSensor(
    const std::string & sensorFrame, SensorState & state);
  bool updateMemory(const ObstacleGrid & grid, const Eigen::Isometry3d & baseFromSensor,
    SensorState & state, const builtin_interfaces::msg::Time & stamp);
  void publishResult(SteadyClock::time_point frameStart, bool shouldUpdateHysteresis);
  void releaseActiveResources();

  ObstacleMonitorParameters parameters_;
  GridGeometry geometry_{};
  std::string baseFrame_;
  std::string memoryFrame_;
  double blindRadiusMeters_, memoryMaxAgeSeconds_, jumpDistanceMeters_, jumpAngleRadians_, memoryMaxRangeMeters_, tfMaxAgeSeconds_, memoryResetReportSeconds_;
  std::optional<ObstacleMemory> memory_;
  std::optional<Eigen::Isometry3d> worldFromBase_;
  bool memoryTfStale_ = true;
  std::optional<SteadyClock::time_point> poseUpdate_, memoryReset_;
  std::string pointsTopic_;
  std::string scanTopic_;
  double pointsTimeoutSeconds_ = 0.5;
  double scanTimeoutSeconds_ = 0.5;
  double tfTimeoutSeconds_ = 0.1;
  double watchdogHz_ = 10.0;

  SensorState pointsState_;
  SensorState scanState_;
  std::optional<SteadyClock::time_point> lastPublish_;
  std::optional<ZoneHysteresis> stopHysteresis_;
  std::optional<ZoneHysteresis> slowHysteresis_;

  std::unique_ptr<tf2_ros::Buffer> tfBuffer_;
  std::shared_ptr<tf2_ros::TransformListener> tfListener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointsSubscription_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scanSubscription_;
  rclcpp_lifecycle::LifecyclePublisher<lightnav_msgs::msg::ObstacleStatus>::SharedPtr
    statusPublisher_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::OccupancyGrid>::SharedPtr gridPublisher_;
  rclcpp::TimerBase::SharedPtr watchdogTimer_;
};

}  // namespace lightnav
