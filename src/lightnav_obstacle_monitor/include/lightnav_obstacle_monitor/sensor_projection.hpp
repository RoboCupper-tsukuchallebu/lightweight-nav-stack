#pragma once

#include <Eigen/Geometry>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "lightnav_obstacle_monitor/obstacle_grid.hpp"

namespace lightnav
{

// Transforms every finite point with the sensor-to-base transform and adds it to the grid.
// Throws std::runtime_error when the cloud has no float32 x/y/z fields.
void projectPointCloud(
  const sensor_msgs::msg::PointCloud2 & cloud, const Eigen::Isometry3d & baseFromSensor,
  ObstacleGrid & grid);

// Adds scan returns within scanMaxUseRangeMeters. Non-finite and out-of-range returns are dropped.
void projectLaserScan(
  const sensor_msgs::msg::LaserScan & scan, const Eigen::Isometry3d & baseFromSensor,
  double scanMaxUseRangeMeters, ObstacleGrid & grid);

}  // namespace lightnav
