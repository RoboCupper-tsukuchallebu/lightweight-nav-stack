#include "lightnav_obstacle_monitor/sensor_projection.hpp"

#include <cmath>
#include <stdexcept>

#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace lightnav
{

namespace
{
bool hasFloat32Field(const sensor_msgs::msg::PointCloud2 & cloud, const char * name)
{
  for (const auto & field : cloud.fields) {
    if (field.name == name) {
      return field.datatype == sensor_msgs::msg::PointField::FLOAT32;
    }
  }
  return false;
}
}  // namespace

void projectPointCloud(
  const sensor_msgs::msg::PointCloud2 & cloud, const Eigen::Isometry3d & baseFromSensor,
  ObstacleGrid & grid)
{
  if (!hasFloat32Field(cloud, "x") || !hasFloat32Field(cloud, "y") ||
    !hasFloat32Field(cloud, "z"))
  {
    throw std::runtime_error("PointCloud2 needs float32 x, y and z fields");
  }
  const Eigen::Matrix3d rotation = baseFromSensor.rotation();
  const Eigen::Vector3d translation = baseFromSensor.translation();
  sensor_msgs::PointCloud2ConstIterator<float> iteratorX(cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iteratorY(cloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iteratorZ(cloud, "z");
  for (; iteratorX != iteratorX.end(); ++iteratorX, ++iteratorY, ++iteratorZ) {
    if (!std::isfinite(*iteratorX) || !std::isfinite(*iteratorY) || !std::isfinite(*iteratorZ)) {
      continue;
    }
    const Eigen::Vector3d pointInBase =
      rotation * Eigen::Vector3d(*iteratorX, *iteratorY, *iteratorZ) + translation;
    grid.addPoint3d(pointInBase.x(), pointInBase.y(), pointInBase.z());
  }
}

void projectLaserScan(
  const sensor_msgs::msg::LaserScan & scan, const Eigen::Isometry3d & baseFromSensor,
  double scanMaxUseRangeMeters, ObstacleGrid & grid)
{
  for (std::size_t beamIndex = 0; beamIndex < scan.ranges.size(); ++beamIndex) {
    const double range = scan.ranges[beamIndex];
    const bool isUsable = std::isfinite(range) && range >= scan.range_min &&
      range <= scan.range_max && range <= scanMaxUseRangeMeters;
    if (!isUsable) {
      continue;
    }
    const double angle = scan.angle_min + static_cast<double>(beamIndex) * scan.angle_increment;
    const Eigen::Vector3d pointInBase =
      baseFromSensor * Eigen::Vector3d(range * std::cos(angle), range * std::sin(angle), 0.0);
    grid.addScanHit(pointInBase.x(), pointInBase.y());
  }
}

}  // namespace lightnav
