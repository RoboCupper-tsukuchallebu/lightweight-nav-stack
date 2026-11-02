#include "lightnav_obstacle_monitor/sensor_projection.hpp"

#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>

#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace lightnav
{

namespace
{
void requireFloat32Field(const sensor_msgs::msg::PointCloud2 & cloud, const char * name)
{
  constexpr std::uint32_t kFloat32ByteCount = 4;
  for (const auto & field : cloud.fields) {
    if (field.name != name) {
      continue;
    }
    const bool isValid = field.datatype == sensor_msgs::msg::PointField::FLOAT32 &&
      field.count >= 1 && field.offset + kFloat32ByteCount <= cloud.point_step;
    if (isValid) {
      return;
    }
    throw std::runtime_error(std::string("PointCloud2 field is not a float32 inside point_step: ") + name);
  }
  throw std::runtime_error(std::string("PointCloud2 has no field ") + name);
}

void validatePointCloud(const sensor_msgs::msg::PointCloud2 & cloud)
{
  if (cloud.is_bigendian) {
    throw std::runtime_error("PointCloud2 is big endian");
  }
  if (cloud.point_step == 0) {
    throw std::runtime_error("PointCloud2 point_step is 0");
  }
  const std::uint64_t rowBytes = static_cast<std::uint64_t>(cloud.width) * cloud.point_step;
  if (cloud.row_step != rowBytes ||
    cloud.data.size() != rowBytes * static_cast<std::uint64_t>(cloud.height))
  {
    throw std::runtime_error("PointCloud2 data size does not match width, height and point_step");
  }
  requireFloat32Field(cloud, "x");
  requireFloat32Field(cloud, "y");
  requireFloat32Field(cloud, "z");
}
}  // namespace

void projectPointCloud(
  const sensor_msgs::msg::PointCloud2 & cloud, const Eigen::Isometry3d & baseFromSensor,
  ObstacleGrid & grid)
{
  validatePointCloud(cloud);
  if (cloud.data.empty()) {
    return;
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
