#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <iostream>
#include <limits>

#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "lightnav_obstacle_monitor/frame_processor.hpp"
#include "lightnav_obstacle_monitor/obstacle_grid.hpp"
#include "lightnav_obstacle_monitor/sensor_projection.hpp"

namespace lightnav
{
namespace
{
constexpr int kRingCount = 32;
constexpr int kAzimuthCount = 2160;  // 32 * 2160 = 69,120 points, like one HDL-32E frame
constexpr double kSensorHeightMeters = 0.7;
constexpr double kCylinderHeightMeters = 1.0;

Eigen::Isometry3d sensorMount()
{
  Eigen::Isometry3d baseFromSensor = Eigen::Isometry3d::Identity();
  baseFromSensor.translation() = Eigen::Vector3d(0.1, 0.0, kSensorHeightMeters);
  return baseFromSensor;
}

// Rings from -30.67 to +10.67 degrees; each ray hits the ground or a 0.5 m radius cylinder
// whose axis is at (2.8, 0.0) in the base frame (1 m wide obstacle, nearest surface at 2.3 m).
sensor_msgs::msg::PointCloud2 makeSyntheticFrame()
{
  sensor_msgs::msg::PointCloud2 cloud;
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(static_cast<std::size_t>(kRingCount) * kAzimuthCount);
  sensor_msgs::PointCloud2Iterator<float> iteratorX(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iteratorY(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iteratorZ(cloud, "z");
  const double cylinderCenterX = 2.8 - 0.1;  // in the sensor frame
  const double cylinderRadius = 0.5;
  for (int ring = 0; ring < kRingCount; ++ring) {
    const double elevation = (-30.67 + ring * (41.34 / (kRingCount - 1))) * M_PI / 180.0;
    for (int azimuthIndex = 0; azimuthIndex < kAzimuthCount; ++azimuthIndex) {
      const double azimuth = 2.0 * M_PI * azimuthIndex / kAzimuthCount;
      const Eigen::Vector3d direction(
        std::cos(elevation) * std::cos(azimuth), std::cos(elevation) * std::sin(azimuth),
        std::sin(elevation));
      double range = 60.0;
      if (direction.z() < -1e-3) {
        range = std::min(range, -kSensorHeightMeters / direction.z());
      }
      // Ray / vertical cylinder intersection in the xy plane (cylinder height 1.0 m).
      const double quadraticA = direction.x() * direction.x() + direction.y() * direction.y();
      const double quadraticB = -2.0 * cylinderCenterX * direction.x();
      const double quadraticC = cylinderCenterX * cylinderCenterX - cylinderRadius * cylinderRadius;
      const double discriminant = quadraticB * quadraticB - 4.0 * quadraticA * quadraticC;
      if (discriminant >= 0.0 && quadraticA > 1e-9) {
        const double hitRange = (-quadraticB - std::sqrt(discriminant)) / (2.0 * quadraticA);
        const bool isBelowTop = kSensorHeightMeters + hitRange * direction.z() <= kCylinderHeightMeters;
        if (hitRange > 0.0 && isBelowTop) {
          range = std::min(range, hitRange);
        }
      }
      *iteratorX = static_cast<float>(range * direction.x());
      *iteratorY = static_cast<float>(range * direction.y());
      *iteratorZ = static_cast<float>(range * direction.z());
      ++iteratorX;
      ++iteratorY;
      ++iteratorZ;
    }
  }
  return cloud;
}

// Same work as one node callback: project, threshold, summarize, build the grid message.
ObstacleSummary processFrame(
  const sensor_msgs::msg::PointCloud2 & cloud, const ObstacleMonitorParameters & parameters,
  nav_msgs::msg::OccupancyGrid & gridMessage)
{
  const auto geometry = GridGeometry::fromParameters(parameters);
  ObstacleGrid grid(geometry, parameters);
  projectPointCloud(cloud, sensorMount(), grid);
  const auto summary = summarizeOccupiedCells(grid.occupiedCells(), geometry, parameters);
  gridMessage = buildOccupancyGridMessage(summary.inflatedCells, geometry, "base_link");
  return summary;
}
}  // namespace

TEST(FrameProcessorTest, SyntheticFrameDetectsCylinderInSlowZoneNotStopZone)
{
  const auto cloud = makeSyntheticFrame();
  ASSERT_EQ(cloud.width * cloud.height, 69120U);
  nav_msgs::msg::OccupancyGrid gridMessage;
  const auto summary = processFrame(cloud, {}, gridMessage);
  EXPECT_EQ(summary.stopZoneCellCount, 0U);
  EXPECT_GE(summary.slowZoneCellCount, 2U);
  EXPECT_NEAR(summary.nearestDistanceMeters, 2.3F, 0.3F);
  EXPECT_EQ(gridMessage.data.size(), 80U * 80U);
}

TEST(FrameProcessorTest, CloudWithoutXyzFieldsThrows)
{
  sensor_msgs::msg::PointCloud2 cloud;
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2Fields(
    1, "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
  modifier.resize(4);
  const ObstacleMonitorParameters parameters;
  ObstacleGrid grid(GridGeometry::fromParameters(parameters), parameters);
  EXPECT_THROW(projectPointCloud(cloud, sensorMount(), grid), std::runtime_error);
}

namespace
{
void expectInvalidCloudIsRejected(const std::function<void(sensor_msgs::msg::PointCloud2 &)> & breakCloud)
{
  auto cloud = makeSyntheticFrame();
  breakCloud(cloud);
  const ObstacleMonitorParameters parameters;
  ObstacleGrid grid(GridGeometry::fromParameters(parameters), parameters);
  EXPECT_THROW(projectPointCloud(cloud, sensorMount(), grid), std::runtime_error);
}
}  // namespace

TEST(FrameProcessorTest, ZeroPointStepIsRejected)
{
  expectInvalidCloudIsRejected([](auto & cloud) {cloud.point_step = 0;});
}

TEST(FrameProcessorTest, DataSizeNotMatchingWidthIsRejected)
{
  expectInvalidCloudIsRejected([](auto & cloud) {cloud.data.pop_back();});
}

TEST(FrameProcessorTest, RowStepNotMatchingIsRejected)
{
  expectInvalidCloudIsRejected([](auto & cloud) {cloud.row_step += 4;});
}

TEST(FrameProcessorTest, FieldBeyondPointStepIsRejected)
{
  expectInvalidCloudIsRejected([](auto & cloud) {cloud.fields[2].offset = cloud.point_step;});
}

TEST(FrameProcessorTest, NonFloatFieldIsRejected)
{
  expectInvalidCloudIsRejected(
    [](auto & cloud) {cloud.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT64;});
}

TEST(FrameProcessorTest, BigEndianCloudIsRejected)
{
  expectInvalidCloudIsRejected([](auto & cloud) {cloud.is_bigendian = true;});
}

TEST(FrameProcessorTest, EmptyCloudIsAcceptedAndAddsNothing)
{
  sensor_msgs::msg::PointCloud2 cloud;
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(0);
  const ObstacleMonitorParameters parameters;
  ObstacleGrid grid(GridGeometry::fromParameters(parameters), parameters);
  projectPointCloud(cloud, sensorMount(), grid);
  const auto occupied = grid.occupiedCells();
  EXPECT_EQ(std::count(occupied.begin(), occupied.end(), 1), 0);
}

TEST(FrameProcessorTest, LaserScanWithinUseRangeOccupiesAndFarReturnIsIgnored)
{
  sensor_msgs::msg::LaserScan scan;
  scan.angle_min = -0.1F;
  scan.angle_increment = 0.1F;
  scan.range_min = 0.05F;
  scan.range_max = 10.0F;
  scan.ranges = {2.0F, std::numeric_limits<float>::infinity(), 6.0F, 0.01F};
  const ObstacleMonitorParameters parameters;
  ObstacleGrid grid(GridGeometry::fromParameters(parameters), parameters);
  projectLaserScan(scan, Eigen::Isometry3d::Identity(), parameters.scanMaxUseRangeMeters, grid);
  const auto occupied = grid.occupiedCells();
  EXPECT_EQ(std::count(occupied.begin(), occupied.end(), 1), 1);
}

TEST(FrameProcessorTest, CombineIgnoresEmptySourceAndUnionsOthers)
{
  const std::vector<std::uint8_t> first = {1, 0, 0};
  const std::vector<std::uint8_t> second = {0, 0, 1};
  EXPECT_EQ(combineOccupiedCells({}, second), second);
  EXPECT_EQ(combineOccupiedCells(first, {}), first);
  EXPECT_EQ(combineOccupiedCells(first, second), (std::vector<std::uint8_t>{1, 0, 1}));
}

TEST(FrameProcessorTest, ProcessingTimeOfOneSeventyThousandPointFrame)
{
  constexpr int kWarmupCount = 5;
  constexpr int kMeasureCount = 200;
  constexpr double kCiLimitMilliseconds = 100.0;  // Issue #1: CI threshold
  const auto cloud = makeSyntheticFrame();
  nav_msgs::msg::OccupancyGrid gridMessage;
  for (int count = 0; count < kWarmupCount; ++count) {
    processFrame(cloud, {}, gridMessage);
  }
  std::vector<double> durationsMilliseconds;
  for (int count = 0; count < kMeasureCount; ++count) {
    const auto start = std::chrono::steady_clock::now();
    processFrame(cloud, {}, gridMessage);
    durationsMilliseconds.push_back(
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count());
  }
  std::sort(durationsMilliseconds.begin(), durationsMilliseconds.end());
  const double median = durationsMilliseconds[kMeasureCount / 2];
  const double p99 = durationsMilliseconds[kMeasureCount * 99 / 100];
  std::cout << "[ MEASURED ] points=" << cloud.width * cloud.height << " median_ms=" << median
            << " p99_ms=" << p99 << std::endl;
  EXPECT_LT(p99, kCiLimitMilliseconds);
}

}  // namespace lightnav
