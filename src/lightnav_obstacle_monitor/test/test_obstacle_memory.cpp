#include <gtest/gtest.h>
#include <cmath>
#include <limits>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include "lightnav_obstacle_monitor/obstacle_memory.hpp"
#include "lightnav_obstacle_monitor/frame_processor.hpp"
#include "lightnav_obstacle_monitor/sensor_projection.hpp"

namespace lightnav
{
namespace
{
sensor_msgs::msg::PointCloud2 frame(double robotX, bool box)
{
  sensor_msgs::msg::PointCloud2 cloud;
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(32 * 2160);
  sensor_msgs::PointCloud2Iterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z");
  for (int ring = 0; ring < 32; ++ring) {
    const double elevation = (-30.67 + ring * 41.34 / 31) * M_PI / 180;
    for (int azimuth = 0; azimuth < 2160; ++azimuth, ++x, ++y, ++z) {
      const double angle = azimuth * 2 * M_PI / 2160;
      const Eigen::Vector3d direction(std::cos(elevation) * std::cos(angle),
        std::cos(elevation) * std::sin(angle), std::sin(elevation));
      double range = direction.z() < 0 ? -0.66 / direction.z() : 100.0;
      // Box world bounds x=[2,2.3], y=[-0.15,0.15], z=[0,0.5].
      if (box) {
        double enter = 0, exit = 100;
        const Eigen::Vector3d origin(robotX, 0, 0.66);
        const Eigen::Vector3d lower(2, -0.15, 0), upper(2.3, 0.15, 0.5);
        for (int axis = 0; axis < 3; ++axis) {
          if (std::abs(direction[axis]) < 1e-10) {
            if (origin[axis] < lower[axis] || origin[axis] > upper[axis]) {exit = -1;}
          } else {
            double a = (lower[axis] - origin[axis]) / direction[axis];
            double b = (upper[axis] - origin[axis]) / direction[axis];
            if (a > b) {std::swap(a, b);}
            enter = std::max(enter, a); exit = std::min(exit, b);
          }
        }
        if (enter <= exit && enter > 0) {range = std::min(range, enter);}
      }
      if (range < 0.9 || range > 20) {range = std::numeric_limits<double>::quiet_NaN();}
      *x = range * direction.x(); *y = range * direction.y(); *z = range * direction.z();
    }
  }
  return cloud;
}
Eigen::Isometry3d pose(double x)
{
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.translation().x() = x;
  return result;
}
}
TEST(ObstacleMemoryTest, BoxApproachAndObservedGroundClears)
{
  ObstacleMonitorParameters parameters;
  parameters.stopZoneXMeters = 0.5;
  const auto geometry = GridGeometry::fromParameters(parameters);
  ObstacleMemory memory(geometry, 0.6, 30, 1, 0.785);
  Eigen::Isometry3d mount = Eigen::Isometry3d::Identity();
  mount.translation().z() = 0.66;
  bool observed = false;
  ZoneHysteresis stop(parameters.minCellCount, parameters.occupiedMinFrames, parameters.clearHoldSeconds);
  for (int step = 0; step <= 65; ++step) {
    const double robotX = step * 0.03; // 0.3 m/s, 10 Hz
    ObstacleGrid grid(geometry, parameters);
    projectPointCloud(frame(robotX, true), mount, grid);
    const auto occupied = grid.occupiedCells();
    if (countOccupiedCellsInZone(occupied, geometry, 2.5, 0.6) >= 2) {observed = true;}
    memory.update(occupied, grid.groundCells(), pose(robotX), mount.translation(), step * 0.1);
    auto combined = combineOccupiedCells(occupied, memory.cells(pose(robotX), step * 0.1));
    const auto stopCount = countOccupiedCellsInZone(combined, geometry, 0.5, 0.4);
    const bool stopped = stop.update(stopCount, step * 0.1);
    if (robotX >= 1.65) {
      EXPECT_GE(stopCount, 2U) << robotX;
      EXPECT_TRUE(stopped) << robotX;
    }
  }
  EXPECT_TRUE(observed);
  // Move back until the floor in the former box cells is visible.
  for (int step = 0; step <= 50; ++step) {
    const double robotX = 1.95 - step * 0.03;
    ObstacleGrid grid(geometry, parameters);
    projectPointCloud(frame(robotX, false), mount, grid);
    memory.update(grid.occupiedCells(), grid.groundCells(), pose(robotX), mount.translation(), 7 + step * 0.1);
  }
  EXPECT_EQ(memory.size(), 0U);
}
TEST(ObstacleMemoryTest, JumpClearsAndAgeBoundsMemory)
{
  ObstacleMonitorParameters parameters;
  const auto geometry = GridGeometry::fromParameters(parameters);
  ObstacleMemory memory(geometry, 0.6, 30, 1, 0.785);
  std::vector<std::uint8_t> occupied(geometry.cellCount(), 0), empty(geometry.cellCount(), 0);
  occupied[*geometry.cellIndex(1.05, 0.05)] = 1;
  EXPECT_FALSE(memory.update(occupied, empty, pose(0), Eigen::Vector3d::Zero(), 0));
  EXPECT_EQ(memory.size(), 1U);
  EXPECT_TRUE(memory.update(empty, empty, pose(2), Eigen::Vector3d::Zero(), 1));
  EXPECT_EQ(memory.size(), 0U);
  memory.update(occupied, empty, pose(2), Eigen::Vector3d::Zero(), 2);
  memory.update(empty, empty, pose(13), Eigen::Vector3d::Zero(), 32);
  EXPECT_EQ(memory.size(), 0U);
}
TEST(ObstacleMemoryTest, UnobservedAndBlindGroundDoNotClear)
{
  ObstacleMonitorParameters parameters;
  const auto geometry = GridGeometry::fromParameters(parameters);
  ObstacleMemory memory(geometry, 0.6, 30, 1, 0.785);
  std::vector<std::uint8_t> occupied(geometry.cellCount(), 0), ground(geometry.cellCount(), 0);
  const auto index = *geometry.cellIndex(0.35, 0.05);
  occupied[index] = 1;
  memory.update(occupied, ground, pose(0), Eigen::Vector3d::Zero(), 0);
  occupied[index] = 0;
  memory.update(occupied, ground, pose(0), Eigen::Vector3d::Zero(), 1);
  EXPECT_EQ(memory.size(), 1U);
  ground[index] = 1;
  memory.update(occupied, ground, pose(0), Eigen::Vector3d::Zero(), 2);
  EXPECT_EQ(memory.size(), 1U);
  memory.cells(pose(0), 30);
  EXPECT_EQ(memory.size(), 1U);
}
TEST(ObstacleMemoryTest, SafetyParameterUpdateKeepsExistingMemory)
{
  ObstacleMonitorParameters parameters;
  const auto geometry = GridGeometry::fromParameters(parameters);
  ObstacleMemory memory(geometry, 0.6, 30, 1, 0.785);
  std::vector<std::uint8_t> occupied(geometry.cellCount(), 0), ground(geometry.cellCount(), 0);
  const auto index = *geometry.cellIndex(1.05, 0.05);
  occupied[index] = 1;
  memory.update(occupied, ground, pose(0), Eigen::Vector3d::Zero(), 0);
  memory.updateSafetyParameters(1.5, 60);
  EXPECT_EQ(memory.size(), 1U);
  occupied[index] = 0;
  ground[index] = 1;
  memory.update(occupied, ground, pose(0), Eigen::Vector3d::Zero(), 40);
  EXPECT_EQ(memory.size(), 1U);
}
TEST(ObstacleMemoryTest, RotationJumpClears)
{
  ObstacleMonitorParameters parameters;
  const auto geometry = GridGeometry::fromParameters(parameters);
  ObstacleMemory memory(geometry, 0.6, 30, 1, 0.785);
  std::vector<std::uint8_t> occupied(geometry.cellCount(), 0), empty(geometry.cellCount(), 0);
  occupied[*geometry.cellIndex(1.05, 0.05)] = 1;
  memory.update(occupied, empty, pose(0), Eigen::Vector3d::Zero(), 0);
  auto rotated = pose(0);
  rotated.rotate(Eigen::AngleAxisd(1.0, Eigen::Vector3d::UnitZ()));
  EXPECT_TRUE(memory.update(empty, empty, rotated, Eigen::Vector3d::Zero(), 1));
  EXPECT_EQ(memory.size(), 0U);
}
}  // namespace lightnav
