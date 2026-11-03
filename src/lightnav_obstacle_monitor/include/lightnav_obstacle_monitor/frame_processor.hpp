#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <nav_msgs/msg/occupancy_grid.hpp>

#include "lightnav_obstacle_monitor/obstacle_grid.hpp"

namespace lightnav
{

// Cell-wise OR. Empty input means "source absent" and is ignored.
std::vector<std::uint8_t> combineOccupiedCells(
  const std::vector<std::uint8_t> & first, const std::vector<std::uint8_t> & second);

struct ObstacleSummary
{
  std::uint32_t stopZoneCellCount;
  std::uint32_t slowZoneCellCount;
  float nearestDistanceMeters;
  std::vector<std::uint8_t> inflatedCells;
};

// Zone counts and the nearest distance use the cells before inflation (Issue #1).
ObstacleSummary summarizeOccupiedCells(
  const std::vector<std::uint8_t> & occupiedCells, const GridGeometry & geometry,
  const ObstacleMonitorParameters & parameters);

nav_msgs::msg::OccupancyGrid buildOccupancyGridMessage(
  const std::vector<std::uint8_t> & inflatedCells, const GridGeometry & geometry,
  const std::string & frameId);

}  // namespace lightnav
