#include "lightnav_obstacle_monitor/frame_processor.hpp"

#include <algorithm>

namespace lightnav
{

std::vector<std::uint8_t> combineOccupiedCells(
  const std::vector<std::uint8_t> & first, const std::vector<std::uint8_t> & second)
{
  if (first.empty()) {
    return second;
  }
  if (second.empty()) {
    return first;
  }
  std::vector<std::uint8_t> combined(std::min(first.size(), second.size()));
  for (std::size_t index = 0; index < combined.size(); ++index) {
    combined[index] = (first[index] != 0 || second[index] != 0) ? 1 : 0;
  }
  return combined;
}

ObstacleSummary summarizeOccupiedCells(
  const std::vector<std::uint8_t> & occupiedCells, const GridGeometry & geometry,
  const ObstacleMonitorParameters & parameters)
{
  return {
    countOccupiedCellsInZone(
      occupiedCells, geometry, parameters.stopZoneXMeters, parameters.stopZoneYHalfMeters),
    countOccupiedCellsInZone(
      occupiedCells, geometry, parameters.slowZoneXMeters, parameters.slowZoneYHalfMeters),
    nearestOccupiedDistance(occupiedCells, geometry),
    inflateOccupiedCells(occupiedCells, geometry, parameters.inflationRadiusMeters)};
}

nav_msgs::msg::OccupancyGrid buildOccupancyGridMessage(
  const std::vector<std::uint8_t> & inflatedCells, const GridGeometry & geometry,
  const std::string & frameId)
{
  constexpr std::int8_t kOccupiedValue = 100;
  nav_msgs::msg::OccupancyGrid message;
  message.header.frame_id = frameId;
  message.info.resolution = static_cast<float>(geometry.cellSizeMeters);
  message.info.width = static_cast<std::uint32_t>(geometry.columnCount);
  message.info.height = static_cast<std::uint32_t>(geometry.rowCount);
  message.info.origin.position.x = geometry.originXMeters;
  message.info.origin.position.y = geometry.originYMeters;
  message.info.origin.orientation.w = 1.0;
  message.data.resize(inflatedCells.size());
  std::transform(
    inflatedCells.begin(), inflatedCells.end(), message.data.begin(),
    [](std::uint8_t cell) {return cell != 0 ? kOccupiedValue : static_cast<std::int8_t>(0);});
  return message;
}

}  // namespace lightnav
