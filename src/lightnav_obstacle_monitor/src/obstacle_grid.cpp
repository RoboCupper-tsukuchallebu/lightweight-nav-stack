#include "lightnav_obstacle_monitor/obstacle_grid.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace lightnav
{

GridGeometry GridGeometry::fromParameters(const ObstacleMonitorParameters & parameters)
{
  const int sideCellCount =
    static_cast<int>(std::lround(parameters.gridSizeMeters / parameters.cellSizeMeters));
  return {parameters.cellSizeMeters, parameters.gridOriginXMeters, parameters.gridOriginYMeters,
    sideCellCount, sideCellCount};
}

std::size_t GridGeometry::cellCount() const
{
  return static_cast<std::size_t>(columnCount) * static_cast<std::size_t>(rowCount);
}

std::optional<std::size_t> GridGeometry::cellIndex(double xMeters, double yMeters) const
{
  const double columnValue = std::floor((xMeters - originXMeters) / cellSizeMeters);
  const double rowValue = std::floor((yMeters - originYMeters) / cellSizeMeters);
  // Written so that NaN also fails the comparison.
  if (!(columnValue >= 0.0 && columnValue < columnCount && rowValue >= 0.0 && rowValue < rowCount)) {
    return std::nullopt;
  }
  return static_cast<std::size_t>(rowValue) * columnCount + static_cast<std::size_t>(columnValue);
}

double GridGeometry::cellCenterX(std::size_t cellIndex) const
{
  return originXMeters + (static_cast<double>(cellIndex % columnCount) + 0.5) * cellSizeMeters;
}

double GridGeometry::cellCenterY(std::size_t cellIndex) const
{
  return originYMeters + (static_cast<double>(cellIndex / columnCount) + 0.5) * cellSizeMeters;
}

ObstacleGrid::ObstacleGrid(
  const GridGeometry & geometry, const ObstacleMonitorParameters & parameters)
: geometry_(geometry),
  parameters_(parameters),
  minHeights_(geometry.cellCount(), std::numeric_limits<float>::infinity()),
  maxHeights_(geometry.cellCount(), -std::numeric_limits<float>::infinity()),
  pointCounts_(geometry.cellCount(), 0),
  scanHits_(geometry.cellCount(), 0)
{
}

void ObstacleGrid::addPoint3d(double xMeters, double yMeters, double zMeters)
{
  if (!(zMeters <= parameters_.zMaxMeters)) {  // also drops NaN
    return;
  }
  const auto index = geometry_.cellIndex(xMeters, yMeters);
  if (!index) {
    return;
  }
  const float height = static_cast<float>(zMeters);
  minHeights_[*index] = std::min(minHeights_[*index], height);
  maxHeights_[*index] = std::max(maxHeights_[*index], height);
  if (pointCounts_[*index] < std::numeric_limits<std::uint16_t>::max()) {
    ++pointCounts_[*index];
  }
}

void ObstacleGrid::addScanHit(double xMeters, double yMeters)
{
  const auto index = geometry_.cellIndex(xMeters, yMeters);
  if (index) {
    scanHits_[*index] = 1;
  }
}

bool ObstacleGrid::hasNeighborStep(std::size_t index) const
{
  if (!(parameters_.stepNeighborRadiusMeters > 0.0)) {
    return false;
  }
  const double radiusCells = parameters_.stepNeighborRadiusMeters / geometry_.cellSizeMeters;
  const int columnRadius = static_cast<int>(std::min(
    std::ceil(radiusCells), static_cast<double>(geometry_.columnCount - 1)));
  const int rowRadius = static_cast<int>(std::min(
    std::ceil(radiusCells), static_cast<double>(geometry_.rowCount - 1)));
  const int column = static_cast<int>(index % geometry_.columnCount);
  const int row = static_cast<int>(index / geometry_.columnCount);
  for (int neighborRow = std::max(0, row - rowRadius);
    neighborRow <= std::min(geometry_.rowCount - 1, row + rowRadius); ++neighborRow)
  {
    for (int neighborColumn = std::max(0, column - columnRadius);
      neighborColumn <= std::min(geometry_.columnCount - 1, column + columnRadius);
      ++neighborColumn)
    {
      const double rowOffset = neighborRow - row;
      const double columnOffset = neighborColumn - column;
      if (rowOffset * rowOffset + columnOffset * columnOffset > radiusCells * radiusCells) {
        continue;
      }
      const auto neighborIndex =
        static_cast<std::size_t>(neighborRow) * geometry_.columnCount + neighborColumn;
      // Existence of a lower neighbor is equivalent to comparing against their minimum z.
      if (neighborIndex != index && pointCounts_[neighborIndex] >= parameters_.minPointsPerCell &&
        maxHeights_[index] - minHeights_[neighborIndex] > parameters_.heightDifferenceThresholdMeters)
      {
        return true;
      }
    }
  }
  return false;
}

std::vector<std::uint8_t> ObstacleGrid::occupiedCells() const
{
  std::vector<std::uint8_t> occupied(geometry_.cellCount(), 0);
  for (std::size_t index = 0; index < occupied.size(); ++index) {
    const bool hasHeightStep =
      pointCounts_[index] >= parameters_.minPointsPerCell &&
      (maxHeights_[index] - minHeights_[index] > parameters_.heightDifferenceThresholdMeters ||
      hasNeighborStep(index));
    occupied[index] = (hasHeightStep || scanHits_[index] != 0) ? 1 : 0;
  }
  return occupied;
}

std::vector<std::uint8_t> inflateOccupiedCells(
  const std::vector<std::uint8_t> & occupiedCells, const GridGeometry & geometry,
  double radiusMeters)
{
  const int radiusCells = static_cast<int>(std::ceil(radiusMeters / geometry.cellSizeMeters));
  const double radiusCellsSquared =
    (radiusMeters / geometry.cellSizeMeters) * (radiusMeters / geometry.cellSizeMeters);
  std::vector<std::uint8_t> inflated(occupiedCells.size(), 0);
  for (std::size_t index = 0; index < occupiedCells.size(); ++index) {
    if (occupiedCells[index] == 0) {
      continue;
    }
    const int column = static_cast<int>(index % geometry.columnCount);
    const int row = static_cast<int>(index / geometry.columnCount);
    for (int rowOffset = -radiusCells; rowOffset <= radiusCells; ++rowOffset) {
      const int neighborRow = row + rowOffset;
      if (neighborRow < 0 || neighborRow >= geometry.rowCount) {
        continue;
      }
      for (int columnOffset = -radiusCells; columnOffset <= radiusCells; ++columnOffset) {
        const int neighborColumn = column + columnOffset;
        const bool isInside = neighborColumn >= 0 && neighborColumn < geometry.columnCount;
        const bool isWithinRadius =
          rowOffset * rowOffset + columnOffset * columnOffset <= radiusCellsSquared;
        if (isInside && isWithinRadius) {
          inflated[static_cast<std::size_t>(neighborRow) * geometry.columnCount + neighborColumn] =
            1;
        }
      }
    }
  }
  return inflated;
}

std::uint32_t countOccupiedCellsInZone(
  const std::vector<std::uint8_t> & occupiedCells, const GridGeometry & geometry,
  double xMeters, double yHalfMeters)
{
  std::uint32_t count = 0;
  for (std::size_t index = 0; index < occupiedCells.size(); ++index) {
    if (occupiedCells[index] == 0) {
      continue;
    }
    // A cell counts when it overlaps the zone even partly (safe side).
    constexpr double kEpsilonMeters = 1e-9;
    const double halfCell = geometry.cellSizeMeters / 2.0;
    const double x = geometry.cellCenterX(index);
    const double y = geometry.cellCenterY(index);
    const bool overlapsX = x + halfCell > kEpsilonMeters && x - halfCell < xMeters - kEpsilonMeters;
    const bool overlapsY = std::abs(y) - halfCell < yHalfMeters - kEpsilonMeters;
    if (overlapsX && overlapsY) {
      ++count;
    }
  }
  return count;
}

float nearestOccupiedDistance(
  const std::vector<std::uint8_t> & occupiedCells, const GridGeometry & geometry)
{
  double nearestSquared = std::numeric_limits<double>::infinity();
  for (std::size_t index = 0; index < occupiedCells.size(); ++index) {
    if (occupiedCells[index] == 0) {
      continue;
    }
    const double x = geometry.cellCenterX(index);
    const double y = geometry.cellCenterY(index);
    nearestSquared = std::min(nearestSquared, x * x + y * y);
  }
  return static_cast<float>(std::sqrt(nearestSquared));
}

ZoneHysteresis::ZoneHysteresis(int minCellCount, int occupiedMinFrames, double clearHoldSeconds)
: minCellCount_(minCellCount),
  occupiedMinFrames_(occupiedMinFrames),
  clearHoldSeconds_(clearHoldSeconds)
{
}

bool ZoneHysteresis::update(std::uint32_t cellCount, double nowSeconds)
{
  if (cellCount >= static_cast<std::uint32_t>(minCellCount_)) {
    ++consecutiveFrames_;
    lastSeenSeconds_ = nowSeconds;
    if (consecutiveFrames_ >= occupiedMinFrames_) {
      isOccupied_ = true;
    }
  } else {
    consecutiveFrames_ = 0;
    if (isOccupied_ && nowSeconds - lastSeenSeconds_ >= clearHoldSeconds_) {
      isOccupied_ = false;
    }
  }
  return isOccupied_;
}

}  // namespace lightnav
