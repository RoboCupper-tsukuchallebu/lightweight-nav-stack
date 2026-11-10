#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace lightnav
{

// Parameters of the local grid and the obstacle decision. Defaults follow Issue #1.
struct ObstacleMonitorParameters
{
  double gridSizeMeters = 8.0;
  double cellSizeMeters = 0.1;
  double gridOriginXMeters = -2.0;
  double gridOriginYMeters = -4.0;
  double zMaxMeters = 1.5;
  double heightDifferenceThresholdMeters = 0.15;
  double stepNeighborRadiusMeters = 0.5;
  int minPointsPerCell = 2;
  double scanMaxUseRangeMeters = 4.0;
  double inflationRadiusMeters = 0.5;
  double stopZoneXMeters = 1.0;
  double stopZoneYHalfMeters = 0.4;
  double slowZoneXMeters = 2.5;
  double slowZoneYHalfMeters = 0.6;
  int minCellCount = 2;
  int occupiedMinFrames = 2;
  double clearHoldSeconds = 1.0;
};

// Cell layout of the grid. Row index follows y, column index follows x.
struct GridGeometry
{
  double cellSizeMeters;
  double originXMeters;
  double originYMeters;
  int columnCount;
  int rowCount;

  static GridGeometry fromParameters(const ObstacleMonitorParameters & parameters);

  std::size_t cellCount() const;
  std::optional<std::size_t> cellIndex(double xMeters, double yMeters) const;
  double cellCenterX(std::size_t cellIndex) const;
  double cellCenterY(std::size_t cellIndex) const;
};

// Per-cell z statistics for 3D points plus a hit flag for 2D scan points.
class ObstacleGrid
{
public:
  ObstacleGrid(const GridGeometry & geometry, const ObstacleMonitorParameters & parameters);

  // Points are in the base frame. Points above zMaxMeters or outside the grid are dropped.
  void addPoint3d(double xMeters, double yMeters, double zMeters);
  void addScanHit(double xMeters, double yMeters);

  // 1 for sufficient 3D points with an internal height difference or neighbor step, or a scan hit.
  std::vector<std::uint8_t> occupiedCells() const;
  std::vector<std::uint8_t> groundCells() const;

  const GridGeometry & geometry() const {return geometry_;}

private:
  bool hasNeighborStep(std::size_t index) const;

  GridGeometry geometry_;
  ObstacleMonitorParameters parameters_;
  std::vector<float> minHeights_;
  std::vector<float> maxHeights_;
  std::vector<std::uint16_t> pointCounts_;
  std::vector<std::uint16_t> groundCounts_;
  std::vector<std::uint8_t> scanHits_;
};

// Marks every cell within radiusMeters of an occupied cell.
std::vector<std::uint8_t> inflateOccupiedCells(
  const std::vector<std::uint8_t> & occupiedCells, const GridGeometry & geometry,
  double radiusMeters);

// Number of occupied cells overlapping the zone 0 <= x <= xMeters, |y| <= yHalfMeters (base frame).
std::uint32_t countOccupiedCellsInZone(
  const std::vector<std::uint8_t> & occupiedCells, const GridGeometry & geometry,
  double xMeters, double yHalfMeters);

// Distance from the base frame origin to the nearest occupied cell. +inf when none.
float nearestOccupiedDistance(
  const std::vector<std::uint8_t> & occupiedCells, const GridGeometry & geometry);

// Turns per-frame cell counts into a stable zone state.
class ZoneHysteresis
{
public:
  ZoneHysteresis(int minCellCount, int occupiedMinFrames, double clearHoldSeconds);

  void updateParameters(int minCellCount, int occupiedMinFrames, double clearHoldSeconds)
  {
    minCellCount_ = minCellCount;
    occupiedMinFrames_ = occupiedMinFrames;
    clearHoldSeconds_ = clearHoldSeconds;
  }

  bool update(std::uint32_t cellCount, double nowSeconds);
  bool isOccupied() const {return isOccupied_;}

private:
  int minCellCount_;
  int occupiedMinFrames_;
  double clearHoldSeconds_;
  int consecutiveFrames_ = 0;
  double lastSeenSeconds_ = 0.0;
  bool isOccupied_ = false;
};

}  // namespace lightnav
