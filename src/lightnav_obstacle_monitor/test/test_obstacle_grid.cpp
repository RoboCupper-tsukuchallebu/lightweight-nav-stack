#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "lightnav_obstacle_monitor/obstacle_grid.hpp"

namespace lightnav
{
namespace
{
constexpr double kDegreesToRadians = M_PI / 180.0;

ObstacleGrid makeGrid(const ObstacleMonitorParameters & parameters)
{
  return ObstacleGrid(GridGeometry::fromParameters(parameters), parameters);
}

// Samples a ground plane with the given slope along +x, 2 cm apart, over the whole grid.
void addSlopedGround(ObstacleGrid & grid, double slopeDegrees)
{
  const double tangent = std::tan(slopeDegrees * kDegreesToRadians);
  for (double x = -1.9; x < 5.9; x += 0.02) {
    for (double y = -3.9; y < 3.9; y += 0.02) {
      grid.addPoint3d(x, y, x * tangent);
    }
  }
}

std::size_t countOccupied(const std::vector<std::uint8_t> & cells)
{
  std::size_t count = 0;
  for (auto cell : cells) {
    count += cell != 0 ? 1 : 0;
  }
  return count;
}

void addRaisedSurfaceWithGround(ObstacleGrid & grid, double minimumHeight, double maximumHeight)
{
  const auto & geometry = grid.geometry();
  const auto raisedCell = geometry.cellIndex(2.05, 0.05);
  for (std::size_t cellIndex = 0; cellIndex < geometry.cellCount(); ++cellIndex) {
    const double centerX = geometry.cellCenterX(cellIndex);
    const double centerY = geometry.cellCenterY(cellIndex);
    grid.addPoint3d(centerX, centerY, cellIndex == *raisedCell ? minimumHeight : 0.0);
    grid.addPoint3d(centerX, centerY, cellIndex == *raisedCell ? maximumHeight : 0.0);
  }
}
}  // namespace

TEST(ObstacleGridTest, FlatGroundHasNoOccupiedCell)
{
  auto grid = makeGrid({});
  addSlopedGround(grid, 0.0);
  EXPECT_EQ(countOccupied(grid.occupiedCells()), 0U);
}

TEST(ObstacleGridTest, TenDegreeSlopeIsNotOccupied)
{
  auto grid = makeGrid({});
  addSlopedGround(grid, 10.0);
  EXPECT_EQ(countOccupied(grid.occupiedCells()), 0U);
}

TEST(ObstacleGridTest, PersonSizedColumnIsOccupied)
{
  auto grid = makeGrid({});
  addSlopedGround(grid, 0.0);
  for (double z = 0.0; z <= 1.0; z += 0.05) {
    grid.addPoint3d(2.05, 0.05, z);
    grid.addPoint3d(2.06, 0.06, z);
  }
  const auto occupied = grid.occupiedCells();
  EXPECT_EQ(countOccupied(occupied), 1U);
  const auto index = grid.geometry().cellIndex(2.05, 0.05);
  ASSERT_TRUE(index.has_value());
  EXPECT_EQ(occupied[*index], 1);
}

TEST(ObstacleGridTest, TenCentimeterCurbIsNotOccupied)
{
  auto grid = makeGrid({});
  addSlopedGround(grid, 0.0);
  grid.addPoint3d(3.05, 0.05, 0.10);
  grid.addPoint3d(3.06, 0.06, 0.10);
  EXPECT_EQ(countOccupied(grid.occupiedCells()), 0U);
}

TEST(ObstacleGridTest, RaisedCellWithOnlyTopPointsAndSurroundingGroundIsOccupied)
{
  const ObstacleMonitorParameters parameters;
  ASSERT_LT(0.30 - 0.25, parameters.heightDifferenceThresholdMeters);
  auto grid = makeGrid(parameters);
  addRaisedSurfaceWithGround(grid, 0.25, 0.30);
  const auto occupied = grid.occupiedCells();
  EXPECT_EQ(occupied[*grid.geometry().cellIndex(2.05, 0.05)], 1);
  EXPECT_EQ(countOccupied(occupied), 1U);

  auto disabledParameters = parameters;
  disabledParameters.stepNeighborRadiusMeters = 0.0;
  auto disabledGrid = makeGrid(disabledParameters);
  addRaisedSurfaceWithGround(disabledGrid, 0.25, 0.30);
  EXPECT_EQ(countOccupied(disabledGrid.occupiedCells()), 0U);
}

TEST(ObstacleGridTest, TenDegreeSlopeWithMultiplePointsInEveryCellIsNotOccupied)
{
  auto grid = makeGrid({});
  const auto & geometry = grid.geometry();
  const double tangent = std::tan(10.0 * kDegreesToRadians);
  for (std::size_t cellIndex = 0; cellIndex < geometry.cellCount(); ++cellIndex) {
    const double centerX = geometry.cellCenterX(cellIndex);
    const double centerY = geometry.cellCenterY(cellIndex);
    grid.addPoint3d(centerX - 0.01, centerY, (centerX - 0.01) * tangent);
    grid.addPoint3d(centerX + 0.01, centerY, (centerX + 0.01) * tangent);
  }
  EXPECT_EQ(countOccupied(grid.occupiedCells()), 0U);
}

TEST(ObstacleGridTest, TenDegreeSlopeAtCellEdgesAndNeighborRadiusBoundaryIsNotOccupied)
{
  const ObstacleMonitorParameters parameters;
  auto grid = makeGrid(parameters);
  const auto & geometry = grid.geometry();
  const auto targetIndex = *geometry.cellIndex(2.05, 0.05);
  const auto neighborIndex = *geometry.cellIndex(1.55, 0.05);
  ASSERT_NEAR(
    geometry.cellCenterX(targetIndex) - geometry.cellCenterX(neighborIndex),
    parameters.stepNeighborRadiusMeters, 1e-12);
  const double tangent = std::tan(10.0 * kDegreesToRadians);
  // Stay just inside each cell so floating-point rounding cannot move a point across its edge.
  const double edgeOffset = parameters.cellSizeMeters / 2.0 - 1e-8;
  for (const auto cellIndex : {targetIndex, neighborIndex}) {
    for (const double offset : {-edgeOffset, edgeOffset}) {
      const double pointX = geometry.cellCenterX(cellIndex) + offset;
      ASSERT_EQ(geometry.cellIndex(pointX, 0.05), cellIndex);
      grid.addPoint3d(pointX, 0.05, pointX * tangent);
    }
  }
  const double maximumHeightDifference =
    (parameters.stepNeighborRadiusMeters + 2.0 * edgeOffset) * tangent;
  ASSERT_NEAR(maximumHeightDifference, 0.6 * tangent, 1e-8);
  ASSERT_LT(maximumHeightDifference, parameters.heightDifferenceThresholdMeters);
  EXPECT_EQ(countOccupied(grid.occupiedCells()), 0U);
}

TEST(ObstacleGridTest, SingleTopPointWithSufficientNeighborGroundIsNotOccupied)
{
  auto grid = makeGrid({});
  grid.addPoint3d(2.05, 0.05, 0.30);
  grid.addPoint3d(2.15, 0.05, 0.0);
  grid.addPoint3d(2.16, 0.05, 0.0);
  EXPECT_EQ(countOccupied(grid.occupiedCells()), 0U);
}

TEST(ObstacleGridTest, TenCentimeterSurfaceWithoutGroundPointsIsNotOccupied)
{
  auto grid = makeGrid({});
  addRaisedSurfaceWithGround(grid, 0.10, 0.10);
  EXPECT_EQ(countOccupied(grid.occupiedCells()), 0U);
}

TEST(ObstacleGridTest, IsolatedSurfaceAndInsufficientGroundSamplesAreNotOccupied)
{
  auto grid = makeGrid({});
  grid.addPoint3d(2.05, 0.05, 0.25);
  grid.addPoint3d(2.05, 0.05, 0.30);
  EXPECT_EQ(countOccupied(grid.occupiedCells()), 0U);
  grid.addPoint3d(2.15, 0.05, 0.0);
  EXPECT_EQ(countOccupied(grid.occupiedCells()), 0U);
}

TEST(ObstacleGridTest, GroundOutsideCircularNeighborRadiusDoesNotOccupySurface)
{
  auto grid = makeGrid({});
  grid.addPoint3d(2.05, 0.05, 0.25);
  grid.addPoint3d(2.05, 0.05, 0.30);
  grid.addPoint3d(2.65, 0.05, 0.0);
  grid.addPoint3d(2.65, 0.05, 0.0);
  grid.addPoint3d(2.45, 0.45, 0.0);
  grid.addPoint3d(2.45, 0.45, 0.0);
  EXPECT_EQ(countOccupied(grid.occupiedCells()), 0U);
}

TEST(ObstacleGridTest, GroundOnNeighborRadiusBoundaryOccupiesSurface)
{
  auto grid = makeGrid({});
  grid.addPoint3d(2.05, 0.05, 0.25);
  grid.addPoint3d(2.05, 0.05, 0.30);
  grid.addPoint3d(2.35, 0.45, 0.0);
  grid.addPoint3d(2.35, 0.45, 0.0);
  const auto occupied = grid.occupiedCells();
  EXPECT_EQ(occupied[*grid.geometry().cellIndex(2.05, 0.05)], 1);
  EXPECT_EQ(countOccupied(occupied), 1U);
}

TEST(ObstacleGridTest, PointsAboveZMaxAreDropped)
{
  auto grid = makeGrid({});
  grid.addPoint3d(1.05, 0.05, 0.0);
  grid.addPoint3d(1.05, 0.05, 1.6);
  grid.addPoint3d(1.06, 0.06, 2.5);
  EXPECT_EQ(countOccupied(grid.occupiedCells()), 0U);
}

TEST(ObstacleGridTest, SinglePointCellIsIgnored)
{
  auto grid = makeGrid({});
  grid.addPoint3d(1.05, 0.05, 0.5);
  EXPECT_EQ(countOccupied(grid.occupiedCells()), 0U);
}

TEST(ObstacleGridTest, NonFinitePointIsIgnored)
{
  auto grid = makeGrid({});
  const double notANumber = std::numeric_limits<double>::quiet_NaN();
  grid.addPoint3d(notANumber, 0.0, 0.0);
  grid.addPoint3d(1.0, notANumber, 0.0);
  grid.addPoint3d(1.0, 0.0, notANumber);
  EXPECT_EQ(countOccupied(grid.occupiedCells()), 0U);
}

TEST(ObstacleGridTest, ScanHitOccupiesWithoutHeightInformation)
{
  auto grid = makeGrid({});
  grid.addScanHit(1.5, -0.5);
  EXPECT_EQ(countOccupied(grid.occupiedCells()), 1U);
}

TEST(ObstacleGridTest, PointsOutsideGridAreIgnored)
{
  auto grid = makeGrid({});
  grid.addScanHit(7.0, 0.0);
  grid.addScanHit(0.0, 4.5);
  grid.addScanHit(-2.5, 0.0);
  EXPECT_EQ(countOccupied(grid.occupiedCells()), 0U);
}

TEST(InflationTest, SingleCellBecomesDisc)
{
  const ObstacleMonitorParameters parameters;
  const auto geometry = GridGeometry::fromParameters(parameters);
  std::vector<std::uint8_t> occupied(geometry.cellCount(), 0);
  occupied[*geometry.cellIndex(2.05, 0.05)] = 1;
  const auto inflated = inflateOccupiedCells(occupied, geometry, 0.5);
  EXPECT_EQ(inflated[*geometry.cellIndex(2.55, 0.05)], 1);
  EXPECT_EQ(inflated[*geometry.cellIndex(1.55, 0.05)], 1);
  EXPECT_EQ(inflated[*geometry.cellIndex(2.65, 0.05)], 0);
  EXPECT_EQ(inflated[*geometry.cellIndex(2.45, 0.45)], 0);  // corner is outside the disc
  EXPECT_GT(countOccupied(inflated), 60U);
}

TEST(InflationTest, EdgeCellDoesNotWrapAround)
{
  const ObstacleMonitorParameters parameters;
  const auto geometry = GridGeometry::fromParameters(parameters);
  std::vector<std::uint8_t> occupied(geometry.cellCount(), 0);
  occupied[*geometry.cellIndex(-1.95, 0.05)] = 1;
  const auto inflated = inflateOccupiedCells(occupied, geometry, 0.5);
  EXPECT_EQ(inflated[*geometry.cellIndex(5.95, 0.05)], 0);
}

TEST(ZoneTest, CountsCellsInsideTheZoneOnly)
{
  const ObstacleMonitorParameters parameters;
  const auto geometry = GridGeometry::fromParameters(parameters);
  std::vector<std::uint8_t> occupied(geometry.cellCount(), 0);
  occupied[*geometry.cellIndex(0.55, 0.05)] = 1;   // stop zone
  occupied[*geometry.cellIndex(1.55, 0.05)] = 1;   // slow zone only
  occupied[*geometry.cellIndex(0.55, 0.55)] = 1;   // slow zone only (beyond stop half width)
  occupied[*geometry.cellIndex(-0.55, 0.05)] = 1;  // behind the robot
  EXPECT_EQ(countOccupiedCellsInZone(occupied, geometry, 1.0, 0.4), 1U);
  EXPECT_EQ(countOccupiedCellsInZone(occupied, geometry, 2.5, 0.6), 3U);
}

TEST(ZoneTest, PartlyOverlappingCellCountsAsInside)
{
  const auto geometry = GridGeometry::fromParameters({});
  std::vector<std::uint8_t> occupied(geometry.cellCount(), 0);
  // Cell y in [0.4, 0.5) overlaps a zone with half width 0.43 although its center is at 0.45.
  occupied[*geometry.cellIndex(0.55, 0.45)] = 1;
  EXPECT_EQ(countOccupiedCellsInZone(occupied, geometry, 1.0, 0.43), 1U);
  EXPECT_EQ(countOccupiedCellsInZone(occupied, geometry, 1.0, 0.4), 0U);
}

TEST(ZoneTest, NearestDistanceIsInfiniteWithoutObstacles)
{
  const auto geometry = GridGeometry::fromParameters({});
  const std::vector<std::uint8_t> occupied(geometry.cellCount(), 0);
  EXPECT_TRUE(std::isinf(nearestOccupiedDistance(occupied, geometry)));
}

TEST(ZoneTest, NearestDistanceUsesCellCenter)
{
  const auto geometry = GridGeometry::fromParameters({});
  std::vector<std::uint8_t> occupied(geometry.cellCount(), 0);
  occupied[*geometry.cellIndex(2.05, 0.05)] = 1;
  EXPECT_NEAR(nearestOccupiedDistance(occupied, geometry), 2.05F, 1e-3F);
}

TEST(ZoneHysteresisTest, NeedsConsecutiveFramesToOccupy)
{
  ZoneHysteresis hysteresis(2, 2, 1.0);
  EXPECT_FALSE(hysteresis.update(5, 0.0));
  EXPECT_FALSE(hysteresis.update(1, 0.1));  // below min_cell_count resets the streak
  EXPECT_FALSE(hysteresis.update(5, 0.2));
  EXPECT_TRUE(hysteresis.update(5, 0.3));
}

TEST(ZoneHysteresisTest, HoldsUntilClearHoldElapsed)
{
  ZoneHysteresis hysteresis(2, 2, 1.0);
  hysteresis.update(5, 0.0);
  ASSERT_TRUE(hysteresis.update(5, 0.1));
  EXPECT_TRUE(hysteresis.update(0, 0.6));
  EXPECT_TRUE(hysteresis.update(0, 1.09));
  EXPECT_FALSE(hysteresis.update(0, 1.1));
}

TEST(ZoneHysteresisTest, ReappearanceRestartsHold)
{
  ZoneHysteresis hysteresis(2, 2, 1.0);
  hysteresis.update(5, 0.0);
  hysteresis.update(5, 0.1);
  hysteresis.update(0, 0.9);
  hysteresis.update(5, 1.0);
  EXPECT_TRUE(hysteresis.update(0, 1.5));
  EXPECT_FALSE(hysteresis.update(0, 2.0));
}

}  // namespace lightnav
