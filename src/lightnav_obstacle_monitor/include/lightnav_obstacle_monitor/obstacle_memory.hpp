#pragma once

#include <Eigen/Geometry>
#include <map>
#include <optional>
#include <utility>
#include <vector>

#include "lightnav_obstacle_monitor/obstacle_grid.hpp"

namespace lightnav
{
// World-quantized cells. Only directly observed ground clears a cell; missing returns never do.
class ObstacleMemory
{
public:
  ObstacleMemory(const GridGeometry & geometry, double blindRadiusMeters, double maxAgeSeconds,
    double jumpDistanceMeters, double jumpAngleRadians, double maxRangeMeters = 10.0);
  bool update(const std::vector<std::uint8_t> & occupied, const std::vector<std::uint8_t> & ground,
    const Eigen::Isometry3d & worldFromBase, const Eigen::Vector3d & sensorInBase, double nowSeconds);
  std::vector<std::uint8_t> cells(const Eigen::Isometry3d & worldFromBase, double nowSeconds);
  void updateSafetyParameters(double blindRadiusMeters, double maxAgeSeconds);
  std::size_t size() const {return cells_.size();}
private:
  using Key = std::pair<long long, long long>;
  Key key(double x, double y) const;
  GridGeometry geometry_;
  double blindRadiusMeters_, maxAgeSeconds_, jumpDistanceMeters_, jumpAngleRadians_, maxRangeMeters_;
  std::map<Key, double> cells_;
  std::optional<Eigen::Isometry3d> previousPose_;
};
}  // namespace lightnav
