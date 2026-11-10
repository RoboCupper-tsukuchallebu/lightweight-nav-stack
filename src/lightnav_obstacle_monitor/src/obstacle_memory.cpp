#include "lightnav_obstacle_monitor/obstacle_memory.hpp"

#include <cmath>

namespace lightnav
{
ObstacleMemory::ObstacleMemory(const GridGeometry & geometry, double blindRadiusMeters,
  double maxAgeSeconds, double jumpDistanceMeters, double jumpAngleRadians, double maxRangeMeters)
: geometry_(geometry), blindRadiusMeters_(blindRadiusMeters), maxAgeSeconds_(maxAgeSeconds),
  jumpDistanceMeters_(jumpDistanceMeters), jumpAngleRadians_(jumpAngleRadians), maxRangeMeters_(maxRangeMeters) {}

void ObstacleMemory::updateSafetyParameters(double blindRadiusMeters, double maxAgeSeconds)
{
  blindRadiusMeters_ = blindRadiusMeters;
  maxAgeSeconds_ = maxAgeSeconds;
}

ObstacleMemory::Key ObstacleMemory::key(double x, double y) const
{
  return {static_cast<long long>(std::floor(x / geometry_.cellSizeMeters)),
    static_cast<long long>(std::floor(y / geometry_.cellSizeMeters))};
}

bool ObstacleMemory::update(const std::vector<std::uint8_t> & occupied,
  const std::vector<std::uint8_t> & ground, const Eigen::Isometry3d & worldFromBase,
  const Eigen::Vector3d & sensorInBase, double nowSeconds)
{
  bool jumped = false;
  if (previousPose_) {
    const auto delta = previousPose_->inverse() * worldFromBase;
    jumped = delta.translation().norm() > jumpDistanceMeters_ ||
      Eigen::AngleAxisd(delta.rotation()).angle() > jumpAngleRadians_;
    if (jumped) {cells_.clear();}
  }
  previousPose_ = worldFromBase;
  const auto baseFromWorld = worldFromBase.inverse();
  for (auto iterator = cells_.begin(); iterator != cells_.end();) {
    const Eigen::Vector3d world((iterator->first.first + 0.5) * geometry_.cellSizeMeters,
      (iterator->first.second + 0.5) * geometry_.cellSizeMeters, 0.0);
    const Eigen::Vector3d base = baseFromWorld * world;
    const auto index = geometry_.cellIndex(base.x(), base.y());
    const double distance = std::hypot(base.x() - sensorInBase.x(), base.y() - sensorInBase.y());
    const bool observedFree = index && *index < ground.size() && ground[*index] &&
      !occupied[*index] && distance >= blindRadiusMeters_;
    const bool observable = index && *index < ground.size() && distance >= blindRadiusMeters_ &&
      (ground[*index] || occupied[*index]);
    const bool expired = observable && (nowSeconds - iterator->second >= maxAgeSeconds_ || nowSeconds < iterator->second);
    if (std::hypot(base.x(), base.y()) > maxRangeMeters_ || expired || observedFree) {
      iterator = cells_.erase(iterator);
    } else {++iterator;}
  }
  for (std::size_t index = 0; index < occupied.size(); ++index) {
    if (!occupied[index]) {continue;}
    const Eigen::Vector3d world = worldFromBase * Eigen::Vector3d(
      geometry_.cellCenterX(index), geometry_.cellCenterY(index), 0.0);
    if (std::hypot(geometry_.cellCenterX(index), geometry_.cellCenterY(index)) <= maxRangeMeters_) {
      cells_[key(world.x(), world.y())] = nowSeconds;
    }
  }
  return jumped;
}

std::vector<std::uint8_t> ObstacleMemory::cells(const Eigen::Isometry3d & worldFromBase, double nowSeconds)
{
  (void)nowSeconds;  // Expiry requires current observation and is performed in update().
  std::vector<std::uint8_t> result(geometry_.cellCount(), 0);
  const auto baseFromWorld = worldFromBase.inverse();
  for (auto iterator = cells_.begin(); iterator != cells_.end();) {
    const Eigen::Vector3d world((iterator->first.first + 0.5) * geometry_.cellSizeMeters,
      (iterator->first.second + 0.5) * geometry_.cellSizeMeters, 0.0);
    const auto base = baseFromWorld * world;
    const auto index = geometry_.cellIndex(base.x(), base.y());
    if (index) {result[*index] = 1;}
    ++iterator;
  }
  return result;
}
}  // namespace lightnav
