#pragma once

#include <cstddef>
#include <vector>

#include "lightnav_controller/types.hpp"

namespace lightnav_controller {

struct AvoidContext {
  Pose2D pose;
  const std::vector<Point2D>& path_points;
  std::size_t closest_index{0};
  bool is_grid_fresh{false};
};

// Priority 1 layer. The real implementation (candidate arcs) arrives in PR4.
class AvoidLayer {
 public:
  AvoidLayer() = default;
  AvoidLayer(const AvoidLayer&) = delete;
  AvoidLayer& operator=(const AvoidLayer&) = delete;
  virtual ~AvoidLayer() = default;

  virtual LayerOutput compute(const AvoidContext& context) = 0;
};

// Never asserts, so the follow layer always wins until PR4.
class NullAvoidLayer final : public AvoidLayer {
 public:
  LayerOutput compute(const AvoidContext&) override { return LayerOutput{}; }
};

}  // namespace lightnav_controller
