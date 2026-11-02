#pragma once

#include "lightnav_controller/types.hpp"

namespace lightnav_controller {

struct ArbitrationInput {
  LayerOutput avoid;
  LayerOutput follow;
  bool is_pose_fresh{false};
  bool is_grid_fresh{false};
};

struct ArbitrationResult {
  Twist2D twist{};
  bool is_hard_stop{true};
  Layer source_layer{Layer::kNone};
  StopReason reason{StopReason::kNoPath};
};

// Priority 1 (avoid) suppresses priority 2 (follow). Priority 0 (emergency
// stop) lives in safety_gate, outside this process.
ArbitrationResult arbitrate(const ArbitrationInput& input);

}  // namespace lightnav_controller
