#pragma once

#include <cstdint>

namespace lightnav_controller {

struct Point2D {
  double x_m{0.0};
  double y_m{0.0};
};

struct Pose2D {
  double x_m{0.0};
  double y_m{0.0};
  double yaw_rad{0.0};
};

struct Twist2D {
  double linear_mps{0.0};
  double angular_rps{0.0};
};

// Values match FollowStatus.state.
enum class StopReason : std::uint8_t {
  kOk = 0,
  kNoPath = 1,
  kPoseStale = 2,
  kObstacleStale = 3,
  kBlocked = 4,
  kAtPathEnd = 5,
  kStuck = 6,
};

// Values match FollowStatus.active_layer.
enum class Layer : std::uint8_t {
  kFollow = 0,
  kAvoid = 1,
  kNone = 2,
};

struct LayerOutput {
  bool is_asserted{false};
  Twist2D twist{};
  StopReason reason{StopReason::kOk};
};

}  // namespace lightnav_controller
