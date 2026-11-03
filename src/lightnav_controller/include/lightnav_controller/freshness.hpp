#pragma once

#include <optional>

namespace lightnav_controller {

struct InputAges {
  std::optional<double> pose_age_s;             // nullopt: never received
  std::optional<double> obstacle_status_age_s;  // nullopt: never received
  std::optional<double> grid_age_s;             // nullopt: never received
  bool is_obstacle_status_input_stale{false};   // ObstacleStatus.is_input_stale
};

struct FreshnessTimeouts {
  double pose_timeout_s{0.5};
  double obstacle_status_timeout_s{0.5};
  double grid_timeout_s{0.5};
};

struct Freshness {
  bool is_pose_fresh{false};
  bool is_obstacle_status_fresh{false};
  bool is_grid_fresh{false};
};

// Never received, negative age (clock went backwards), non-finite age, or age
// beyond the timeout all count as stale.
Freshness evaluateFreshness(const InputAges& ages, const FreshnessTimeouts& timeouts);

}  // namespace lightnav_controller
