#include "lightnav_controller/freshness.hpp"

#include <cmath>

namespace lightnav_controller {

namespace {

bool isWithinTimeout(const std::optional<double>& age_s, double timeout_s) {
  return age_s.has_value() && std::isfinite(*age_s) && *age_s >= 0.0 && *age_s <= timeout_s;
}

}  // namespace

Freshness evaluateFreshness(const InputAges& ages, const FreshnessTimeouts& timeouts) {
  Freshness freshness;
  freshness.is_pose_fresh = isWithinTimeout(ages.pose_age_s, timeouts.pose_timeout_s);
  freshness.is_obstacle_status_fresh =
      !ages.is_obstacle_status_input_stale &&
      isWithinTimeout(ages.obstacle_status_age_s, timeouts.obstacle_status_timeout_s);
  return freshness;
}

}  // namespace lightnav_controller
