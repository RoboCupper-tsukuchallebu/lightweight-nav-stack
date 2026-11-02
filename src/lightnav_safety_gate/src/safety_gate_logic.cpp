#include "lightnav_safety_gate/safety_gate_logic.hpp"

#include <algorithm>
#include <cmath>

namespace lightnav_safety_gate
{

namespace
{

bool isFreshWithin(bool has_message, double age_s, double timeout_s)
{
  // Negated comparison so that a NaN age counts as stale.
  return has_message && (age_s >= 0.0) && (age_s <= timeout_s);
}

double limitAxis(
  double previous, double target, double acceleration_max, double deceleration_max,
  double period_s)
{
  if (target == 0.0) {
    return 0.0;
  }
  const bool is_reversing = (previous * target) < 0.0;
  const bool is_speeding_up = !is_reversing && (std::abs(target) > std::abs(previous));
  const double step_limit = (is_speeding_up ? acceleration_max : deceleration_max) * period_s;
  const double effective_target = is_reversing ? 0.0 : target;
  const double delta = std::clamp(effective_target - previous, -step_limit, step_limit);
  return previous + delta;
}

}  // namespace

bool isFinite(const PlanarVelocity & velocity)
{
  return std::isfinite(velocity.linear_mps) && std::isfinite(velocity.angular_rps);
}

bool shouldEmergencyStop(
  const EmergencyStopInputs & inputs, const EmergencyStopTimeouts & timeouts)
{
  return !inputs.is_active ||
         !isFreshWithin(
           inputs.has_obstacle_status, inputs.obstacle_status_age_s,
           timeouts.obstacle_status_timeout_s) ||
         inputs.is_stop_zone_occupied || inputs.is_input_stale ||
         !isFreshWithin(inputs.has_command, inputs.command_age_s, timeouts.command_timeout_s) ||
         inputs.is_hard_stop;
}

PlanarVelocity saturate(const PlanarVelocity & command, const SafetyGateLimits & limits)
{
  return {
    std::clamp(command.linear_mps, -limits.linear_max_mps, limits.linear_max_mps),
    std::clamp(command.angular_rps, -limits.angular_max_rps, limits.angular_max_rps)};
}

PlanarVelocity limitRate(
  const PlanarVelocity & previous_output, const PlanarVelocity & target,
  const SafetyGateLimits & limits, double period_s)
{
  return {
    limitAxis(
      previous_output.linear_mps, target.linear_mps, limits.acceleration_max_mps2,
      limits.deceleration_max_mps2, period_s),
    limitAxis(
      previous_output.angular_rps, target.angular_rps, limits.angular_acceleration_max_rps2,
      limits.angular_deceleration_max_rps2, period_s)};
}

bool areLimitsValid(const SafetyGateLimits & limits)
{
  const double values[] = {
    limits.linear_max_mps, limits.angular_max_rps, limits.acceleration_max_mps2,
    limits.deceleration_max_mps2, limits.angular_acceleration_max_rps2,
    limits.angular_deceleration_max_rps2};
  return std::all_of(std::begin(values), std::end(values), [](double value) {
           return std::isfinite(value) && value > 0.0;
         });
}

}  // namespace lightnav_safety_gate
