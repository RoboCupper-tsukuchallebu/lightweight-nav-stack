#ifndef LIGHTNAV_SAFETY_GATE__SAFETY_GATE_LOGIC_HPP_
#define LIGHTNAV_SAFETY_GATE__SAFETY_GATE_LOGIC_HPP_

namespace lightnav_safety_gate
{

struct PlanarVelocity
{
  double linear_mps{0.0};
  double angular_rps{0.0};
};

struct SafetyGateLimits
{
  double linear_max_mps{0.0};
  double angular_max_rps{0.0};
  double acceleration_max_mps2{0.0};
  double deceleration_max_mps2{0.0};
  double angular_acceleration_max_rps2{0.0};
  double angular_deceleration_max_rps2{0.0};
};

// Snapshot of everything the priority-0 emergency stop looks at.
// Ages are seconds since the message was received; "has_*" is false until the first message.
struct EmergencyStopInputs
{
  bool is_active{false};
  bool has_obstacle_status{false};
  double obstacle_status_age_s{0.0};
  bool is_stop_zone_occupied{false};
  bool is_input_stale{false};
  bool has_command{false};
  double command_age_s{0.0};
  bool is_hard_stop{false};
};

struct EmergencyStopTimeouts
{
  double command_timeout_s{0.0};
  double obstacle_status_timeout_s{0.0};
};

bool isFinite(const PlanarVelocity & velocity);

// Any doubt (missing input, NaN age, timeout, not active) resolves to "stop".
bool shouldEmergencyStop(const EmergencyStopInputs & inputs, const EmergencyStopTimeouts & timeouts);

PlanarVelocity saturate(const PlanarVelocity & command, const SafetyGateLimits & limits);

// Rate limiting applies unless the whole target is zero, which is returned immediately.
// A target with one zero axis (e.g. rotate in place) is still rate limited per axis.
// A sign reversal first decelerates toward zero.
PlanarVelocity limitRate(
  const PlanarVelocity & previous_output, const PlanarVelocity & target,
  const SafetyGateLimits & limits, double period_s);

bool areLimitsValid(const SafetyGateLimits & limits);

}  // namespace lightnav_safety_gate

#endif  // LIGHTNAV_SAFETY_GATE__SAFETY_GATE_LOGIC_HPP_
