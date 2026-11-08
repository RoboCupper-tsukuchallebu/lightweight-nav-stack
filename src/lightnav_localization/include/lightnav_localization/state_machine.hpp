#pragma once

namespace lightnav_localization {

enum class LocalizationState { kInitializing, kTracking, kGpsOnly, kOdomOnly, kLost, kInputStale };

struct StateMachineParameters {
  double tracking_window_s = 5.0;
  double lost_position_std_m = 2.0;
  double odom_only_max_distance_m = 50.0;
  double odom_only_max_s = 60.0;
  double wheel_timeout_s = 0.2;
  double scan_timeout_s = 0.5;
  double gps_timeout_s = 3.0;
};

struct StateInputs {
  double now_sec = 0.0;
  double last_wheel_sec = 0.0;
  double last_scan_sec = 0.0;
  double last_map_match_sec = -1e9;  // accepted prior-map match
  double last_gps_sec = -1e9;        // accepted GPS factor
  double position_std_m = 0.0;
  double odom_only_distance_m = 0.0;  // distance since the last map match or GPS
  double odom_only_duration_s = 0.0;
  bool requires_scans = true;
  bool allow_uncertain_map_tracking = false;  // temporary localizer policy pending field trials
};

// LOST latches until reset(); output is withheld in INITIALIZING, LOST and INPUT_STALE.
class StateMachine {
 public:
  explicit StateMachine(const StateMachineParameters& parameters) : parameters_(parameters) {}

  void start_tracking() { state_ = LocalizationState::kOdomOnly; is_started_ = true; }
  void reset() { state_ = LocalizationState::kInitializing; is_started_ = false; }

  LocalizationState update(const StateInputs& inputs) {
    if (!is_started_ || state_ == LocalizationState::kLost) return state_;
    if (inputs.now_sec - inputs.last_wheel_sec > parameters_.wheel_timeout_s ||
        (inputs.requires_scans && inputs.now_sec - inputs.last_scan_sec > parameters_.scan_timeout_s)) {
      return state_ = LocalizationState::kInputStale;
    }
    const bool has_map = inputs.now_sec - inputs.last_map_match_sec <= parameters_.tracking_window_s;
    if (inputs.position_std_m > parameters_.lost_position_std_m &&
        !(inputs.allow_uncertain_map_tracking && has_map)) {
      return state_ = LocalizationState::kLost;
    }
    const bool has_gps = inputs.now_sec - inputs.last_gps_sec <= parameters_.gps_timeout_s;
    if (has_map) return state_ = LocalizationState::kTracking;
    if (has_gps) return state_ = LocalizationState::kGpsOnly;
    const bool is_over_limit = inputs.odom_only_distance_m > parameters_.odom_only_max_distance_m ||
                               inputs.odom_only_duration_s > parameters_.odom_only_max_s;
    return state_ = is_over_limit ? LocalizationState::kLost : LocalizationState::kOdomOnly;
  }

  LocalizationState state() const { return state_; }
  bool should_publish_odom() const {
    return state_ == LocalizationState::kTracking || state_ == LocalizationState::kGpsOnly ||
           state_ == LocalizationState::kOdomOnly;
  }

 private:
  StateMachineParameters parameters_;
  LocalizationState state_ = LocalizationState::kInitializing;
  bool is_started_ = false;
};

}  // namespace lightnav_localization
