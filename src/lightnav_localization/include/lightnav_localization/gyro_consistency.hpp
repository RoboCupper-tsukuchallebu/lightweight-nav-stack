#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <deque>
#include <optional>

namespace lightnav_localization {

// Whether the gyro yaw increments may replace the wheel yaw increments (Issue #92, section 3.3).
//   kUnverified           right after configuration; the gyro is not used until it agrees with LiDAR / wheels
//   kActive               verified; used for yaw replacement and rotation factors
//   kSuspended            wheels and LiDAR together outvoted the gyro; not used until verified again
//   kDisabledInconsistent repeatedly inconsistent; not used until the node is reconfigured
// This header is intentionally free of ROS and GTSAM.
enum class GyroConsistencyState { kUnverified, kActive, kSuspended, kDisabledInconsistent };

// Numbers of the consistency check (Issue #92, section 3.3 step 3). Constants, not parameters.
namespace gyro_consistency_constants {
constexpr double kAgreementFloorRad = 0.05;      // x agrees with b when |x - b| <= max(0.05, 0.2 |b|)
constexpr double kAgreementRatio = 0.2;
constexpr double kVerificationThresholdRad = 0.2;  // a check needs |W| or |L| above this
// Without LiDAR: |G| / |W| must be in this range (version 3.2; narrowed from 0.4 - 1.8 so that a gyro of
// twice the scale does not pass beside a wheel that over-reports by 1.4). May change with real recordings.
constexpr double kMinimumGyroToWheelRatio = 0.6;
constexpr double kMaximumGyroToWheelRatio = 1.2;
constexpr double kMaximumAccumulationSec = 20.0;
constexpr double kMinimumGyroCoverageRatio = 0.95;  // share of a keyframe interval the gyro must cover
constexpr std::size_t kMaximumRejectionsBeforeWarning = 3;
constexpr std::size_t kMaximumOutvotesBeforeDisabling = 3;
}  // namespace gyro_consistency_constants

// The yaw changes of one keyframe interval. `lidar_yaw_rad` exists only when the LiDAR match of that
// keyframe was accepted.
struct GyroIntervalObservation {
  double wheel_yaw_rad = 0.0;  // raw wheel yaw change (w)
  double gyro_yaw_rad = 0.0;   // gyro yaw change (g); meaningful when `is_gyro_covered`
  std::optional<double> lidar_yaw_rad;  // l
  double duration_sec = 0.0;
  bool is_gyro_covered = false;  // at least 95 % of the interval is covered by gyro increments
};

enum class GyroConsistencyEvent {
  kNone,
  kConfirmed,  // kUnverified / kSuspended -> kActive
  kRejected,   // a check failed (the state stays)
  kOutvoted,   // kActive -> kSuspended (or kDisabledInconsistent): wheel and LiDAR agree against the gyro
};

struct GyroConsistencyResult {
  GyroConsistencyEvent event = GyroConsistencyEvent::kNone;
  // For kConfirmed: the gyro yaw and the seconds of the whole verification span (first accumulated
  // interval until now), over which one rotation factor is added.
  double verified_gyro_yaw_rad = 0.0;
  double verified_duration_sec = 0.0;
};

struct GyroVerificationSums {
  double wheel_rad = 0.0;           // W: all intervals
  double gyro_rad = 0.0;            // G: all intervals
  double lidar_rad = 0.0;           // L: intervals with a LiDAR yaw
  double gyro_on_lidar_rad = 0.0;   // G_l: the gyro over the same intervals
  bool has_interval_without_lidar = false;
  double duration_sec = 0.0;
};

// Decides when the gyro may be used. Fed once per keyframe interval; knows nothing about ROS or GTSAM.
class GyroConsistencyJudge {
 public:
  // `initial_outvoted_total`: the losses counted before (carried over when a Localizer is rebuilt). Three in
  // total disable the gyro, so a start with three or more is disabled whatever state was asked for.
  explicit GyroConsistencyJudge(GyroConsistencyState initial_state = GyroConsistencyState::kUnverified,
                                std::size_t initial_outvoted_total = 0)
      : state_(initial_outvoted_total >= gyro_consistency_constants::kMaximumOutvotesBeforeDisabling
                   ? GyroConsistencyState::kDisabledInconsistent
                   : initial_state),
        outvoted_total_(initial_outvoted_total) {}

  GyroConsistencyState state() const { return state_; }
  std::size_t confirmed_total() const { return confirmed_total_; }
  std::size_t outvoted_total() const { return outvoted_total_; }
  std::size_t rejected_total() const { return rejected_total_; }
  // Still kUnverified although the check failed at least three times.
  bool is_inconsistent_warning() const {
    return state_ == GyroConsistencyState::kUnverified &&
           rejected_total_ >= gyro_consistency_constants::kMaximumRejectionsBeforeWarning;
  }
  const GyroVerificationSums& sums() const { return sums_; }
  // True while a verification span has started (some interval was accumulated).
  bool is_accumulating() const { return sums_.duration_sec > 0.0; }
  // Starts the verification span over (the window was rebuilt, for example).
  void restart_accumulation() { sums_ = GyroVerificationSums(); }

  GyroConsistencyResult observe_interval(const GyroIntervalObservation& interval) {
    switch (state_) {
      case GyroConsistencyState::kDisabledInconsistent:
        return {};
      case GyroConsistencyState::kActive:
        return observe_while_active(interval);
      case GyroConsistencyState::kUnverified:
      case GyroConsistencyState::kSuspended:
        return observe_while_verifying(interval);
    }
    return {};
  }

 private:
  static bool agrees(double value, double reference) {
    using namespace gyro_consistency_constants;
    return std::abs(value - reference) <= std::max(kAgreementFloorRad, kAgreementRatio * std::abs(reference));
  }

  GyroConsistencyResult observe_while_active(const GyroIntervalObservation& interval) {
    if (!is_outvoted(interval)) return {};
    ++outvoted_total_;
    state_ = outvoted_total_ >= gyro_consistency_constants::kMaximumOutvotesBeforeDisabling
                 ? GyroConsistencyState::kDisabledInconsistent
                 : GyroConsistencyState::kSuspended;
    restart_accumulation();
    return {GyroConsistencyEvent::kOutvoted, 0.0, 0.0};
  }

  // Wheel and LiDAR agree and the gyro does not (the vote against the gyro).
  static bool is_outvoted(const GyroIntervalObservation& interval) {
    return interval.is_gyro_covered && interval.lidar_yaw_rad &&
           agrees(interval.wheel_yaw_rad, *interval.lidar_yaw_rad) &&
           !agrees(interval.gyro_yaw_rad, *interval.lidar_yaw_rad);
  }

  GyroConsistencyResult observe_while_verifying(const GyroIntervalObservation& interval) {
    using namespace gyro_consistency_constants;
    // A suspended gyro keeps being voted on: the third loss in total disables it for good.
    if (state_ == GyroConsistencyState::kSuspended && is_outvoted(interval)) {
      ++outvoted_total_;
      if (outvoted_total_ >= kMaximumOutvotesBeforeDisabling) state_ = GyroConsistencyState::kDisabledInconsistent;
      restart_accumulation();
      return {GyroConsistencyEvent::kOutvoted, 0.0, 0.0};
    }
    if (!interval.is_gyro_covered) {
      restart_accumulation();
      return {};
    }
    sums_.wheel_rad += interval.wheel_yaw_rad;
    sums_.gyro_rad += interval.gyro_yaw_rad;
    sums_.duration_sec += interval.duration_sec;
    if (interval.lidar_yaw_rad) {
      sums_.lidar_rad += *interval.lidar_yaw_rad;
      sums_.gyro_on_lidar_rad += interval.gyro_yaw_rad;
    } else {
      sums_.has_interval_without_lidar = true;
    }
    const bool is_checkable = std::abs(sums_.wheel_rad) > kVerificationThresholdRad ||
                              std::abs(sums_.lidar_rad) > kVerificationThresholdRad;
    if (is_checkable) {
      const bool does_gyro_agree_with_lidar = agrees(sums_.gyro_on_lidar_rad, sums_.lidar_rad);
      const bool is_confirmed_by_lidar =
          does_gyro_agree_with_lidar && std::abs(sums_.lidar_rad) > kVerificationThresholdRad;
      const bool is_confirmed_by_wheel = does_gyro_agree_with_lidar && sums_.has_interval_without_lidar &&
                                         std::abs(sums_.wheel_rad) > kVerificationThresholdRad &&
                                         is_gyro_roughly_matching_wheel();
      // After the gyro was suspended only the LiDAR may bring it back.
      const bool is_confirmed =
          state_ == GyroConsistencyState::kSuspended ? is_confirmed_by_lidar
                                                     : (is_confirmed_by_lidar || is_confirmed_by_wheel);
      if (is_confirmed) return confirm();
      const bool is_rejected =
          !does_gyro_agree_with_lidar ||
          (sums_.has_interval_without_lidar && std::abs(sums_.wheel_rad) > kVerificationThresholdRad &&
           !is_confirmed_by_lidar && !is_confirmed_by_wheel);
      if (is_rejected) {
        ++rejected_total_;
        restart_accumulation();
        return {GyroConsistencyEvent::kRejected, 0.0, 0.0};
      }
    }
    if (sums_.duration_sec > kMaximumAccumulationSec) restart_accumulation();
    return {};
  }

  bool is_gyro_roughly_matching_wheel() const {
    using namespace gyro_consistency_constants;
    const bool has_same_sign = (sums_.gyro_rad > 0.0) == (sums_.wheel_rad > 0.0) && sums_.gyro_rad != 0.0;
    const double ratio = std::abs(sums_.gyro_rad) / std::abs(sums_.wheel_rad);
    return has_same_sign && ratio >= kMinimumGyroToWheelRatio && ratio <= kMaximumGyroToWheelRatio;
  }

  GyroConsistencyResult confirm() {
    const GyroConsistencyResult result{GyroConsistencyEvent::kConfirmed, sums_.gyro_rad, sums_.duration_sec};
    ++confirmed_total_;
    state_ = GyroConsistencyState::kActive;
    restart_accumulation();
    return result;
  }

  GyroConsistencyState state_;
  GyroVerificationSums sums_;
  std::size_t confirmed_total_ = 0;
  std::size_t outvoted_total_ = 0;
  std::size_t rejected_total_ = 0;
};

// What one wheel sample contributed to the yaw: `is_covered` when a gyro increment came with it. The sample
// covers the time (stamp_sec - interval_sec, stamp_sec].
struct WheelYawSample {
  double stamp_sec;
  double interval_sec;
  bool is_covered;
  double gyro_yaw_rad;  // meaningful when covered
  double raw_wheel_yaw_rad;
};

// Sums over the wheel samples of one keyframe interval.
struct WheelYawInterval {
  double total_sec = 0.0;
  double covered_sec = 0.0;
  double gyro_yaw_sum_rad = 0.0;
  double uncovered_wheel_yaw_sum_rad = 0.0;
  double raw_wheel_yaw_sum_rad = 0.0;
  bool is_sufficiently_covered() const {
    return total_sec > 0.0 &&
           covered_sec >= gyro_consistency_constants::kMinimumGyroCoverageRatio * total_sec;
  }
  // Gyro yaw over the covered part plus the raw wheel yaw over the rest.
  double measured_yaw_rad() const { return gyro_yaw_sum_rad + uncovered_wheel_yaw_sum_rad; }
};

// Sums the part of every wheel sample that lies inside (begin_sec, end_sec]. A sample that straddles a
// keyframe time is split in proportion to the time on each side (the yaw is taken as uniform inside the
// sample), so consecutive keyframe intervals partition the samples exactly.
inline WheelYawInterval summarize_wheel_yaw_samples(const std::deque<WheelYawSample>& samples, double begin_sec,
                                                    double end_sec) {
  WheelYawInterval interval;
  for (const WheelYawSample& sample : samples) {
    if (sample.interval_sec <= 0.0) continue;
    const double overlap_sec = std::min(sample.stamp_sec, end_sec) - std::max(sample.stamp_sec - sample.interval_sec, begin_sec);
    if (overlap_sec <= 0.0) continue;
    const double share = overlap_sec / sample.interval_sec;
    interval.total_sec += overlap_sec;
    interval.raw_wheel_yaw_sum_rad += share * sample.raw_wheel_yaw_rad;
    if (sample.is_covered) {
      interval.covered_sec += overlap_sec;
      interval.gyro_yaw_sum_rad += share * sample.gyro_yaw_rad;
    } else {
      interval.uncovered_wheel_yaw_sum_rad += share * sample.raw_wheel_yaw_rad;
    }
  }
  return interval;
}

}  // namespace lightnav_localization
