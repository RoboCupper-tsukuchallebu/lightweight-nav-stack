#include "lightnav_localization/gyro_yaw.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace lightnav_localization {

namespace {

constexpr double kSampleRetentionSec = 3.0;           // same as kWheelBufferSec
constexpr double kMaxExtrapolationSec = 0.03;         // how far past the newest sample a query may reach
constexpr double kTimeToleranceSec = 1e-9;
constexpr double kStationaryTranslationSpeedMetersPerSec = 0.01;
constexpr double kStationaryRotationSpeedRadPerSec = 0.02;
constexpr double kSettleAfterStopSec = 0.5;           // shaking right after the wheels stop
constexpr double kMaxWheelSegmentSec = 1.0;           // a longer wheel silence is not evidence of standstill
constexpr double kGravityMetersPerSecSquared = 9.80665;
constexpr double kGravityToleranceMetersPerSecSquared = 0.5;
constexpr double kSampleRateWindowSec = 1.0;
constexpr double kDegreesPerRadian = 180.0 / 3.14159265358979323846;

bool is_finite_and_positive(double value) { return std::isfinite(value) && value > 0.0; }

}  // namespace

GyroYawTracker::GyroYawTracker(const GyroYawParameters& parameters) : parameters_(parameters) {
  if (parameters.stationary_window_s >= kSampleRetentionSec) {
    throw std::invalid_argument("GyroYawParameters: stationary_window_s must be shorter than the held sample time");
  }
  const double values[] = {parameters.max_gap_s,         parameters.max_rate_rad_s,
                           parameters.stationary_window_s, parameters.bias_max_abs_rad_s,
                           parameters.bias_max_std_rad_s,  parameters.up_axis_warn_deg,
                           parameters.timeout_s};
  for (const double value : values) {
    if (!is_finite_and_positive(value)) {
      throw std::invalid_argument("GyroYawParameters: every value must be finite and positive");
    }
  }
}

void GyroYawTracker::add_imu_sample(double stamp_sec, const Eigen::Vector3d& angular_velocity_rad_s,
                                    const Eigen::Vector3d& linear_acceleration_m_s2) {
  const bool is_finite = std::isfinite(stamp_sec) && angular_velocity_rad_s.allFinite() &&
                         linear_acceleration_m_s2.allFinite();
  if (!is_finite || angular_velocity_rad_s.norm() > parameters_.max_rate_rad_s) {
    ++dropped_samples_total_;
    return;
  }
  if (!samples_.empty() && stamp_sec <= samples_.back().stamp_sec) {
    ++dropped_samples_total_;
    const bool is_duplicate_stamp = stamp_sec == samples_.back().stamp_sec;
    // The clock went backwards: what is held no longer belongs to this timeline. The sample
    // itself is dropped too (not held), as the header says.
    if (!is_duplicate_stamp) samples_.clear();
    return;
  }
  samples_.push_back({stamp_sec, angular_velocity_rad_s, linear_acceleration_m_s2});
  prune_old_samples();
}

void GyroYawTracker::prune_old_samples() {
  const double oldest_kept_sec = samples_.back().stamp_sec - kSampleRetentionSec;
  while (samples_.front().stamp_sec < oldest_kept_sec) samples_.pop_front();
}

void GyroYawTracker::add_wheel_pose(double stamp_sec, const Eigen::Isometry3d& odom_to_base) {
  if (!std::isfinite(stamp_sec)) return;
  const bool went_backwards = latest_wheel_stamp_sec_ && stamp_sec < *latest_wheel_stamp_sec_;
  if (latest_wheel_stamp_sec_ && stamp_sec == *latest_wheel_stamp_sec_) return;
  const auto previous_pose = previous_wheel_pose_;
  const double previous_stamp_sec = latest_wheel_stamp_sec_.value_or(stamp_sec);
  latest_wheel_stamp_sec_ = stamp_sec;
  previous_wheel_pose_ = odom_to_base;
  if (went_backwards || !previous_pose) {
    stationary_window_start_sec_.reset();
    return;
  }

  const double segment_sec = stamp_sec - previous_stamp_sec;
  const Eigen::Isometry3d motion = previous_pose->inverse() * odom_to_base;
  const double translation_speed = motion.translation().norm() / segment_sec;
  const double rotation_speed = Eigen::AngleAxisd(motion.rotation()).angle() / segment_sec;
  const bool is_stationary = segment_sec <= kMaxWheelSegmentSec &&
                             translation_speed < kStationaryTranslationSpeedMetersPerSec &&
                             rotation_speed < kStationaryRotationSpeedRadPerSec;
  if (!is_stationary) {
    stationary_window_start_sec_.reset();
    return;
  }
  if (!stationary_window_start_sec_) stationary_window_start_sec_ = previous_stamp_sec + kSettleAfterStopSec;
  evaluate_stationary_windows(stamp_sec);
}

void GyroYawTracker::evaluate_stationary_windows(double wheel_stamp_sec) {
  while (stationary_window_start_sec_) {
    const double window_start_sec = *stationary_window_start_sec_;
    const double window_end_sec = window_start_sec + parameters_.stationary_window_s;
    const bool is_wheel_past_window = window_end_sec <= wheel_stamp_sec;
    const bool is_imu_past_window = !samples_.empty() && window_end_sec <= samples_.back().stamp_sec;
    if (!is_wheel_past_window || !is_imu_past_window) return;
    evaluate_window(window_start_sec, window_end_sec);
    stationary_window_start_sec_ = window_end_sec;
  }
}

void GyroYawTracker::evaluate_window(double window_start_sec, double window_end_sec) {
  std::vector<const ImuSample*> window;
  for (const ImuSample& sample : samples_) {
    if (sample.stamp_sec >= window_start_sec && sample.stamp_sec <= window_end_sec) window.push_back(&sample);
  }
  // Samples that do not cover the whole window (late start, dropped values) give no verdict.
  bool is_covered = window.size() >= 2 &&
                    window.front()->stamp_sec - window_start_sec <= parameters_.max_gap_s &&
                    window_end_sec - window.back()->stamp_sec <= parameters_.max_gap_s;
  for (std::size_t index = 1; is_covered && index < window.size(); ++index) {
    is_covered = window[index]->stamp_sec - window[index - 1]->stamp_sec <= parameters_.max_gap_s;
  }
  if (!is_covered) return;

  Eigen::Vector3d gyro_sum = Eigen::Vector3d::Zero();
  Eigen::Vector3d acceleration_sum = Eigen::Vector3d::Zero();
  for (const ImuSample* sample : window) {
    gyro_sum += sample->angular_velocity_rad_s;
    acceleration_sum += sample->linear_acceleration_m_s2;
  }
  const double count = static_cast<double>(window.size());
  const Eigen::Vector3d gyro_mean = gyro_sum / count;
  const Eigen::Vector3d acceleration_mean = acceleration_sum / count;
  Eigen::Vector3d squared_deviation_sum = Eigen::Vector3d::Zero();
  for (const ImuSample* sample : window) {
    squared_deviation_sum += (sample->angular_velocity_rad_s - gyro_mean).cwiseAbs2();
  }
  const double gyro_std = std::sqrt(squared_deviation_sum.maxCoeff() / count);

  const bool is_rotating = gyro_mean.norm() > parameters_.bias_max_abs_rad_s;
  const bool is_shaking = gyro_std > parameters_.bias_max_std_rad_s;
  const bool is_gravity_off =
      std::abs(acceleration_mean.norm() - kGravityMetersPerSecSquared) > kGravityToleranceMetersPerSecSquared;
  if (is_rotating || is_shaking || is_gravity_off) {
    ++bias_rejected_total_;
    return;
  }
  gyro_bias_rad_s_ = gyro_mean;
  bias_stamp_sec_ = window_end_sec;
  ++bias_updates_total_;
  if (!up_axis_in_imu_) up_axis_in_imu_ = acceleration_mean.normalized();  // fixed for the whole run
}

double GyroYawTracker::yaw_rate_of(const Eigen::Vector3d& angular_velocity_rad_s) const {
  return (angular_velocity_rad_s - *gyro_bias_rad_s_).dot(*up_axis_in_imu_);
}

std::optional<double> GyroYawTracker::integrate_yaw(double from_sec, double to_sec) const {
  if (!gyro_bias_rad_s_ || !up_axis_in_imu_ || samples_.size() < 2) return std::nullopt;
  if (!std::isfinite(from_sec) || !std::isfinite(to_sec) || to_sec < from_sec) return std::nullopt;
  const double newest_sec = samples_.back().stamp_sec;
  if (from_sec < samples_.front().stamp_sec - kTimeToleranceSec ||
      to_sec > newest_sec + kMaxExtrapolationSec + kTimeToleranceSec) {
    return std::nullopt;
  }

  // Start at the sample before from_sec (or the first one) and walk forward over the intervals.
  const auto first_at_or_after = std::lower_bound(
      samples_.begin(), samples_.end(), from_sec,
      [](const ImuSample& sample, double stamp) { return sample.stamp_sec < stamp; });
  const auto start = first_at_or_after == samples_.begin() ? first_at_or_after : std::prev(first_at_or_after);

  double yaw_delta_rad = 0.0;
  double segment_start_sec = from_sec;
  for (auto next = start; std::next(next) != samples_.end() && segment_start_sec < to_sec; ++next) {
    const auto following = std::next(next);
    if (following->stamp_sec <= from_sec) continue;
    if (following->stamp_sec - next->stamp_sec > parameters_.max_gap_s) return std::nullopt;
    const double rate_before = yaw_rate_of(next->angular_velocity_rad_s);
    const double rate_after = yaw_rate_of(following->angular_velocity_rad_s);
    const double interval_sec = following->stamp_sec - next->stamp_sec;
    const auto rate_at = [&](double stamp) {
      return rate_before + (rate_after - rate_before) * (stamp - next->stamp_sec) / interval_sec;
    };
    const double segment_end_sec = std::min(following->stamp_sec, to_sec);
    const double start_rate = rate_at(segment_start_sec);
    const double end_rate = rate_at(segment_end_sec);
    yaw_delta_rad += 0.5 * (start_rate + end_rate) * (segment_end_sec - segment_start_sec);
    segment_start_sec = segment_end_sec;
  }
  if (segment_start_sec < to_sec) {  // hold the last rate past the newest sample
    yaw_delta_rad += yaw_rate_of(samples_.back().angular_velocity_rad_s) * (to_sec - segment_start_sec);
  }
  return yaw_delta_rad;
}

void GyroYawTracker::set_expected_up_axis(const std::optional<Eigen::Vector3d>& up_axis_in_imu) {
  if (up_axis_in_imu && up_axis_in_imu->allFinite() && up_axis_in_imu->norm() > 0.0) {
    expected_up_axis_in_imu_ = up_axis_in_imu->normalized();
  } else {
    expected_up_axis_in_imu_.reset();
  }
}

GyroYawStatus GyroYawTracker::status() const {
  GyroYawStatus status;
  status.bias_updates_total = bias_updates_total_;
  status.bias_rejected_total = bias_rejected_total_;
  status.dropped_samples_total = dropped_samples_total_;
  status.gyro_bias_rad_s = gyro_bias_rad_s_;
  status.up_axis_in_imu = up_axis_in_imu_;
  if (gyro_bias_rad_s_ && up_axis_in_imu_) status.yaw_bias_rad_s = gyro_bias_rad_s_->dot(*up_axis_in_imu_);
  if (bias_stamp_sec_ && latest_wheel_stamp_sec_) {
    status.bias_age_s = std::max(0.0, *latest_wheel_stamp_sec_ - *bias_stamp_sec_);
  }
  if (up_axis_in_imu_ && expected_up_axis_in_imu_) {
    const double cosine = std::clamp(up_axis_in_imu_->dot(*expected_up_axis_in_imu_), -1.0, 1.0);
    const double disagreement_deg = std::acos(cosine) * kDegreesPerRadian;
    status.tf_up_axis_disagreement_deg = disagreement_deg;
    status.is_up_axis_warning = disagreement_deg > parameters_.up_axis_warn_deg;
  }
  if (!samples_.empty()) {
    const double newest_sec = samples_.back().stamp_sec;
    const auto first_in_window = std::upper_bound(
        samples_.begin(), samples_.end(), newest_sec - kSampleRateWindowSec,
        [](double stamp, const ImuSample& sample) { return stamp < sample.stamp_sec; });
    status.sample_rate_hz = static_cast<double>(std::distance(first_in_window, samples_.end())) / kSampleRateWindowSec;
    if (gyro_bias_rad_s_ && up_axis_in_imu_) {
      status.yaw_rate_rad_s = yaw_rate_of(samples_.back().angular_velocity_rad_s);
    }
    if (latest_wheel_stamp_sec_) {
      status.last_sample_age_s = std::max(0.0, *latest_wheel_stamp_sec_ - newest_sec);
    }
  }

  if (samples_.empty()) {
    status.state = GyroYawState::kWaitingForData;
  } else if (status.last_sample_age_s && *status.last_sample_age_s > parameters_.timeout_s) {
    status.state = GyroYawState::kStale;
  } else if (!gyro_bias_rad_s_) {
    status.state = GyroYawState::kWaitingForStationary;
  } else {
    status.state = GyroYawState::kReady;
  }
  return status;
}

}  // namespace lightnav_localization
