#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <random>
#include <stdexcept>
#include <string>

#include "lightnav_localization/gyro_yaw.hpp"

using lightnav_localization::GyroYawParameters;
using lightnav_localization::GyroYawState;
using lightnav_localization::GyroYawTracker;

namespace {

constexpr double kImuRateHz = 966.0;
constexpr double kWheelPeriodSec = 0.02;
constexpr double kWheelSpeedMetersPerSec = 0.5;
constexpr double kGravityMetersPerSecSquared = 9.80665;
constexpr double kPi = 3.14159265358979323846;

const Eigen::Vector3d kTrueBias(0.01, -0.02, 0.015);

Eigen::Vector3d tilted_up_axis(double tilt_deg) {
  const double tilt_rad = tilt_deg * kPi / 180.0;
  return Eigen::Vector3d(std::sin(tilt_rad), 0.0, std::cos(tilt_rad));
}

// What the simulated robot does during one call of Simulator::run().
struct Motion {
  bool is_wheel_moving = false;
  double yaw_rate_rad_s = 0.0;                          // about the true up axis
  Eigen::Vector3d extra_angular_velocity_rad_s = Eigen::Vector3d::Zero();
  double gyro_noise_std_rad_s = 0.0;
  double acceleration_magnitude_m_s2 = kGravityMetersPerSecSquared;
};

// Feeds a tracker the way the node does: IMU samples (966 Hz) up to a wheel stamp first, then
// the wheel pose (50 Hz).
class Simulator {
 public:
  Simulator(GyroYawTracker& tracker, const Eigen::Vector3d& up_axis_in_imu = Eigen::Vector3d::UnitZ(),
            const Eigen::Vector3d& gyro_bias_rad_s = kTrueBias)
      : tracker_(tracker), up_axis_in_imu_(up_axis_in_imu.normalized()), gyro_bias_rad_s_(gyro_bias_rad_s) {}

  void set_up_axis(const Eigen::Vector3d& up_axis_in_imu) { up_axis_in_imu_ = up_axis_in_imu.normalized(); }

  void run(double duration_sec, const Motion& motion = Motion()) {
    const double end_time_sec = end_time_sec_ + duration_sec;
    while (wheel_tick_index_ * kWheelPeriodSec <= end_time_sec + 1e-9) {
      const double wheel_stamp_sec = wheel_tick_index_ * kWheelPeriodSec;
      while (imu_index_ / kImuRateHz <= wheel_stamp_sec) {
        feed_imu_sample(imu_index_ / kImuRateHz, motion);
        ++imu_index_;
      }
      if (motion.is_wheel_moving) wheel_x_m_ += kWheelSpeedMetersPerSec * kWheelPeriodSec;
      Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
      pose.translation().x() = wheel_x_m_;
      tracker_.add_wheel_pose(wheel_stamp_sec, pose);
      ++wheel_tick_index_;
    }
    end_time_sec_ = end_time_sec;
  }

  double last_imu_stamp_sec() const { return (imu_index_ - 1) / kImuRateHz; }

 private:
  void feed_imu_sample(double stamp_sec, const Motion& motion) {
    Eigen::Vector3d angular_velocity = up_axis_in_imu_ * motion.yaw_rate_rad_s + gyro_bias_rad_s_ +
                                       motion.extra_angular_velocity_rad_s;
    if (motion.gyro_noise_std_rad_s > 0.0) {
      std::normal_distribution<double> noise(0.0, motion.gyro_noise_std_rad_s);
      angular_velocity += Eigen::Vector3d(noise(random_engine_), noise(random_engine_), noise(random_engine_));
    }
    tracker_.add_imu_sample(stamp_sec, angular_velocity, up_axis_in_imu_ * motion.acceleration_magnitude_m_s2);
  }

  GyroYawTracker& tracker_;
  Eigen::Vector3d up_axis_in_imu_;
  Eigen::Vector3d gyro_bias_rad_s_;
  std::mt19937 random_engine_{12345};
  std::size_t imu_index_ = 0;
  std::size_t wheel_tick_index_ = 0;
  double end_time_sec_ = 0.0;
  double wheel_x_m_ = 0.0;
};

Motion rotating(double yaw_rate_rad_s) {
  Motion motion;
  motion.is_wheel_moving = true;
  motion.yaw_rate_rad_s = yaw_rate_rad_s;
  return motion;
}

constexpr double kStationarySettleSec = 3.0;

}  // namespace

// --- integration ---------------------------------------------------------------------------

TEST(GyroYawTracker, constant_rate_integrates_to_rate_times_duration_between_samples) {
  GyroYawTracker tracker{GyroYawParameters()};
  Simulator simulator(tracker);
  simulator.run(kStationarySettleSec);
  simulator.run(2.0, rotating(0.5));
  const double from_sec = kStationarySettleSec + 0.3 / kImuRateHz + 0.4;
  const double to_sec = from_sec + 1.0 + 0.6 / kImuRateHz;

  const auto yaw_delta = tracker.integrate_yaw(from_sec, to_sec);

  ASSERT_TRUE(yaw_delta.has_value());
  EXPECT_NEAR(0.5 * (to_sec - from_sec), *yaw_delta, 0.5 * (to_sec - from_sec) * 1e-6);
}

TEST(GyroYawTracker, linearly_increasing_rate_integrates_exactly_between_samples) {
  GyroYawTracker tracker{GyroYawParameters()};
  Simulator simulator(tracker);
  simulator.run(kStationarySettleSec);
  const double ramp_start_sec = simulator.last_imu_stamp_sec();
  constexpr double kRampRadPerSecSquared = 0.4;
  const Eigen::Vector3d gravity(0.0, 0.0, kGravityMetersPerSecSquared);
  for (int index = 1; index <= 1000; ++index) {
    const double elapsed_sec = index / kImuRateHz;
    const Eigen::Vector3d measured = kTrueBias + Eigen::Vector3d(0.0, 0.0, kRampRadPerSecSquared * elapsed_sec);
    tracker.add_imu_sample(ramp_start_sec + elapsed_sec, measured, gravity);
  }
  const double from_elapsed_sec = 0.2 + 0.3 / kImuRateHz;
  const double to_elapsed_sec = 0.9 + 0.7 / kImuRateHz;

  const auto yaw_delta =
      tracker.integrate_yaw(ramp_start_sec + from_elapsed_sec, ramp_start_sec + to_elapsed_sec);

  ASSERT_TRUE(yaw_delta.has_value());
  const double expected = 0.5 * kRampRadPerSecSquared * (to_elapsed_sec * to_elapsed_sec - from_elapsed_sec * from_elapsed_sec);
  EXPECT_NEAR(expected, *yaw_delta, 1e-9);
}

TEST(GyroYawTracker, rate_is_projected_on_the_up_axis_after_subtracting_the_bias) {
  GyroYawTracker tracker{GyroYawParameters()};
  Simulator simulator(tracker);
  simulator.run(kStationarySettleSec);
  simulator.run(1.0, rotating(-0.3));

  const auto status = tracker.status();

  ASSERT_TRUE(status.yaw_rate_rad_s.has_value());
  EXPECT_NEAR(-0.3, *status.yaw_rate_rad_s, 1e-9);
  ASSERT_TRUE(status.yaw_bias_rad_s.has_value());
  EXPECT_NEAR(kTrueBias.z(), *status.yaw_bias_rad_s, 1e-9);
}

TEST(GyroYawTracker, yaw_is_unknown_while_the_bias_is_unknown) {
  GyroYawTracker tracker{GyroYawParameters()};
  Simulator simulator(tracker);
  simulator.run(1.0, rotating(0.5));

  const auto yaw_delta = tracker.integrate_yaw(0.2, 0.8);

  EXPECT_FALSE(yaw_delta.has_value());
}

TEST(GyroYawTracker, yaw_is_unknown_outside_the_held_range) {
  GyroYawTracker tracker{GyroYawParameters()};
  Simulator simulator(tracker);
  simulator.run(kStationarySettleSec);
  const double newest_sec = simulator.last_imu_stamp_sec();

  EXPECT_FALSE(tracker.integrate_yaw(-0.5, 0.5).has_value());
  EXPECT_FALSE(tracker.integrate_yaw(newest_sec + 0.1, newest_sec + 0.2).has_value());
  EXPECT_FALSE(tracker.integrate_yaw(1.0, std::numeric_limits<double>::quiet_NaN()).has_value());
  EXPECT_FALSE(tracker.integrate_yaw(2.0, 1.0).has_value());
}

TEST(GyroYawTracker, yaw_extends_the_last_rate_by_at_most_0_03_seconds_past_the_newest_sample) {
  GyroYawTracker tracker{GyroYawParameters()};
  Simulator simulator(tracker);
  simulator.run(kStationarySettleSec);
  simulator.run(1.0, rotating(0.5));
  const double newest_sec = simulator.last_imu_stamp_sec();

  const auto within_extension = tracker.integrate_yaw(newest_sec - 0.1, newest_sec + 0.03);
  const auto beyond_extension = tracker.integrate_yaw(newest_sec - 0.1, newest_sec + 0.04);

  ASSERT_TRUE(within_extension.has_value());
  EXPECT_NEAR(0.5 * 0.13, *within_extension, 1e-6);
  EXPECT_FALSE(beyond_extension.has_value());
}

TEST(GyroYawTracker, yaw_is_unknown_across_a_gap_longer_than_the_maximum_and_known_around_it) {
  GyroYawTracker tracker{GyroYawParameters()};
  Simulator simulator(tracker);
  simulator.run(kStationarySettleSec);
  const Eigen::Vector3d measured = kTrueBias + Eigen::Vector3d(0.0, 0.0, 0.5);
  const Eigen::Vector3d gravity(0.0, 0.0, kGravityMetersPerSecSquared);
  const double block_start_sec = simulator.last_imu_stamp_sec() + 1.0 / kImuRateHz;
  for (int index = 0; index < 200; ++index) {
    tracker.add_imu_sample(block_start_sec + index / kImuRateHz, measured, gravity);
  }
  const double first_block_end_sec = block_start_sec + 199.0 / kImuRateHz;
  const double second_block_start_sec = first_block_end_sec + 0.06;
  for (int index = 0; index < 200; ++index) {
    tracker.add_imu_sample(second_block_start_sec + index / kImuRateHz, measured, gravity);
  }
  const double second_block_end_sec = second_block_start_sec + 199.0 / kImuRateHz;

  const auto across_gap = tracker.integrate_yaw(first_block_end_sec - 0.05, second_block_start_sec + 0.05);
  const auto before_gap = tracker.integrate_yaw(block_start_sec + 0.01, first_block_end_sec - 0.01);
  const auto after_gap = tracker.integrate_yaw(second_block_start_sec + 0.01, second_block_end_sec - 0.01);

  EXPECT_FALSE(across_gap.has_value());
  ASSERT_TRUE(before_gap.has_value());
  EXPECT_NEAR(0.5 * (first_block_end_sec - block_start_sec - 0.02), *before_gap, 1e-6);
  ASSERT_TRUE(after_gap.has_value());
  EXPECT_NEAR(0.5 * (second_block_end_sec - second_block_start_sec - 0.02), *after_gap, 1e-6);
}

// --- bias ----------------------------------------------------------------------------------

TEST(GyroYawTracker, bias_is_estimated_within_2e_4_from_three_seconds_of_noisy_standstill) {
  GyroYawTracker tracker{GyroYawParameters()};
  Simulator simulator(tracker);
  Motion noisy_standstill;
  noisy_standstill.gyro_noise_std_rad_s = 1.05e-3;

  simulator.run(kStationarySettleSec, noisy_standstill);

  const auto status = tracker.status();
  ASSERT_TRUE(status.gyro_bias_rad_s.has_value());
  for (int axis = 0; axis < 3; ++axis) {
    EXPECT_NEAR(kTrueBias[axis], (*status.gyro_bias_rad_s)[axis], 2e-4) << "axis " << axis;
  }
  EXPECT_EQ(1u, status.bias_updates_total);
  EXPECT_EQ(0u, status.bias_rejected_total);
}

TEST(GyroYawTracker, bias_is_not_updated_while_the_wheels_move) {
  GyroYawTracker tracker{GyroYawParameters()};
  Simulator simulator(tracker);
  simulator.run(kStationarySettleSec);
  const auto bias_before = tracker.status().gyro_bias_rad_s;
  Motion moving_with_drift = rotating(0.0);
  moving_with_drift.extra_angular_velocity_rad_s = Eigen::Vector3d(0.0, 0.0, 0.02);

  simulator.run(6.0, moving_with_drift);

  const auto status = tracker.status();
  ASSERT_TRUE(bias_before.has_value());
  ASSERT_TRUE(status.gyro_bias_rad_s.has_value());
  EXPECT_EQ(*bias_before, *status.gyro_bias_rad_s);
  EXPECT_EQ(1u, status.bias_updates_total);
  EXPECT_EQ(0u, status.bias_rejected_total);
}

TEST(GyroYawTracker, bias_is_updated_by_every_standstill_window_of_a_long_standstill) {
  GyroYawTracker tracker{GyroYawParameters()};
  Simulator simulator(tracker);

  simulator.run(0.5 + 3 * 2.0 + 0.01);

  EXPECT_EQ(3u, tracker.status().bias_updates_total);
}

TEST(GyroYawTracker, standstill_window_rotating_at_0_1_rad_s_is_rejected) {
  GyroYawTracker tracker{GyroYawParameters()};
  Simulator simulator(tracker);
  Motion turned_by_hand;
  turned_by_hand.extra_angular_velocity_rad_s = Eigen::Vector3d(0.0, 0.0, 0.1);

  simulator.run(kStationarySettleSec, turned_by_hand);

  const auto status = tracker.status();
  EXPECT_EQ(1u, status.bias_rejected_total);
  EXPECT_EQ(0u, status.bias_updates_total);
  EXPECT_FALSE(status.gyro_bias_rad_s.has_value());
  EXPECT_FALSE(status.up_axis_in_imu.has_value());
}

TEST(GyroYawTracker, standstill_window_with_acceleration_magnitude_12_is_rejected) {
  GyroYawTracker tracker{GyroYawParameters()};
  Simulator simulator(tracker);
  Motion carried;
  carried.acceleration_magnitude_m_s2 = 12.0;

  simulator.run(kStationarySettleSec, carried);

  const auto status = tracker.status();
  EXPECT_EQ(1u, status.bias_rejected_total);
  EXPECT_EQ(0u, status.bias_updates_total);
  EXPECT_FALSE(status.gyro_bias_rad_s.has_value());
}

TEST(GyroYawTracker, standstill_window_with_gyro_standard_deviation_0_2_is_rejected) {
  GyroYawTracker tracker{GyroYawParameters()};
  Simulator simulator(tracker);
  Motion shaken;
  shaken.gyro_noise_std_rad_s = 0.2;

  simulator.run(kStationarySettleSec, shaken);

  const auto status = tracker.status();
  EXPECT_EQ(1u, status.bias_rejected_total);
  EXPECT_EQ(0u, status.bias_updates_total);
  EXPECT_FALSE(status.gyro_bias_rad_s.has_value());
}

// --- up axis -------------------------------------------------------------------------------

struct UpAxisCase {
  std::string name;
  Eigen::Vector3d up_axis_in_imu;
};

class GyroYawUpAxis : public testing::TestWithParam<UpAxisCase> {};

TEST_P(GyroYawUpAxis, yaw_rate_about_the_gravity_axis_has_the_right_value_and_sign) {
  GyroYawTracker tracker{GyroYawParameters()};
  Simulator simulator(tracker, GetParam().up_axis_in_imu);
  simulator.run(kStationarySettleSec);

  simulator.run(1.0, rotating(0.5));

  const auto status = tracker.status();
  ASSERT_TRUE(status.yaw_rate_rad_s.has_value());
  EXPECT_NEAR(0.5, *status.yaw_rate_rad_s, 0.5e-3);
  ASSERT_TRUE(status.up_axis_in_imu.has_value());
  EXPECT_NEAR(1.0, status.up_axis_in_imu->dot(GetParam().up_axis_in_imu.normalized()), 1e-9);
  const double newest_sec = simulator.last_imu_stamp_sec();
  const auto yaw_delta = tracker.integrate_yaw(newest_sec - 0.5, newest_sec);
  ASSERT_TRUE(yaw_delta.has_value());
  EXPECT_NEAR(0.25, *yaw_delta, 0.25e-3);
}

INSTANTIATE_TEST_SUITE_P(
    FourMountings, GyroYawUpAxis,
    testing::Values(UpAxisCase{"board_flat_up_is_plus_z", Eigen::Vector3d::UnitZ()},
                    UpAxisCase{"board_on_its_side_up_is_plus_y", Eigen::Vector3d::UnitY()},
                    UpAxisCase{"board_upside_down_up_is_minus_z", -Eigen::Vector3d::UnitZ()},
                    UpAxisCase{"board_tilted_30_degrees", tilted_up_axis(30.0)}),
    [](const testing::TestParamInfo<UpAxisCase>& info) { return info.param.name; });

TEST(GyroYawTracker, up_axis_stays_at_the_first_value_after_a_later_standstill_tilted_by_10_degrees) {
  GyroYawTracker tracker{GyroYawParameters()};
  Simulator simulator(tracker);
  simulator.run(kStationarySettleSec);
  simulator.run(1.0, rotating(0.0));
  simulator.set_up_axis(tilted_up_axis(10.0));

  simulator.run(kStationarySettleSec);

  const auto status = tracker.status();
  EXPECT_EQ(2u, status.bias_updates_total);
  ASSERT_TRUE(status.up_axis_in_imu.has_value());
  EXPECT_NEAR(1.0, status.up_axis_in_imu->dot(Eigen::Vector3d::UnitZ()), 1e-12);
}

TEST(GyroYawTracker, expected_up_axis_20_degrees_away_is_reported_and_warns) {
  GyroYawTracker tracker{GyroYawParameters()};
  Simulator simulator(tracker);
  simulator.run(kStationarySettleSec);

  tracker.set_expected_up_axis(tilted_up_axis(20.0));

  const auto status = tracker.status();
  ASSERT_TRUE(status.tf_up_axis_disagreement_deg.has_value());
  EXPECT_NEAR(20.0, *status.tf_up_axis_disagreement_deg, 0.5);
  EXPECT_TRUE(status.is_up_axis_warning);
}

TEST(GyroYawTracker, expected_up_axis_5_degrees_away_is_reported_without_a_warning) {
  GyroYawTracker tracker{GyroYawParameters()};
  Simulator simulator(tracker);
  simulator.run(kStationarySettleSec);

  tracker.set_expected_up_axis(tilted_up_axis(5.0));

  const auto status = tracker.status();
  ASSERT_TRUE(status.tf_up_axis_disagreement_deg.has_value());
  EXPECT_NEAR(5.0, *status.tf_up_axis_disagreement_deg, 0.5);
  EXPECT_FALSE(status.is_up_axis_warning);
}

TEST(GyroYawTracker, disagreement_is_unavailable_without_an_expected_up_axis) {
  GyroYawTracker tracker{GyroYawParameters()};
  Simulator simulator(tracker);
  simulator.run(kStationarySettleSec);

  const auto without_expectation = tracker.status();
  tracker.set_expected_up_axis(tilted_up_axis(20.0));
  tracker.set_expected_up_axis(std::nullopt);
  const auto withdrawn_expectation = tracker.status();

  EXPECT_FALSE(without_expectation.tf_up_axis_disagreement_deg.has_value());
  EXPECT_FALSE(without_expectation.is_up_axis_warning);
  EXPECT_FALSE(withdrawn_expectation.tf_up_axis_disagreement_deg.has_value());
  EXPECT_FALSE(withdrawn_expectation.is_up_axis_warning);
}

// --- dropped samples -----------------------------------------------------------------------

TEST(GyroYawTracker, non_finite_values_and_rates_above_the_limit_are_dropped_and_counted) {
  GyroYawTracker tracker{GyroYawParameters()};
  Simulator simulator(tracker);
  simulator.run(kStationarySettleSec);
  const double newest_sec = simulator.last_imu_stamp_sec();
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double infinity = std::numeric_limits<double>::infinity();
  const Eigen::Vector3d gravity(0.0, 0.0, kGravityMetersPerSecSquared);

  tracker.add_imu_sample(newest_sec + 0.001, Eigen::Vector3d(nan, 0.0, 0.0), gravity);
  tracker.add_imu_sample(newest_sec + 0.002, Eigen::Vector3d::Zero(), Eigen::Vector3d(0.0, infinity, 0.0));
  tracker.add_imu_sample(newest_sec + 0.003, Eigen::Vector3d(0.0, 0.0, 10.5), gravity);
  tracker.add_imu_sample(nan, Eigen::Vector3d::Zero(), gravity);

  const auto status = tracker.status();
  EXPECT_EQ(4u, status.dropped_samples_total);
  const auto yaw_delta = tracker.integrate_yaw(newest_sec - 0.1, newest_sec);
  ASSERT_TRUE(yaw_delta.has_value());
  EXPECT_NEAR(0.0, *yaw_delta, 1e-9);
}

TEST(GyroYawTracker, backwards_imu_stamp_discards_the_samples_but_keeps_bias_and_up_axis) {
  GyroYawTracker tracker{GyroYawParameters()};
  Simulator simulator(tracker);
  simulator.run(kStationarySettleSec);
  const double newest_sec = simulator.last_imu_stamp_sec();
  const auto bias_before = tracker.status().gyro_bias_rad_s;
  const auto up_axis_before = tracker.status().up_axis_in_imu;

  tracker.add_imu_sample(newest_sec - 1.0, kTrueBias, Eigen::Vector3d(0.0, 0.0, kGravityMetersPerSecSquared));

  const auto status = tracker.status();
  EXPECT_EQ(1u, status.dropped_samples_total);
  EXPECT_EQ(bias_before, status.gyro_bias_rad_s);
  EXPECT_EQ(up_axis_before, status.up_axis_in_imu);
  EXPECT_FALSE(tracker.integrate_yaw(newest_sec - 0.5, newest_sec).has_value());
  EXPECT_FALSE(tracker.integrate_yaw(newest_sec - 1.0, newest_sec - 1.0).has_value());  // the late sample is not held
}

// --- retention and state -------------------------------------------------------------------

TEST(GyroYawTracker, held_samples_cover_three_seconds_within_10_percent_after_60_seconds) {
  GyroYawTracker tracker{GyroYawParameters()};
  Simulator simulator(tracker);
  simulator.run(kStationarySettleSec);
  simulator.run(57.0, rotating(0.1));
  const double newest_sec = simulator.last_imu_stamp_sec();

  const auto shortest_covered = tracker.integrate_yaw(newest_sec - 2.7, newest_sec);
  const auto longest_covered = tracker.integrate_yaw(newest_sec - 3.3, newest_sec);

  EXPECT_TRUE(shortest_covered.has_value());
  EXPECT_FALSE(longest_covered.has_value());
}

TEST(GyroYawTracker, state_follows_the_data_the_bias_and_the_timeout) {
  GyroYawTracker tracker{GyroYawParameters()};
  EXPECT_EQ(GyroYawState::kWaitingForData, tracker.status().state);
  Simulator simulator(tracker);
  simulator.run(1.0, rotating(0.0));
  EXPECT_EQ(GyroYawState::kWaitingForStationary, tracker.status().state);
  simulator.run(0.1);
  EXPECT_EQ(GyroYawState::kWaitingForStationary, tracker.status().state);
  simulator.run(kStationarySettleSec);
  EXPECT_EQ(GyroYawState::kReady, tracker.status().state);
  const double newest_imu_sec = simulator.last_imu_stamp_sec();

  tracker.add_wheel_pose(newest_imu_sec + 0.4, Eigen::Isometry3d::Identity());
  const auto delayed = tracker.status();
  tracker.add_wheel_pose(newest_imu_sec + 0.6, Eigen::Isometry3d::Identity());
  const auto stale = tracker.status();

  EXPECT_EQ(GyroYawState::kReady, delayed.state);
  EXPECT_EQ(GyroYawState::kStale, stale.state);
  ASSERT_TRUE(stale.last_sample_age_s.has_value());
  EXPECT_NEAR(0.6, *stale.last_sample_age_s, 1e-9);
  ASSERT_TRUE(stale.bias_age_s.has_value());
  EXPECT_GT(*stale.bias_age_s, 0.6);
}

TEST(GyroYawTracker, state_is_waiting_for_stationary_when_only_imu_samples_arrived) {
  GyroYawTracker tracker{GyroYawParameters()};

  tracker.add_imu_sample(0.0, Eigen::Vector3d::Zero(), Eigen::Vector3d(0.0, 0.0, kGravityMetersPerSecSquared));

  const auto status = tracker.status();
  EXPECT_EQ(GyroYawState::kWaitingForStationary, status.state);
  EXPECT_FALSE(status.last_sample_age_s.has_value());
}

TEST(GyroYawTracker, sample_rate_counts_the_stamps_of_the_last_second) {
  GyroYawTracker tracker{GyroYawParameters()};
  Simulator simulator(tracker);
  simulator.run(2.0);

  EXPECT_NEAR(kImuRateHz, tracker.status().sample_rate_hz, 2.0);
}

// --- parameters ----------------------------------------------------------------------------

TEST(GyroYawTracker, constructor_rejects_parameters_that_are_not_finite_and_positive) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double infinity = std::numeric_limits<double>::infinity();
  double GyroYawParameters::*const fields[] = {
      &GyroYawParameters::max_gap_s,       &GyroYawParameters::max_rate_rad_s,
      &GyroYawParameters::stationary_window_s, &GyroYawParameters::bias_max_abs_rad_s,
      &GyroYawParameters::bias_max_std_rad_s,  &GyroYawParameters::up_axis_warn_deg,
      &GyroYawParameters::timeout_s};

  for (const auto field : fields) {
    for (const double bad_value : {0.0, -1.0, nan, infinity}) {
      GyroYawParameters parameters;
      parameters.*field = bad_value;
      EXPECT_THROW(GyroYawTracker{parameters}, std::invalid_argument) << bad_value;
    }
  }
  EXPECT_NO_THROW(GyroYawTracker{GyroYawParameters()});
}

TEST(GyroYawTracker, constructor_rejects_a_standstill_window_that_the_held_samples_cannot_cover) {
  GyroYawParameters too_long;
  too_long.stationary_window_s = 3.0;
  GyroYawParameters far_too_long;
  far_too_long.stationary_window_s = 5.0;
  GyroYawParameters longest_allowed;
  longest_allowed.stationary_window_s = 2.9;

  EXPECT_THROW(GyroYawTracker{too_long}, std::invalid_argument);
  EXPECT_THROW(GyroYawTracker{far_too_long}, std::invalid_argument);
  EXPECT_NO_THROW(GyroYawTracker{longest_allowed});
}
