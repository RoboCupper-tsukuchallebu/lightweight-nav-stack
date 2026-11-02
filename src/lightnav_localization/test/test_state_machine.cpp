#include <gtest/gtest.h>

#include "lightnav_localization/state_machine.hpp"

using namespace lightnav_localization;

namespace {
StateInputs fresh_inputs(double now_sec) {
  StateInputs inputs;
  inputs.now_sec = now_sec;
  inputs.last_wheel_sec = now_sec;
  inputs.last_scan_sec = now_sec;
  return inputs;
}
}  // namespace

TEST(StateMachine, FallsFromTrackingToLostAndStaysLost) {
  StateMachine machine(StateMachineParameters{});
  EXPECT_EQ(machine.update(fresh_inputs(0.0)), LocalizationState::kInitializing);
  machine.start_tracking();

  StateInputs inputs = fresh_inputs(10.0);
  inputs.last_map_match_sec = 9.0;
  EXPECT_EQ(machine.update(inputs), LocalizationState::kTracking);

  inputs = fresh_inputs(20.0);  // map match 11 s ago, GPS fresh
  inputs.last_map_match_sec = 9.0;
  inputs.last_gps_sec = 19.5;
  EXPECT_EQ(machine.update(inputs), LocalizationState::kGpsOnly);

  inputs = fresh_inputs(30.0);
  inputs.last_map_match_sec = 9.0;
  inputs.last_gps_sec = 19.5;
  inputs.odom_only_distance_m = 10.0;
  EXPECT_EQ(machine.update(inputs), LocalizationState::kOdomOnly);
  EXPECT_TRUE(machine.should_publish_odom());

  inputs.odom_only_distance_m = 50.1;
  EXPECT_EQ(machine.update(inputs), LocalizationState::kLost);
  EXPECT_FALSE(machine.should_publish_odom());

  inputs.last_map_match_sec = 30.0;  // fresh data does not leave LOST by itself
  EXPECT_EQ(machine.update(inputs), LocalizationState::kLost);
  machine.reset();
  EXPECT_EQ(machine.state(), LocalizationState::kInitializing);
}

TEST(StateMachine, LostOnLargePositionStd) {
  StateMachine machine(StateMachineParameters{});
  machine.start_tracking();
  StateInputs inputs = fresh_inputs(5.0);
  inputs.position_std_m = 2.5;
  EXPECT_EQ(machine.update(inputs), LocalizationState::kLost);
}

TEST(StateMachine, StaleWheelWithholdsOdom) {
  StateMachine machine(StateMachineParameters{});
  machine.start_tracking();
  StateInputs inputs = fresh_inputs(5.0);
  inputs.last_map_match_sec = 5.0;
  inputs.last_wheel_sec = 4.7;  // 0.3 s > 0.2 s timeout
  EXPECT_EQ(machine.update(inputs), LocalizationState::kInputStale);
  EXPECT_FALSE(machine.should_publish_odom());
}

TEST(StateMachine, LargePositionStdIsLostEvenWhileTrackingOrGpsOnly) {
  for (const bool has_map : {true, false}) {
    StateMachine machine(StateMachineParameters{});
    machine.start_tracking();
    StateInputs inputs = fresh_inputs(5.0);
    (has_map ? inputs.last_map_match_sec : inputs.last_gps_sec) = 5.0;
    inputs.position_std_m = 2.5;
    EXPECT_EQ(machine.update(inputs), LocalizationState::kLost);
  }
}
