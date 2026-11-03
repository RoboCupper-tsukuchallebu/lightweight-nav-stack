#include <gtest/gtest.h>

#include "lightnav_controller/arbitrate.hpp"

using namespace lightnav_controller;

namespace {

LayerOutput asserted(double linear_mps) {
  return LayerOutput{true, Twist2D{linear_mps, 0.0}, StopReason::kOk};
}

LayerOutput notAsserted(StopReason reason) { return LayerOutput{false, Twist2D{}, reason}; }

}  // namespace

// Full table: avoid asserted x follow asserted x pose fresh x grid fresh.
TEST(Arbitrate, FullCombinationTable) {
  for (int bits = 0; bits < 16; ++bits) {
    const bool is_avoid_asserted = bits & 1;
    const bool is_follow_asserted = bits & 2;
    const bool is_pose_fresh = bits & 4;
    const bool is_grid_fresh = bits & 8;

    ArbitrationInput input;
    input.avoid = is_avoid_asserted ? asserted(0.25) : notAsserted(StopReason::kOk);
    input.follow = is_follow_asserted ? asserted(0.5) : notAsserted(StopReason::kBlocked);
    input.is_pose_fresh = is_pose_fresh;
    input.is_grid_fresh = is_grid_fresh;
    const ArbitrationResult result = arbitrate(input);

    SCOPED_TRACE(bits);
    if (!is_pose_fresh) {
      EXPECT_TRUE(result.is_hard_stop);
      EXPECT_EQ(result.reason, StopReason::kPoseStale);
      EXPECT_EQ(result.source_layer, Layer::kNone);
      EXPECT_DOUBLE_EQ(result.twist.linear_mps, 0.0);
    } else if (is_avoid_asserted && is_grid_fresh) {
      EXPECT_FALSE(result.is_hard_stop);
      EXPECT_EQ(result.source_layer, Layer::kAvoid);
      EXPECT_DOUBLE_EQ(result.twist.linear_mps, 0.25);
    } else if (is_follow_asserted) {
      EXPECT_FALSE(result.is_hard_stop);
      EXPECT_EQ(result.source_layer, Layer::kFollow);
      EXPECT_DOUBLE_EQ(result.twist.linear_mps, 0.5);
    } else {
      EXPECT_TRUE(result.is_hard_stop);
      EXPECT_EQ(result.reason, StopReason::kBlocked);
      EXPECT_EQ(result.source_layer, Layer::kNone);
    }
  }
}

TEST(Arbitrate, ReportsNoPathWhenFollowGivesNoReason) {
  ArbitrationInput input;
  input.is_pose_fresh = true;
  input.is_grid_fresh = true;
  const ArbitrationResult result = arbitrate(input);
  EXPECT_TRUE(result.is_hard_stop);
  EXPECT_EQ(result.reason, StopReason::kNoPath);
}
