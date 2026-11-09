#include <gtest/gtest.h>

#include "lightnav_controller/arc_markers.hpp"

using namespace lightnav_controller;

TEST(ArcMarkers, ReflectsSyntheticGridEvaluationAndSelection) {
  LocalGrid grid;
  grid.width_cells = 40;
  grid.height_cells = 40;
  grid.resolution_m = 0.1;
  grid.origin_x_m = -1.0;
  grid.origin_y_m = -2.0;
  grid.cells.resize(1600, 0);
  grid.cells[20 * 40 + 15] = 100;
  AvoidParameters parameters;
  parameters.arc_candidate_count = 5;
  parameters.arc_length_m = 1.0;
  parameters.arc_step_m = 0.1;
  parameters.max_path_deviation_m = 2.0;
  const auto candidates = evaluateArcCandidates(grid, {}, {{0.0, 0.0}, {2.0, 0.0}}, 0, 0.0, parameters);
  const auto best = selectBestCandidate(candidates);
  ASSERT_TRUE(best);
  const auto markers = makeArcMarkers(candidates, best, 1.0, 0.1, "base_footprint", rclcpp::Time(0), true, StopReason::kOk, 2.0);
  ASSERT_EQ(markers.markers.size(), candidates.size() + 2);
  bool rejected = false;
  bool selected = false;
  for (std::size_t index = 0; index < candidates.size(); ++index) {
    const auto& marker = markers.markers[index + 1];
    EXPECT_EQ(marker.header.frame_id, "base_footprint");
    EXPECT_EQ(marker.color.g, candidates[index].is_feasible ? 1.0 : 0.0);
    if (!candidates[index].is_feasible) { rejected = true; EXPECT_EQ(marker.ns, "collision"); }
    if (marker.scale.x > 0.05) { selected = true; EXPECT_EQ(candidates[index].curvature_per_m, best->curvature_per_m); }
  }
  EXPECT_TRUE(rejected);
  EXPECT_TRUE(selected);
  EXPECT_EQ(markers.markers.back().text, "OK");
  EXPECT_EQ(makeArcMarkers({}, {}, 1.0, 0.1, "base_footprint", rclcpp::Time(0), false, StopReason::kOk, 2.0).markers.front().action,
            visualization_msgs::msg::Marker::DELETEALL);
}

TEST(ArcMarkers, RateCapAndDisabled) {
  ArcMarkerRate rate;
  EXPECT_FALSE(rate.ready(0.0, 0.0, true));
  EXPECT_FALSE(rate.ready(0.0, 2.0, false));
  EXPECT_TRUE(rate.ready(0.0, 2.0, true));
  EXPECT_FALSE(rate.ready(0.49, 2.0, true));
  EXPECT_TRUE(rate.ready(0.5, 2.0, true));
}

TEST(ArcMarkers, EveryMarkerHasFiniteLifetime) {
  const std::vector<ArcCandidate> candidates{{0.0, true, 0.0, ""}};
  for (const auto rate : {0.5, 2.0, 10.0}) {
    const auto markers = makeArcMarkers(candidates, candidates.front(), 1.0, 0.1,
        "base_footprint", rclcpp::Time(0), true, StopReason::kOk, rate);
    for (const auto& marker : markers.markers) {
      EXPECT_DOUBLE_EQ(rclcpp::Duration(marker.lifetime).seconds(), std::max(1.0, 2.0 / rate));
    }
  }
}
