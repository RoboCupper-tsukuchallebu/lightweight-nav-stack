#include <gtest/gtest.h>

#include <vector>
#include <stdexcept>
#include <cmath>

#include "lightnav_localization/global_initialization.hpp"

using namespace lightnav_localization;

TEST(GlobalInitializationField, ComputesEuclideanGridDistance) {
  const PointList points{Eigen::Vector3f(1.0f, 1.0f, 0.0f)};
  const GlobalInitializationLikelihoodField field(points, {0.0, 0.0}, {3.0, 3.0}, 1.0, 16);
  EXPECT_EQ(field.cell_count(), 16u);
  EXPECT_DOUBLE_EQ(field.distance({1.0, 1.0}), 0.0);
  EXPECT_DOUBLE_EQ(field.distance({3.0, 2.0}), std::sqrt(5.0));
  EXPECT_TRUE(std::isinf(field.distance({-1.0, 0.0})));
  EXPECT_THROW(GlobalInitializationLikelihoodField(points, {0.0, 0.0}, {3.0, 3.0}, 1.0, 15),
               std::length_error);
}

TEST(GlobalInitializationPeaks, RejectsCompetitorBelowFifthCoarseRank) {
  std::vector<GlobalInitializationPeak> peaks(7);
  for (std::size_t index = 0; index < peaks.size(); ++index) {
    peaks[index].position.x() = 2.0 * static_cast<double>(index);
    peaks[index].verification_score = 0.1;
  }
  peaks[0].verification_score = 0.95;
  // Limiting verification to the first five candidates would miss this competing peak.
  peaks[6].verification_score = 0.94;
  const auto decision = decide_global_initialization_peaks(peaks, {});
  EXPECT_FALSE(decision.accepted);
  EXPECT_EQ(decision.error, "ambiguous");
}

TEST(GlobalInitializationPeaks, RejectsEleventhNearbyCoarseScore) {
  std::vector<GlobalInitializationPeak> peaks(11);
  for (auto& peak : peaks) peak.coarse_score = 0.9;
  peaks.back().coarse_score = 0.81;
  EXPECT_TRUE(has_too_many_global_initialization_peaks(peaks, {}));
  peaks.back().coarse_score = 0.79;
  EXPECT_FALSE(has_too_many_global_initialization_peaks(peaks, {}));
}

TEST(GlobalInitializationPeaks, RejectsUnsortedCoarseScores) {
  std::vector<GlobalInitializationPeak> peaks(2);
  peaks[0].coarse_score = 0.1;
  peaks[1].coarse_score = 0.9;
  EXPECT_THROW(has_too_many_global_initialization_peaks(peaks, {}), std::invalid_argument);
}

TEST(GlobalInitializationPeaks, IgnoresSamePeakAfterRefinement) {
  std::vector<GlobalInitializationPeak> peaks(2);
  peaks[0].verification_score = 0.95;
  peaks[1].verification_score = 0.94;
  peaks[1].position.x() = 0.1;
  EXPECT_TRUE(decide_global_initialization_peaks(peaks, {}).accepted);
}

TEST(GlobalInitializationSearch, CancelsBeforeStarting) {
  std::atomic<bool> abort{true};
  const auto result = search_global_initialization({}, {}, {}, abort);
  EXPECT_EQ(result.decision.error, "cancelled");
}

TEST(GlobalInitializationSearch, StopsAtTimeBudget) {
  std::atomic<bool> abort{false};
  GlobalInitializationSearchParameters parameters;
  parameters.time_budget_s = 0.0;
  const auto result = search_global_initialization({}, {}, parameters, abort);
  EXPECT_EQ(result.decision.error, "time_budget");
}

TEST(GlobalInitializationSearch, RejectsInsufficientScan) {
  std::atomic<bool> abort{false};
  const auto result = search_global_initialization({}, {}, {}, abort);
  EXPECT_EQ(result.decision.error, "not_enough_scan_points");
}

TEST(GlobalInitializationSearch, RejectsSymmetricRoomThroughSearch) {
  PointList map;
  PointList scan;
  for (int index = -20; index <= 20; ++index) {
    const float coordinate = static_cast<float>(index) * 0.1f;
    for (float height : {0.5f, 1.0f, 1.5f}) {
      map.emplace_back(coordinate, -2.0f, height);
      map.emplace_back(coordinate, 2.0f, height);
      map.emplace_back(-2.0f, coordinate, height);
      map.emplace_back(2.0f, coordinate, height);
    }
  }
  for (const auto& point : map) scan.emplace_back(point.x() - 0.5f, point.y(), point.z());
  GlobalInitializationSearchParameters parameters;
  parameters.minimum = {-1.0, -1.0};
  parameters.maximum = {1.0, 1.0};
  parameters.maximum_range_m = 5.0;
  parameters.coarse_yaw_step_rad = M_PI / 2.0;
  parameters.fine_yaw_step_rad = M_PI / 2.0;
  parameters.fine_translation_step_m = 0.5;
  parameters.peaks.peak_count = 2;
  parameters.peaks.candidate_margin = 1.0;
  parameters.time_budget_s = 60.0;
  std::atomic<bool> abort{false};
  const auto result = search_global_initialization(map, scan, parameters, abort);
  EXPECT_FALSE(result.decision.accepted);
  EXPECT_EQ(result.decision.error, "ambiguous");
}

TEST(GlobalInitializationSearch, RejectsWhenCandidateBudgetExpires) {
  PointList map;
  PointList scan;
  for (int index = 0; index < 30; ++index) {
    const float x = static_cast<float>(index) * 0.1f;
    map.emplace_back(x, 0.0f, 0.0f);
    map.emplace_back(x, 0.0f, 1.0f);
    scan.emplace_back(x + 0.1f, 1.0f, 1.0f);
  }
  GlobalInitializationSearchParameters parameters;
  parameters.minimum = {1.0, 1.0};
  parameters.maximum = {1.5, 1.5};
  parameters.maximum_range_m = 3.0;
  parameters.minimum_scan_points = 1;
  parameters.candidate_budget = 1;
  std::atomic<bool> abort{false};
  EXPECT_EQ(search_global_initialization(map, scan, parameters, abort).decision.error, "time_budget");
}

TEST(GlobalInitializationSearch, RejectsLikelihoodFieldAboveCellLimit) {
  PointList scan;
  for (int index = 0; index < 36; ++index) {
    const double angle = index * M_PI / 18.0;
    scan.emplace_back(std::cos(angle), std::sin(angle), 1.0f);
  }
  GlobalInitializationSearchParameters parameters;
  parameters.minimum = {-1.0, -1.0};
  parameters.maximum = {1.0, 1.0};
  parameters.maximum_range_m = 2.0;
  parameters.maximum_grid_cells = 100;
  std::atomic<bool> abort{false};
  const auto result = search_global_initialization({}, scan, parameters, abort);
  EXPECT_EQ(result.decision.error, "map_too_large");
  EXPECT_EQ(result.evaluated_candidates, 0u);
}
