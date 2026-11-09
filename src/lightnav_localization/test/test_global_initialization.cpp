#include <gtest/gtest.h>

#include <vector>
#include <cmath>
#include <stdexcept>

#include "lightnav_localization/global_initialization.hpp"

using namespace lightnav_localization;

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
  peaks.front().coarse_score = 0.9;
  peaks.back().coarse_score = 0.8;
  EXPECT_TRUE(has_too_many_global_initialization_peaks(peaks, {}));
}

TEST(GlobalInitializationPeaks, IgnoresSamePeakAfterRefinement) {
  std::vector<GlobalInitializationPeak> peaks(2);
  peaks[0].verification_score = 0.95;
  peaks[1].verification_score = 0.94;
  peaks[1].position.x() = 0.1;
  EXPECT_TRUE(decide_global_initialization_peaks(peaks, {}).accepted);
}

TEST(GlobalInitializationField, ComputesDistanceAndBounds) {
  const PointList points{Eigen::Vector3f(1.0f, 1.0f, 0.0f)};
  const GlobalInitializationLikelihoodField field(points, {0.0, 0.0}, {2.0, 2.0}, 1.0, 9);
  EXPECT_EQ(field.cell_count(), 9u);
  EXPECT_DOUBLE_EQ(field.distance({1.0, 1.0}), 0.0);
  EXPECT_DOUBLE_EQ(field.distance({0.0, 1.0}), 1.0);
  EXPECT_TRUE(std::isinf(field.distance({-1.0, 0.0})));
}

TEST(GlobalInitializationField, RejectsOversizedGrid) {
  EXPECT_THROW(GlobalInitializationLikelihoodField({}, {0.0, 0.0}, {2.0, 2.0}, 1.0, 8),
               std::length_error);
}
