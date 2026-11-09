#pragma once

#include <Eigen/Core>
#include <cstddef>
#include <string>
#include <vector>

namespace lightnav_localization {

// A candidate after either coarse scoring or verification. Scores from the two stages are
// intentionally kept separate: coarse likelihood is not an upper bound on verification score.
struct GlobalInitializationPeak {
  Eigen::Vector2d position = Eigen::Vector2d::Zero();
  double yaw_rad = 0.0;
  double coarse_score = 0.0;
  double verification_score = 0.0;
};

struct GlobalInitializationPeakParameters {
  std::size_t peak_count = 10;
  double candidate_margin = 0.1;
  double distinct_translation_m = 1.0;
  double distinct_yaw_rad = 0.3490658503988659;
  double score_min = 0.7;
  double margin_min = 0.15;
};

// Input must be sorted by decreasing coarse score after non-maximum suppression.
// Only the coarse 2D score is compared; a discarded peak can still rank higher in 3D.
bool has_too_many_global_initialization_peaks(
    const std::vector<GlobalInitializationPeak>& coarse_peaks,
    const GlobalInitializationPeakParameters& parameters);

struct GlobalInitializationPeakDecision {
  bool accepted = false;
  std::size_t best_index = 0;
  double best_score = 0.0;
  double second_score = 0.0;
  std::string error;
};

// Compare all verified peaks, including those with low coarse-score rank. Other acceptance
// requirements (convergence, degeneracy and GNSS window) belong to the caller.
GlobalInitializationPeakDecision decide_global_initialization_peaks(
    const std::vector<GlobalInitializationPeak>& verified_peaks,
    const GlobalInitializationPeakParameters& parameters);

}  // namespace lightnav_localization
