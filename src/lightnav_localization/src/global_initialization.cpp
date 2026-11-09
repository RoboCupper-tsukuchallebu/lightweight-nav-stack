#include "lightnav_localization/global_initialization.hpp"

#include <algorithm>
#include <cmath>

namespace lightnav_localization {
namespace {

constexpr double kFullTurnRadians = 6.2831853071795864769;

bool are_distinct(const GlobalInitializationPeak& left, const GlobalInitializationPeak& right,
                  const GlobalInitializationPeakParameters& parameters) {
  const double translation = (left.position - right.position).norm();
  const double yaw = std::abs(std::remainder(left.yaw_rad - right.yaw_rad, kFullTurnRadians));
  return translation >= parameters.distinct_translation_m || yaw >= parameters.distinct_yaw_rad;
}

}  // namespace

bool has_too_many_global_initialization_peaks(
    const std::vector<GlobalInitializationPeak>& coarse_peaks,
    const GlobalInitializationPeakParameters& parameters) {
  if (coarse_peaks.empty() || coarse_peaks.size() <= parameters.peak_count) return false;
  return coarse_peaks[parameters.peak_count].coarse_score >=
         coarse_peaks.front().coarse_score - parameters.candidate_margin;
}

GlobalInitializationPeakDecision decide_global_initialization_peaks(
    const std::vector<GlobalInitializationPeak>& verified_peaks,
    const GlobalInitializationPeakParameters& parameters) {
  GlobalInitializationPeakDecision decision;
  if (verified_peaks.empty()) {
    decision.error = "low_score";
    return decision;
  }
  for (std::size_t index = 0; index < verified_peaks.size(); ++index) {
    const double score = verified_peaks[index].verification_score;
    if (!std::isfinite(score)) {
      decision.error = "low_score";
      return decision;
    }
    if (index == 0 || score > decision.best_score) {
      decision.best_score = score;
      decision.best_index = index;
    }
  }
  const auto& best = verified_peaks[decision.best_index];
  for (const auto& peak : verified_peaks) {
    if (are_distinct(best, peak, parameters)) {
      decision.second_score = std::max(decision.second_score, peak.verification_score);
    }
  }
  if (decision.best_score < parameters.score_min) decision.error = "low_score";
  else if (decision.best_score - decision.second_score < parameters.margin_min) {
    decision.error = "ambiguous";
  } else {
    decision.accepted = true;
  }
  return decision;
}

}  // namespace lightnav_localization
