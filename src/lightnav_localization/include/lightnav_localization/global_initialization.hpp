#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <atomic>
#include <cstddef>
#include <string>
#include <vector>
#include "lightnav_localization/pcd_io.hpp"

namespace lightnav_localization {

// The field covers only the supplied search window, including its range padding.
class GlobalInitializationLikelihoodField {
 public:
  GlobalInitializationLikelihoodField(const PointList& points, const Eigen::Vector2d& minimum,
                                      const Eigen::Vector2d& maximum, double resolution,
                                      std::size_t maximum_cells);
  double distance(const Eigen::Vector2d& position) const;
  std::size_t cell_count() const { return distances_.size(); }

 private:
  Eigen::Vector2d minimum_;
  double resolution_;
  std::size_t width_;
  std::size_t height_;
  std::vector<double> distances_;
};

// A candidate after either coarse scoring or verification. Scores from the two stages are
// intentionally kept separate: coarse likelihood is not an upper bound on verification score.
struct GlobalInitializationPeak {
  Eigen::Vector2d position = Eigen::Vector2d::Zero();
  double yaw_rad = 0.0;
  double coarse_score = 0.0;
  double verification_score = 0.0;
  bool degenerate = false;
};

struct GlobalInitializationPeakParameters {
  std::size_t peak_count = 10;
  double candidate_margin = 0.1;
  double distinct_translation_m = 1.0;
  double distinct_yaw_rad = 0.3490658503988659;
  double score_min = 0.7;
  double margin_min = 0.15;
};

// Input must have finite scores sorted by decreasing coarse score after non-maximum suppression;
// invalid input throws std::invalid_argument.
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

// The caller owns the abort flag and sets it when motion or cancellation is detected.
// Candidate and wall-clock budgets are checked between bounded units of work; an in-flight
// PreparedTarget construction or match_scan call cannot be interrupted.
struct GlobalInitializationSearchParameters {
  Eigen::Vector2d minimum = Eigen::Vector2d::Zero();
  Eigen::Vector2d maximum = Eigen::Vector2d::Zero();
  double grid_resolution_m = 0.1;
  double likelihood_sigma_m = 0.3;
  double maximum_range_m = 30.0;
  double maximum_area_m2 = 2500.0;
  std::size_t maximum_grid_cells = 4000000;
  double coarse_translation_step_m = 0.5;
  double coarse_yaw_step_rad = 0.08726646259971647;
  double fine_translation_step_m = 0.1;
  double fine_yaw_step_rad = 0.017453292519943295;
  double height_min_m = 0.3;
  double height_max_m = 2.0;
  double correspondence_m = 0.2;
  double see_through_m = 0.5;
  double degeneracy_ratio = 0.05;
  double time_budget_s = 20.0;
  std::size_t candidate_budget = 10000000;
  std::size_t minimum_scan_points = 10;
  GlobalInitializationPeakParameters peaks;
};

struct GlobalInitializationSearchResult {
  GlobalInitializationPeakDecision decision;
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  std::size_t evaluated_candidates = 0;
};

GlobalInitializationSearchResult search_global_initialization(
    const PointList& map, const PointList& scan,
    const GlobalInitializationSearchParameters& parameters, const std::atomic<bool>& abort);

}  // namespace lightnav_localization
