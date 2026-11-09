#pragma once

#include <Eigen/Geometry>
#include <cstddef>
#include <functional>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include "lightnav_localization/pose_graph.hpp"

namespace lightnav_localization {

struct KeyframeCloud;
struct MatchResult;

struct LoopClosureParameters {
  bool enabled = true;
  double min_travel_m = 8.0;
  double search_radius_m = 3.0;
  double candidate_spacing_m = 0.5;
  int max_verifications = 200;
  int submap_half_keyframes = 5;
  double coarse_correspondence_m = 2.0;
  double fine_correspondence_m = 0.5;
  double inlier_ratio_min = 0.7;
  double max_drift_ratio = 0.1;
  double max_rotation_correction_rad = 0.35;
  int min_support = 1;
  double covariance_scale = 1.0;
  double outlier_chi2 = 12.6;
  double time_budget_s = 25.0;
};

struct LoopCandidate {
  std::size_t older_index = 0;
  std::size_t newer_index = 0;
  double travel_m = 0.0;
};

struct VerifiedLoop {
  LoopCandidate candidate;
  Eigen::Isometry3d older_to_newer = Eigen::Isometry3d::Identity();
  Eigen::Matrix<double, 6, 6> information = Eigen::Matrix<double, 6, 6>::Zero();
};

std::vector<double> loop_path_lengths(const std::vector<BatchPose>& poses);
std::vector<LoopCandidate> select_loop_candidates(const std::vector<BatchPose>& poses,
                                                   const std::vector<double>& path_lengths,
                                                   const LoopClosureParameters& parameters);
std::vector<VerifiedLoop> supported_loops(const std::vector<VerifiedLoop>& loops,
                                         const std::vector<BatchPose>& poses,
                                         const std::vector<double>& path_lengths,
                                         const LoopClosureParameters& parameters);
double loop_translation_gate(const LoopCandidate& candidate, const std::vector<BatchPose>& poses,
                             const std::vector<double>& path_lengths, const std::set<int>& gps_indices,
                             const LoopClosureParameters& parameters);
enum class LoopVerificationFailure { kTarget, kCoarse, kFine, kInlier, kDegenerate, kCorrectionGate };
std::optional<VerifiedLoop> assess_loop_match(
    const LoopCandidate& candidate, const MatchResult& match, const std::vector<BatchPose>& poses,
    const std::vector<double>& path_lengths, const std::set<int>& gps_indices,
    const LoopClosureParameters& parameters, double degeneracy_ratio,
    LoopVerificationFailure* failure = nullptr);
std::optional<VerifiedLoop> verify_loop_candidate(
    const LoopCandidate& candidate, const std::vector<BatchPose>& poses,
    const std::vector<KeyframeCloud>& keyframes, const std::vector<double>& path_lengths,
    const std::set<int>& gps_indices, const LoopClosureParameters& parameters,
    double degeneracy_ratio, LoopVerificationFailure* failure = nullptr,
    double* fine_inlier_ratio = nullptr);
// Optional seams for deterministic verification of save-time failure and budget handling.
struct LoopClosureTestHooks {
  std::function<std::optional<VerifiedLoop>(const LoopCandidate&)> verify;
  std::function<double()> elapsed_seconds;
  std::function<void(int)> before_batch;
  std::function<void()> before_add_factor;
  std::optional<double> iteration_seconds;
};

struct LoopClosureResult {
  std::vector<BatchPose> poses;
  int count = 0;
  std::string warning;
  std::string statistics;
};
LoopClosureResult close_loops(PoseGraph& graph, const std::vector<KeyframeCloud>& keyframes,
                              const LoopClosureParameters& parameters, int batch_max_iterations,
                              double degeneracy_ratio, double elapsed_before_pass_zero_s,
                              const LoopClosureTestHooks* hooks = nullptr);
int loop_iteration_limit(double remaining_s, double iteration_s, int batch_max_iterations);

}  // namespace lightnav_localization
