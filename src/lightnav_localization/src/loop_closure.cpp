#include "lightnav_localization/loop_closure.hpp"

#include <algorithm>
#include <chrono>
#include <sstream>
#include <cmath>
#include <limits>

#include "lightnav_localization/initial_alignment.hpp"
#include "lightnav_localization/map_builder.hpp"
#include "lightnav_localization/matcher.hpp"
#include "lightnav_localization/match_information.hpp"

namespace lightnav_localization {
namespace {
constexpr double kSupportTranslationM = 0.3;
constexpr double kSupportRotationRad = 0.05;
constexpr double kTimeSafetyFactor = 1.5;
constexpr double kGateBaseM = 0.5;
constexpr double kGateSigmaMultiplier = 3.0;
constexpr double kSupportSpacingMultiplier = 2.0;
constexpr double kLoopSigmaFloorM = 0.05;
constexpr double kLoopSigmaFloorRad = 0.01;

const char* loop_verification_failure_string(LoopVerificationFailure failure) {
  switch (failure) {
    case LoopVerificationFailure::kTarget: return "target";
    case LoopVerificationFailure::kCoarse: return "coarse non-finite";
    case LoopVerificationFailure::kFine: return "fine";
    case LoopVerificationFailure::kInlier: return "inlier";
    case LoopVerificationFailure::kDegenerate: return "degenerate";
    case LoopVerificationFailure::kCorrectionGate: return "correction gate";
  }
  return "unknown";
}
}

std::vector<double> loop_path_lengths(const std::vector<BatchPose>& poses) {
  std::vector<double> lengths(poses.size(), 0.0);
  for (std::size_t index = 1; index < poses.size(); ++index) {
    lengths[index] = lengths[index - 1] +
        (poses[index].map_to_base.translation() - poses[index - 1].map_to_base.translation()).norm();
  }
  return lengths;
}

std::vector<LoopCandidate> select_loop_candidates(const std::vector<BatchPose>& poses,
                                                   const std::vector<double>& path_lengths,
                                                   const LoopClosureParameters& parameters) {
  std::vector<LoopCandidate> candidates;
  if (poses.size() != path_lengths.size() || poses.empty()) return candidates;
  double last_selected_length = -std::numeric_limits<double>::infinity();
  for (std::size_t newer_index = 0; newer_index < poses.size(); ++newer_index) {
    if (path_lengths[newer_index] - last_selected_length < parameters.candidate_spacing_m) continue;
    last_selected_length = path_lengths[newer_index];
    double closest_distance = parameters.search_radius_m;
    std::size_t closest_index = poses.size();
    for (std::size_t older_index = 0; older_index < newer_index; ++older_index) {
      if (path_lengths[newer_index] - path_lengths[older_index] < parameters.min_travel_m) continue;
      const double distance = (poses[newer_index].map_to_base.translation() -
                               poses[older_index].map_to_base.translation()).head<2>().norm();
      if (distance <= closest_distance && (closest_index == poses.size() || distance < closest_distance)) {
        closest_distance = distance;
        closest_index = older_index;
      }
    }
    if (closest_index != poses.size()) {
      candidates.push_back({closest_index, newer_index,
                            path_lengths[newer_index] - path_lengths[closest_index]});
    }
  }
  std::stable_sort(candidates.begin(), candidates.end(),
                   [](const LoopCandidate& left, const LoopCandidate& right) {
                     return left.travel_m > right.travel_m;
                   });
  return candidates;
}

std::vector<VerifiedLoop> supported_loops(const std::vector<VerifiedLoop>& loops,
                                         const std::vector<BatchPose>& poses,
                                         const std::vector<double>& path_lengths,
                                         const LoopClosureParameters& parameters) {
  std::vector<VerifiedLoop> supported;
  for (std::size_t first_index = 0; first_index < loops.size(); ++first_index) {
    const VerifiedLoop& first = loops[first_index];
    int support_count = 0;
    for (std::size_t second_index = 0; second_index < loops.size(); ++second_index) {
      if (first_index == second_index) continue;
      const VerifiedLoop& second = loops[second_index];
      if (std::abs(path_lengths[first.candidate.newer_index] -
                   path_lengths[second.candidate.newer_index]) < kSupportSpacingMultiplier * parameters.candidate_spacing_m) continue;
      const Eigen::Isometry3d cycle = first.older_to_newer.inverse() *
          poses[first.candidate.older_index].map_to_base.inverse() *
          poses[second.candidate.older_index].map_to_base * second.older_to_newer *
          poses[second.candidate.newer_index].map_to_base.inverse() *
          poses[first.candidate.newer_index].map_to_base;
      if (cycle.translation().norm() <= kSupportTranslationM &&
          Eigen::AngleAxisd(cycle.rotation()).angle() <= kSupportRotationRad) ++support_count;
    }
    if (support_count >= parameters.min_support) supported.push_back(first);
  }
  return supported;
}

double loop_translation_gate(const LoopCandidate& candidate, const std::vector<BatchPose>& poses,
                             const std::vector<double>& path_lengths, const std::set<int>& gps_indices,
                             const LoopClosureParameters& parameters) {
  const std::size_t older_index = candidate.older_index;
  const std::size_t newer_index = candidate.newer_index;
  double gate = kGateBaseM + parameters.max_drift_ratio * candidate.travel_m;
  if (!gps_indices.empty() && poses[older_index].position_and_yaw_std && poses[newer_index].position_and_yaw_std &&
      poses[older_index].position_and_yaw_std->allFinite() &&
      poses[newer_index].position_and_yaw_std->allFinite()) {
    const auto uncertainty = [&](std::size_t index) {
      double distance = std::numeric_limits<double>::infinity();
      for (std::size_t gps_index = 0; gps_index < poses.size(); ++gps_index) {
        if (gps_indices.count(poses[gps_index].index)) {
          distance = std::min(distance, std::abs(path_lengths[index] - path_lengths[gps_index]));
        }
      }
      const Eigen::Vector3d& standard_deviation = *poses[index].position_and_yaw_std;
      return std::hypot(std::max(standard_deviation.x(), standard_deviation.y()),
                        parameters.max_drift_ratio * distance);
    };
    gate = std::min(gate, kGateBaseM + kGateSigmaMultiplier * std::hypot(uncertainty(older_index), uncertainty(newer_index)));
  }
  return gate;
}

std::optional<VerifiedLoop> assess_loop_match(
    const LoopCandidate& candidate, const MatchResult& match, const std::vector<BatchPose>& poses,
    const std::vector<double>& path_lengths, const std::set<int>& gps_indices,
    const LoopClosureParameters& parameters, double degeneracy_ratio,
    LoopVerificationFailure* failure) {
  if (failure) *failure = LoopVerificationFailure::kFine;
  if (!match.converged || !match.target_to_source.matrix().allFinite() || !match.information.allFinite()) {
    return std::nullopt;
  }
  if (!std::isfinite(match.inlier_ratio) || match.inlier_ratio < parameters.inlier_ratio_min) {
    if (failure) *failure = LoopVerificationFailure::kInlier;
    return std::nullopt;
  }
  const Eigen::Matrix3d block = match.information.bottomRightCorner<3, 3>();
  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigenvalues(0.5 * (block + block.transpose()));
  if (eigenvalues.info() != Eigen::Success || eigenvalues.eigenvalues()[2] <= 0.0 ||
      eigenvalues.eigenvalues()[0] < degeneracy_ratio * eigenvalues.eigenvalues()[2]) {
    if (failure) *failure = LoopVerificationFailure::kDegenerate;
    return std::nullopt;
  }
  const Eigen::Isometry3d prediction = poses[candidate.older_index].map_to_base.inverse() *
                                         poses[candidate.newer_index].map_to_base;
  const Eigen::Isometry3d correction = match.target_to_source.inverse() * prediction;
  const double gate = loop_translation_gate(candidate, poses, path_lengths, gps_indices, parameters);
  if (correction.translation().norm() > gate ||
      Eigen::AngleAxisd(correction.rotation()).angle() > parameters.max_rotation_correction_rad) {
    if (failure) *failure = LoopVerificationFailure::kCorrectionGate;
    return std::nullopt;
  }
  InformationLimits limits;
  limits.covariance_scale = parameters.covariance_scale;
  limits.sigma_floor_m = kLoopSigmaFloorM;
  limits.sigma_floor_rad = kLoopSigmaFloorRad;
  limits.degeneracy_ratio = degeneracy_ratio;
  return VerifiedLoop{candidate, match.target_to_source, information_from_hessian(match.information, limits)};
}

std::optional<VerifiedLoop> verify_loop_candidate(
    const LoopCandidate& candidate, const std::vector<BatchPose>& poses,
    const std::vector<KeyframeCloud>& keyframes, const std::vector<double>& path_lengths,
    const std::set<int>& gps_indices, const LoopClosureParameters& parameters,
    double degeneracy_ratio, LoopVerificationFailure* failure, double* fine_inlier_ratio,
    LoopVerificationDiagnostics* diagnostics, const LoopMatcherForTest* matcher_for_test) {
  if (fine_inlier_ratio) *fine_inlier_ratio = std::numeric_limits<double>::quiet_NaN();
  if (failure) *failure = LoopVerificationFailure::kTarget;
  const std::size_t older_index = candidate.older_index;
  const std::size_t newer_index = candidate.newer_index;
  const std::size_t half = static_cast<std::size_t>(std::max(0, parameters.submap_half_keyframes));
  const std::size_t begin = older_index > half ? older_index - half : 0;
  const std::size_t end = std::min(keyframes.size(), older_index + half + 1);
  PointList target_points;
  for (std::size_t index = begin; index < end; ++index) {
    const Eigen::Isometry3d transform = poses[older_index].map_to_base.inverse() * poses[index].map_to_base;
    for (const Eigen::Vector3f& point : keyframes[index].points_in_base) {
      target_points.push_back((transform * point.cast<double>()).cast<float>());
    }
  }
  const PreparedTarget target(target_points);
  if (!target.is_valid()) return std::nullopt;
  const Eigen::Isometry3d prediction = poses[older_index].map_to_base.inverse() * poses[newer_index].map_to_base;
  MatcherParameters matcher_parameters;
  const InitialAlignmentParameters alignment_defaults;
  matcher_parameters.max_iterations = alignment_defaults.coarse_max_iterations;
  matcher_parameters.max_correspondence_distance_m = parameters.coarse_correspondence_m;
  matcher_parameters.gate_translation_m = std::numeric_limits<double>::max();
  matcher_parameters.gate_rotation_rad = std::numeric_limits<double>::max();
  const auto run_match = [&](const Eigen::Isometry3d& initial_pose) {
    return matcher_for_test ? (*matcher_for_test)(target, keyframes[newer_index].points_in_base, initial_pose, matcher_parameters)
                            : match_scan(target, keyframes[newer_index].points_in_base, initial_pose, matcher_parameters);
  };
  const MatchResult coarse = run_match(prediction);
  if (diagnostics) diagnostics->coarse_converged = coarse.converged;
  if (!coarse.target_to_source.matrix().allFinite()) {
    if (failure) *failure = LoopVerificationFailure::kCoarse;
    return std::nullopt;
  }
  matcher_parameters.max_iterations = alignment_defaults.fine_max_iterations;
  matcher_parameters.max_correspondence_distance_m = parameters.fine_correspondence_m;
  const MatchResult fine = run_match(coarse.target_to_source);
  if (diagnostics) {
    diagnostics->fine_converged = fine.converged;
    diagnostics->fine_inlier_ratio = fine.inlier_ratio;
    if (fine.target_to_source.matrix().allFinite()) {
      const Eigen::Isometry3d correction = fine.target_to_source.inverse() * prediction;
      diagnostics->correction_translation_m = correction.translation().norm();
      diagnostics->correction_rotation_deg = Eigen::AngleAxisd(correction.rotation()).angle() * 180.0 / std::acos(-1.0);
    }
  }
  if (fine_inlier_ratio) *fine_inlier_ratio = fine.inlier_ratio;
  return assess_loop_match(candidate, fine, poses, path_lengths, gps_indices, parameters,
                           degeneracy_ratio, failure);
}

LoopClosureResult close_loops(PoseGraph& graph, const std::vector<KeyframeCloud>& keyframes,
                              const LoopClosureParameters& parameters, int batch_max_iterations,
                              double degeneracy_ratio, double elapsed_before_pass_zero_s,
                              const LoopClosureTestHooks* hooks) {
  using Clock = std::chrono::steady_clock;
  constexpr int kMaximumRemovals = 3;
  const auto start = Clock::now();
  const auto elapsed = [&]() {
    if (hooks && hooks->elapsed_seconds) return hooks->elapsed_seconds();
    return elapsed_before_pass_zero_s + std::chrono::duration<double>(Clock::now() - start).count();
  };
  const auto log_info = [&](const std::string& message) {
    if (parameters.info_logger) parameters.info_logger(message);
  };
  LoopClosureResult result;
  result.poses = graph.batch_poses();
  if (!parameters.enabled) return result;
  if (elapsed() >= parameters.time_budget_s) {
    result.warning = "loop closure skipped: time budget";
    result.statistics = "pass0_iterations=" + std::to_string(graph.batch_iterations()) +
        ", pass0_iteration_s=" + std::to_string(
            graph.batch_optimization_seconds() / std::max(1, graph.batch_iterations()));
    return result;
  }
  const bool pass_zero_marginals_failed = graph.batch_marginals_failed();
  const double marginals_seconds = graph.batch_marginals_seconds();
  double iteration_seconds = graph.batch_optimization_seconds() / std::max(1, graph.batch_iterations());
  if (hooks && hooks->iteration_seconds) iteration_seconds = *hooks->iteration_seconds;
  std::vector<BatchPose> poses;
  std::vector<KeyframeCloud> aligned_keyframes;
  for (const KeyframeCloud& keyframe : keyframes) {
    const auto found = std::find_if(result.poses.begin(), result.poses.end(),
        [&](const BatchPose& pose) { return pose.index == keyframe.node_index; });
    if (found != result.poses.end()) {
      poses.push_back(*found);
      aligned_keyframes.push_back(keyframe);
    }
  }
  const std::vector<double> lengths = loop_path_lengths(poses);
  const std::vector<LoopCandidate> candidates = select_loop_candidates(poses, lengths, parameters);
  std::vector<VerifiedLoop> accepted;
  std::vector<LoopVerificationDiagnostics> accepted_diagnostics;
  int verification_count = 0;
  int rejected_target = 0;
  int rejected_coarse = 0;
  int rejected_fine = 0;
  int rejected_inlier = 0;
  int rejected_degenerate = 0;
  int rejected_gate = 0;
  double best_fine_inlier_ratio = 0.0;
  for (const LoopCandidate& candidate : candidates) {
    if (verification_count >= parameters.max_verifications || elapsed() >= parameters.time_budget_s) break;
    ++verification_count;
    LoopVerificationFailure failure = LoopVerificationFailure::kFine;
    double fine_inlier_ratio = std::numeric_limits<double>::quiet_NaN();
    LoopVerificationDiagnostics diagnostics;
    const auto verified = hooks && hooks->verify ? hooks->verify(candidate) :
        verify_loop_candidate(candidate, poses, aligned_keyframes, lengths,
                              graph.gps_factor_indices(), parameters, degeneracy_ratio, &failure, &fine_inlier_ratio, &diagnostics);
    if (std::isfinite(fine_inlier_ratio)) {
      best_fine_inlier_ratio = std::max(best_fine_inlier_ratio, fine_inlier_ratio);
    }
    std::ostringstream candidate_log;
    candidate_log << "loop " << (verified ? "accepted" : "rejected")
                  << ": older_index=" << candidate.older_index << " newer_index=" << candidate.newer_index
                  << " travel_m=" << candidate.travel_m << " reason=" << (verified ? "none" : loop_verification_failure_string(failure))
                  << " correction_translation_m=" << diagnostics.correction_translation_m
                  << " correction_rotation_deg=" << diagnostics.correction_rotation_deg
                  << " fine_inlier_ratio=" << diagnostics.fine_inlier_ratio
                  << " fine_converged=" << diagnostics.fine_converged << " coarse_converged=" << diagnostics.coarse_converged;
    log_info(candidate_log.str());
    if (verified) {
      accepted.push_back(*verified);
      accepted_diagnostics.push_back(diagnostics);
    } else if (failure == LoopVerificationFailure::kDegenerate) {
      ++rejected_degenerate;
    } else if (failure == LoopVerificationFailure::kCorrectionGate) {
      ++rejected_gate;
    } else if (failure == LoopVerificationFailure::kTarget) {
      ++rejected_target;
    } else if (failure == LoopVerificationFailure::kCoarse) {
      ++rejected_coarse;
    } else if (failure == LoopVerificationFailure::kInlier) {
      ++rejected_inlier;
    } else {
      ++rejected_fine;
    }
  }
  const std::vector<VerifiedLoop> supported = supported_loops(accepted, poses, lengths, parameters);
  for (std::size_t accepted_index = 0; accepted_index < accepted.size(); ++accepted_index) {
    const VerifiedLoop& loop = accepted[accepted_index];
    const LoopVerificationDiagnostics& diagnostics = accepted_diagnostics[accepted_index];
    const bool has_support = std::any_of(supported.begin(), supported.end(), [&](const VerifiedLoop& item) {
      return item.candidate.older_index == loop.candidate.older_index && item.candidate.newer_index == loop.candidate.newer_index;
    });
    if (!has_support) {
      std::ostringstream unsupported_log;
      unsupported_log << "loop rejected: older_index=" << loop.candidate.older_index
                      << " newer_index=" << loop.candidate.newer_index << " travel_m=" << loop.candidate.travel_m
                      << " reason=unsupported correction_translation_m=" << diagnostics.correction_translation_m
                      << " correction_rotation_deg=" << diagnostics.correction_rotation_deg
                      << " fine_inlier_ratio=" << diagnostics.fine_inlier_ratio;
      log_info(unsupported_log.str());
    }
  }
  std::vector<VerifiedLoop> active_loops = supported;
  double minimum_source_length = std::numeric_limits<double>::infinity();
  double maximum_source_length = 0.0;
  for (const VerifiedLoop& loop : accepted) {
    const double source_length = lengths[loop.candidate.newer_index];
    minimum_source_length = std::min(minimum_source_length, source_length);
    maximum_source_length = std::max(maximum_source_length, source_length);
  }
  const double source_span = accepted.empty() ? 0.0 : maximum_source_length - minimum_source_length;
  std::ostringstream statistics;
  statistics << "loop candidates=" << candidates.size() << ", verified=" << verification_count
             << ", accepted=" << accepted.size() << ", unsupported=" << accepted.size() - supported.size()
             << ", rejected_target=" << rejected_target << ", rejected_coarse=" << rejected_coarse
             << ", rejected_fine=" << rejected_fine << ", rejected_inlier=" << rejected_inlier
             << ", rejected_degenerate=" << rejected_degenerate
             << ", rejected_gate=" << rejected_gate << ", best_fine_inlier=" << best_fine_inlier_ratio
             << ", accepted_source_span_m=" << source_span
             << ", pass0_iterations=" << graph.batch_iterations()
             << ", pass0_iteration_s=" << iteration_seconds;
  for (const VerifiedLoop& loop : supported) {
    if (hooks && hooks->before_add_factor) hooks->before_add_factor();
    graph.add_loop_factor(poses[loop.candidate.older_index].index,
                          poses[loop.candidate.newer_index].index, loop.older_to_newer, loop.information);
    ++result.count;
  }
  if (result.count == 0) {
    statistics << ", removed=0, retained=0, elapsed_s=" << elapsed();
    result.statistics = statistics.str();
    return result;
  }
  std::string failure;
  int removals = 0;
  try {
    while (true) {
      const double reserve = pass_zero_marginals_failed ? 0.0 : marginals_seconds * kTimeSafetyFactor;
      const double remaining = parameters.time_budget_s - elapsed() - reserve;
      const int limit = loop_iteration_limit(remaining, iteration_seconds, batch_max_iterations);
      if (limit < 1) { failure = "time budget"; break; }
      if (hooks && hooks->before_batch) hooks->before_batch(limit);
      result.count = static_cast<int>(graph.loop_factor_count());
      if (!graph.batch_optimize(limit, true, false)) { failure = "optimization failed"; break; }
      const double measured_iteration_seconds =
          graph.batch_optimization_seconds() / std::max(1, graph.batch_iterations());
      statistics << ", pass_iterations=" << graph.batch_iterations()
                 << ", iteration_s=" << measured_iteration_seconds;
      iteration_seconds = std::max(iteration_seconds, measured_iteration_seconds);
      if (graph.batch_iterations() >= limit) {
        failure = limit < batch_max_iterations ? "time budget" : "not converged";
        break;
      }
      const std::vector<double> chi_squared = graph.loop_factor_chi_squared();
      const auto worst = std::max_element(chi_squared.begin(), chi_squared.end());
      if (worst == chi_squared.end() || *worst <= parameters.outlier_chi2) break;
      if (removals >= kMaximumRemovals) { failure = "inconsistent loops"; break; }
      statistics << ", removed chi2=" << *worst;
      const std::size_t removed_index = static_cast<std::size_t>(worst - chi_squared.begin());
      std::ostringstream removal_log;
      removal_log << "loop removed: older_index=" << active_loops[removed_index].candidate.older_index
                  << " newer_index=" << active_loops[removed_index].candidate.newer_index << " chi2=" << *worst;
      log_info(removal_log.str());
      active_loops.erase(active_loops.begin() + static_cast<std::ptrdiff_t>(removed_index));
      graph.remove_loop_factor(removed_index);
      --result.count;
      ++removals;
      if (result.count == 0) break;
    }
    if (failure.empty() && result.count > 0) {
      if (pass_zero_marginals_failed || elapsed() + marginals_seconds * kTimeSafetyFactor > parameters.time_budget_s) {
        result.warning = pass_zero_marginals_failed ? "covariance skipped: unavailable" : "covariance skipped: time budget";
      } else if (!graph.calculate_batch_marginals()) {
        result.warning = "covariance skipped: unavailable";
      }
      result.poses = graph.batch_poses();
    }
  } catch (const std::exception&) { failure = "optimization failed"; }
  if (failure.empty()) {
    for (const VerifiedLoop& loop : active_loops) {
      std::ostringstream retained_log;
      retained_log << "loop retained: older_index=" << loop.candidate.older_index
                   << " newer_index=" << loop.candidate.newer_index;
      log_info(retained_log.str());
    }
  }
  const int retained_count = result.count;
  graph.clear_loop_factors();
  result.count = retained_count;
  if (!failure.empty()) {
    result.count = 0;
    result.warning = "loop closure skipped: " + failure;
  }
  statistics << ", removed=" << removals << ", retained=" << result.count << ", elapsed_s=" << elapsed();
  result.statistics = statistics.str();
  return result;
}

int loop_iteration_limit(double remaining_s, double iteration_s, int batch_max_iterations) {
  if (remaining_s <= 0.0 || batch_max_iterations < 1) return 0;
  if (iteration_s <= 0.0) return batch_max_iterations;
  const double affordable = std::floor(remaining_s / (iteration_s * kTimeSafetyFactor));
  if (affordable < 1.0) return 0;
  return static_cast<int>(std::min(affordable, static_cast<double>(batch_max_iterations)));
}

}  // namespace lightnav_localization
