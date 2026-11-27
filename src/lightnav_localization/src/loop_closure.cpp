#include "lightnav_localization/loop_closure.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <limits>
#include <map>
#include <optional>
#include <stdexcept>

#include "lightnav_localization/downsample.hpp"

namespace lightnav_localization {
namespace {

Eigen::Isometry3d invert_isometry(const Eigen::Isometry3d& transform) {
  Eigen::Isometry3d inverse = Eigen::Isometry3d::Identity();
  inverse.linear() = transform.linear().transpose();
  inverse.translation() = -(inverse.linear() * transform.translation());
  return inverse;
}

double horizontal_distance(const Eigen::Isometry3d& first, const Eigen::Isometry3d& second) {
  return (first.translation() - second.translation()).head<2>().norm();
}

// Path length from the first keyframe to each keyframe, on the current estimate.
std::vector<double> cumulative_path_length(const std::vector<Eigen::Isometry3d>& map_to_base) {
  std::vector<double> lengths(map_to_base.size(), 0.0);
  for (std::size_t index = 1; index < map_to_base.size(); ++index) {
    lengths[index] = lengths[index - 1] + horizontal_distance(map_to_base[index - 1], map_to_base[index]);
  }
  return lengths;
}

// Clouds of the keyframes center-half_window .. center+half_window, expressed in the base frame of `center`.
PointList merge_local_cloud(const std::vector<KeyframeCloud>& keyframes,
                            const std::vector<Eigen::Isometry3d>& map_to_base, std::size_t center,
                            int half_window, double voxel_size_m) {
  const std::size_t first = center >= static_cast<std::size_t>(half_window) ? center - half_window : 0;
  const std::size_t last = std::min(keyframes.size() - 1, center + static_cast<std::size_t>(half_window));
  const Eigen::Isometry3d center_from_map = invert_isometry(map_to_base[center]);
  PointList merged;
  for (std::size_t index = first; index <= last; ++index) {
    const Eigen::Isometry3f center_from_base = (center_from_map * map_to_base[index]).cast<float>();
    for (const Eigen::Vector3f& point : keyframes[index].points_in_base) merged.push_back(center_from_base * point);
  }
  return downsample_voxel(merged, voxel_size_m);
}

double search_radius(const LoopClosureParameters& parameters, double path_gap_m) {
  return std::min(parameters.maximum_search_radius_m,
                  parameters.search_radius_m + parameters.search_radius_per_m * path_gap_m);
}

// The earlier keyframe that is closest to `query` on the estimate, among those far enough along the path
// and inside their own search radius.
std::optional<std::size_t> nearest_target(const std::vector<Eigen::Isometry3d>& map_to_base,
                                          const std::vector<double>& path_length, std::size_t query,
                                          const LoopClosureParameters& parameters) {
  std::optional<std::size_t> best;
  double best_distance = std::numeric_limits<double>::infinity();
  for (std::size_t target = 0; target < query; ++target) {
    const double path_gap = path_length[query] - path_length[target];
    if (path_gap < parameters.minimum_path_gap_m) break;  // later targets are even closer along the path
    const double distance = horizontal_distance(map_to_base[query], map_to_base[target]);
    if (distance <= search_radius(parameters, path_gap) && distance < best_distance) {
      best = target;
      best_distance = distance;
    }
  }
  return best;
}

// Earlier keyframes that look most like `query`: ring keys pick a short list, the full descriptors rank it.
std::vector<std::pair<double, std::size_t>> descriptor_targets(
    const std::vector<PlaceDescriptor>& descriptors, const std::vector<double>& path_length, std::size_t query,
    const LoopClosureParameters& parameters) {
  std::vector<std::pair<double, std::size_t>> by_ring_key;
  for (std::size_t target = 0; target < query; ++target) {
    if (path_length[query] - path_length[target] < parameters.minimum_path_gap_m) break;
    double squared_difference = 0.0;
    for (std::size_t ring = 0; ring < descriptors[query].ring_key.size(); ++ring) {
      const double difference = descriptors[query].ring_key[ring] - descriptors[target].ring_key[ring];
      squared_difference += difference * difference;
    }
    by_ring_key.emplace_back(squared_difference, target);
  }
  const std::size_t short_list_size =
      std::min(by_ring_key.size(), static_cast<std::size_t>(std::max(1, parameters.descriptor_ring_key_candidates)));
  std::partial_sort(by_ring_key.begin(), by_ring_key.begin() + static_cast<std::ptrdiff_t>(short_list_size),
                    by_ring_key.end());
  std::vector<std::pair<double, std::size_t>> ranked;
  for (std::size_t index = 0; index < short_list_size; ++index) {
    const std::size_t target = by_ring_key[index].second;
    const double distance = place_descriptor_distance(descriptors[query], descriptors[target]);
    if (distance <= parameters.descriptor_distance_max) ranked.emplace_back(distance, target);
  }
  std::sort(ranked.begin(), ranked.end());
  return ranked;
}

bool is_usable(const LoopClosureParameters& parameters) {
  const double positive_values[] = {parameters.minimum_path_gap_m, parameters.query_spacing_m,
                                    parameters.maximum_search_radius_m, parameters.local_cloud_voxel_m,
                                    parameters.sigma_translation_m, parameters.sigma_rotation_rad};
  for (const double value : positive_values) {
    if (!std::isfinite(value) || value <= 0.0) return false;
  }
  return std::isfinite(parameters.search_radius_m) && parameters.search_radius_m >= 0.0 &&
         std::isfinite(parameters.search_radius_per_m) && parameters.search_radius_per_m >= 0.0 &&
         std::isfinite(parameters.descriptor_target_shift_m) && parameters.descriptor_target_shift_m >= 0.0 &&
         std::isfinite(parameters.descriptor_max_correction_per_m) &&
         parameters.descriptor_max_correction_per_m >= 0.0 &&
         parameters.target_half_window >= 0 && parameters.query_half_window >= 0 &&
         parameters.maximum_rounds >= 1;
}

}  // namespace

GlobalInitializationSearchParameters loop_closure_search_defaults() {
  GlobalInitializationSearchParameters search;
  search.peaks.candidate_margin = 0.0;
  search.peaks.peak_count = 30;
  search.peaks.score_min = 0.5;
  search.peaks.margin_min = 0.15;
  search.time_budget_s = 60.0;
  return search;
}

PlaceDescriptor make_place_descriptor(const PointList& points_in_base, const PlaceDescriptorParameters& parameters) {
  if (parameters.ring_count < 1 || parameters.sector_count < 1 || !(parameters.maximum_range_m > 0.0) ||
      !(parameters.height_max_m > parameters.height_min_m)) {
    throw std::invalid_argument("make_place_descriptor: invalid parameters");
  }
  PlaceDescriptor descriptor;
  descriptor.ring_count = parameters.ring_count;
  descriptor.sector_count = parameters.sector_count;
  descriptor.cells.assign(static_cast<std::size_t>(parameters.ring_count * parameters.sector_count), 0.0F);
  const double ring_width_m = parameters.maximum_range_m / parameters.ring_count;
  const double sector_width_rad = 2.0 * M_PI / parameters.sector_count;
  for (const Eigen::Vector3f& point : points_in_base) {
    if (!point.allFinite() || point.z() < parameters.height_min_m || point.z() > parameters.height_max_m) continue;
    const double range = point.head<2>().norm();
    if (range >= parameters.maximum_range_m) continue;
    const int ring = std::min(parameters.ring_count - 1, static_cast<int>(range / ring_width_m));
    const double angle = std::atan2(point.y(), point.x()) + M_PI;
    const int sector = std::min(parameters.sector_count - 1, static_cast<int>(angle / sector_width_rad));
    float& cell = descriptor.cells[static_cast<std::size_t>(ring * parameters.sector_count + sector)];
    cell = std::max(cell, static_cast<float>(point.z() - parameters.height_min_m));
  }
  descriptor.ring_key.assign(static_cast<std::size_t>(parameters.ring_count), 0.0F);
  for (int ring = 0; ring < parameters.ring_count; ++ring) {
    int occupied = 0;
    for (int sector = 0; sector < parameters.sector_count; ++sector) {
      if (descriptor.cells[static_cast<std::size_t>(ring * parameters.sector_count + sector)] > 0.0F) ++occupied;
    }
    descriptor.ring_key[static_cast<std::size_t>(ring)] = static_cast<float>(occupied) / parameters.sector_count;
  }
  return descriptor;
}

double place_descriptor_distance(const PlaceDescriptor& first, const PlaceDescriptor& second) {
  if (first.ring_count != second.ring_count || first.sector_count != second.sector_count || first.cells.empty()) {
    throw std::invalid_argument("place_descriptor_distance: descriptors differ in shape");
  }
  const int ring_count = first.ring_count;
  const int sector_count = first.sector_count;
  // Squared norm of every sector column, once.
  const auto column_norms = [&](const PlaceDescriptor& descriptor) {
    std::vector<double> norms(static_cast<std::size_t>(sector_count), 0.0);
    for (int sector = 0; sector < sector_count; ++sector) {
      for (int ring = 0; ring < ring_count; ++ring) {
        const double value = descriptor.cells[static_cast<std::size_t>(ring * sector_count + sector)];
        norms[static_cast<std::size_t>(sector)] += value * value;
      }
    }
    return norms;
  };
  const std::vector<double> first_norms = column_norms(first);
  const std::vector<double> second_norms = column_norms(second);
  double best_distance = 1.0;
  for (int shift = 0; shift < sector_count; ++shift) {
    double similarity_sum = 0.0;
    int compared_columns = 0;
    for (int sector = 0; sector < sector_count; ++sector) {
      const int shifted = (sector + shift) % sector_count;
      const double first_norm = first_norms[static_cast<std::size_t>(sector)];
      const double second_norm = second_norms[static_cast<std::size_t>(shifted)];
      if (first_norm <= 0.0 && second_norm <= 0.0) continue;  // empty in both: says nothing
      ++compared_columns;
      if (first_norm <= 0.0 || second_norm <= 0.0) continue;  // empty in one: similarity 0
      double dot = 0.0;
      for (int ring = 0; ring < ring_count; ++ring) {
        dot += static_cast<double>(first.cells[static_cast<std::size_t>(ring * sector_count + sector)]) *
               second.cells[static_cast<std::size_t>(ring * sector_count + shifted)];
      }
      similarity_sum += dot / std::sqrt(first_norm * second_norm);
    }
    if (compared_columns == 0) continue;
    best_distance = std::min(best_distance, 1.0 - similarity_sum / compared_columns);
  }
  return best_distance;
}

LoopClosureCandidateReport match_keyframe_pair(const std::vector<KeyframeCloud>& keyframes,
                                               const std::vector<Eigen::Isometry3d>& map_to_base,
                                               std::size_t target, std::size_t query,
                                               const Eigen::Vector2d& expected_translation,
                                               double search_radius_m,
                                               const LoopClosureParameters& parameters,
                                               Eigen::Isometry3d& target_to_query) {
  const std::vector<double> path_length = cumulative_path_length(map_to_base);
  const PointList target_cloud = merge_local_cloud(keyframes, map_to_base, target, parameters.target_half_window,
                                                   parameters.local_cloud_voxel_m);
  const PointList query_cloud = merge_local_cloud(keyframes, map_to_base, query, parameters.query_half_window,
                                                  parameters.local_cloud_voxel_m);
  GlobalInitializationSearchParameters search = parameters.search;
  search.gnss_window.reset();
  const Eigen::Vector2d half_extent = Eigen::Vector2d::Constant(search_radius_m);
  search.minimum = expected_translation - half_extent;
  search.maximum = expected_translation + half_extent;
  search.maximum_area_m2 = std::max(search.maximum_area_m2, 4.0 * search_radius_m * search_radius_m + 1.0);

  const std::atomic<bool> abort{false};
  const auto search_started = std::chrono::steady_clock::now();
  const GlobalInitializationSearchResult result =
      search_global_initialization(target_cloud, query_cloud, search, abort);
  LoopClosureCandidateReport report;
  report.target_node_index = keyframes[target].node_index;
  report.query_node_index = keyframes[query].node_index;
  report.path_gap_m = path_length[query] - path_length[target];
  report.estimate_distance_m = horizontal_distance(map_to_base[query], map_to_base[target]);
  report.search_radius_m = search_radius_m;
  report.accepted = result.decision.accepted;
  report.best_score = result.decision.best_score;
  report.second_score = result.decision.second_score;
  report.seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - search_started).count();
  if (!result.decision.accepted) {
    report.rejection = result.decision.error.empty() ? "not_accepted" : result.decision.error;
    return report;
  }
  target_to_query = result.pose;
  const Eigen::Isometry3d predicted = invert_isometry(map_to_base[target]) * map_to_base[query];
  report.correction_m = (result.pose.translation() - predicted.translation()).head<2>().norm();
  return report;
}

namespace {

// The keyframe whose path length is closest to that of `target` + `offset_m`, as long as it stays at
// least `minimum_path_gap_m` before `query` along the path.
std::optional<std::size_t> target_at_path_offset(const std::vector<double>& path_length, std::size_t target,
                                                 double offset_m, std::size_t query,
                                                 double minimum_path_gap_m) {
  const double wanted_path_length = path_length[target] + offset_m;
  const auto upper = std::lower_bound(path_length.begin(), path_length.end(), wanted_path_length);
  std::size_t closest = static_cast<std::size_t>(std::distance(path_length.begin(), upper));
  if (closest >= path_length.size()) closest = path_length.size() - 1;
  if (closest > 0 && std::abs(path_length[closest - 1] - wanted_path_length) <
                         std::abs(path_length[closest] - wanted_path_length)) {
    --closest;
  }
  if (path_length[query] - path_length[closest] < minimum_path_gap_m) return std::nullopt;
  return closest;
}

// One candidate pair: where to look for the query in the target frame, and how far around it.
struct Candidate {
  std::size_t target = 0;
  Eigen::Vector2d expected_translation = Eigen::Vector2d::Zero();
  double radius_m = 0.0;
  bool is_descriptor_candidate = false;
  double descriptor_distance = 0.0;
};

// Appearance candidates of `query`: the best-ranked keyframes and, for each, the keyframes a few metres
// before and after it (the place is right, but the local map around the chosen keyframe can be poor: a
// turn on the spot smears it). Candidates that would move the estimate implausibly far are counted in
// `implausible_count` and left out.
std::vector<Candidate> descriptor_candidates(const std::vector<Eigen::Isometry3d>& map_to_base,
                                             const std::vector<double>& path_length,
                                             const std::vector<PlaceDescriptor>& descriptors, std::size_t query,
                                             const LoopClosureParameters& parameters,
                                             std::size_t& implausible_count) {
  std::vector<Candidate> candidates;
  const auto ranked = descriptor_targets(descriptors, path_length, query, parameters);
  for (std::size_t index = 0; index < ranked.size() &&
                              index < static_cast<std::size_t>(parameters.descriptor_candidates); ++index) {
    for (const double shift_m :
         {0.0, -parameters.descriptor_target_shift_m, parameters.descriptor_target_shift_m}) {
      const std::optional<std::size_t> target = target_at_path_offset(
          path_length, ranked[index].second, shift_m, query, parameters.minimum_path_gap_m);
      const bool is_new_target = target && std::none_of(
          candidates.begin(), candidates.end(),
          [&](const Candidate& existing_candidate) { return existing_candidate.target == *target; });
      if (!is_new_target) continue;
      // The match is searched around "the same place", so it would move the query by at least this much.
      const double smallest_correction_m = horizontal_distance(map_to_base[query], map_to_base[*target]) -
                                           parameters.descriptor_search_radius_m;
      const double plausible_correction_m =
          parameters.descriptor_max_correction_per_m * (path_length[query] - path_length[*target]);
      if (smallest_correction_m > plausible_correction_m) {
        ++implausible_count;
        continue;
      }
      Candidate candidate;
      candidate.target = *target;
      candidate.radius_m = parameters.descriptor_search_radius_m;
      candidate.is_descriptor_candidate = true;
      candidate.descriptor_distance = ranked[index].first;
      candidates.push_back(candidate);
    }
  }
  return candidates;
}

// Every pair to match for `query`: the nearest earlier keyframe on the estimate first, then the appearance
// candidates (`descriptors` is empty when they are off).
std::vector<Candidate> collect_candidates(const std::vector<Eigen::Isometry3d>& map_to_base,
                                          const std::vector<double>& path_length,
                                          const std::vector<PlaceDescriptor>& descriptors, std::size_t query,
                                          const LoopClosureParameters& parameters,
                                          std::size_t& implausible_count) {
  std::vector<Candidate> candidates;
  if (const std::optional<std::size_t> target = nearest_target(map_to_base, path_length, query, parameters)) {
    Candidate candidate;
    candidate.target = *target;
    candidate.expected_translation =
        (invert_isometry(map_to_base[*target]) * map_to_base[query]).translation().head<2>();
    candidate.radius_m = search_radius(parameters, path_length[query] - path_length[*target]);
    candidates.push_back(candidate);
  }
  if (descriptors.empty()) return candidates;
  const std::vector<Candidate> by_appearance =
      descriptor_candidates(map_to_base, path_length, descriptors, query, parameters, implausible_count);
  candidates.insert(candidates.end(), by_appearance.begin(), by_appearance.end());
  return candidates;
}

}  // namespace

std::vector<LoopClosure> find_loop_closures(const std::vector<KeyframeCloud>& keyframes,
                                            const std::vector<Eigen::Isometry3d>& map_to_base,
                                            const LoopClosureParameters& parameters,
                                            const std::vector<LoopClosure>& existing,
                                            LoopClosureSearchSummary* summary) {
  if (keyframes.size() != map_to_base.size()) {
    throw std::invalid_argument("find_loop_closures: keyframes and poses differ in size");
  }
  if (!is_usable(parameters)) throw std::invalid_argument("find_loop_closures: invalid parameters");

  std::vector<LoopClosure> closures;
  LoopClosureSearchSummary local_summary;
  std::map<std::string, std::size_t> rejections;
  const std::vector<double> path_length = cumulative_path_length(map_to_base);

  std::map<int, std::size_t> position_of_node;
  for (std::size_t index = 0; index < keyframes.size(); ++index) position_of_node[keyframes[index].node_index] = index;
  std::vector<double> closed_query_path_lengths;
  for (const LoopClosure& closure : existing) {
    const auto found = position_of_node.find(closure.query_node_index);
    if (found != position_of_node.end()) closed_query_path_lengths.push_back(path_length[found->second]);
  }

  std::vector<PlaceDescriptor> descriptors;
  if (parameters.descriptor_candidates > 0) {
    descriptors.reserve(keyframes.size());
    for (std::size_t index = 0; index < keyframes.size(); ++index) {
      descriptors.push_back(make_place_descriptor(
          merge_local_cloud(keyframes, map_to_base, index, parameters.query_half_window, parameters.local_cloud_voxel_m),
          parameters.descriptor));
    }
  }

  double next_query_path_length = parameters.minimum_path_gap_m;
  for (std::size_t query = 0; query < keyframes.size(); ++query) {
    // The last keyframe is always a query: a drive that ends where it started closes its loop there, and the
    // vehicle often stands still at the end (keyframes by time, no path length), so the spacing would skip it.
    const bool is_last_keyframe = query + 1 == keyframes.size();
    if (path_length[query] < next_query_path_length && !is_last_keyframe) continue;
    next_query_path_length = path_length[query] + parameters.query_spacing_m;
    ++local_summary.query_count;
    const bool is_already_closed = std::any_of(
        closed_query_path_lengths.begin(), closed_query_path_lengths.end(),
        [&](double closed) { return std::abs(closed - path_length[query]) < parameters.query_spacing_m; });
    if (is_already_closed) continue;

    std::size_t implausible_count = 0;
    const std::vector<Candidate> candidates =
        collect_candidates(map_to_base, path_length, descriptors, query, parameters, implausible_count);
    if (implausible_count > 0) rejections["implausible"] += implausible_count;
    for (const Candidate& candidate : candidates) {
      ++local_summary.candidate_count;
      Eigen::Isometry3d target_to_query = Eigen::Isometry3d::Identity();
      LoopClosureCandidateReport report = match_keyframe_pair(
          keyframes, map_to_base, candidate.target, query, candidate.expected_translation, candidate.radius_m,
          parameters, target_to_query);
      report.is_descriptor_candidate = candidate.is_descriptor_candidate;
      report.descriptor_distance = candidate.descriptor_distance;
      local_summary.candidates.push_back(report);
      if (!report.accepted) {
        ++rejections[report.rejection];
        continue;
      }
      LoopClosure closure;
      closure.target_node_index = report.target_node_index;
      closure.query_node_index = report.query_node_index;
      closure.target_to_query = target_to_query;
      closure.score = report.best_score;
      closure.correction_m = report.correction_m;
      closures.push_back(closure);
      ++local_summary.accepted_count;
      break;  // one closure per query
    }
  }
  for (const auto& entry : rejections) {
    if (!local_summary.rejections.empty()) local_summary.rejections += ", ";
    local_summary.rejections += entry.first + "=" + std::to_string(entry.second);
  }
  if (summary) *summary = local_summary;
  return closures;
}

}  // namespace lightnav_localization
