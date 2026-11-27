// Offline loop-closure trial on a saved map directory (map.yaml, trajectory.csv, keyframes.bin).
// Development tool: runs the same detection as save_map() without replaying the bag.
//
//   loop_closure_probe <map directory> [name=value ...]
//
// Names: min_path_gap_m, query_spacing_m, search_radius_m, search_radius_per_m, max_search_radius_m,
// target_half_window, query_half_window, local_cloud_voxel_m, score_min, margin_min, peak_count,
// candidate_margin, height_min_m, height_max_m, correspondence_m, time_budget_s, coarse_translation_step_m.
// time_map_build=1 measures the map build of the save instead.

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <map>
#include <string>
#include <vector>

#include "lightnav_localization/loop_closure.hpp"
#include "lightnav_localization/map_builder.hpp"

using namespace lightnav_localization;

namespace {

bool apply_setting(LoopClosureParameters& parameters, const std::string& name, double value) {
  const std::map<std::string, double*> real_settings{
      {"min_path_gap_m", &parameters.minimum_path_gap_m},
      {"query_spacing_m", &parameters.query_spacing_m},
      {"search_radius_m", &parameters.search_radius_m},
      {"search_radius_per_m", &parameters.search_radius_per_m},
      {"max_search_radius_m", &parameters.maximum_search_radius_m},
      {"local_cloud_voxel_m", &parameters.local_cloud_voxel_m},
      {"score_min", &parameters.search.peaks.score_min},
      {"margin_min", &parameters.search.peaks.margin_min},
      {"candidate_margin", &parameters.search.peaks.candidate_margin},
      {"height_min_m", &parameters.search.height_min_m},
      {"height_max_m", &parameters.search.height_max_m},
      {"correspondence_m", &parameters.search.correspondence_m},
      {"time_budget_s", &parameters.search.time_budget_s},
      {"coarse_translation_step_m", &parameters.search.coarse_translation_step_m},
      {"descriptor_distance_max", &parameters.descriptor_distance_max},
      {"descriptor_search_radius_m", &parameters.descriptor_search_radius_m},
      {"descriptor_target_shift_m", &parameters.descriptor_target_shift_m},
      {"descriptor_max_correction_per_m", &parameters.descriptor_max_correction_per_m},
      {"descriptor_range_m", &parameters.descriptor.maximum_range_m},
  };
  const auto real_setting = real_settings.find(name);
  if (real_setting != real_settings.end()) {
    *real_setting->second = value;
    return true;
  }
  if (name == "target_half_window") { parameters.target_half_window = static_cast<int>(value); return true; }
  if (name == "query_half_window") { parameters.query_half_window = static_cast<int>(value); return true; }
  if (name == "peak_count") { parameters.search.peaks.peak_count = static_cast<std::size_t>(value); return true; }
  if (name == "descriptor_candidates") { parameters.descriptor_candidates = static_cast<int>(value); return true; }
  if (name == "descriptor_ring_key_candidates") {
    parameters.descriptor_ring_key_candidates = static_cast<int>(value);
    return true;
  }
  return false;
}

}  // namespace

int main(int argument_count, char** arguments) {
  if (argument_count < 2) {
    std::fprintf(stderr, "usage: loop_closure_probe <map directory> [name=value ...]\n");
    return 2;
  }
  const std::string directory = arguments[1];
  LoopClosureParameters parameters;
  parameters.enabled = true;
  // pair_target= and pair_query= (positions in keyframes.bin) test explicit pairs instead of the detection:
  // pair_count pairs, stepping the target forward and the query backward by pair_step keyframes.
  int forced_target = -1;
  int forced_query = -1;
  int forced_count = 1;
  int forced_step = 5;
  double forced_radius_m = 10.0;
  // time_map_build=1 only measures how long the map of the save takes to build from these keyframes.
  bool should_time_map_build = false;
  for (int index = 2; index < argument_count; ++index) {
    const std::string setting = arguments[index];
    const std::size_t separator = setting.find('=');
    if (separator != std::string::npos) {
      const std::string name = setting.substr(0, separator);
      const double value = std::atof(setting.substr(separator + 1).c_str());
      if (name == "pair_target") { forced_target = static_cast<int>(value); continue; }
      if (name == "pair_query") { forced_query = static_cast<int>(value); continue; }
      if (name == "pair_count") { forced_count = static_cast<int>(value); continue; }
      if (name == "pair_step") { forced_step = static_cast<int>(value); continue; }
      if (name == "pair_radius_m") { forced_radius_m = value; continue; }
      if (name == "time_map_build") { should_time_map_build = value != 0.0; continue; }
    }
    if (separator == std::string::npos ||
        !apply_setting(parameters, setting.substr(0, separator), std::atof(setting.substr(separator + 1).c_str()))) {
      std::fprintf(stderr, "unknown setting: %s\n", setting.c_str());
      return 2;
    }
  }
  try {
    const std::vector<KeyframeCloud> keyframes = read_keyframe_clouds(directory + "/keyframes.bin");
    const std::vector<TrajectoryPoint> trajectory = read_trajectory_csv(directory + "/trajectory.csv");
    if (keyframes.size() != trajectory.size()) {
      std::fprintf(stderr, "keyframes.bin has %zu keyframes but trajectory.csv has %zu rows\n",
                   keyframes.size(), trajectory.size());
      return 1;
    }
    std::vector<Eigen::Isometry3d> poses;
    poses.reserve(trajectory.size());
    for (const TrajectoryPoint& point : trajectory) poses.push_back(point.map_to_base);
    if (should_time_map_build) {
      const double map_voxel_size_m = 0.2;
      std::size_t keyframe_point_count = 0;
      for (const KeyframeCloud& keyframe : keyframes) keyframe_point_count += keyframe.points_in_base.size();
      const auto started = std::chrono::steady_clock::now();
      const PointList map_points = build_map_points(keyframes, trajectory, map_voxel_size_m);
      const double seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
      std::printf("keyframes %zu, keyframe points %zu, map points %zu, build_map_points %.2f s\n", keyframes.size(),
                  keyframe_point_count, map_points.size(), seconds);
      return 0;
    }
    if (forced_target >= 0 && forced_query >= 0) {
      // One explicit pair, searched around "the same place" (zero translation) instead of the estimate.
      std::printf("%8s %8s %9s %9s %8s %7s %7s %7s  %s\n", "target", "query", "gap[m]", "apart[m]", "radius",
                  "best", "second", "sec", "result (translation of the query in the target frame, yaw)");
      for (int offset = 0; offset < forced_count; ++offset) {
        const std::size_t target = static_cast<std::size_t>(forced_target + offset * forced_step);
        const std::size_t query = static_cast<std::size_t>(forced_query - offset * forced_step);
        if (target >= keyframes.size() || query >= keyframes.size()) break;
        Eigen::Isometry3d target_to_query = Eigen::Isometry3d::Identity();
        const LoopClosureCandidateReport report = match_keyframe_pair(
            keyframes, poses, target, query, Eigen::Vector2d::Zero(), forced_radius_m, parameters, target_to_query);
        std::printf("%8zu %8zu %9.1f %9.2f %8.1f %7.3f %7.3f %7.2f  ", target, query, report.path_gap_m,
                    report.estimate_distance_m, report.search_radius_m, report.best_score, report.second_score,
                    report.seconds);
        if (report.accepted) {
          std::printf("ACCEPTED (%.2f, %.2f) m, %.1f deg\n", target_to_query.translation().x(),
                      target_to_query.translation().y(),
                      std::atan2(target_to_query.linear()(1, 0), target_to_query.linear()(0, 0)) * 180.0 / M_PI);
        } else {
          std::printf("%s\n", report.rejection.c_str());
        }
      }
      return 0;
    }
    LoopClosureSearchSummary summary;
    const std::vector<LoopClosure> closures = find_loop_closures(keyframes, poses, parameters, {}, &summary);
    std::printf("keyframes %zu, queries %zu, candidates %zu, accepted %zu\n", keyframes.size(),
                summary.query_count, summary.candidate_count, summary.accepted_count);
    std::printf("%8s %8s %9s %9s %8s %7s %7s %10s %7s %11s  %s\n", "target", "query", "gap[m]", "apart[m]",
                "radius", "best", "second", "moved[m]", "sec", "descriptor", "result");
    for (const LoopClosureCandidateReport& report : summary.candidates) {
      char descriptor_text[16] = "-";
      if (report.is_descriptor_candidate) {
        std::snprintf(descriptor_text, sizeof(descriptor_text), "%.3f", report.descriptor_distance);
      }
      std::printf("%8d %8d %9.1f %9.2f %8.1f %7.3f %7.3f %10.2f %7.2f %11s  %s\n", report.target_node_index,
                  report.query_node_index, report.path_gap_m, report.estimate_distance_m,
                  report.search_radius_m, report.best_score, report.second_score, report.correction_m,
                  report.seconds, descriptor_text, report.accepted ? "ACCEPTED" : report.rejection.c_str());
    }
    (void)closures;
  } catch (const std::exception& exception) {
    std::fprintf(stderr, "error: %s\n", exception.what());
    return 1;
  }
  return 0;
}
