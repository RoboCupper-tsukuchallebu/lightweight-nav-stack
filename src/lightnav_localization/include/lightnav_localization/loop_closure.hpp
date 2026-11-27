#pragma once

// Loop closures for the mapping batch optimisation. No ROS types.
//
// Mapping has no absolute reference without GNSS, so the heading error of the odometry accumulates and a
// drive that returns to a place it has seen ends somewhere else on the map. This module looks for
// keyframe pairs that are far apart along the path but close on the current estimate, checks each pair by
// matching the two local clouds (the same 2D search + 3D verification that global initialisation uses),
// and returns the measured relative poses. The pose graph adds them as factors and optimises again.

#include <Eigen/Geometry>
#include <cstddef>
#include <string>
#include <vector>

#include "lightnav_localization/global_initialization.hpp"
#include "lightnav_localization/map_builder.hpp"

namespace lightnav_localization {

// Appearance of the surroundings of one keyframe, independent of where the estimate puts it: the highest
// point in every cell of a polar grid around the vehicle (the "scan context" idea). Two keyframes taken at
// the same place give similar descriptors whatever the heading, because a turn only shifts the sectors.
struct PlaceDescriptorParameters {
  int ring_count = 20;
  int sector_count = 60;
  double maximum_range_m = 40.0;
  double height_min_m = -0.5;  // relative to the base frame; cells keep (height - height_min_m)
  double height_max_m = 6.0;
};

struct PlaceDescriptor {
  int ring_count = 0;
  int sector_count = 0;
  std::vector<float> cells;      // ring * sector_count + sector
  std::vector<float> ring_key;   // occupied share of every ring: does not change with the heading
};

PlaceDescriptor make_place_descriptor(const PointList& points_in_base, const PlaceDescriptorParameters& parameters);
// 0 = identical, 1 = nothing in common; the smallest value over all sector shifts.
double place_descriptor_distance(const PlaceDescriptor& first, const PlaceDescriptor& second);

// The search of global_initialization.hpp with the thresholds of loop closing: the "too many coarse peaks"
// shortcut is off (candidate_margin 0, every one of 30 peaks is verified in 3D), and a verified match counts
// from score 0.5 with a 0.15 lead over the runner-up. Measured on the 2026-11-25 drive: true pairs score
// 0.60 to 0.68 (runner-up about 0.3), wrong pairs 0.2 to 0.35 with the runner-up within 0.02.
GlobalInitializationSearchParameters loop_closure_search_defaults();

struct LoopClosureParameters {
  bool enabled = false;
  // Two keyframes are a candidate pair only when the path between them is at least this long.
  double minimum_path_gap_m = 50.0;
  // One query keyframe every this distance along the path.
  double query_spacing_m = 10.0;
  // A target must lie within search_radius_m + search_radius_per_m * (path between the two keyframes),
  // capped at maximum_search_radius_m. The same radius bounds the translation search of the match.
  double search_radius_m = 5.0;
  double search_radius_per_m = 0.05;
  double maximum_search_radius_m = 20.0;
  // Keyframes on each side of the target / query keyframe that are merged into the matched clouds.
  int target_half_window = 5;
  int query_half_window = 1;
  double local_cloud_voxel_m = 0.25;
  // Noise of an accepted closure in the pose graph.
  double sigma_translation_m = 0.2;
  double sigma_rotation_rad = 0.03;
  // Detection and optimisation are repeated (a closure shrinks the error for the next search) up to this many times.
  int maximum_rounds = 3;
  // Candidates by appearance (place descriptors), for a drift too large for the radius above: the best
  // `descriptor_candidates` earlier keyframes of every query are matched around "the same place"
  // (zero translation) within descriptor_search_radius_m. 0 candidates turns this off.
  int descriptor_candidates = 1;
  int descriptor_ring_key_candidates = 20;
  // Measured on the 2026-11-25 drive: true pairs 0.22 to 0.36, most wrong pairs 0.38 to 0.50.
  double descriptor_distance_max = 0.4;
  double descriptor_search_radius_m = 10.0;
  // A descriptor candidate is also matched against the keyframes this far before and after it along the
  // path (0 = only the chosen keyframe).
  double descriptor_target_shift_m = 6.0;
  // An appearance candidate is tried only when closing it would move the query by at most this share of
  // the path between the two keyframes. No odometry is wrong by more, and a place that merely looks the
  // same (a second identical corridor) must not close a loop: one wrong closure ruins a good map.
  // The 2026-11-25 drive needed 0.13 (78 m over 600 m, after two heading jumps of 17 degrees).
  double descriptor_max_correction_per_m = 0.2;
  PlaceDescriptorParameters descriptor;
  // Wall-clock limit of one pair. Field resolution, steps, score thresholds: see global_initialization.hpp.
  // loop_closure_search_defaults() gives the values that suit keyframe-to-keyframe matching.
  GlobalInitializationSearchParameters search = loop_closure_search_defaults();
};

struct LoopClosure {
  int target_node_index = 0;  // the earlier keyframe
  int query_node_index = 0;   // the later keyframe
  Eigen::Isometry3d target_to_query = Eigen::Isometry3d::Identity();  // pose of the query base in the target base
  double score = 0.0;
  double correction_m = 0.0;  // how far the match moved the query from where the estimate had it
};

// What happened to one candidate pair (for logs and offline tuning).
struct LoopClosureCandidateReport {
  int target_node_index = 0;
  int query_node_index = 0;
  double path_gap_m = 0.0;
  double estimate_distance_m = 0.0;
  double search_radius_m = 0.0;
  bool accepted = false;
  std::string rejection;  // empty when accepted
  double best_score = 0.0;
  double second_score = 0.0;
  double correction_m = 0.0;
  double seconds = 0.0;
  bool is_descriptor_candidate = false;
  double descriptor_distance = 0.0;
};

struct LoopClosureSearchSummary {
  std::size_t query_count = 0;
  std::size_t candidate_count = 0;
  std::size_t accepted_count = 0;
  // Rejection reason of the matcher (e.g. "low_score", "ambiguous", "degenerate") -> count, as "reason=count"
  // items. "implausible" counts the appearance candidates that were not matched at all (see
  // descriptor_max_correction_per_m).
  std::string rejections;
  std::vector<LoopClosureCandidateReport> candidates;
};

// Matches the local cloud around keyframe `query` against the one around keyframe `target` (positions in
// `keyframes`). The translation of the query in the target frame is searched within `search_radius_m` of
// `expected_translation`; every heading is tried. The report says whether the match was accepted; on
// acceptance `target_to_query` is filled.
LoopClosureCandidateReport match_keyframe_pair(const std::vector<KeyframeCloud>& keyframes,
                                               const std::vector<Eigen::Isometry3d>& map_to_base,
                                               std::size_t target, std::size_t query,
                                               const Eigen::Vector2d& expected_translation,
                                               double search_radius_m,
                                               const LoopClosureParameters& parameters,
                                               Eigen::Isometry3d& target_to_query);

// `map_to_base` holds the current estimate of each keyframe, in the same order as `keyframes`.
// Pairs whose query lies within query_spacing_m (along the path) of an entry of `existing` are skipped.
// Throws std::invalid_argument when the two vectors differ in size or a parameter is not usable.
std::vector<LoopClosure> find_loop_closures(const std::vector<KeyframeCloud>& keyframes,
                                            const std::vector<Eigen::Isometry3d>& map_to_base,
                                            const LoopClosureParameters& parameters,
                                            const std::vector<LoopClosure>& existing,
                                            LoopClosureSearchSummary* summary = nullptr);

}  // namespace lightnav_localization
