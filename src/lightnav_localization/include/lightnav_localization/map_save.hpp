#pragma once

// The steps of Localizer::save_map() that only need the pose graph and the keyframe clouds. No ROS types.

#include <string>
#include <vector>

#include "lightnav_localization/loop_closure.hpp"
#include "lightnav_localization/map_builder.hpp"
#include "lightnav_localization/pose_graph.hpp"

namespace lightnav_localization {

// One trajectory row per keyframe that has a pose. Poses without a standard deviation (the stages before
// the last, or a failed marginal solve) give nan: it must not read as a perfect 0 m in trajectory.csv.
std::vector<TrajectoryPoint> make_trajectory(const std::vector<KeyframeCloud>& keyframes,
                                             const std::vector<BatchPose>& poses);

// Detects loop closures on the current batch result, adds them to the graph and optimises again, up to
// maximum_rounds times (a closure shrinks the error that the next search has to bridge). `closures` gets
// the accepted ones and `report` one item per round.
// Returns false with `error` set when the detection or an optimisation failed.
bool close_loops(PoseGraph& graph, const std::vector<KeyframeCloud>& keyframes,
                 const LoopClosureParameters& parameters, int batch_max_iterations,
                 std::vector<LoopClosure>& closures, std::string& report, std::string& error);

// The success message of ~/save_map, e.g.
//   saved 1899 keyframes, 1135156 map points to <dir>; loop closures: 6; 191.2 s
// "loop closures: off" when the save did not close loops.
std::string format_save_map_message(const MapOutputResult& result, const std::string& output_directory,
                                    bool did_close_loops);

// The success message of a ~/save_map that finds the map already saved (no keyframe added since).
extern const char* const kMapAlreadySavedMessage;

}  // namespace lightnav_localization
