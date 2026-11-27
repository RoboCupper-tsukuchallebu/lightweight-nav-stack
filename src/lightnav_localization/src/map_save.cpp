#include "lightnav_localization/map_save.hpp"

#include <algorithm>
#include <limits>
#include <map>
#include <stdexcept>

namespace lightnav_localization {

std::vector<TrajectoryPoint> make_trajectory(const std::vector<KeyframeCloud>& keyframes,
                                             const std::vector<BatchPose>& poses) {
  std::map<int, const BatchPose*> pose_of_node;
  for (const BatchPose& pose : poses) pose_of_node[pose.index] = &pose;
  std::vector<TrajectoryPoint> trajectory;
  trajectory.reserve(keyframes.size());
  for (const KeyframeCloud& keyframe : keyframes) {
    const auto found = pose_of_node.find(keyframe.node_index);
    if (found == pose_of_node.end()) continue;
    const BatchPose& pose = *found->second;
    TrajectoryPoint point;
    point.node_index = keyframe.node_index;
    point.stamp_sec = keyframe.stamp_sec;
    point.map_to_base = pose.map_to_base;
    point.position_std_m = pose.position_and_yaw_std
                               ? std::max((*pose.position_and_yaw_std)[0], (*pose.position_and_yaw_std)[1])
                               : std::numeric_limits<double>::quiet_NaN();
    trajectory.push_back(point);
  }
  return trajectory;
}

bool close_loops(PoseGraph& graph, const std::vector<KeyframeCloud>& keyframes,
                 const LoopClosureParameters& parameters, int batch_max_iterations,
                 std::vector<LoopClosure>& closures, std::string& report, std::string& error) {
  try {
    for (int round = 1; round <= parameters.maximum_rounds; ++round) {
      std::map<int, Eigen::Isometry3d> pose_of_node;
      for (const BatchPose& pose : graph.batch_poses()) pose_of_node[pose.index] = pose.map_to_base;
      std::vector<KeyframeCloud> keyframes_with_pose;
      std::vector<Eigen::Isometry3d> keyframe_poses;
      for (const KeyframeCloud& keyframe : keyframes) {
        const auto found = pose_of_node.find(keyframe.node_index);
        if (found == pose_of_node.end()) continue;
        keyframes_with_pose.push_back(keyframe);
        keyframe_poses.push_back(found->second);
      }
      LoopClosureSearchSummary summary;
      const std::vector<LoopClosure> found_closures =
          find_loop_closures(keyframes_with_pose, keyframe_poses, parameters, closures, &summary);
      double largest_correction_m = 0.0;
      for (const LoopClosure& closure : found_closures) {
        graph.add_loop_closure_factor(closure.target_node_index, closure.query_node_index,
                                      closure.target_to_query, parameters.sigma_translation_m,
                                      parameters.sigma_rotation_rad);
        largest_correction_m = std::max(largest_correction_m, closure.correction_m);
        closures.push_back(closure);
      }
      report += "round " + std::to_string(round) + ": " + std::to_string(summary.query_count) +
                " queries, " + std::to_string(summary.candidate_count) + " candidates, " +
                std::to_string(summary.accepted_count) + " accepted (largest correction " +
                std::to_string(largest_correction_m) + " m)" +
                (summary.rejections.empty() ? "" : ", rejected: " + summary.rejections) + "; ";
      if (found_closures.empty()) break;
      if (!graph.batch_optimize(batch_max_iterations, false)) {
        error = "batch optimization with loop closures failed: " + graph.last_error();
        return false;
      }
    }
  } catch (const std::exception& exception) {
    error = std::string("loop closure detection failed: ") + exception.what();
    return false;
  }
  return true;
}

}  // namespace lightnav_localization
