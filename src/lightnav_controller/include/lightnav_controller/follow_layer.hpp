#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "lightnav_controller/types.hpp"

namespace lightnav_controller {

struct FollowParameters {
  double lookahead_m{1.0};
  double search_window_m{5.0};
  double v_max_mps{0.6};
  double v_min_mps{0.1};
  double v_slow_mps{0.25};
  double w_max_rps{0.4};
  double w_rotate_rps{0.3};
  double k_end_per_s{0.5};
  double goal_tolerance_m{0.3};
  double rotate_in_place_heading_rad{1.0};
};

struct FollowInputs {
  bool is_obstacle_status_fresh{false};  // received, not timed out, not is_input_stale
  bool is_stop_zone_occupied{false};
  bool is_slow_zone_occupied{false};
};

struct FollowResult {
  LayerOutput output;
  std::size_t closest_index{0};
  double remaining_distance_m{0.0};
  double path_deviation_m{0.0};
};

// Pure pursuit on a polyline. Holds the accepted path and the last nearest index.
class FollowLayer {
 public:
  explicit FollowLayer(const FollowParameters& parameters);

  // Acceptance rule: a different session_id always replaces the held path;
  // within the same session only path_id >= held path_id is accepted.
  // session_id 0 is invalid. Returns true when the path was taken.
  bool tryAcceptPath(std::uint32_t session_id, std::uint32_t path_id,
                     std::vector<Point2D> path_points);

  FollowResult compute(const Pose2D& pose, const FollowInputs& inputs);

  std::uint32_t sessionId() const { return session_id_; }
  std::uint32_t pathId() const { return path_id_; }
  const std::vector<Point2D>& pathPoints() const { return path_points_; }

 private:
  FollowParameters parameters_;
  std::uint32_t session_id_{0};
  std::uint32_t path_id_{0};
  std::vector<Point2D> path_points_;
  std::vector<double> cumulative_length_m_;  // arc length from index 0
  std::size_t closest_index_{0};
};

}  // namespace lightnav_controller
