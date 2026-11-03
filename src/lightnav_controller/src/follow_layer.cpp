#include "lightnav_controller/follow_layer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace lightnav_controller {

namespace {

constexpr double kMinimumCurvature = 1e-3;

double distanceBetween(const Point2D& first, const Point2D& second) {
  return std::hypot(first.x_m - second.x_m, first.y_m - second.y_m);
}

FollowResult makeStopResult(StopReason reason) {
  FollowResult result;
  result.output.reason = reason;
  return result;
}

}  // namespace

FollowLayer::FollowLayer(const FollowParameters& parameters) : parameters_(parameters) {}

bool FollowLayer::tryAcceptPath(std::uint32_t session_id, std::uint32_t path_id,
                                std::vector<Point2D> path_points) {
  if (session_id == 0) {
    return false;
  }
  const bool is_new_session = session_id != session_id_;
  if (!is_new_session && path_id < path_id_) {
    return false;
  }
  session_id_ = session_id;
  path_id_ = path_id;
  path_points_ = std::move(path_points);
  cumulative_length_m_.assign(path_points_.size(), 0.0);
  for (std::size_t index = 1; index < path_points_.size(); ++index) {
    cumulative_length_m_[index] =
        cumulative_length_m_[index - 1] + distanceBetween(path_points_[index - 1], path_points_[index]);
  }
  closest_index_ = 0;
  return true;
}

FollowResult FollowLayer::compute(const Pose2D& pose, const FollowInputs& inputs) {
  if (path_id_ == 0 || path_points_.empty()) {
    return makeStopResult(StopReason::kNoPath);
  }

  // Nearest point: search forward from the previous index within search_window_m.
  const Point2D body{pose.x_m, pose.y_m};
  double nearest_distance_m = std::numeric_limits<double>::infinity();
  std::size_t nearest_index = closest_index_;
  for (std::size_t index = closest_index_; index < path_points_.size(); ++index) {
    if (cumulative_length_m_[index] - cumulative_length_m_[closest_index_] >
        parameters_.search_window_m) {
      break;
    }
    const double distance_m = distanceBetween(body, path_points_[index]);
    if (distance_m < nearest_distance_m) {
      nearest_distance_m = distance_m;
      nearest_index = index;
    }
  }
  closest_index_ = nearest_index;

  FollowResult result;
  result.closest_index = nearest_index;
  result.path_deviation_m = nearest_distance_m;
  result.remaining_distance_m = cumulative_length_m_.back() - cumulative_length_m_[nearest_index];

  if (!inputs.is_obstacle_status_fresh) {
    result.output.reason = StopReason::kObstacleStale;
    return result;
  }
  if (inputs.is_stop_zone_occupied) {
    result.output.reason = StopReason::kBlocked;
    return result;
  }
  if (result.remaining_distance_m < parameters_.goal_tolerance_m) {
    result.output.reason = StopReason::kAtPathEnd;
    return result;
  }

  // Lookahead target: first point at least lookahead_m of arc length ahead.
  std::size_t target_index = nearest_index;
  while (target_index + 1 < path_points_.size() &&
         cumulative_length_m_[target_index] - cumulative_length_m_[nearest_index] <
             parameters_.lookahead_m) {
    ++target_index;
  }
  const Point2D& target = path_points_[target_index];
  const double delta_x_m = target.x_m - pose.x_m;
  const double delta_y_m = target.y_m - pose.y_m;
  const double forward_m = std::cos(pose.yaw_rad) * delta_x_m + std::sin(pose.yaw_rad) * delta_y_m;
  const double lateral_m = -std::sin(pose.yaw_rad) * delta_x_m + std::cos(pose.yaw_rad) * delta_y_m;
  const double squared_distance_m2 = delta_x_m * delta_x_m + delta_y_m * delta_y_m;
  const double heading_error_rad = std::atan2(lateral_m, forward_m);

  result.output.is_asserted = true;
  result.output.reason = StopReason::kOk;

  if (std::fabs(heading_error_rad) > parameters_.rotate_in_place_heading_rad ||
      squared_distance_m2 < kMinimumCurvature * kMinimumCurvature) {
    const double turn_sign = lateral_m >= 0.0 ? 1.0 : -1.0;
    result.output.twist = Twist2D{0.0, turn_sign * parameters_.w_rotate_rps};
    return result;
  }

  const double curvature_per_m = 2.0 * lateral_m / squared_distance_m2;
  const double curve_limit_mps =
      std::min(parameters_.v_max_mps,
               parameters_.w_max_rps / std::max(std::fabs(curvature_per_m), kMinimumCurvature));
  // The v_min floor does not apply near the end, where k_end must reach 0.
  double linear_mps = std::min(std::max(curve_limit_mps, parameters_.v_min_mps),
                               parameters_.k_end_per_s * result.remaining_distance_m);
  if (inputs.is_slow_zone_occupied) {
    linear_mps = std::min(linear_mps, parameters_.v_slow_mps);
  }
  const double angular_rps = std::clamp(linear_mps * curvature_per_m, -parameters_.w_max_rps,
                                        parameters_.w_max_rps);
  result.output.twist = Twist2D{linear_mps, angular_rps};
  return result;
}

}  // namespace lightnav_controller
