#include "lightnav_controller/avoid_layer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace lightnav_controller {

namespace {

constexpr std::int8_t kOccupiedThreshold = 50;
constexpr double kStraightCurvature = 1e-6;

double distanceBetween(const Point2D& first, const Point2D& second) {
  return std::hypot(first.x_m - second.x_m, first.y_m - second.y_m);
}

double wrapAngle(double angle_rad) { return std::atan2(std::sin(angle_rad), std::cos(angle_rad)); }

Point2D toBody(const Pose2D& pose, const Point2D& world_point) {
  const double delta_x_m = world_point.x_m - pose.x_m;
  const double delta_y_m = world_point.y_m - pose.y_m;
  return Point2D{std::cos(pose.yaw_rad) * delta_x_m + std::sin(pose.yaw_rad) * delta_y_m,
                 -std::sin(pose.yaw_rad) * delta_x_m + std::cos(pose.yaw_rad) * delta_y_m};
}

Point2D toWorld(const Pose2D& pose, const Point2D& body_point) {
  return Point2D{
      pose.x_m + std::cos(pose.yaw_rad) * body_point.x_m - std::sin(pose.yaw_rad) * body_point.y_m,
      pose.y_m + std::sin(pose.yaw_rad) * body_point.x_m + std::cos(pose.yaw_rad) * body_point.y_m};
}

// Position on an arc of the given curvature after travelling arc_length_m.
Point2D arcPointInBody(double curvature_per_m, double arc_length_m) {
  if (std::fabs(curvature_per_m) < kStraightCurvature) {
    return Point2D{arc_length_m, 0.0};
  }
  return Point2D{std::sin(curvature_per_m * arc_length_m) / curvature_per_m,
                 (1.0 - std::cos(curvature_per_m * arc_length_m)) / curvature_per_m};
}

double distanceToSegment(const Point2D& point, const Point2D& start, const Point2D& end) {
  const double segment_x_m = end.x_m - start.x_m;
  const double segment_y_m = end.y_m - start.y_m;
  const double squared_length_m2 = segment_x_m * segment_x_m + segment_y_m * segment_y_m;
  if (squared_length_m2 <= 0.0) {
    return distanceBetween(point, start);
  }
  const double ratio = std::clamp(((point.x_m - start.x_m) * segment_x_m +
                                   (point.y_m - start.y_m) * segment_y_m) / squared_length_m2,
                                  0.0, 1.0);
  return distanceBetween(point, Point2D{start.x_m + ratio * segment_x_m, start.y_m + ratio * segment_y_m});
}

// Searches only path_points[first, last]; callers bound the window so a path
// that doubles back is not matched against a far-away section.
double distanceToPath(const Point2D& point, const std::vector<Point2D>& path_points,
                      std::size_t first, std::size_t last) {
  double nearest_m = distanceBetween(point, path_points[first]);
  for (std::size_t index = first + 1; index <= last; ++index) {
    nearest_m = std::min(nearest_m, distanceToSegment(point, path_points[index - 1], path_points[index]));
  }
  return nearest_m;
}

// Last index within distance_m of arc length ahead of first.
std::size_t windowEnd(const std::vector<Point2D>& path_points, std::size_t first, double distance_m) {
  std::size_t index = first;
  double travelled_m = 0.0;
  while (index + 1 < path_points.size() && travelled_m < distance_m) {
    travelled_m += distanceBetween(path_points[index], path_points[index + 1]);
    ++index;
  }
  return index;
}

// First path point at least distance_m of arc length ahead of closest_index
// (or the last point when the path is shorter).
Point2D pathPointAhead(const std::vector<Point2D>& path_points, std::size_t closest_index,
                       double distance_m) {
  std::size_t index = std::min(closest_index, path_points.size() - 1);
  double travelled_m = 0.0;
  while (index + 1 < path_points.size() && travelled_m < distance_m) {
    travelled_m += distanceBetween(path_points[index], path_points[index + 1]);
    ++index;
  }
  return path_points[index];
}

// True when any path point within ahead_m of arc length from closest_index is
// on an occupied cell.
bool isPathBlocked(const LocalGrid& grid, const Pose2D& pose,
                   const std::vector<Point2D>& path_points, std::size_t closest_index,
                   double ahead_m) {
  if (path_points.empty()) {
    return false;
  }
  double travelled_m = 0.0;
  for (std::size_t index = std::min(closest_index, path_points.size() - 1);
       index < path_points.size(); ++index) {
    if (index > closest_index) {
      travelled_m += distanceBetween(path_points[index - 1], path_points[index]);
    }
    if (travelled_m > ahead_m) {
      break;
    }
    const Point2D body_point = toBody(pose, path_points[index]);
    if (grid.isOccupiedInBody(body_point.x_m, body_point.y_m)) {
      return true;
    }
  }
  return false;
}

}  // namespace

bool LocalGrid::isOccupiedInBody(double body_x_m, double body_y_m) const {
  const double column = std::floor((body_x_m - origin_x_m) / resolution_m);
  const double row = std::floor((body_y_m - origin_y_m) / resolution_m);
  if (column < 0.0 || row < 0.0 || column >= static_cast<double>(width_cells) ||
      row >= static_cast<double>(height_cells)) {
    return false;
  }
  const std::size_t cell_index = static_cast<std::size_t>(row) * width_cells + static_cast<std::size_t>(column);
  return cell_index < cells.size() && cells[cell_index] >= kOccupiedThreshold;
}

std::vector<ArcCandidate> evaluateArcCandidates(const LocalGrid& grid, const Pose2D& pose,
                                                const std::vector<Point2D>& path_points,
                                                std::size_t closest_index,
                                                double previous_curvature_per_m,
                                                const AvoidParameters& parameters) {
  std::vector<ArcCandidate> candidates;
  if (parameters.arc_candidate_count < 1 || path_points.empty() ||
      !std::isfinite(parameters.arc_length_m) || parameters.arc_length_m <= 0.0 ||
      !std::isfinite(parameters.arc_step_m) || parameters.arc_step_m <= 0.0 ||
      !std::isfinite(parameters.max_path_deviation_m) || parameters.max_path_deviation_m <= 0.0) {
    return candidates;
  }
  const double curvature_limit = parameters.w_max_rps / parameters.v_avoid_mps;
  const std::size_t first_index = std::min(closest_index, path_points.size() - 1);
  const std::size_t last_index = windowEnd(
      path_points, first_index, parameters.arc_length_m + 2.0 * parameters.max_path_deviation_m);
  const Point2D target = pathPointAhead(path_points, closest_index, parameters.arc_length_m);
  candidates.reserve(static_cast<std::size_t>(parameters.arc_candidate_count));

  for (int candidate_index = 0; candidate_index < parameters.arc_candidate_count; ++candidate_index) {
    const double ratio = parameters.arc_candidate_count == 1
                             ? 0.5
                             : static_cast<double>(candidate_index) /
                                   static_cast<double>(parameters.arc_candidate_count - 1);
    ArcCandidate candidate;
    candidate.curvature_per_m = -curvature_limit + 2.0 * curvature_limit * ratio;

    candidate.is_feasible = true;
    for (double travelled_m = parameters.arc_step_m;
         travelled_m <= parameters.arc_length_m + 1e-9; travelled_m += parameters.arc_step_m) {
      const Point2D sample = arcPointInBody(candidate.curvature_per_m, travelled_m);
      if (grid.isOccupiedInBody(sample.x_m, sample.y_m) ||
          distanceToPath(toWorld(pose, sample), path_points, first_index, last_index) >
              parameters.max_path_deviation_m) {
        candidate.is_feasible = false;
        break;
      }
    }
    if (candidate.is_feasible) {
      const Point2D end_world =
          toWorld(pose, arcPointInBody(candidate.curvature_per_m, parameters.arc_length_m));
      const double deviation_m = distanceToPath(end_world, path_points, first_index, last_index);
      {
        const double end_yaw_rad = pose.yaw_rad + candidate.curvature_per_m * parameters.arc_length_m;
        const double bearing_rad = std::atan2(target.y_m - end_world.y_m, target.x_m - end_world.x_m);
        candidate.score = parameters.w_path * deviation_m +
                          parameters.w_heading * std::fabs(wrapAngle(bearing_rad - end_yaw_rad)) +
                          parameters.w_smooth *
                              std::fabs(candidate.curvature_per_m - previous_curvature_per_m);
      }
    }
    candidates.push_back(candidate);
  }
  return candidates;
}

std::optional<ArcCandidate> selectBestCandidate(const std::vector<ArcCandidate>& candidates) {
  std::optional<ArcCandidate> best;
  for (const ArcCandidate& candidate : candidates) {
    if (candidate.is_feasible && (!best.has_value() || candidate.score < best->score)) {
      best = candidate;
    }
  }
  return best;
}

bool ArcAvoidLayer::isPathEndOccupied(const LocalGrid& grid, const AvoidContext& context) const {
  const Point2D body_point = toBody(context.pose, context.path_points.back());
  return grid.isOccupiedInBody(body_point.x_m, body_point.y_m);
}

ArcAvoidLayer::ArcAvoidLayer(const AvoidParameters& parameters) : parameters_(parameters) {}

void ArcAvoidLayer::reset() {
  is_avoiding_ = false;
  release_since_s_.reset();
  waiting_since_s_.reset();
  previous_curvature_per_m_ = 0.0;
}

LayerOutput ArcAvoidLayer::compute(const AvoidContext& context) {
  if (!context.is_grid_fresh || context.grid == nullptr || context.path_points.empty()) {
    reset();
    return LayerOutput{};
  }
  const LocalGrid& grid = *context.grid;
  const bool is_trigger_blocked = isPathBlocked(grid, context.pose, context.path_points,
                                                context.closest_index, parameters_.avoid_trigger_m);
  if (!is_avoiding_ && !is_trigger_blocked) {
    return LayerOutput{};
  }
  is_avoiding_ = true;

  const bool is_release_clear = !isPathBlocked(grid, context.pose, context.path_points,
                                               context.closest_index, parameters_.avoid_release_m);
  const std::size_t closest = std::min(context.closest_index, context.path_points.size() - 1);
  const bool is_near_path =
      distanceBetween(Point2D{context.pose.x_m, context.pose.y_m}, context.path_points[closest]) <=
      parameters_.rejoin_distance_m;
  if (is_release_clear && is_near_path) {
    if (!release_since_s_.has_value()) {
      release_since_s_ = context.now_s;
    }
    if (context.now_s - *release_since_s_ >= parameters_.avoid_release_hold_s) {
      reset();
      return LayerOutput{};
    }
  } else {
    release_since_s_.reset();
  }

  const bool is_at_path_end = context.remaining_distance_m < parameters_.goal_tolerance_m;
  if (is_at_path_end && !isPathEndOccupied(grid, context)) {
    reset();  // follow layer reports AT_PATH_END
    return LayerOutput{};
  }
  if (!context.is_obstacle_status_fresh) {
    waiting_since_s_.reset();
    LayerOutput output;
    output.is_asserted = true;
    output.reason = StopReason::kObstacleStale;
    return output;
  }
  if (context.is_stop_zone_occupied || is_at_path_end) {
    return makeWaiting(context.now_s, context.is_obstacle_status_fresh
                                          ? StopReason::kBlocked
                                          : StopReason::kObstacleStale);
  }

  const auto best = selectBestCandidate(evaluateArcCandidates(
      grid, context.pose, context.path_points, context.closest_index, previous_curvature_per_m_,
      parameters_));
  LayerOutput output;
  output.is_asserted = true;
  if (best.has_value()) {
    waiting_since_s_.reset();
    previous_curvature_per_m_ = best->curvature_per_m;
    const double linear_mps = std::min(parameters_.v_avoid_mps,
                                       parameters_.k_end_per_s * context.remaining_distance_m);
    output.twist = Twist2D{linear_mps, linear_mps * best->curvature_per_m};
    output.reason = StopReason::kOk;
    return output;
  }
  return makeWaiting(context.now_s, StopReason::kBlocked);
}

LayerOutput ArcAvoidLayer::makeWaiting(double now_s, StopReason reason) {
  if (!waiting_since_s_.has_value()) {
    waiting_since_s_ = now_s;
    rotation_sign_ = -rotation_sign_;
  }
  LayerOutput output;
  output.is_asserted = true;
  const double waiting_s = now_s - *waiting_since_s_;
  output.reason = waiting_s >= parameters_.stuck_timeout_s ? StopReason::kStuck : reason;
  if (output.reason == StopReason::kStuck && waiting_s >= parameters_.stuck_timeout_s + parameters_.stuck_rotate_after_s) {
    output.twist.angular_rps = rotation_sign_ * parameters_.w_rotate_rps;
  }
  return output;
}

}  // namespace lightnav_controller
