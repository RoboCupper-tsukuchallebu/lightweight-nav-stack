#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <string>
#include <vector>

#include "lightnav_controller/avoid_layer.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/duration.hpp"

namespace lightnav_controller {

class ArcMarkerRate {
 public:
  bool ready(double now_s, double rate_hz, bool has_subscribers) {
    if (rate_hz <= 0.0 || !has_subscribers) return false;
    if (last_s_ && now_s >= *last_s_ && now_s - *last_s_ < 1.0 / rate_hz) return false;
    last_s_ = now_s;
    return true;
  }
 private:
  std::optional<double> last_s_;
};

inline bool arcMarkersActive(bool is_pose_fresh, bool is_avoid_asserted) {
  return is_pose_fresh && is_avoid_asserted;
}

inline visualization_msgs::msg::MarkerArray makeArcMarkers(
    const std::vector<ArcCandidate>& candidates, const std::optional<ArcCandidate>& selected,
    double length_m, double step_m, const std::string& frame, const rclcpp::Time& stamp,
    bool active, StopReason reason, double rate_hz) {
  using Marker = visualization_msgs::msg::Marker;
  visualization_msgs::msg::MarkerArray array;
  Marker clear;
  clear.header.frame_id = frame;
  clear.header.stamp = stamp;
  clear.action = Marker::DELETEALL;
  const double lifetime_s = std::max(1.0, std::min(2.0 / rate_hz,
      static_cast<double>(std::numeric_limits<int32_t>::max())));
  clear.lifetime = rclcpp::Duration::from_seconds(lifetime_s);
  array.markers.push_back(clear);
  if (!active) return array;
  for (std::size_t index = 0; index < candidates.size(); ++index) {
    const auto& candidate = candidates[index];
    Marker marker;
    marker.header = clear.header;
    marker.lifetime = clear.lifetime;
    marker.ns = candidate.is_feasible ? "feasible" : candidate.rejection_reason;
    marker.id = static_cast<int>(index);
    marker.type = Marker::LINE_STRIP;
    marker.action = Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = selected && candidate.is_feasible &&
                     candidate.curvature_per_m == selected->curvature_per_m ? 0.08 : 0.025;
    marker.color.a = 1.0;
    marker.color.g = candidate.is_feasible ? 1.0 : 0.0;
    marker.color.r = candidate.is_feasible ? 0.0 : 1.0;
    for (double distance = 0.0; distance < length_m; distance += step_m) {
      geometry_msgs::msg::Point point;
      const auto sample = arcPointInBody(candidate.curvature_per_m, distance);
      point.x = sample.x_m;
      point.y = sample.y_m;
      marker.points.push_back(point);
    }
    geometry_msgs::msg::Point end;
    const auto sample = arcPointInBody(candidate.curvature_per_m, length_m);
    end.x = sample.x_m;
    end.y = sample.y_m;
    marker.points.push_back(end);
    array.markers.push_back(marker);
  }
  Marker text;
  text.header = clear.header;
  text.lifetime = clear.lifetime;
  text.ns = "state";
  text.type = Marker::TEXT_VIEW_FACING;
  text.action = Marker::ADD;
  text.pose.orientation.w = 1.0;
  text.pose.position.z = 0.5;
  text.scale.z = 0.2;
  text.color.r = text.color.g = text.color.b = text.color.a = 1.0;
  text.text = reason == StopReason::kStuck ? "STUCK: timeout" :
              reason == StopReason::kObstacleStale ? "BLOCKED: obstacle stale" :
              reason == StopReason::kBlocked ? "BLOCKED: obstacle or no feasible arc" : "OK";
  array.markers.push_back(text);
  return array;
}
}  // namespace lightnav_controller
