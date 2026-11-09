#pragma once

// Synthetic corridor world for tests: walls around a polyline of axis-aligned legs.

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <vector>

#include "lightnav_localization/deskew.hpp"

namespace lightnav_localization::testing {

struct WallSegment {
  Eigen::Vector2d start;
  Eigen::Vector2d end;
};

constexpr double kWallHeightM = 3.0;
constexpr double kSensorHeightM = 1.0;
constexpr double kCorridorHalfWidthM = 2.0;

class SyntheticWorld {
 public:
  // `centerline` must have axis-aligned legs. Walls end flush at the course ends (closed by caps).
  // `has_end_caps = false` leaves the ends open (only the max range limits the view), which makes
  // the corridor axis unobservable for scan matching.
  explicit SyntheticWorld(const std::vector<Eigen::Vector2d>& centerline, bool has_end_caps = true,
                          bool closed_course = false) {
    centerline_ = centerline;
    // Use the stored copy: reading the caller's vector (often a two-point initializer_list) here
    // makes GCC 15 report a false -Warray-bounds for Eigen's vectorized 2-vector operations.
    if (closed_course) {
      std::vector<Eigen::Vector2d> cyclic_line;
      cyclic_line.push_back(centerline_[centerline_.size() - 2]);
      cyclic_line.insert(cyclic_line.end(), centerline_.begin(), centerline_.end());
      cyclic_line.push_back(centerline_[1]);
      for (std::size_t leg = 1; leg + 2 < cyclic_line.size(); ++leg) {
        add_leg_walls(cyclic_line, leg);
      }
    } else {
      for (std::size_t leg = 0; leg + 1 < centerline_.size(); ++leg) {
        add_leg_walls(centerline_, leg);
      }
    }
    if (has_end_caps && !closed_course) {
      walls_.push_back({front_cap_start(centerline_), front_cap_end(centerline_)});
      walls_.push_back({back_cap_start(centerline_), back_cap_end(centerline_)});
    }
  }

  double distance_to_wall(const Eigen::Vector2d& point) const {
    double nearest_distance = std::numeric_limits<double>::infinity();
    for (const WallSegment& wall : walls_) {
      const Eigen::Vector2d segment = wall.end - wall.start;
      const double fraction = std::clamp((point - wall.start).dot(segment) / segment.squaredNorm(), 0.0, 1.0);
      nearest_distance = std::min(nearest_distance, (point - (wall.start + fraction * segment)).norm());
    }
    return nearest_distance;
  }

  // Boxes along each leg at `spacing_m` intervals, starting half an interval from the leg start
  // (with the default 10 m spacing that is 5 m, not the old 10 m start). `half_length_m` is half
  // the size along the travel direction, `half_width_m` half the size across it. The defaults
  // (0.2 m both, left side) give 0.4 m pillars for PillarsAppearInMapAndScan; the start offset
  // changed, so the default layout is not the old 10 m-start pillar set. A thin, wide box (small
  // half_length_m, large half_width_m) adds faces perpendicular to the travel direction without
  // adding much lateral surface, which is what raises the along-track translation eigenvalue ratio
  // in a straight corridor.
  void add_pillars(double half_length_m = 0.2, double half_width_m = 0.2, double spacing_m = 10.0,
                   double lateral_offset_m = 1.5, bool both_sides = false) {
    for (std::size_t leg = 0; leg + 1 < centerline_.size(); ++leg) {
      const Eigen::Vector2d displacement = centerline_[leg + 1] - centerline_[leg];
      const double length = displacement.norm();
      if (length <= 0.0) continue;
      const Eigen::Vector2d direction = displacement / length;
      const Eigen::Vector2d normal(-direction.y(), direction.x());
      for (double distance = 0.5 * spacing_m; distance < length; distance += spacing_m) {
        const int side_count = both_sides ? 2 : 1;
        for (int side_index = 0; side_index < side_count; ++side_index) {
          const double side = side_index == 0 ? 1.0 : -1.0;
          const Eigen::Vector2d center =
              centerline_[leg] + distance * direction + side * lateral_offset_m * normal;
          const Eigen::Vector2d front_left =
              center + half_length_m * direction + half_width_m * normal;
          const Eigen::Vector2d front_right =
              center + half_length_m * direction - half_width_m * normal;
          const Eigen::Vector2d back_left =
              center - half_length_m * direction + half_width_m * normal;
          const Eigen::Vector2d back_right =
              center - half_length_m * direction - half_width_m * normal;
          walls_.push_back({front_left, front_right});
          walls_.push_back({front_right, back_right});
          walls_.push_back({back_right, back_left});
          walls_.push_back({back_left, front_left});
        }
      }
    }
  }

  // Floor rises at `angle_rad` from x = begin_x over `horizontal_length_m`, then stays level.
  void set_max_range(double range_m) { max_range_m_ = range_m; }
  void set_ramp(double begin_x, double horizontal_length_m, double angle_rad) {
    ramp_begin_x_ = begin_x;
    ramp_end_x_ = begin_x + horizontal_length_m;
    ramp_slope_ = std::tan(angle_rad);
    has_ramp_ = true;
  }
  double floor_height(double x) const {
    if (!has_ramp_ || x <= ramp_begin_x_) return 0.0;
    return ramp_slope_ * (std::min(x, ramp_end_x_) - ramp_begin_x_);
  }
  bool is_on_ramp(double x) const { return has_ramp_ && x > ramp_begin_x_ && x < ramp_end_x_; }
  double ramp_angle_rad() const { return std::atan(ramp_slope_); }

  // Wall and floor surface samples at `spacing_m`, world frame (floor at z = 0).
  PointList sample_map(double spacing_m) const {
    PointList points = sample_floor(spacing_m);
    for (const WallSegment& wall : walls_) {
      const double length = (wall.end - wall.start).norm();
      for (double along = 0.0; along <= length; along += spacing_m) {
        const Eigen::Vector2d xy = wall.start + (wall.end - wall.start) * (along / length);
        for (double height = 0.0; height <= kWallHeightM; height += spacing_m) {
          points.emplace_back(xy.x(), xy.y(), height + floor_height(xy.x()));
        }
      }
    }
    return points;
  }

  // Simulates one rotation. Each ray is cast from the true sensor pose at its own emission time and
  // reported in the sensor frame at that time (the distortion a moving LiDAR really has).
  // `stamp_at_scan_end` selects the time_offset convention: true -> offsets are negative (driver
  // default), false -> offsets are positive (timestamp_first_packet: true).
  std::vector<TimedPoint> synthesize_scan(const std::function<Eigen::Isometry3d(double)>& pose_at,
                                          double scan_start_sec, double scan_duration_sec,
                                          bool stamp_at_scan_end, int azimuth_steps = 900,
                                          int ring_count = 16) const {
    const double stamp = stamp_at_scan_end ? scan_start_sec + scan_duration_sec : scan_start_sec;
    std::vector<TimedPoint> scan;
    for (int step = 0; step < azimuth_steps; ++step) {
      const double fraction = static_cast<double>(step) / azimuth_steps;
      const double emission_time = scan_start_sec + fraction * scan_duration_sec;
      const Eigen::Isometry3d pose = pose_at(emission_time);
      const double azimuth = 2.0 * M_PI * fraction;
      for (int ring = 0; ring < ring_count; ++ring) {
        const double elevation = (-15.0 + 30.0 * ring / (ring_count - 1)) * M_PI / 180.0;
        const Eigen::Vector3d direction(std::cos(elevation) * std::cos(azimuth),
                                        std::cos(elevation) * std::sin(azimuth),
                                        std::sin(elevation));
        const double range = cast_ray(pose.translation(), pose.linear() * direction);
        if (!std::isfinite(range)) continue;
        scan.push_back({(range * direction).cast<float>(),
                        static_cast<float>(emission_time - stamp)});
      }
    }
    return scan;
  }

 private:
  PointList sample_floor(double spacing_m) const {
    PointList points;
    for (std::size_t leg = 0; leg + 1 < centerline_.size(); ++leg) {
      const Eigen::Vector2d direction = (centerline_[leg + 1] - centerline_[leg]).normalized();
      const Eigen::Vector2d normal(-direction.y(), direction.x());
      const double length = (centerline_[leg + 1] - centerline_[leg]).norm();
      const double begin = leg > 0 ? -kCorridorHalfWidthM : 0.0;
      const double end = length + (leg + 2 < centerline_.size() ? kCorridorHalfWidthM : 0.0);
      for (double along = begin; along <= end; along += spacing_m) {
        for (double across = -kCorridorHalfWidthM; across <= kCorridorHalfWidthM; across += spacing_m) {
          const Eigen::Vector2d xy = centerline_[leg] + along * direction + across * normal;
          points.emplace_back(xy.x(), xy.y(), floor_height(xy.x()));
        }
      }
    }
    return points;
  }

  static Eigen::Vector2d normal_of(const std::vector<Eigen::Vector2d>& line, std::size_t leg) {
    const Eigen::Vector2d direction = (line[leg + 1] - line[leg]).normalized();
    return {-direction.y(), direction.x()};
  }

  // Wall endpoint on side `side` (+1 left, -1 right) at centerline vertex `vertex` of leg `leg`.
  static Eigen::Vector2d wall_end_point(const std::vector<Eigen::Vector2d>& line, std::size_t leg,
                                        bool at_leg_end, double side) {
    const std::size_t vertex = at_leg_end ? leg + 1 : leg;
    const bool is_corner = at_leg_end ? (leg + 2 < line.size()) : (leg > 0);
    if (!is_corner) return line[vertex] + side * kCorridorHalfWidthM * normal_of(line, leg);
    const std::size_t before = at_leg_end ? leg : leg - 1;
    const Eigen::Vector2d normal_before = normal_of(line, before);
    const Eigen::Vector2d normal_after = normal_of(line, before + 1);
    const Eigen::Vector2d incoming = (line[before + 1] - line[before]).normalized();
    const Eigen::Vector2d outgoing = (line[before + 2] - line[before + 1]).normalized();
    const double turn_sign = incoming.x() * outgoing.y() - incoming.y() * outgoing.x() > 0 ? 1.0 : -1.0;
    const double corner_sign = (side == turn_sign) ? 1.0 : -1.0;  // inner side is +1
    return line[vertex] + corner_sign * turn_sign * kCorridorHalfWidthM * (normal_before + normal_after);
  }

  void add_leg_walls(const std::vector<Eigen::Vector2d>& line, std::size_t leg) {
    for (const double side : {1.0, -1.0}) {
      walls_.push_back({wall_end_point(line, leg, false, side), wall_end_point(line, leg, true, side)});
    }
  }

  Eigen::Vector2d front_cap_start(const std::vector<Eigen::Vector2d>& line) const {
    return wall_end_point(line, 0, false, 1.0);
  }
  Eigen::Vector2d front_cap_end(const std::vector<Eigen::Vector2d>& line) const {
    return wall_end_point(line, 0, false, -1.0);
  }
  Eigen::Vector2d back_cap_start(const std::vector<Eigen::Vector2d>& line) const {
    return wall_end_point(line, line.size() - 2, true, 1.0);
  }
  Eigen::Vector2d back_cap_end(const std::vector<Eigen::Vector2d>& line) const {
    return wall_end_point(line, line.size() - 2, true, -1.0);
  }

  // Nearest hit distance of a ray against walls and floor; infinity if nothing is hit.
  double cast_ray(const Eigen::Vector3d& origin, const Eigen::Vector3d& direction) const {
    double nearest = std::numeric_limits<double>::infinity();
    nearest = std::min(nearest, cast_floor(origin, direction));
    for (const WallSegment& wall : walls_) {
      const Eigen::Vector2d edge = wall.end - wall.start;
      const Eigen::Vector2d ray_xy = direction.head<2>();
      const double denominator = ray_xy.x() * edge.y() - ray_xy.y() * edge.x();
      if (std::abs(denominator) < 1e-12) continue;
      const Eigen::Vector2d to_start = wall.start - origin.head<2>();
      const double range = (to_start.x() * edge.y() - to_start.y() * edge.x()) / denominator;
      const double edge_ratio = (to_start.x() * ray_xy.y() - to_start.y() * ray_xy.x()) / denominator;
      const double hit_height = origin.z() + range * direction.z() -
                                floor_height(origin.x() + range * direction.x());
      if (range > 0.0 && edge_ratio >= 0.0 && edge_ratio <= 1.0 && hit_height >= 0.0 &&
          hit_height <= kWallHeightM) {
        nearest = std::min(nearest, range);
      }
    }
    return nearest < max_range_m_ ? nearest : std::numeric_limits<double>::infinity();
  }

  // Floor pieces: level before the ramp, the ramp plane, level after it (z = slope * x + offset).
  double cast_floor(const Eigen::Vector3d& origin, const Eigen::Vector3d& direction) const {
    struct Piece { double slope; double offset; double x_min; double x_max; };
    const double inf = std::numeric_limits<double>::infinity();
    std::vector<Piece> pieces{{0.0, 0.0, -inf, inf}};
    if (has_ramp_) {
      const double top = ramp_slope_ * (ramp_end_x_ - ramp_begin_x_);
      pieces = {{0.0, 0.0, -inf, ramp_begin_x_},
                {ramp_slope_, -ramp_slope_ * ramp_begin_x_, ramp_begin_x_, ramp_end_x_},
                {0.0, top, ramp_end_x_, inf}};
    }
    double nearest = inf;
    for (const Piece& piece : pieces) {
      const double denominator = direction.z() - piece.slope * direction.x();
      if (std::abs(denominator) < 1e-12) continue;
      const double range = (piece.slope * origin.x() + piece.offset - origin.z()) / denominator;
      const double hit_x = origin.x() + range * direction.x();
      if (range > 0.0 && hit_x >= piece.x_min && hit_x <= piece.x_max) nearest = std::min(nearest, range);
    }
    return nearest;
  }

  double max_range_m_ = 60.0;
  bool has_ramp_ = false;
  double ramp_begin_x_ = 0.0;
  double ramp_end_x_ = 0.0;
  double ramp_slope_ = 0.0;
  std::vector<WallSegment> walls_;
  std::vector<Eigen::Vector2d> centerline_;
};

// Constant-speed, constant-yaw-rate motion starting at `start_xy` heading `start_yaw` at t = 0.
inline Eigen::Isometry3d arc_pose(double time_sec, const Eigen::Vector2d& start_xy, double start_yaw,
                                  double speed_mps, double yaw_rate_rps) {
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  const double yaw = start_yaw + yaw_rate_rps * time_sec;
  // Small-arc integration is exact enough here: heading changes <= 0.05 rad per scan.
  const double mean_yaw = start_yaw + 0.5 * yaw_rate_rps * time_sec;
  pose.translation() = Eigen::Vector3d(start_xy.x() + speed_mps * time_sec * std::cos(mean_yaw),
                                       start_xy.y() + speed_mps * time_sec * std::sin(mean_yaw),
                                       kSensorHeightM);
  pose.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  return pose;
}

}  // namespace lightnav_localization::testing
