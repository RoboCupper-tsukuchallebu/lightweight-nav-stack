#pragma once

#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <vector>

#include "lightnav_controller/types.hpp"

namespace lightnav_controller {

// Inflated occupancy grid in base_link (nav_msgs/OccupancyGrid layout: row
// major, cell (column, row) covers origin + [column, column + 1) * resolution).
struct LocalGrid {
  std::size_t width_cells{0};
  std::size_t height_cells{0};
  double resolution_m{0.1};
  double origin_x_m{0.0};
  double origin_y_m{0.0};
  std::vector<std::int8_t> cells;  // 0 free, >= occupied_threshold occupied

  // A point outside the grid is treated as free: nothing is known there and
  // the obstacle monitor already stops the robot on its own stop zone.
  bool isOccupiedInBody(double body_x_m, double body_y_m) const;
};

struct AvoidParameters {
  double v_avoid_mps{0.25};
  double w_max_rps{0.4};
  int arc_candidate_count{15};
  double arc_length_m{2.0};
  double arc_step_m{0.1};
  double max_path_deviation_m{std::numeric_limits<double>::quiet_NaN()};
  double w_path{1.0};
  double w_heading{0.5};
  double w_smooth{0.2};
  double avoid_trigger_m{2.5};
  double avoid_release_m{3.0};
  double rejoin_distance_m{0.5};
  double avoid_release_hold_s{1.0};
  double stuck_timeout_s{10.0};
  double goal_tolerance_m{0.3};
  double k_end_per_s{0.5};
};

struct ArcCandidate {
  double curvature_per_m{0.0};
  bool is_feasible{false};
  double score{0.0};
  const char* rejection_reason{""};
};

// Position on an arc in the robot body frame.
Point2D arcPointInBody(double curvature_per_m, double arc_length_m);

// Pure function: scores every candidate arc. Infeasible when a sample hits an
// occupied cell or the end point is farther than max_path_deviation_m from the
// path. The grid is in the body frame, the path and pose in the map frame.
std::vector<ArcCandidate> evaluateArcCandidates(const LocalGrid& grid, const Pose2D& pose,
                                                const std::vector<Point2D>& path_points,
                                                std::size_t closest_index,
                                                double previous_curvature_per_m,
                                                const AvoidParameters& parameters);

// Lowest-score feasible candidate; nullopt when none is feasible.
std::optional<ArcCandidate> selectBestCandidate(const std::vector<ArcCandidate>& candidates);

struct AvoidContext {
  Pose2D pose;
  const std::vector<Point2D>& path_points;
  std::size_t closest_index{0};
  bool is_grid_fresh{false};
  const LocalGrid* grid{nullptr};
  double now_s{0.0};
  double remaining_distance_m{std::numeric_limits<double>::infinity()};
  bool is_stop_zone_occupied{false};
  bool is_obstacle_status_fresh{true};
};

// Priority 1 layer.
class AvoidLayer {
 public:
  AvoidLayer() = default;
  AvoidLayer(const AvoidLayer&) = delete;
  AvoidLayer& operator=(const AvoidLayer&) = delete;
  virtual ~AvoidLayer() = default;

  virtual LayerOutput compute(const AvoidContext& context) = 0;
};

// Never asserts; used by tests that only exercise the follow layer.
class NullAvoidLayer final : public AvoidLayer {
 public:
  LayerOutput compute(const AvoidContext&) override { return LayerOutput{}; }
};

// Candidate-arc avoidance with trigger/release hysteresis and stuck detection.
// Asserts with reason kBlocked while no arc is feasible, kStuck after
// stuck_timeout_s of waiting.
class ArcAvoidLayer final : public AvoidLayer {
 public:
  explicit ArcAvoidLayer(const AvoidParameters& parameters);
  void updateParameters(const AvoidParameters& parameters) { parameters_ = parameters; }

  LayerOutput compute(const AvoidContext& context) override;

  bool isAvoiding() const { return is_avoiding_; }
  const std::vector<ArcCandidate>& candidates() const { return candidates_; }
  const std::optional<ArcCandidate>& selectedCandidate() const { return selected_candidate_; }

 private:
  void reset();
  bool isPathEndOccupied(const LocalGrid& grid, const AvoidContext& context) const;
  LayerOutput makeWaiting(double now_s, StopReason reason);

  std::vector<ArcCandidate> candidates_;
  std::optional<ArcCandidate> selected_candidate_;
  AvoidParameters parameters_;
  bool is_avoiding_{false};
  std::optional<double> release_since_s_;
  std::optional<double> waiting_since_s_;
  double previous_curvature_per_m_{0.0};
};

}  // namespace lightnav_controller
