#include "lightnav_localization/global_initialization.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <limits>
#include <chrono>
#include <array>
#include <map>
#include <tuple>
#include <Eigen/Eigenvalues>
#include "lightnav_localization/matcher.hpp"

namespace lightnav_localization {
namespace {

constexpr double kFullTurnRadians = 6.2831853071795864769;

// Squared Euclidean distance transform of a single row or column, in linear time.
std::vector<double> transform_distances(const std::vector<double>& input) {
  const std::size_t size = input.size();
  std::vector<std::size_t> sites(size);
  std::vector<double> boundaries(size + 1);
  std::vector<double> output(size, std::numeric_limits<double>::infinity());
  std::size_t count = 0;
  for (std::size_t index = 0; index < size; ++index) {
    if (!std::isfinite(input[index])) continue;
    double boundary = -std::numeric_limits<double>::infinity();
    while (count > 0) {
      const double current = static_cast<double>(index);
      const double previous = static_cast<double>(sites[count - 1]);
      boundary = ((input[index] + current * current) -
                  (input[sites[count - 1]] + previous * previous)) / (2.0 * (current - previous));
      if (boundary > boundaries[count - 1]) break;
      --count;
    }
    sites[count] = index;
    boundaries[count] = count == 0 ? -std::numeric_limits<double>::infinity() : boundary;
    ++count;
  }
  if (count == 0) return output;
  boundaries[count] = std::numeric_limits<double>::infinity();
  std::size_t site = 0;
  for (std::size_t index = 0; index < size; ++index) {
    while (boundaries[site + 1] < static_cast<double>(index)) ++site;
    const double difference = static_cast<double>(index) - static_cast<double>(sites[site]);
    output[index] = difference * difference + input[sites[site]];
  }
  return output;
}

bool are_distinct(const GlobalInitializationPeak& left, const GlobalInitializationPeak& right,
                  const GlobalInitializationPeakParameters& parameters) {
  const double translation = (left.position - right.position).norm();
  const double yaw = std::abs(std::remainder(left.yaw_rad - right.yaw_rad, kFullTurnRadians));
  return translation >= parameters.distinct_translation_m || yaw >= parameters.distinct_yaw_rad;
}

}  // namespace

GlobalInitializationLikelihoodField::GlobalInitializationLikelihoodField(
    const PointList& points, const Eigen::Vector2d& minimum, const Eigen::Vector2d& maximum,
    double resolution, std::size_t maximum_cells)
    : minimum_(minimum), resolution_(resolution), width_(0), height_(0) {
  if (!minimum.allFinite() || !maximum.allFinite() || !std::isfinite(resolution) ||
      resolution <= 0.0 || (maximum.array() < minimum.array()).any()) {
    throw std::invalid_argument("invalid likelihood field bounds");
  }
  const double width = std::floor((maximum.x() - minimum.x()) / resolution) + 1.0;
  const double height = std::floor((maximum.y() - minimum.y()) / resolution) + 1.0;
  if (width > static_cast<double>(maximum_cells) || height > static_cast<double>(maximum_cells) ||
      width * height > static_cast<double>(maximum_cells)) throw std::length_error("map_too_large");
  width_ = static_cast<std::size_t>(width);
  height_ = static_cast<std::size_t>(height);
  distances_.assign(width_ * height_, std::numeric_limits<double>::infinity());
  for (const auto& point : points) {
    if (!point.allFinite()) continue;
    const Eigen::Vector2d cell = (point.head<2>().cast<double>() - minimum_) / resolution_;
    if ((cell.array() < 0.0).any() || cell.x() >= width_ || cell.y() >= height_) continue;
    distances_[static_cast<std::size_t>(cell.y()) * width_ + static_cast<std::size_t>(cell.x())] = 0.0;
  }
  for (std::size_t row = 0; row < height_; ++row) {
    const auto first = distances_.begin() + row * width_;
    const auto transformed = transform_distances(std::vector<double>(first, first + width_));
    std::copy(transformed.begin(), transformed.end(), first);
  }
  for (std::size_t column = 0; column < width_; ++column) {
    std::vector<double> values(height_);
    for (std::size_t row = 0; row < height_; ++row) values[row] = distances_[row * width_ + column];
    const auto transformed = transform_distances(values);
    for (std::size_t row = 0; row < height_; ++row) {
      distances_[row * width_ + column] = std::sqrt(transformed[row]) * resolution_;
    }
  }
}

double GlobalInitializationLikelihoodField::distance(const Eigen::Vector2d& position) const {
  if (!position.allFinite()) return std::numeric_limits<double>::infinity();
  const Eigen::Vector2d cell = (position - minimum_) / resolution_;
  if ((cell.array() < 0.0).any() || cell.x() >= width_ || cell.y() >= height_)
    return std::numeric_limits<double>::infinity();
  return distances_[static_cast<std::size_t>(cell.y()) * width_ + static_cast<std::size_t>(cell.x())];
}

bool has_too_many_global_initialization_peaks(
    const std::vector<GlobalInitializationPeak>& coarse_peaks,
    const GlobalInitializationPeakParameters& parameters) {
  for (std::size_t index = 0; index < coarse_peaks.size(); ++index) {
    if (!std::isfinite(coarse_peaks[index].coarse_score) ||
        (index > 0 && coarse_peaks[index - 1].coarse_score < coarse_peaks[index].coarse_score)) {
      throw std::invalid_argument("coarse peaks must have finite scores in descending order");
    }
  }
  if (coarse_peaks.empty() || coarse_peaks.size() <= parameters.peak_count) return false;
  return coarse_peaks[parameters.peak_count].coarse_score >=
         coarse_peaks.front().coarse_score - parameters.candidate_margin;
}

GlobalInitializationPeakDecision decide_global_initialization_peaks(
    const std::vector<GlobalInitializationPeak>& verified_peaks,
    const GlobalInitializationPeakParameters& parameters) {
  GlobalInitializationPeakDecision decision;
  if (verified_peaks.empty()) {
    decision.error = "low_score";
    return decision;
  }
  for (std::size_t index = 0; index < verified_peaks.size(); ++index) {
    const double score = verified_peaks[index].verification_score;
    if (!std::isfinite(score)) {
      decision.error = "low_score";
      return decision;
    }
    if (index == 0 || score > decision.best_score) {
      decision.best_score = score;
      decision.best_index = index;
    }
  }
  const auto& best = verified_peaks[decision.best_index];
  for (const auto& peak : verified_peaks) {
    if (are_distinct(best, peak, parameters)) {
      decision.second_score = std::max(decision.second_score, peak.verification_score);
    }
  }
  if (decision.best_score < parameters.score_min) decision.error = "low_score";
  else if (decision.best_score - decision.second_score < parameters.margin_min) {
    decision.error = "ambiguous";
  } else {
    decision.accepted = true;
  }
  return decision;
}

namespace {
using Clock = std::chrono::steady_clock;

struct SearchContext {
  const GlobalInitializationSearchParameters& parameters;
  const std::atomic<bool>& abort;
  Clock::time_point started = Clock::now();
  std::size_t candidates = 0;
  std::string error;
  GlobalInitializationSearchResult& measurements;
  bool stopped() {
    if (abort.load()) error = "cancelled";
    else if (std::chrono::duration<double>(Clock::now() - started).count() >=
             parameters.time_budget_s) error = "time_budget";
    else if (candidates >= parameters.candidate_budget) error = "time_budget";
    return !error.empty();
  }
};

PointList select_band(const PointList& scan, const GlobalInitializationSearchParameters& parameters) {
  std::array<float, 360> ranges;
  ranges.fill(std::numeric_limits<float>::infinity());
  std::array<Eigen::Vector3f, 360> points;
  for (const auto& point : scan) {
    if (!point.allFinite() || point.z() < parameters.height_min_m ||
        point.z() > parameters.height_max_m) continue;
    const float range = point.head<2>().norm();
    if (range > parameters.maximum_range_m) continue;
    const double angle = std::atan2(point.y(), point.x());
    const int bin = (static_cast<int>(std::floor((angle + M_PI) * 180.0 / M_PI + 0.5)) + 360) % 360;
    if (range < ranges[bin]) { ranges[bin] = range; points[bin] = point; }
  }
  PointList result;
  for (std::size_t index = 0; index < ranges.size(); ++index)
    if (std::isfinite(ranges[index])) result.push_back(points[index]);
  return result;
}

PointList map_height_band(const PointList& map, const GlobalInitializationSearchParameters& parameters) {
  using Cell = std::pair<int, int>;
  std::map<Cell, float> ground;
  for (const auto& point : map) {
    if (!point.allFinite()) continue;
    const Cell cell{static_cast<int>(std::floor(point.x())), static_cast<int>(std::floor(point.y()))};
    const auto found = ground.find(cell);
    if (found == ground.end()) ground.emplace(cell, point.z());
    else found->second = std::min(found->second, point.z());
  }
  PointList band;
  for (const auto& point : map) {
    if (!point.allFinite()) continue;
    const int x = static_cast<int>(std::floor(point.x()));
    const int y = static_cast<int>(std::floor(point.y()));
    float height = std::numeric_limits<float>::infinity();
    for (int dx = -1; dx <= 1; ++dx) for (int dy = -1; dy <= 1; ++dy) {
      const auto found = ground.find({x + dx, y + dy});
      if (found != ground.end()) height = std::min(height, found->second);
    }
    if (point.z() - height >= parameters.height_min_m &&
        point.z() - height <= parameters.height_max_m) band.push_back(point);
  }
  return band;
}

double likelihood(const GlobalInitializationLikelihoodField& field, const PointList& scan,
                  const GlobalInitializationPeak& peak, double sigma) {
  const Eigen::Rotation2Dd rotation(peak.yaw_rad);
  double sum = 0.0;
  for (const auto& point : scan) {
    const double distance = field.distance(peak.position + rotation * point.head<2>().cast<double>());
    sum += std::exp(-distance * distance / (2.0 * sigma * sigma));
  }
  return sum / static_cast<double>(scan.size());
}

std::vector<GlobalInitializationPeak> suppress_peaks(
    std::vector<GlobalInitializationPeak> candidates, const GlobalInitializationPeakParameters& parameters,
    bool verified) {
  std::sort(candidates.begin(), candidates.end(), [verified](const auto& left, const auto& right) {
    return verified ? left.verification_score > right.verification_score : left.coarse_score > right.coarse_score;
  });
  std::vector<GlobalInitializationPeak> result;
  for (const auto& candidate : candidates) {
    bool distinct = true;
    for (const auto& previous : result) if (!are_distinct(candidate, previous, parameters)) { distinct = false; break; }
    if (distinct) result.push_back(candidate);
    if (!verified && result.size() > parameters.peak_count) break;
  }
  return result;
}

std::vector<GlobalInitializationPeak> coarse_search(
    const GlobalInitializationLikelihoodField& field, const PointList& scan, SearchContext& context,
    const Eigen::Vector2d& minimum, const Eigen::Vector2d& maximum) {
  const auto& parameters = context.parameters;
  std::vector<GlobalInitializationPeak> candidates;
  for (double x = minimum.x(); x <= maximum.x() + 1e-9; x += parameters.coarse_translation_step_m) {
    for (double y = minimum.y(); y <= maximum.y() + 1e-9; y += parameters.coarse_translation_step_m) {
      const double distance = field.distance({x, y});
      if (distance < 0.3 || distance > parameters.maximum_range_m) continue;
      for (double yaw = -M_PI; yaw < M_PI - 1e-9; yaw += parameters.coarse_yaw_step_rad) {
        if (context.stopped()) return {};
        GlobalInitializationPeak peak;
        peak.position = {x, y}; peak.yaw_rad = yaw;
        if (parameters.gnss_window) {
          const auto& window = *parameters.gnss_window;
          if ((peak.position - window.antenna_position).norm() > window.radius_m) continue;
          peak.position -= Eigen::Rotation2Dd(yaw) * window.lever_arm;
        }
        if ((peak.position.array() < parameters.minimum.array()).any() ||
            (peak.position.array() > parameters.maximum.array()).any()) continue;
        peak.coarse_score = likelihood(field, scan, peak, parameters.likelihood_sigma_m);
        ++context.candidates;
        candidates.push_back(peak);
      }
    }
  }
  return suppress_peaks(std::move(candidates), parameters.peaks, false);
}

bool inside_gnss_window(const Eigen::Vector2d& base_position, double yaw,
                        const GlobalInitializationSearchParameters& parameters) {
  if (!parameters.gnss_window) return true;
  const auto& window = *parameters.gnss_window;
  const Eigen::Vector2d antenna = base_position + Eigen::Rotation2Dd(yaw) * window.lever_arm;
  return (antenna - window.antenna_position).norm() <= window.radius_m;
}

GlobalInitializationPeak refine_peak(const GlobalInitializationLikelihoodField& field,
                                     const PointList& scan, GlobalInitializationPeak peak,
                                     SearchContext& context) {
  const auto& parameters = context.parameters;
  const auto center = peak;
  for (double x = -parameters.coarse_translation_step_m; x <= parameters.coarse_translation_step_m + 1e-9; x += parameters.fine_translation_step_m)
    for (double y = -parameters.coarse_translation_step_m; y <= parameters.coarse_translation_step_m + 1e-9; y += parameters.fine_translation_step_m)
      for (double yaw = -parameters.coarse_yaw_step_rad; yaw <= parameters.coarse_yaw_step_rad + 1e-9; yaw += parameters.fine_yaw_step_rad) {
        if (context.stopped()) return peak;
        GlobalInitializationPeak candidate = center;
        candidate.position += Eigen::Vector2d(x, y); candidate.yaw_rad += yaw;
        if (!inside_gnss_window(candidate.position, candidate.yaw_rad, parameters)) continue;
        if ((candidate.position.array() < parameters.minimum.array()).any() ||
            (candidate.position.array() > parameters.maximum.array()).any()) continue;
        candidate.coarse_score = likelihood(field, scan, candidate, parameters.likelihood_sigma_m);
        ++context.candidates;
        if (candidate.coarse_score > peak.coarse_score) peak = candidate;
      }
  return peak;
}

double scan_to_map(const PointList& scan, const PointList& map, const Eigen::Isometry3d& pose, double threshold) {
  using Cell = std::tuple<int, int, int>;
  const auto cell_of = [threshold](const Eigen::Vector3d& point) {
    return Cell{static_cast<int>(std::floor(point.x() / threshold)),
                static_cast<int>(std::floor(point.y() / threshold)),
                static_cast<int>(std::floor(point.z() / threshold))};
  };
  std::map<Cell, PointList> cells;
  for (const auto& point : map) if (point.allFinite()) cells[cell_of(point.cast<double>())].push_back(point);
  std::size_t inliers = 0;
  for (const auto& point : scan) {
    if (!point.allFinite()) continue;
    const Eigen::Vector3d transformed = pose * point.cast<double>();
    const auto [x, y, z] = cell_of(transformed);
    bool found = false;
    for (int dx = -1; dx <= 1 && !found; ++dx)
      for (int dy = -1; dy <= 1 && !found; ++dy)
        for (int dz = -1; dz <= 1 && !found; ++dz) {
          const auto cell = cells.find({x + dx, y + dy, z + dz});
          if (cell == cells.end()) continue;
          for (const auto& target : cell->second) {
            if ((transformed - target.cast<double>()).squaredNorm() <= threshold * threshold) {
              found = true; break;
            }
          }
        }
    if (found) ++inliers;
  }
  return scan.empty() ? 0.0 : static_cast<double>(inliers) / scan.size();
}

double map_to_scan(const GlobalInitializationLikelihoodField& field, const PointList& scan,
                   const GlobalInitializationPeak& peak, const GlobalInitializationSearchParameters& parameters) {
  std::array<double, 360> ranges;
  ranges.fill(std::numeric_limits<double>::infinity());
  for (const auto& point : scan) {
    const double angle = std::atan2(point.y(), point.x());
    const int bin = (static_cast<int>(std::floor((angle + M_PI) * 180.0 / M_PI + 0.5)) + 360) % 360;
    ranges[bin] = std::min(ranges[bin], static_cast<double>(point.head<2>().norm()));
  }
  std::size_t visible = 0, consistent = 0;
  for (int bin = 0; bin < 360; ++bin) {
    const double angle = peak.yaw_rad + bin * M_PI / 180.0 - M_PI;
    const Eigen::Vector2d direction(std::cos(angle), std::sin(angle));
    for (double range = 0.3; range <= parameters.maximum_range_m; range += parameters.grid_resolution_m) {
      if (field.distance(peak.position + range * direction) <= parameters.grid_resolution_m * 0.5) {
        ++visible;
        if (ranges[bin] <= range + parameters.see_through_m) ++consistent;
        break;
      }
    }
  }
  return visible == 0 ? 0.0 : static_cast<double>(consistent) / visible;
}

bool verify_peak(GlobalInitializationPeak& peak, const PointList& map, const PointList& scan,
                 const PointList& band, const GlobalInitializationLikelihoodField& field,
                 SearchContext& context, Eigen::Isometry3d& pose, bool& degenerate) {
  if (context.stopped()) return false;
  PointList local_map;
  for (const auto& point : map)
    if ((point.head<2>().cast<double>() - peak.position).norm() <= context.parameters.maximum_range_m) local_map.push_back(point);
  const auto target_started = Clock::now();
  PreparedTarget target(local_map);
  context.measurements.target_seconds.push_back(std::chrono::duration<double>(Clock::now() - target_started).count());
  if (context.stopped() || !target.is_valid()) return false;
  pose = Eigen::Isometry3d::Identity();
  pose.linear() = Eigen::AngleAxisd(peak.yaw_rad, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  pose.translation().head<2>() = peak.position;
  double ground = std::numeric_limits<double>::infinity();
  const Eigen::Vector2d cell = peak.position.array().floor();
  for (const auto& point : local_map) {
    if (!point.allFinite()) continue;
    const Eigen::Vector2d offset = point.head<2>().cast<double>().array().floor().matrix() - cell;
    if (std::abs(offset.x()) <= 1.0 && std::abs(offset.y()) <= 1.0)
      ground = std::min(ground, static_cast<double>(point.z()));
  }
  if (!std::isfinite(ground)) return false;
  pose.translation().z() = ground;
  MatcherParameters parameters;
  parameters.max_correspondence_distance_m = 2.0;
  const auto matching_started = Clock::now();
  auto result = match_scan(target, scan, pose, parameters);
  if (context.stopped() || !result.converged) return false;
  parameters.max_correspondence_distance_m = 0.5;
  result = match_scan(target, scan, result.target_to_source, parameters);
  context.measurements.matching_seconds.push_back(std::chrono::duration<double>(Clock::now() - matching_started).count());
  if (context.stopped() || !result.converged || !result.target_to_source.matrix().allFinite()) return false;
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigenvalues(result.information.block<3, 3>(3, 3));
  degenerate = eigenvalues.info() != Eigen::Success || eigenvalues.eigenvalues().maxCoeff() <= 0.0 ||
      eigenvalues.eigenvalues().minCoeff() / eigenvalues.eigenvalues().maxCoeff() < context.parameters.degeneracy_ratio;
  pose = result.target_to_source;
  if ((pose.translation().head<2>().array() < context.parameters.minimum.array()).any() ||
      (pose.translation().head<2>().array() > context.parameters.maximum.array()).any()) return false;
  peak.position = pose.translation().head<2>();
  peak.yaw_rad = std::atan2(pose.linear()(1, 0), pose.linear()(0, 0));
  peak.verification_score = std::min(scan_to_map(scan, local_map, pose, context.parameters.correspondence_m),
                                    map_to_scan(field, band, peak, context.parameters));
  return true;
}
}  // namespace

GlobalInitializationSearchResult search_global_initialization(
    const PointList& map, const PointList& scan, const GlobalInitializationSearchParameters& parameters,
    const std::atomic<bool>& abort) {
  GlobalInitializationSearchResult output;
  SearchContext context{parameters, abort, Clock::now(), 0, {}, output};
  if (context.stopped()) { output.decision.error = context.error; return output; }
  const Eigen::Vector2d extent = parameters.maximum - parameters.minimum;
  const std::array<double, 8> positive_values{
      parameters.grid_resolution_m, parameters.likelihood_sigma_m, parameters.maximum_range_m,
      parameters.maximum_area_m2, parameters.coarse_translation_step_m, parameters.coarse_yaw_step_rad,
      parameters.fine_translation_step_m, parameters.fine_yaw_step_rad};
  if (!parameters.minimum.allFinite() || !parameters.maximum.allFinite() || !extent.allFinite() ||
      (extent.array() < 0.0).any() ||
      std::any_of(positive_values.begin(), positive_values.end(),
                  [](double value) { return !std::isfinite(value) || value <= 0.0; }) ||
      !std::isfinite(parameters.correspondence_m) || parameters.correspondence_m <= 0.0) {
    output.decision.error = "invalid_parameters";
    return output;
  }
  if (parameters.gnss_window &&
      (!parameters.gnss_window->antenna_position.allFinite() ||
       !parameters.gnss_window->lever_arm.allFinite() ||
       !std::isfinite(parameters.gnss_window->radius_m) || parameters.gnss_window->radius_m <= 0.0)) {
    output.decision.error = "invalid_parameters";
    return output;
  }
  Eigen::Vector2d search_minimum = parameters.minimum;
  Eigen::Vector2d search_maximum = parameters.maximum;
  if (parameters.gnss_window) {
    const auto& window = *parameters.gnss_window;
    const Eigen::Vector2d radius = Eigen::Vector2d::Constant(window.radius_m + window.lever_arm.norm());
    search_minimum = search_minimum.cwiseMax(window.antenna_position - radius);
    search_maximum = search_maximum.cwiseMin(window.antenna_position + radius);
    if ((search_maximum.array() < search_minimum.array()).any()) {
      output.decision.error = "low_score";
      return output;
    }
  }
  if ((search_maximum - search_minimum).prod() > parameters.maximum_area_m2) {
    output.decision.error = "map_too_large";
    return output;
  }
  const PointList band = select_band(scan, parameters);
  if (band.size() < parameters.minimum_scan_points) { output.decision.error = "not_enough_scan_points"; return output; }
  try {
    const Eigen::Vector2d padding = Eigen::Vector2d::Constant(parameters.maximum_range_m);
    auto stage_started = Clock::now();
    GlobalInitializationLikelihoodField field(map_height_band(map, parameters), search_minimum - padding, search_maximum + padding,
                                               parameters.grid_resolution_m, parameters.maximum_grid_cells);
    output.field_cells = field.cell_count();
    output.field_seconds = std::chrono::duration<double>(Clock::now() - stage_started).count();
    stage_started = Clock::now();
    auto peaks = coarse_search(field, band, context, search_minimum, search_maximum);
    output.coarse_seconds = std::chrono::duration<double>(Clock::now() - stage_started).count();
    output.coarse_peaks = peaks;
    if (context.stopped()) { output.decision.error = context.error; return output; }
    if (has_too_many_global_initialization_peaks(peaks, parameters.peaks)) {
      output.decision.error = "ambiguous"; return output;
    }
    if (peaks.size() > parameters.peaks.peak_count) peaks.resize(parameters.peaks.peak_count);
    std::vector<GlobalInitializationPeak> verified;
    std::vector<Eigen::Isometry3d> poses;
    for (auto peak : peaks) {
      stage_started = Clock::now();
      peak = refine_peak(field, band, peak, context);
      output.fine_seconds += std::chrono::duration<double>(Clock::now() - stage_started).count();
      stage_started = Clock::now();
      Eigen::Isometry3d pose;
      bool degenerate = false;
      if (verify_peak(peak, map, scan, band, field, context, pose, degenerate)) {
        peak.degenerate = degenerate;
        verified.push_back(peak); poses.push_back(pose);
      }
      output.verification_seconds += std::chrono::duration<double>(Clock::now() - stage_started).count();
      if (context.stopped()) { output.decision.error = context.error; return output; }
    }
    // All converged peaks compete for b2, even if degenerate; only b1 is checked below.
    output.decision = decide_global_initialization_peaks(verified, parameters.peaks);
    if (output.decision.accepted && verified[output.decision.best_index].degenerate) {
      output.decision.accepted = false;
      output.decision.error = "degenerate";
    }
    if (output.decision.accepted && !inside_gnss_window(
        poses[output.decision.best_index].translation().head<2>(),
        verified[output.decision.best_index].yaw_rad, parameters)) {
      output.decision.accepted = false;
      output.decision.error = "outside_gnss_window";
    }
    if (output.decision.accepted) output.pose = poses[output.decision.best_index];
    output.evaluated_candidates = context.candidates;
  } catch (const std::length_error&) { output.decision.error = "map_too_large"; }
  return output;
}

}  // namespace lightnav_localization
