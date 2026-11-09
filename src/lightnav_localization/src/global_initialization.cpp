#include "lightnav_localization/global_initialization.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <limits>

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

}  // namespace lightnav_localization
