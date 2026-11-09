#include "lightnav_localization/matcher.hpp"

#include <cmath>
#include <small_gicp/ann/kdtree.hpp>
#include <small_gicp/factors/gicp_factor.hpp>
#include <small_gicp/points/point_cloud.hpp>
#include <small_gicp/registration/reduction.hpp>
#include <small_gicp/registration/registration.hpp>
#include <small_gicp/util/normal_estimation.hpp>

namespace lightnav_localization {
namespace {

constexpr std::size_t kMinimumPointCount = 10;
constexpr int kCovarianceNeighborCount = 10;

std::shared_ptr<small_gicp::PointCloud> make_cloud_with_covariances(const PointList& points) {
  auto cloud = std::make_shared<small_gicp::PointCloud>(points);
  small_gicp::estimate_covariances(*cloud, kCovarianceNeighborCount);
  return cloud;
}

double rotation_angle_rad(const Eigen::Isometry3d& transform) {
  return Eigen::AngleAxisd(transform.rotation()).angle();
}

}  // namespace

struct PreparedTarget::Implementation {
  explicit Implementation(const PointList& points)
      : cloud(std::make_shared<small_gicp::PointCloud>(points)), tree(cloud) {
    small_gicp::estimate_covariances(*cloud, tree, kCovarianceNeighborCount);
  }
  std::shared_ptr<small_gicp::PointCloud> cloud;
  small_gicp::KdTree<small_gicp::PointCloud> tree;
};

PreparedTarget::PreparedTarget(const PointList& points) {
  if (points.size() >= kMinimumPointCount) {
    implementation_ = std::make_shared<const Implementation>(points);
  }
}

MatchResult match_scan(const PreparedTarget& target, const PointList& source,
                       const Eigen::Isometry3d& prediction, const MatcherParameters& parameters) {
  MatchResult result;
  if (!target.is_valid() || source.size() < kMinimumPointCount) return result;

  const auto source_cloud = make_cloud_with_covariances(source);
  small_gicp::Registration<small_gicp::GICPFactor, small_gicp::SerialReduction> registration;
  registration.rejector.max_dist_sq =
      parameters.max_correspondence_distance_m * parameters.max_correspondence_distance_m;
  registration.optimizer.max_iterations = parameters.max_iterations;
  const auto raw = registration.align(*target.implementation_->cloud, *source_cloud,
                                      target.implementation_->tree, prediction);

  result.target_to_source = raw.T_target_source;
  result.converged = raw.converged;
  result.iterations = raw.iterations;
  result.information = raw.H;
  result.inlier_ratio = static_cast<double>(raw.num_inliers) / static_cast<double>(source.size());

  const Eigen::Isometry3d difference = prediction.inverse() * raw.T_target_source;
  result.is_accepted = result.converged && result.inlier_ratio >= parameters.inlier_ratio_min &&
                       difference.translation().norm() <= parameters.gate_translation_m &&
                       rotation_angle_rad(difference) <= parameters.gate_rotation_rad;
  return result;
}

}  // namespace lightnav_localization
