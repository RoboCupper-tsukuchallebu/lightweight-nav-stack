#include "lightnav_localization/pose_graph.hpp"

#include <chrono>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <exception>
#include <stdexcept>

namespace lightnav_localization {
namespace {

using gtsam::noiseModel::Diagonal;
using gtsam::noiseModel::Robust;
using gtsam::noiseModel::mEstimator::Huber;

// GPS supplies horizontal position only; altitude must not flatten lidar-derived terrain.
class HorizontalGpsFactor final : public gtsam::NoiseModelFactor1<gtsam::Pose3> {
 public:
  HorizontalGpsFactor(gtsam::Key key, const gtsam::Point2& position,
                      const gtsam::SharedNoiseModel& noise)
      : gtsam::NoiseModelFactor1<gtsam::Pose3>(noise, key), position_(position) {}

  gtsam::Vector evaluateError(const gtsam::Pose3& pose,
                             boost::optional<gtsam::Matrix&> jacobian = boost::none) const override {
    gtsam::Matrix36 translation_jacobian;
    const gtsam::Point3 translation = pose.translation(translation_jacobian);
    if (jacobian) *jacobian = translation_jacobian.topRows<2>();
    return translation.head<2>() - position_;
  }

 private:
  gtsam::Point2 position_;
};

gtsam::Key key_of(int index) { return gtsam::Symbol('x', static_cast<std::uint64_t>(index)); }

gtsam::Pose3 to_gtsam(const Eigen::Isometry3d& pose) { return gtsam::Pose3(pose.matrix()); }

Eigen::Isometry3d invert_isometry(const Eigen::Isometry3d& transform) {
  Eigen::Isometry3d inverse = Eigen::Isometry3d::Identity();
  inverse.linear() = transform.linear().transpose();
  inverse.translation() = -(inverse.linear() * transform.translation());
  return inverse;
}

Eigen::Isometry3d from_gtsam(const gtsam::Pose3& pose) {
  Eigen::Isometry3d result;
  result.matrix() = pose.matrix();
  return result;
}

// GTSAM orders Pose3 tangent as (rotation xyz, translation xyz).
gtsam::SharedNoiseModel pose_noise(const gtsam::Vector3& rotation_sigma,
                                   const gtsam::Vector3& translation_sigma) {
  gtsam::Vector6 sigmas;
  sigmas << rotation_sigma, translation_sigma;
  return Diagonal::Sigmas(sigmas);
}

gtsam::SharedNoiseModel robust(const gtsam::SharedNoiseModel& base, double huber_k) {
  return Robust::Create(Huber::Create(huber_k), base);
}

}  // namespace

struct PoseGraph::Implementation {
  explicit Implementation(const PoseGraphParameters& parameters_in)
      : parameters(parameters_in),
        smoother(std::make_unique<gtsam::IncrementalFixedLagSmoother>(parameters_in.window_s)) {}
  PoseGraphParameters parameters;
  std::unique_ptr<gtsam::IncrementalFixedLagSmoother> smoother;
  gtsam::NonlinearFactorGraph pending_factors;
  gtsam::Values pending_values;
  gtsam::FixedLagSmoother::KeyTimestampMap pending_stamps;
  gtsam::Values estimate;
  std::map<int, double> stamps;
  gtsam::NonlinearFactorGraph history_factors;
  gtsam::Values history_values;
  std::map<int, double> history_stamps;
  std::vector<BatchPose> batch_poses;
  gtsam::Values batch_values;
  std::vector<boost::shared_ptr<gtsam::BetweenFactor<gtsam::Pose3>>> loop_factors;
  std::set<int> gps_indices;
  int batch_iterations = 0;
  double optimization_seconds = 0.0;
  double marginals_seconds = 0.0;
  bool marginals_failed = false;
  int marginal_construction_attempts = 0;
};

PoseGraph::PoseGraph(const PoseGraphParameters& parameters)
    : implementation_(std::make_unique<Implementation>(parameters)) {}
PoseGraph::~PoseGraph() = default;
PoseGraph::PoseGraph(PoseGraph&&) noexcept = default;
PoseGraph& PoseGraph::operator=(PoseGraph&&) noexcept = default;

int PoseGraph::add_first_node(double stamp_sec, const Eigen::Isometry3d& pose, double sigma_xy_m,
                              double sigma_yaw_rad) {
  Implementation& graph = *implementation_;
  latest_index_ = 0;
  graph.pending_values.insert(key_of(0), to_gtsam(pose));
  graph.pending_stamps[key_of(0)] = stamp_sec;
  graph.stamps[0] = stamp_sec;
  boost::shared_ptr<gtsam::PriorFactor<gtsam::Pose3>> factor(
      new gtsam::PriorFactor<gtsam::Pose3>(
          key_of(0), to_gtsam(pose),
          pose_noise({0.1, 0.1, sigma_yaw_rad}, {sigma_xy_m, sigma_xy_m, 1.0})));
  graph.pending_factors.push_back(factor);
  if (graph.parameters.record_history) {
    graph.history_factors.push_back(factor);
    graph.history_values.insert(key_of(0), to_gtsam(pose));
    graph.history_stamps[0] = stamp_sec;
  }
  return 0;
}

int PoseGraph::restart_window(double stamp_sec, const Eigen::Isometry3d& pose,
                              const Eigen::Isometry3d& previous_pose, double sigma_xy_m,
                              double sigma_yaw_rad) {
  Implementation& graph = *implementation_;
  const int previous_index = latest_index_;
  const int index = previous_index + 1;
  latest_index_ = index;
  graph.smoother = std::make_unique<gtsam::IncrementalFixedLagSmoother>(
      graph.parameters.window_s);
  graph.pending_factors.resize(0);
  graph.pending_values.clear();
  graph.pending_stamps.clear();
  graph.estimate = gtsam::Values();
  graph.stamps.clear();

  graph.pending_values.insert(key_of(index), to_gtsam(pose));
  graph.pending_stamps[key_of(index)] = stamp_sec;
  graph.stamps[index] = stamp_sec;
  boost::shared_ptr<gtsam::PriorFactor<gtsam::Pose3>> prior(
      new gtsam::PriorFactor<gtsam::Pose3>(
          key_of(index), to_gtsam(pose),
          pose_noise({0.1, 0.1, sigma_yaw_rad}, {sigma_xy_m, sigma_xy_m, 1.0})));
  graph.pending_factors.push_back(prior);
  if (graph.parameters.record_history) {
    // No prior at the restarted node: its estimate may be exactly what failed. The history is
    // connected by the between factor below instead.
    graph.history_values.insert(key_of(index), to_gtsam(pose));
    graph.history_stamps[index] = stamp_sec;
  }

  if (previous_index >= 0) {
    const Eigen::Isometry3d inverse_previous = invert_isometry(previous_pose);
    boost::shared_ptr<gtsam::BetweenFactor<gtsam::Pose3>> connection(
        new gtsam::BetweenFactor<gtsam::Pose3>(
            key_of(previous_index), key_of(index), to_gtsam(inverse_previous * pose),
            pose_noise({0.1, 0.1, sigma_yaw_rad}, {sigma_xy_m, sigma_xy_m, 1.0})));
    // Only the batch history knows the previous node; the new online smoother must contain just
    // the new prior, exactly like the localization-mode restart.
    if (graph.parameters.record_history) graph.history_factors.push_back(connection);
  }
  return index;
}

int PoseGraph::add_node_with_wheel_factor(double stamp_sec, const Eigen::Isometry3d& guess,
                                          const Eigen::Isometry3d& wheel_relative) {
  Implementation& graph = *implementation_;
  const PoseGraphParameters& parameters = graph.parameters;
  const int previous = latest_index_;
  const int index = ++latest_index_;
  const double distance = wheel_relative.translation().head<2>().norm();
  const double yaw = std::abs(Eigen::AngleAxisd(wheel_relative.rotation()).angle());
  const double sigma_translation =
      std::max(parameters.sigma_floor_m, parameters.wheel_sigma_translation_per_m * distance);
  const double sigma_yaw = std::max(
      parameters.sigma_floor_rad,
      parameters.wheel_sigma_yaw_per_rad * yaw + parameters.wheel_sigma_yaw_per_m * distance);
  const double sigma_z = std::max(parameters.sigma_floor_m, parameters.planar_sigma_z_per_m * distance);
  // Weak planar constraint: dz = droll = dpitch = 0 with the looser planar sigmas.
  Eigen::Isometry3d planar = wheel_relative;
  planar.translation().z() = 0.0;
  const gtsam::Vector3 rotation_sigma(parameters.planar_sigma_tilt_rad,
                                      parameters.planar_sigma_tilt_rad, sigma_yaw);
  const gtsam::Vector3 translation_sigma(sigma_translation, sigma_translation, sigma_z);
  boost::shared_ptr<gtsam::BetweenFactor<gtsam::Pose3>> factor(
      new gtsam::BetweenFactor<gtsam::Pose3>(
          key_of(previous), key_of(index), to_gtsam(planar),
          pose_noise(rotation_sigma, translation_sigma)));
  graph.pending_factors.push_back(factor);
  graph.pending_values.insert(key_of(index), to_gtsam(guess));
  graph.pending_stamps[key_of(index)] = stamp_sec;
  graph.stamps[index] = stamp_sec;
  if (graph.parameters.record_history) {
    graph.history_factors.push_back(factor);
    graph.history_values.insert(key_of(index), to_gtsam(guess));
    graph.history_stamps[index] = stamp_sec;
  }
  return index;
}

void PoseGraph::add_lidar_odometry_factor(int from_index, int to_index,
                                          const Eigen::Isometry3d& relative,
                                          const Eigen::Matrix<double, 6, 6>& information) {
  boost::shared_ptr<gtsam::BetweenFactor<gtsam::Pose3>> factor(
      new gtsam::BetweenFactor<gtsam::Pose3>(
          key_of(from_index), key_of(to_index), to_gtsam(relative),
          gtsam::noiseModel::Gaussian::Information(information)));
  implementation_->pending_factors.push_back(factor);
  if (implementation_->parameters.record_history) {
    implementation_->history_factors.push_back(factor);
  }
}

void PoseGraph::add_map_factor(int index, const Eigen::Isometry3d& pose,
                               const Eigen::Matrix<double, 6, 6>& information) {
  boost::shared_ptr<gtsam::PriorFactor<gtsam::Pose3>> factor(
      new gtsam::PriorFactor<gtsam::Pose3>(
          key_of(index), to_gtsam(pose),
          robust(gtsam::noiseModel::Gaussian::Information(information),
                 implementation_->parameters.huber_k)));
  implementation_->pending_factors.push_back(factor);
  if (implementation_->parameters.record_history) {
    implementation_->history_factors.push_back(factor);
  }
}

void PoseGraph::add_gps_factor(int index, const Eigen::Vector3d& position,
                               const Eigen::Vector3d& sigma_m) {
  boost::shared_ptr<HorizontalGpsFactor> factor(new HorizontalGpsFactor(
      key_of(index), gtsam::Point2(position.head<2>()),
      robust(Diagonal::Sigmas(sigma_m.head<2>()), implementation_->parameters.huber_k)));
  implementation_->pending_factors.push_back(factor);
  if (implementation_->parameters.record_history) {
    implementation_->history_factors.push_back(factor);
    implementation_->gps_indices.insert(index);
  }
}

bool PoseGraph::optimize() {
  Implementation& graph = *implementation_;
  try {
    if (should_fail_once_) {
      should_fail_once_ = false;
      throw std::runtime_error("injected optimization failure");
    }
    graph.smoother->update(graph.pending_factors, graph.pending_values, graph.pending_stamps);
    graph.estimate = graph.smoother->calculateEstimate();
    if (graph.parameters.record_history) {
      // Keep the batch initial values close to the latest window estimate: nodes leave the
      // fixed-lag window after window_s, and their creation guesses would otherwise keep the
      // accumulated odometry drift forever.
      for (const auto& entry : graph.history_stamps) {
        const gtsam::Key key = key_of(entry.first);
        if (graph.estimate.exists(key)) {
          graph.history_values.update(key, graph.estimate.at(key));
        }
      }
    }
  } catch (const std::exception& exception) {
    last_error_ = exception.what();
    // Never leave a half-consumed batch behind: a later update with the same keys would fail
    // with "already exists" forever. The caller restarts the window from this estimate.
    graph.pending_factors.resize(0);
    graph.pending_values.clear();
    graph.pending_stamps.clear();
    return false;
  }
  graph.pending_factors.resize(0);
  graph.pending_values.clear();
  graph.pending_stamps.clear();
  return true;
}

bool PoseGraph::batch_optimize(int max_iterations, bool start_from_previous, bool calculate_marginals) {
  Implementation& graph = *implementation_;
  if (!graph.parameters.record_history) {
    last_error_ = "batch optimization requires record_history";
    return false;
  }
  if (max_iterations < 1) {
    last_error_ = "batch optimization requires max_iterations >= 1";
    return false;
  }
  if (graph.history_factors.empty() || graph.history_values.empty()) {
    last_error_ = "batch optimization has no graph";
    return false;
  }
  try {
    gtsam::LevenbergMarquardtParams optimizer_parameters;
    optimizer_parameters.maxIterations = max_iterations;
    const gtsam::Values& initial_values =
        start_from_previous && !graph.batch_values.empty() ? graph.batch_values : graph.history_values;
    gtsam::LevenbergMarquardtOptimizer optimizer(graph.history_factors, initial_values,
                                                 optimizer_parameters);
    const auto optimization_start = std::chrono::steady_clock::now();
    const gtsam::Values result = optimizer.optimize();
    graph.optimization_seconds = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - optimization_start).count();
    graph.batch_iterations = static_cast<int>(optimizer.iterations());
    graph.batch_values = result;
    graph.marginals_failed = false;
    graph.marginal_construction_attempts = 0;
    graph.batch_poses.clear();
    graph.batch_poses.reserve(graph.history_stamps.size());
    std::unique_ptr<gtsam::Marginals> marginals;
    const auto marginals_start = std::chrono::steady_clock::now();
    if (calculate_marginals) {
      try {
        ++graph.marginal_construction_attempts;
        if (should_fail_marginals_once_) {
          should_fail_marginals_once_ = false;
          throw std::runtime_error("injected marginal construction failure");
        }
        marginals = std::make_unique<gtsam::Marginals>(graph.history_factors, result);
      } catch (const std::exception&) {
        graph.marginals_failed = true;
      }
    }
    for (const auto& entry : graph.history_stamps) {
      BatchPose batch_pose;
      batch_pose.index = entry.first;
      batch_pose.stamp_sec = entry.second;
      batch_pose.map_to_base = from_gtsam(result.at<gtsam::Pose3>(key_of(entry.first)));
      try {
        if (!marginals) {
          graph.batch_poses.push_back(std::move(batch_pose));
          continue;
        }
        const gtsam::Matrix covariance = marginals->marginalCovariance(key_of(entry.first));
        const Eigen::Matrix3d map_rotation = batch_pose.map_to_base.rotation();
        const Eigen::Matrix3d map_translation_covariance =
            map_rotation * covariance.bottomRightCorner<3, 3>() * map_rotation.transpose();
        batch_pose.position_and_yaw_std = Eigen::Vector3d(
            std::sqrt(std::max(0.0, map_translation_covariance(0, 0))),
            std::sqrt(std::max(0.0, map_translation_covariance(1, 1))),
            std::sqrt(std::max(0.0, covariance(2, 2))));
      } catch (const std::exception&) {
        // A rank-deficient graph can still yield usable poses; the std is left absent.
      }
      graph.batch_poses.push_back(std::move(batch_pose));
    }
    graph.marginals_seconds = calculate_marginals ? std::chrono::duration<double>(
        std::chrono::steady_clock::now() - marginals_start).count() : 0.0;
  } catch (const std::exception& exception) {
    last_error_ = exception.what();
    return false;
  }
  return true;
}

bool PoseGraph::calculate_batch_marginals() {
  Implementation& graph = *implementation_;
  graph.marginals_failed = false;
  graph.marginal_construction_attempts = 1;
  try {
    if (should_fail_marginals_once_) {
      should_fail_marginals_once_ = false;
      throw std::runtime_error("injected marginal construction failure");
    }
    const gtsam::Marginals marginals(graph.history_factors, graph.batch_values);
    for (BatchPose& pose : graph.batch_poses) {
      pose.position_and_yaw_std.reset();
      try {
        const gtsam::Matrix covariance = marginals.marginalCovariance(key_of(pose.index));
        const Eigen::Matrix3d rotation = pose.map_to_base.rotation();
        const Eigen::Matrix3d translation_covariance =
            rotation * covariance.bottomRightCorner<3, 3>() * rotation.transpose();
        pose.position_and_yaw_std = Eigen::Vector3d(
            std::sqrt(std::max(0.0, translation_covariance(0, 0))),
            std::sqrt(std::max(0.0, translation_covariance(1, 1))),
            std::sqrt(std::max(0.0, covariance(2, 2))));
      } catch (const std::exception&) {
        // Individual marginal failures leave just this node without an uncertainty.
      }
    }
  } catch (const std::exception&) {
    graph.marginals_failed = true;
    for (BatchPose& pose : graph.batch_poses) pose.position_and_yaw_std.reset();
    return false;
  }
  return true;
}

bool PoseGraph::add_loop_factor(int from_index, int to_index, const Eigen::Isometry3d& relative,
                                const Eigen::Matrix<double, 6, 6>& information) {
  Implementation& graph = *implementation_;
  if (!graph.parameters.record_history) return false;
  boost::shared_ptr<gtsam::BetweenFactor<gtsam::Pose3>> factor(
      new gtsam::BetweenFactor<gtsam::Pose3>(key_of(from_index), key_of(to_index),
          to_gtsam(relative), gtsam::noiseModel::Gaussian::Information(information)));
  graph.history_factors.push_back(factor);
  graph.loop_factors.push_back(factor);
  return true;
}

bool PoseGraph::remove_loop_factor(std::size_t loop_index) {
  Implementation& graph = *implementation_;
  if (loop_index >= graph.loop_factors.size()) return false;
  const auto& factor = graph.loop_factors[loop_index];
  for (std::size_t index = 0; index < graph.history_factors.size(); ++index) {
    if (graph.history_factors[index] == factor) {
      graph.history_factors.remove(index);
      break;
    }
  }
  graph.loop_factors.erase(graph.loop_factors.begin() + loop_index);
  return true;
}

void PoseGraph::remove_history_prior_for_test() {
  if (!implementation_->history_factors.empty()) implementation_->history_factors.remove(0);
}

std::size_t PoseGraph::loop_factor_count() const { return implementation_->loop_factors.size(); }

void PoseGraph::clear_loop_factors() {
  while (!implementation_->loop_factors.empty()) remove_loop_factor(0);
}

std::vector<double> PoseGraph::loop_factor_chi_squared() const {
  std::vector<double> result;
  for (const auto& factor : implementation_->loop_factors) {
    result.push_back(factor->whitenedError(implementation_->batch_values).squaredNorm());
  }
  return result;
}

const std::set<int>& PoseGraph::gps_factor_indices() const { return implementation_->gps_indices; }
int PoseGraph::batch_iterations() const { return implementation_->batch_iterations; }
double PoseGraph::batch_optimization_seconds() const { return implementation_->optimization_seconds; }
double PoseGraph::batch_marginals_seconds() const { return implementation_->marginals_seconds; }
bool PoseGraph::batch_marginals_failed() const { return implementation_->marginals_failed; }
int PoseGraph::marginal_construction_attempts() const {
  return implementation_->marginal_construction_attempts;
}

const std::vector<BatchPose>& PoseGraph::batch_poses() const {
  return implementation_->batch_poses;
}

std::optional<Eigen::Isometry3d> PoseGraph::pose(int index) const {
  const gtsam::Values& estimate = implementation_->estimate;
  if (!estimate.exists(key_of(index))) return std::nullopt;
  return from_gtsam(estimate.at<gtsam::Pose3>(key_of(index)));
}

std::optional<Eigen::Vector3d> PoseGraph::position_and_yaw_std(int index) const {
  const gtsam::Values& estimate = implementation_->estimate;
  if (!estimate.exists(key_of(index))) return std::nullopt;
  gtsam::Matrix covariance;
  try {
    covariance = implementation_->smoother->getISAM2().marginalCovariance(key_of(index));
  } catch (const std::exception& exception) {
    covariance_error_ = exception.what();
    return std::nullopt;
  }
  const Eigen::Matrix3d map_rotation = estimate.at<gtsam::Pose3>(key_of(index)).rotation().matrix();
  const Eigen::Matrix3d map_translation_covariance =
      map_rotation * covariance.bottomRightCorner<3, 3>() * map_rotation.transpose();
  return Eigen::Vector3d(std::sqrt(map_translation_covariance(0, 0)),
                         std::sqrt(map_translation_covariance(1, 1)),
                         std::sqrt(covariance(2, 2)));
}

}  // namespace lightnav_localization
