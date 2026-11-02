#include "lightnav_localization/pose_graph.hpp"

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>
#include <gtsam/navigation/GPSFactor.h>
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

gtsam::Key key_of(int index) { return gtsam::Symbol('x', static_cast<std::uint64_t>(index)); }

gtsam::Pose3 to_gtsam(const Eigen::Isometry3d& pose) { return gtsam::Pose3(pose.matrix()); }

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
      : parameters(parameters_in), smoother(parameters_in.window_s) {}
  PoseGraphParameters parameters;
  gtsam::IncrementalFixedLagSmoother smoother;
  gtsam::NonlinearFactorGraph pending_factors;
  gtsam::Values pending_values;
  gtsam::FixedLagSmoother::KeyTimestampMap pending_stamps;
  gtsam::Values estimate;
  std::map<int, double> stamps;
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
  graph.pending_factors.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
      key_of(0), to_gtsam(pose),
      pose_noise({0.1, 0.1, sigma_yaw_rad}, {sigma_xy_m, sigma_xy_m, 1.0}));
  return 0;
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
  graph.pending_factors.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
      key_of(previous), key_of(index), to_gtsam(planar), pose_noise(rotation_sigma, translation_sigma));
  graph.pending_values.insert(key_of(index), to_gtsam(guess));
  graph.pending_stamps[key_of(index)] = stamp_sec;
  graph.stamps[index] = stamp_sec;
  return index;
}

void PoseGraph::add_lidar_odometry_factor(int from_index, int to_index,
                                          const Eigen::Isometry3d& relative,
                                          const Eigen::Matrix<double, 6, 6>& information) {
  implementation_->pending_factors.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
      key_of(from_index), key_of(to_index), to_gtsam(relative),
      gtsam::noiseModel::Gaussian::Information(information));
}

void PoseGraph::add_map_factor(int index, const Eigen::Isometry3d& pose,
                               const Eigen::Matrix<double, 6, 6>& information) {
  implementation_->pending_factors.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
      key_of(index), to_gtsam(pose),
      robust(gtsam::noiseModel::Gaussian::Information(information),
             implementation_->parameters.huber_k));
}

void PoseGraph::add_gps_factor(int index, const Eigen::Vector3d& position,
                               const Eigen::Vector3d& sigma_m) {
  implementation_->pending_factors.emplace_shared<gtsam::GPSFactor>(
      key_of(index), gtsam::Point3(position),
      robust(Diagonal::Sigmas(sigma_m), implementation_->parameters.huber_k));
}

bool PoseGraph::optimize() {
  Implementation& graph = *implementation_;
  try {
    if (should_fail_once_) {
      should_fail_once_ = false;
      throw std::runtime_error("injected optimization failure");
    }
    graph.smoother.update(graph.pending_factors, graph.pending_values, graph.pending_stamps);
    graph.estimate = graph.smoother.calculateEstimate();
  } catch (const std::exception& exception) {
    last_error_ = exception.what();
    return false;
  }
  graph.pending_factors.resize(0);
  graph.pending_values.clear();
  graph.pending_stamps.clear();
  return true;
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
    covariance = implementation_->smoother.getISAM2().marginalCovariance(key_of(index));
  } catch (const std::exception& exception) {
    covariance_error_ = exception.what();
    return std::nullopt;
  }
  return Eigen::Vector3d(std::sqrt(covariance(3, 3)), std::sqrt(covariance(4, 4)),
                         std::sqrt(covariance(2, 2)));
}

}  // namespace lightnav_localization
