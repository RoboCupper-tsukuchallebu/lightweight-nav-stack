#include "lightnav_localization/pose_graph.hpp"

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
#include <optional>
#include <utility>
#include <stdexcept>

namespace lightnav_localization {
namespace {

using gtsam::noiseModel::Diagonal;
using gtsam::noiseModel::Robust;
using gtsam::noiseModel::mEstimator::Huber;
using gtsam::noiseModel::mEstimator::Cauchy;

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

// Pulls the body z axis towards the map z axis: the error is the horizontal part of "up" seen from
// the body, which is zero when the vehicle is level and about (-pitch, roll) for small angles.
class LevelAttitudeFactor final : public gtsam::NoiseModelFactor1<gtsam::Pose3> {
 public:
  LevelAttitudeFactor(gtsam::Key key, const gtsam::SharedNoiseModel& noise)
      : gtsam::NoiseModelFactor1<gtsam::Pose3>(noise, key) {}

  gtsam::Vector evaluateError(const gtsam::Pose3& pose,
                             boost::optional<gtsam::Matrix&> jacobian = boost::none) const override {
    gtsam::Matrix36 rotation_jacobian;
    gtsam::Matrix33 unrotate_jacobian;
    const gtsam::Rot3 rotation = pose.rotation(rotation_jacobian);
    const gtsam::Point3 up_in_body = rotation.unrotate(gtsam::Point3(0.0, 0.0, 1.0), unrotate_jacobian);
    if (jacobian) *jacobian = (unrotate_jacobian * rotation_jacobian).topRows<2>();
    return up_in_body.head<2>();
  }
};

// Keeps the map-frame height near a reference height (the first node), loosely enough for real terrain.
class HeightFactor final : public gtsam::NoiseModelFactor1<gtsam::Pose3> {
 public:
  HeightFactor(gtsam::Key key, double reference_height_m, const gtsam::SharedNoiseModel& noise)
      : gtsam::NoiseModelFactor1<gtsam::Pose3>(noise, key), reference_height_m_(reference_height_m) {}

  gtsam::Vector evaluateError(const gtsam::Pose3& pose,
                             boost::optional<gtsam::Matrix&> jacobian = boost::none) const override {
    gtsam::Matrix36 translation_jacobian;
    const gtsam::Point3 translation = pose.translation(translation_jacobian);
    if (jacobian) *jacobian = translation_jacobian.row(2);
    return gtsam::Vector1(translation.z() - reference_height_m_);
  }

 private:
  double reference_height_m_;
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
        smoother(std::make_unique<gtsam::IncrementalFixedLagSmoother>(parameters_in.window_s)) {
    if ((parameters.wheel_robust_kernel != "huber" && parameters.wheel_robust_kernel != "cauchy") ||
        !std::isfinite(parameters.wheel_robust_k) || parameters.wheel_robust_k <= 0.0 ||
        !std::isfinite(parameters.wheel_yaw_slip_threshold_rad) || parameters.wheel_yaw_slip_threshold_rad < 0.0) {
      throw std::invalid_argument("invalid wheel noise parameters");
    }
    if (!std::isfinite(parameters.gyro_noise_density_rad_s_sqrt_hz) ||
        parameters.gyro_noise_density_rad_s_sqrt_hz <= 0.0 ||
        !std::isfinite(parameters.gyro_bias_sigma_rad_s) || parameters.gyro_bias_sigma_rad_s <= 0.0 ||
        !std::isfinite(parameters.gyro_scale_sigma) || parameters.gyro_scale_sigma <= 0.0) {
      throw std::invalid_argument("invalid gyro noise parameters");
    }
    if (!std::isfinite(parameters.absolute_sigma_tilt_rad) || parameters.absolute_sigma_tilt_rad < 0.0 ||
        !std::isfinite(parameters.absolute_sigma_z_m) || parameters.absolute_sigma_z_m < 0.0) {
      throw std::invalid_argument("invalid absolute tilt / height sigmas");
    }
  }

  // Absolute level-attitude and height factors of one node (each only when its sigma is positive).
  void add_ground_vehicle_factors(int index, bool should_record_history) {
    if (parameters.absolute_sigma_tilt_rad > 0.0) {
      boost::shared_ptr<LevelAttitudeFactor> factor(new LevelAttitudeFactor(
          key_of(index), Diagonal::Sigmas(gtsam::Vector2::Constant(parameters.absolute_sigma_tilt_rad))));
      pending_factors.push_back(factor);
      if (should_record_history) history_factors.push_back(factor);
    }
    if (parameters.absolute_sigma_z_m > 0.0 && reference_height_m) {
      boost::shared_ptr<HeightFactor> factor(new HeightFactor(
          key_of(index), *reference_height_m, Diagonal::Sigmas(gtsam::Vector1(parameters.absolute_sigma_z_m))));
      pending_factors.push_back(factor);
      if (should_record_history) history_factors.push_back(factor);
    }
  }

  PoseGraphParameters parameters;
  std::optional<double> reference_height_m;  // height of the first node; kept across restart_window()
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
  gtsam::Values batch_values;  // result of the last batch_optimize(); initial guess of the next one
  // Kept apart from history_factors so they can be removed again (a failed loop closing must leave the
  // graph as it was). batch_optimize() uses both.
  gtsam::NonlinearFactorGraph loop_closure_factors;
  // batch_values / batch_poses as they were when the first loop closure factor was added.
  std::optional<std::pair<gtsam::Values, std::vector<BatchPose>>> batch_state_before_loop_closures;

  gtsam::NonlinearFactorGraph batch_factors() const {
    gtsam::NonlinearFactorGraph all_factors = history_factors;
    all_factors.push_back(loop_closure_factors);
    return all_factors;
  }
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
  graph.reference_height_m = pose.translation().z();
  graph.add_ground_vehicle_factors(0, graph.parameters.record_history);
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
  if (!graph.reference_height_m) graph.reference_height_m = pose.translation().z();
  graph.add_ground_vehicle_factors(index, graph.parameters.record_history);

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

double wheel_yaw_sigma(const PoseGraphParameters& parameters, const Eigen::Isometry3d& wheel_relative,
                       double yaw_disagreement_rad) {
  const double distance = wheel_relative.translation().head<2>().norm();
  const double yaw = std::abs(Eigen::AngleAxisd(wheel_relative.rotation()).angle());
  double sigma = std::max(parameters.sigma_floor_rad,
      parameters.wheel_sigma_yaw_per_rad * yaw + parameters.wheel_sigma_yaw_per_m * distance);
  if (parameters.wheel_yaw_slip_threshold_rad > 0.0 &&
      yaw_disagreement_rad > parameters.wheel_yaw_slip_threshold_rad) {
    sigma = std::max(sigma, yaw_disagreement_rad);
  }
  return sigma;
}

double gyro_yaw_sigma(const PoseGraphParameters& parameters, double interval_sec, double yaw_delta_rad,
                      double uncovered_wheel_yaw_rad) {
  if (!std::isfinite(interval_sec) || interval_sec < 0.0 || !std::isfinite(yaw_delta_rad) ||
      !std::isfinite(uncovered_wheel_yaw_rad)) {
    throw std::invalid_argument("invalid gyro yaw sigma arguments");
  }
  const double white_noise = parameters.gyro_noise_density_rad_s_sqrt_hz;
  const double bias_drift = parameters.gyro_bias_sigma_rad_s * interval_sec;
  const double scale_error = parameters.gyro_scale_sigma * std::abs(yaw_delta_rad);
  const double variance = white_noise * white_noise * interval_sec + bias_drift * bias_drift +
                          scale_error * scale_error + uncovered_wheel_yaw_rad * uncovered_wheel_yaw_rad;
  return std::max(parameters.sigma_floor_rad, std::sqrt(variance));
}

int PoseGraph::add_node_with_wheel_factor(double stamp_sec, const Eigen::Isometry3d& guess,
                                          const Eigen::Isometry3d& wheel_relative, double yaw_disagreement_rad,
                                          double minimum_yaw_sigma_rad) {
  Implementation& graph = *implementation_;
  const PoseGraphParameters& parameters = graph.parameters;
  const int previous = latest_index_;
  const int index = ++latest_index_;
  const double distance = wheel_relative.translation().head<2>().norm();
  const double sigma_translation =
      std::max(parameters.sigma_floor_m, parameters.wheel_sigma_translation_per_m * distance);
  const double sigma_yaw =
      std::max(wheel_yaw_sigma(parameters, wheel_relative, yaw_disagreement_rad), minimum_yaw_sigma_rad);
  const double sigma_z = std::max(parameters.sigma_floor_m, parameters.planar_sigma_z_per_m * distance);
  // Weak planar constraint: dz = droll = dpitch = 0 with the looser planar sigmas.
  Eigen::Isometry3d planar = wheel_relative;
  planar.translation().z() = 0.0;
  const gtsam::Vector3 rotation_sigma(parameters.planar_sigma_tilt_rad,
                                      parameters.planar_sigma_tilt_rad, sigma_yaw);
  const gtsam::Vector3 translation_sigma(sigma_translation, sigma_translation, sigma_z);
  gtsam::SharedNoiseModel wheel_noise = pose_noise(rotation_sigma, translation_sigma);
  if (parameters.wheel_yaw_slip_threshold_rad > 0.0 &&
      yaw_disagreement_rad > parameters.wheel_yaw_slip_threshold_rad) {
    wheel_noise = Robust::Create(parameters.wheel_robust_kernel == "cauchy"
        ? gtsam::noiseModel::mEstimator::Base::shared_ptr(Cauchy::Create(parameters.wheel_robust_k))
        : gtsam::noiseModel::mEstimator::Base::shared_ptr(Huber::Create(parameters.wheel_robust_k)), wheel_noise);
  }
  boost::shared_ptr<gtsam::BetweenFactor<gtsam::Pose3>> factor(
      new gtsam::BetweenFactor<gtsam::Pose3>(key_of(previous), key_of(index), to_gtsam(planar), wheel_noise));
  graph.pending_factors.push_back(factor);
  graph.pending_values.insert(key_of(index), to_gtsam(guess));
  graph.pending_stamps[key_of(index)] = stamp_sec;
  graph.stamps[index] = stamp_sec;
  if (graph.parameters.record_history) {
    graph.history_factors.push_back(factor);
    graph.history_values.insert(key_of(index), to_gtsam(guess));
    graph.history_stamps[index] = stamp_sec;
  }
  graph.add_ground_vehicle_factors(index, graph.parameters.record_history);
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

void PoseGraph::add_rotation_factor(int from_index, int to_index, double yaw_delta_rad,
                                    double yaw_sigma_rad) {
  if (!std::isfinite(yaw_delta_rad) || !std::isfinite(yaw_sigma_rad) || yaw_sigma_rad <= 0.0) {
    throw std::invalid_argument("invalid rotation factor");
  }
  // Roll, pitch and translation are not measured. 100 rad and 1000 m are far above any plausible
  // error, so these rows carry (almost) no information, yet stay finite so that the noise model
  // remains a proper Gaussian (a hard "unconstrained" would need a different factor type).
  constexpr double kUnconstrainedRotationSigmaRad = 100.0;
  constexpr double kUnconstrainedTranslationSigmaM = 1000.0;
  const gtsam::Pose3 measured_relative(gtsam::Rot3::Yaw(yaw_delta_rad), gtsam::Point3(0.0, 0.0, 0.0));
  boost::shared_ptr<gtsam::BetweenFactor<gtsam::Pose3>> factor(
      new gtsam::BetweenFactor<gtsam::Pose3>(
          key_of(from_index), key_of(to_index), measured_relative,
          pose_noise({kUnconstrainedRotationSigmaRad, kUnconstrainedRotationSigmaRad, yaw_sigma_rad},
                     {kUnconstrainedTranslationSigmaM, kUnconstrainedTranslationSigmaM,
                      kUnconstrainedTranslationSigmaM})));
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

void PoseGraph::add_loop_closure_factor(int from_index, int to_index, const Eigen::Isometry3d& relative,
                                        double sigma_translation_m, double sigma_rotation_rad) {
  Implementation& graph = *implementation_;
  if (!graph.parameters.record_history) return;
  boost::shared_ptr<gtsam::BetweenFactor<gtsam::Pose3>> factor(
      new gtsam::BetweenFactor<gtsam::Pose3>(
          key_of(from_index), key_of(to_index), to_gtsam(relative),
          robust(pose_noise(gtsam::Vector3::Constant(sigma_rotation_rad),
                            gtsam::Vector3::Constant(sigma_translation_m)),
                 graph.parameters.huber_k)));
  if (graph.loop_closure_factors.empty()) {
    graph.batch_state_before_loop_closures = std::make_pair(graph.batch_values, graph.batch_poses);
  }
  graph.loop_closure_factors.push_back(factor);
}

std::size_t PoseGraph::loop_closure_factor_count() const {
  return implementation_->loop_closure_factors.size();
}

void PoseGraph::remove_loop_closure_factors() {
  Implementation& graph = *implementation_;
  graph.loop_closure_factors.resize(0);
  if (graph.batch_state_before_loop_closures) {
    graph.batch_values = graph.batch_state_before_loop_closures->first;
    graph.batch_poses = graph.batch_state_before_loop_closures->second;
    graph.batch_state_before_loop_closures.reset();
  }
}

bool PoseGraph::batch_optimize(int max_iterations, bool should_compute_marginals) {
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
  if (batch_failure_countdown_ >= 0 && batch_failure_countdown_-- == 0) {
    last_error_ = "injected batch optimization failure";
    return false;
  }
  try {
    const gtsam::NonlinearFactorGraph all_factors = graph.batch_factors();
    gtsam::LevenbergMarquardtParams optimizer_parameters;
    optimizer_parameters.maxIterations = max_iterations;
    gtsam::Values initial_values = graph.history_values;
    for (const gtsam::Key key : graph.batch_values.keys()) {
      if (initial_values.exists(key)) initial_values.update(key, graph.batch_values.at(key));
    }
    gtsam::LevenbergMarquardtOptimizer optimizer(all_factors, initial_values,
                                                 optimizer_parameters);
    const gtsam::Values result = optimizer.optimize();
    graph.batch_values = result;
    graph.batch_poses.clear();
    graph.batch_poses.reserve(graph.history_stamps.size());
    std::unique_ptr<gtsam::Marginals> marginals;
    for (const auto& entry : graph.history_stamps) {
      BatchPose batch_pose;
      batch_pose.index = entry.first;
      batch_pose.stamp_sec = entry.second;
      batch_pose.map_to_base = from_gtsam(result.at<gtsam::Pose3>(key_of(entry.first)));
      if (!should_compute_marginals) {
        graph.batch_poses.push_back(std::move(batch_pose));
        continue;
      }
      try {
        if (!marginals) {
          marginals = std::make_unique<gtsam::Marginals>(all_factors, result);
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
  } catch (const std::exception& exception) {
    last_error_ = exception.what();
    return false;
  }
  return true;
}

const std::vector<BatchPose>& PoseGraph::batch_poses() const {
  return implementation_->batch_poses;
}

std::vector<BatchPose> PoseGraph::recorded_poses() const {
  const Implementation& graph = *implementation_;
  std::vector<BatchPose> poses;
  poses.reserve(graph.history_stamps.size());
  for (const auto& entry : graph.history_stamps) {
    const gtsam::Key key = key_of(entry.first);
    if (!graph.history_values.exists(key)) continue;
    BatchPose pose;
    pose.index = entry.first;
    pose.stamp_sec = entry.second;
    pose.map_to_base = from_gtsam(graph.history_values.at<gtsam::Pose3>(key));
    poses.push_back(std::move(pose));
  }
  return poses;
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
