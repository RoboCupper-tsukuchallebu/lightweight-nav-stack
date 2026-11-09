#include "lightnav_obstacle_monitor/obstacle_monitor_node.hpp"

#include <cmath>
#include <exception>
#include <limits>
#include <algorithm>
#include <climits>
#include <set>

#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/transform_listener.h>

#include "lightnav_obstacle_monitor/frame_processor.hpp"
#include "lightnav_obstacle_monitor/freshness.hpp"
#include "lightnav_obstacle_monitor/sensor_projection.hpp"

namespace lightnav
{

namespace
{
double secondsSince(
  ObstacleMonitorNode::SteadyClock::time_point origin,
  ObstacleMonitorNode::SteadyClock::time_point now)
{
  return std::chrono::duration<double>(now - origin).count();
}
}  // namespace

ObstacleMonitorNode::ObstacleMonitorNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("obstacle_monitor", options)
{
  const ObstacleMonitorParameters defaults;
  rcl_interfaces::msg::ParameterDescriptor readOnly;
  readOnly.read_only = true;
  declare_parameter("points_topic", "/velodyne_points", readOnly);
  declare_parameter("points_qos_reliability", "reliable", readOnly);
  declare_parameter("scan_topic", "", readOnly);
  declare_parameter("base_frame", "base_link", readOnly);
  declare_parameter("points_timeout_s", 0.5);
  declare_parameter("scan_timeout_s", 0.5);
  declare_parameter("tf_timeout_s", 0.1);
  declare_parameter("grid_size_m", defaults.gridSizeMeters, readOnly);
  declare_parameter("cell_size_m", defaults.cellSizeMeters, readOnly);
  declare_parameter("grid_origin_x_m", defaults.gridOriginXMeters, readOnly);
  declare_parameter("grid_origin_y_m", defaults.gridOriginYMeters, readOnly);
  declare_parameter("z_max_m", defaults.zMaxMeters);
  declare_parameter("height_diff_threshold_m", defaults.heightDifferenceThresholdMeters);
  declare_parameter("step_neighbor_radius_m", defaults.stepNeighborRadiusMeters, readOnly);
  declare_parameter("min_points_per_cell", defaults.minPointsPerCell);
  declare_parameter("scan_max_use_range_m", defaults.scanMaxUseRangeMeters);
  declare_parameter("inflation_radius_m", defaults.inflationRadiusMeters);
  declare_parameter("stop_zone_x_m", defaults.stopZoneXMeters);
  declare_parameter("stop_zone_y_half_m", defaults.stopZoneYHalfMeters);
  declare_parameter("slow_zone_x_m", defaults.slowZoneXMeters);
  declare_parameter("slow_zone_y_half_m", defaults.slowZoneYHalfMeters);
  declare_parameter("min_cell_count", defaults.minCellCount);
  declare_parameter("occupied_min_frames", defaults.occupiedMinFrames);
  declare_parameter("clear_hold_s", defaults.clearHoldSeconds);
  declare_parameter("watchdog_hz", 10.0, readOnly);
  for (const auto & name : list_parameters({}, 1).names) {
    startupParameters_.emplace(name, get_parameter(name));
  }
  appliedParameters_ = startupParameters_;
  validationHandle_ = add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & changes) {
      auto result = validateParameters(changes, true);
      if (!result.successful) {RCLCPP_WARN(get_logger(), "parameter rejected: %s", result.reason.c_str());}
      return result;
    });
  applicationHandle_ = add_post_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & changes) {applyParameters(changes);});
}


rcl_interfaces::msg::SetParametersResult ObstacleMonitorNode::validateParameters(
  const std::vector<rclcpp::Parameter> & changes, bool runtime) const
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = false;
  auto reject = [&](const std::string & reason) {result.reason = reason; return result;};
  const std::set<std::string> increasing = {"z_max_m", "scan_max_use_range_m",
    "inflation_radius_m", "stop_zone_x_m", "stop_zone_y_half_m", "slow_zone_x_m",
    "slow_zone_y_half_m", "clear_hold_s"};
  const std::set<std::string> decreasing = {"height_diff_threshold_m", "min_points_per_cell",
    "min_cell_count", "occupied_min_frames", "points_timeout_s", "scan_timeout_s", "tf_timeout_s"};
  auto values = startupParameters_;
  for (auto & entry : values) {entry.second = get_parameter(entry.first);}
  for (const auto & parameter : changes) {
    const auto & name = parameter.get_name();
    if (name.rfind("qos_overrides.", 0) == 0 && !has_parameter(name)) {
      continue;  // rclcpp declares read-only QoS overrides when subscriptions are created.
    }
    if (name == "use_sim_time") {return reject("use_sim_time is read-only after construction");}
    if (!increasing.count(name) && !decreasing.count(name)) {
      return reject(name + ": read-only parameter");
    }
    const auto & baseline = startupParameters_.at(name);
    if (parameter.get_type() != baseline.get_type()) {return reject(name + ": wrong type");}
    const double requested = parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER ?
      static_cast<double>(parameter.as_int()) : parameter.as_double();
    const double initial = baseline.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER ?
      static_cast<double>(baseline.as_int()) : baseline.as_double();
    if (!(std::isfinite(requested) && std::isfinite(initial)) ||
      (runtime && !((increasing.count(name) && requested >= initial) ||
      (decreasing.count(name) && requested <= initial)))) {
      return reject(name + ": must be finite and on the conservative side of startup value (baseline=" +
        baseline.value_to_string() + ", requested=" + parameter.value_to_string() + ")");
    }
    values.insert_or_assign(name, parameter);
  }
  auto number = [&](const char * name) {
      const auto & value = values.at(name);
      return value.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER ?
             static_cast<double>(value.as_int()) : value.as_double();
    };
  for (const auto & name : increasing) {
    if (!std::isfinite(number(name.c_str())) || number(name.c_str()) < 0.0) {
      return reject(name + ": must be finite and nonnegative");
    }
  }
  for (const auto & name : decreasing) {
    const double value = number(name.c_str());
    if (!std::isfinite(value) || value <= 0.0 || value > INT_MAX) {
      return reject(name + ": must be finite and positive");
    }
  }
  const double size = number("grid_size_m"), cell = number("cell_size_m");
  const double originX = number("grid_origin_x_m"), originY = number("grid_origin_y_m");
  if (!(std::isfinite(size) && std::isfinite(cell) && std::isfinite(originX) &&
    std::isfinite(originY) && size > 0 && cell > 0 && size / cell <= INT_MAX &&
    originX <= 0 && originY <= -number("slow_zone_y_half_m") &&
    originY + size >= number("slow_zone_y_half_m") &&
    originX + size >= number("slow_zone_x_m") &&
    number("stop_zone_x_m") <= number("slow_zone_x_m") &&
    number("stop_zone_y_half_m") <= number("slow_zone_y_half_m"))) {
    return reject("stop zone must fit inside slow zone and grid");
  }
  result.successful = true;
  return result;
}

void ObstacleMonitorNode::applyParameters(const std::vector<rclcpp::Parameter> & changes)
{
  auto updated = parameters_;
  auto pointsTimeout = pointsTimeoutSeconds_, scanTimeout = scanTimeoutSeconds_;
  auto tfTimeout = tfTimeoutSeconds_;
  for (const auto & value : changes) {
    const auto & name = value.get_name();
    if (name == "z_max_m") {updated.zMaxMeters = value.as_double();}
    if (name == "height_diff_threshold_m") {updated.heightDifferenceThresholdMeters = value.as_double();}
    if (name == "min_points_per_cell") {updated.minPointsPerCell = static_cast<int>(value.as_int());}
    if (name == "scan_max_use_range_m") {updated.scanMaxUseRangeMeters = value.as_double();}
    if (name == "inflation_radius_m") {updated.inflationRadiusMeters = value.as_double();}
    if (name == "stop_zone_x_m") {updated.stopZoneXMeters = value.as_double();}
    if (name == "stop_zone_y_half_m") {updated.stopZoneYHalfMeters = value.as_double();}
    if (name == "slow_zone_x_m") {updated.slowZoneXMeters = value.as_double();}
    if (name == "slow_zone_y_half_m") {updated.slowZoneYHalfMeters = value.as_double();}
    if (name == "min_cell_count") {updated.minCellCount = static_cast<int>(value.as_int());}
    if (name == "occupied_min_frames") {updated.occupiedMinFrames = static_cast<int>(value.as_int());}
    if (name == "clear_hold_s") {updated.clearHoldSeconds = value.as_double();}
    if (name == "points_timeout_s") {pointsTimeout = value.as_double();}
    if (name == "scan_timeout_s") {scanTimeout = value.as_double();}
    if (name == "tf_timeout_s") {tfTimeout = value.as_double();}
  }
  parameters_ = updated;
  pointsTimeoutSeconds_ = pointsTimeout;
  scanTimeoutSeconds_ = scanTimeout;
  tfTimeoutSeconds_ = tfTimeout;
  if (stopHysteresis_) {
    stopHysteresis_->updateParameters(updated.minCellCount, updated.occupiedMinFrames, updated.clearHoldSeconds);
    slowHysteresis_->updateParameters(updated.minCellCount, updated.occupiedMinFrames, updated.clearHoldSeconds);
  }
  for (const auto & value : changes) {
    auto previousEntry = appliedParameters_.find(value.get_name());
    if (previousEntry == appliedParameters_.end()) {continue;}
    auto & previous = previousEntry->second;
    if (previous != value) {
      RCLCPP_INFO(get_logger(), "parameter changed: %s %s -> %s", value.get_name().c_str(),
        previous.value_to_string().c_str(), value.value_to_string().c_str());
      previous = value;
    }
  }
}

ObstacleMonitorNode::CallbackReturn ObstacleMonitorNode::on_configure(const rclcpp_lifecycle::State &)
{
  parameters_.gridSizeMeters = get_parameter("grid_size_m").as_double();
  parameters_.cellSizeMeters = get_parameter("cell_size_m").as_double();
  parameters_.gridOriginXMeters = get_parameter("grid_origin_x_m").as_double();
  parameters_.gridOriginYMeters = get_parameter("grid_origin_y_m").as_double();
  parameters_.zMaxMeters = get_parameter("z_max_m").as_double();
  parameters_.heightDifferenceThresholdMeters = get_parameter("height_diff_threshold_m").as_double();
  parameters_.stepNeighborRadiusMeters = get_parameter("step_neighbor_radius_m").as_double();
  parameters_.minPointsPerCell = static_cast<int>(get_parameter("min_points_per_cell").as_int());
  parameters_.scanMaxUseRangeMeters = get_parameter("scan_max_use_range_m").as_double();
  parameters_.inflationRadiusMeters = get_parameter("inflation_radius_m").as_double();
  parameters_.stopZoneXMeters = get_parameter("stop_zone_x_m").as_double();
  parameters_.stopZoneYHalfMeters = get_parameter("stop_zone_y_half_m").as_double();
  parameters_.slowZoneXMeters = get_parameter("slow_zone_x_m").as_double();
  parameters_.slowZoneYHalfMeters = get_parameter("slow_zone_y_half_m").as_double();
  parameters_.minCellCount = static_cast<int>(get_parameter("min_cell_count").as_int());
  parameters_.occupiedMinFrames = static_cast<int>(get_parameter("occupied_min_frames").as_int());
  parameters_.clearHoldSeconds = get_parameter("clear_hold_s").as_double();
  baseFrame_ = get_parameter("base_frame").as_string();
  pointsTopic_ = get_parameter("points_topic").as_string();
  scanTopic_ = get_parameter("scan_topic").as_string();
  const std::string pointsQosReliability = get_parameter("points_qos_reliability").as_string();
  pointsTimeoutSeconds_ = get_parameter("points_timeout_s").as_double();
  scanTimeoutSeconds_ = get_parameter("scan_timeout_s").as_double();
  tfTimeoutSeconds_ = get_parameter("tf_timeout_s").as_double();
  watchdogHz_ = get_parameter("watchdog_hz").as_double();

  const auto validation = validateParameters({}, false);
  if (!validation.successful) {
    RCLCPP_ERROR(get_logger(), "invalid parameters: %s", validation.reason.c_str());
    return CallbackReturn::FAILURE;
  }

  rclcpp::QoS pointsQos(5);
  if (pointsQosReliability == "reliable") {
    pointsQos.reliable();
  } else if (pointsQosReliability == "best_effort") {
    pointsQos.best_effort();
  } else {
    RCLCPP_ERROR(
      get_logger(), "invalid points_qos_reliability '%s': must be 'reliable' or 'best_effort'",
      pointsQosReliability.c_str());
    return CallbackReturn::FAILURE;
  }
  pointsQos.durability_volatile();

  const bool isValid = parameters_.cellSizeMeters > 0.0 && parameters_.gridSizeMeters > 0.0 &&
    std::isfinite(parameters_.stepNeighborRadiusMeters) &&
    parameters_.stepNeighborRadiusMeters >= 0.0 && watchdogHz_ > 0.0 &&
    (!pointsTopic_.empty() || !scanTopic_.empty());
  if (!isValid) {
    RCLCPP_ERROR(get_logger(), "invalid parameters: sizes and watchdog_hz must be positive, step_neighbor_radius_m must be finite and nonnegative, and at least one input topic is needed");
    return CallbackReturn::FAILURE;
  }
  geometry_ = GridGeometry::fromParameters(parameters_);
  stopHysteresis_.emplace(
    parameters_.minCellCount, parameters_.occupiedMinFrames, parameters_.clearHoldSeconds);
  slowHysteresis_.emplace(
    parameters_.minCellCount, parameters_.occupiedMinFrames, parameters_.clearHoldSeconds);
  pointsState_ = {};
  scanState_ = {};
  lastPublish_.reset();

  tfBuffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_, this, false);
  statusPublisher_ = create_publisher<lightnav_msgs::msg::ObstacleStatus>(
    "/lightnav/obstacle_status", rclcpp::QoS(10).reliable().durability_volatile());
  gridPublisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
    "/lightnav/local_grid", rclcpp::QoS(1).reliable().durability_volatile());
  if (!pointsTopic_.empty()) {
    pointsSubscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      pointsTopic_, pointsQos,
      [this](sensor_msgs::msg::PointCloud2::SharedPtr cloud) {handlePointCloud(cloud);});
  }
  if (!scanTopic_.empty()) {
    scanSubscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scanTopic_, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::LaserScan::SharedPtr scan) {handleLaserScan(scan);});
  }
  return CallbackReturn::SUCCESS;
}

ObstacleMonitorNode::CallbackReturn ObstacleMonitorNode::on_activate(const rclcpp_lifecycle::State &)
{
  statusPublisher_->on_activate();
  gridPublisher_->on_activate();
  watchdogTimer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / watchdogHz_)),
    [this]() {handleWatchdog();});
  return CallbackReturn::SUCCESS;
}

void ObstacleMonitorNode::releaseActiveResources()
{
  watchdogTimer_.reset();
  if (statusPublisher_) {
    statusPublisher_->on_deactivate();
  }
  if (gridPublisher_) {
    gridPublisher_->on_deactivate();
  }
}

ObstacleMonitorNode::CallbackReturn ObstacleMonitorNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  releaseActiveResources();
  return CallbackReturn::SUCCESS;
}

ObstacleMonitorNode::CallbackReturn ObstacleMonitorNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  pointsSubscription_.reset();
  scanSubscription_.reset();
  statusPublisher_.reset();
  gridPublisher_.reset();
  tfListener_.reset();
  tfBuffer_.reset();
  return CallbackReturn::SUCCESS;
}

ObstacleMonitorNode::CallbackReturn ObstacleMonitorNode::on_shutdown(const rclcpp_lifecycle::State &)
{
  releaseActiveResources();
  pointsSubscription_.reset();
  scanSubscription_.reset();
  return CallbackReturn::SUCCESS;
}

std::optional<Eigen::Isometry3d> ObstacleMonitorNode::lookupBaseFromSensor(
  const std::string & sensorFrame, SensorState & state)
{
  state.sensorFrame = sensorFrame;
  try {
    const auto transform =
      tfBuffer_->lookupTransform(baseFrame_, sensorFrame, tf2::TimePointZero);
    state.hasLastTransformFailed = false;
    return tf2::transformToEigen(transform);
  } catch (const tf2::TransformException & exception) {
    state.hasLastTransformFailed = true;
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000, "TF %s -> %s unavailable: %s", sensorFrame.c_str(),
      baseFrame_.c_str(), exception.what());
    return std::nullopt;
  }
}

void ObstacleMonitorNode::handlePointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr cloud)
{
  const auto frameStart = SteadyClock::now();
  const auto baseFromSensor = lookupBaseFromSensor(cloud->header.frame_id, pointsState_);
  if (!baseFromSensor) {
    publishResult(frameStart, false);
    return;
  }
  ObstacleGrid grid(geometry_, parameters_);
  try {
    projectPointCloud(*cloud, *baseFromSensor, grid);
  } catch (const std::exception & exception) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "%s", exception.what());
    publishResult(frameStart, false);
    return;
  }
  pointsState_.occupiedCells = grid.occupiedCells();
  pointsState_.lastUpdate = SteadyClock::now();
  publishResult(frameStart, true);
}

void ObstacleMonitorNode::handleLaserScan(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
  const auto frameStart = SteadyClock::now();
  const auto baseFromSensor = lookupBaseFromSensor(scan->header.frame_id, scanState_);
  if (!baseFromSensor) {
    return;
  }
  ObstacleGrid grid(geometry_, parameters_);
  projectLaserScan(*scan, *baseFromSensor, parameters_.scanMaxUseRangeMeters, grid);
  scanState_.occupiedCells = grid.occupiedCells();
  scanState_.lastUpdate = SteadyClock::now();
  // The 3D cloud drives publishing when enabled; the scan drives it only when it is alone.
  if (pointsTopic_.empty()) {
    publishResult(frameStart, true);
  }
}

void ObstacleMonitorNode::handleWatchdog()
{
  const auto now = SteadyClock::now();
  const double publishPeriodSeconds = 1.0 / watchdogHz_;
  if (!lastPublish_ || secondsSince(*lastPublish_, now) >= publishPeriodSeconds) {
    publishResult(now, false);
  }
}

void ObstacleMonitorNode::publishResult(
  SteadyClock::time_point frameStart, bool shouldUpdateHysteresis)
{
  if (!statusPublisher_ || !statusPublisher_->is_activated()) {
    return;
  }
  const auto now = SteadyClock::now();
  const double nowSeconds = std::chrono::duration<double>(now.time_since_epoch()).count();
  const auto toSeconds = [](const std::optional<SteadyClock::time_point> & time) {
      return time ? std::optional<double>(
        std::chrono::duration<double>(time->time_since_epoch()).count()) : std::nullopt;
    };
  lightnav_msgs::msg::ObstacleStatus status;
  std::vector<std::uint8_t> occupiedCells;
  const auto collect = [&](const char * name, const std::string & topic, const SensorState & state,
      double timeoutSeconds) {
      if (topic.empty()) {
        return;
      }
      const auto staleSources = findStaleSources(
        name, state.sensorFrame, toSeconds(state.lastUpdate), state.hasLastTransformFailed,
        nowSeconds, timeoutSeconds);
      status.stale_sources.insert(
        status.stale_sources.end(), staleSources.begin(), staleSources.end());
      if (staleSources.empty()) {
        occupiedCells = combineOccupiedCells(occupiedCells, state.occupiedCells);
      }
    };
  collect("points", pointsTopic_, pointsState_, pointsTimeoutSeconds_);
  collect("scan", scanTopic_, scanState_, scanTimeoutSeconds_);
  status.is_input_stale = !status.stale_sources.empty();
  status.header.stamp = get_clock()->now();
  status.header.frame_id = baseFrame_;
  status.nearest_distance_m = std::numeric_limits<float>::infinity();

  if (!status.is_input_stale) {
    const auto summary = summarizeOccupiedCells(occupiedCells, geometry_, parameters_);
    if (shouldUpdateHysteresis) {
      stopHysteresis_->update(summary.stopZoneCellCount, nowSeconds);
      slowHysteresis_->update(summary.slowZoneCellCount, nowSeconds);
    }
    status.stop_zone_cell_count = summary.stopZoneCellCount;
    status.slow_zone_cell_count = summary.slowZoneCellCount;
    status.nearest_distance_m = summary.nearestDistanceMeters;
    auto gridMessage = buildOccupancyGridMessage(summary.inflatedCells, geometry_, baseFrame_);
    gridMessage.header.stamp = status.header.stamp;
    gridPublisher_->publish(gridMessage);
  }
  // While an input is stale no grid is published, so the controller's grid timeout fires.
  status.is_stop_zone_occupied = stopHysteresis_->isOccupied();
  status.is_slow_zone_occupied = slowHysteresis_->isOccupied();
  status.processing_time_ms =
    std::chrono::duration<float, std::milli>(SteadyClock::now() - frameStart).count();
  statusPublisher_->publish(status);
  lastPublish_ = now;
}

}  // namespace lightnav
