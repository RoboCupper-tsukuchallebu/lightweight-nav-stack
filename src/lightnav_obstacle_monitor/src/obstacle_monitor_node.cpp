#include "lightnav_obstacle_monitor/obstacle_monitor_node.hpp"

#include <cmath>
#include <exception>
#include <limits>

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
  declare_parameter("points_topic", "/velodyne_points");
  declare_parameter("scan_topic", "");
  declare_parameter("base_frame", "base_link");
  declare_parameter("points_timeout_s", 0.5);
  declare_parameter("scan_timeout_s", 0.5);
  declare_parameter("tf_timeout_s", 0.1);
  declare_parameter("grid_size_m", defaults.gridSizeMeters);
  declare_parameter("cell_size_m", defaults.cellSizeMeters);
  declare_parameter("grid_origin_x_m", defaults.gridOriginXMeters);
  declare_parameter("grid_origin_y_m", defaults.gridOriginYMeters);
  declare_parameter("z_max_m", defaults.zMaxMeters);
  declare_parameter("height_diff_threshold_m", defaults.heightDifferenceThresholdMeters);
  declare_parameter("step_neighbor_radius_m", defaults.stepNeighborRadiusMeters);
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
  declare_parameter("watchdog_hz", 10.0);
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
  pointsTimeoutSeconds_ = get_parameter("points_timeout_s").as_double();
  scanTimeoutSeconds_ = get_parameter("scan_timeout_s").as_double();
  tfTimeoutSeconds_ = get_parameter("tf_timeout_s").as_double();
  watchdogHz_ = get_parameter("watchdog_hz").as_double();

  const bool isValid = parameters_.cellSizeMeters > 0.0 && parameters_.gridSizeMeters > 0.0 &&
    std::isfinite(parameters_.stepNeighborRadiusMeters) &&
    parameters_.stepNeighborRadiusMeters >= 0.0 && watchdogHz_ > 0.0 && (!pointsTopic_.empty() || !scanTopic_.empty());
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
      pointsTopic_, rclcpp::SensorDataQoS(),
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
