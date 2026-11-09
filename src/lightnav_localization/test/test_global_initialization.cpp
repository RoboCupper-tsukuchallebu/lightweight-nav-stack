#include <chrono>

#include <gtest/gtest.h>

#include <vector>
#include <stdexcept>
#include <cmath>
#include <iostream>

#include "lightnav_localization/global_initialization.hpp"
#include "synthetic_world.hpp"

using namespace lightnav_localization;

TEST(GlobalInitializationField, ComputesEuclideanGridDistance) {
  const PointList points{Eigen::Vector3f(1.0f, 1.0f, 0.0f)};
  const GlobalInitializationLikelihoodField field(points, {0.0, 0.0}, {3.0, 3.0}, 1.0, 16);
  EXPECT_EQ(field.cell_count(), 16u);
  EXPECT_DOUBLE_EQ(field.distance({1.0, 1.0}), 0.0);
  EXPECT_DOUBLE_EQ(field.distance({3.0, 2.0}), std::sqrt(5.0));
  EXPECT_TRUE(std::isinf(field.distance({-1.0, 0.0})));
  EXPECT_THROW(GlobalInitializationLikelihoodField(points, {0.0, 0.0}, {3.0, 3.0}, 1.0, 15),
               std::length_error);
}

TEST(GlobalInitializationPeaks, RejectsCompetitorBelowFifthCoarseRank) {
  std::vector<GlobalInitializationPeak> peaks(7);
  for (std::size_t index = 0; index < peaks.size(); ++index) {
    peaks[index].position.x() = 2.0 * static_cast<double>(index);
    peaks[index].verification_score = 0.1;
  }
  peaks[0].verification_score = 0.95;
  // Limiting verification to the first five candidates would miss this competing peak.
  peaks[6].verification_score = 0.94;
  const auto decision = decide_global_initialization_peaks(peaks, {});
  EXPECT_FALSE(decision.accepted);
  EXPECT_EQ(decision.error, "ambiguous");
}

TEST(GlobalInitializationPeaks, DegenerateSecondPeakStillMakesResultAmbiguous) {
  std::vector<GlobalInitializationPeak> peaks(2);
  peaks[0].verification_score = 0.95;
  peaks[1].position.x() = 2.0;
  peaks[1].verification_score = 0.94;
  peaks[1].degenerate = true;
  const auto decision = decide_global_initialization_peaks(peaks, {});
  EXPECT_FALSE(decision.accepted);
  EXPECT_EQ(decision.error, "ambiguous");
  EXPECT_DOUBLE_EQ(decision.second_score, 0.94);
}

TEST(GlobalInitializationPeaks, RejectsEleventhNearbyCoarseScore) {
  std::vector<GlobalInitializationPeak> peaks(11);
  for (auto& peak : peaks) peak.coarse_score = 0.9;
  peaks.back().coarse_score = 0.81;
  EXPECT_TRUE(has_too_many_global_initialization_peaks(peaks, {}));
  peaks.back().coarse_score = 0.79;
  EXPECT_FALSE(has_too_many_global_initialization_peaks(peaks, {}));
}

TEST(GlobalInitializationPeaks, RejectsUnsortedCoarseScores) {
  std::vector<GlobalInitializationPeak> peaks(2);
  peaks[0].coarse_score = 0.1;
  peaks[1].coarse_score = 0.9;
  EXPECT_THROW(has_too_many_global_initialization_peaks(peaks, {}), std::invalid_argument);
}

TEST(GlobalInitializationPeaks, IgnoresSamePeakAfterRefinement) {
  std::vector<GlobalInitializationPeak> peaks(2);
  peaks[0].verification_score = 0.95;
  peaks[1].verification_score = 0.94;
  peaks[1].position.x() = 0.1;
  EXPECT_TRUE(decide_global_initialization_peaks(peaks, {}).accepted);
}

TEST(GlobalInitializationSearch, CancelsBeforeStarting) {
  std::atomic<bool> abort{true};
  const auto result = search_global_initialization({}, {}, {}, abort);
  EXPECT_EQ(result.decision.error, "cancelled");
}

TEST(GlobalInitializationSearch, StopsAtTimeBudget) {
  std::atomic<bool> abort{false};
  GlobalInitializationSearchParameters parameters;
  parameters.time_budget_s = 0.0;
  const auto result = search_global_initialization({}, {}, parameters, abort);
  EXPECT_EQ(result.decision.error, "time_budget");
}

TEST(GlobalInitializationSearch, RejectsInsufficientScan) {
  std::atomic<bool> abort{false};
  const auto result = search_global_initialization({}, {}, {}, abort);
  EXPECT_EQ(result.decision.error, "not_enough_scan_points");
}

TEST(GlobalInitializationSearch, RejectsSymmetricRoomThroughSearch) {
  using lightnav_localization::testing::SyntheticWorld;
  SyntheticWorld world({{0.0, 0.0}, {6.0, 0.0}});
  const PointList map = world.sample_map(0.1);
  Eigen::Isometry3d sensor_pose = Eigen::Isometry3d::Identity();
  sensor_pose.translation() = Eigen::Vector3d(1.0, 0.0, 1.0);
  const auto timed = world.synthesize_scan([&](double) { return sensor_pose; }, 0.0, 0.1, true, 360, 8);
  PointList scan;
  for (const auto& point : timed) scan.push_back(point.position + Eigen::Vector3f(0.0f, 0.0f, 1.0f));
  GlobalInitializationSearchParameters parameters;
  parameters.minimum = {0.0, -2.0};
  parameters.maximum = {6.0, 2.0};
  parameters.maximum_range_m = 8.0;
  std::atomic<bool> abort{false};
  const auto result = search_global_initialization(map, scan, parameters, abort);
  EXPECT_FALSE(result.decision.accepted);
  EXPECT_EQ(result.decision.error, "ambiguous");
  EXPECT_GE(result.decision.best_score, parameters.peaks.score_min);
  EXPECT_GT(result.decision.second_score, 0.0);
  EXPECT_LT(result.decision.best_score - result.decision.second_score, parameters.peaks.margin_min);
}

TEST(GlobalInitializationSearch, RejectsWhenCandidateBudgetExpires) {
  PointList map;
  PointList scan;
  for (int index = 0; index < 30; ++index) {
    const float x = static_cast<float>(index) * 0.1f;
    map.emplace_back(x, 0.0f, 0.0f);
    map.emplace_back(x, 0.0f, 1.0f);
    scan.emplace_back(x + 0.1f, 1.0f, 1.0f);
  }
  GlobalInitializationSearchParameters parameters;
  parameters.minimum = {1.0, 1.0};
  parameters.maximum = {1.5, 1.5};
  parameters.maximum_range_m = 3.0;
  parameters.minimum_scan_points = 1;
  parameters.candidate_budget = 1;
  std::atomic<bool> abort{false};
  EXPECT_EQ(search_global_initialization(map, scan, parameters, abort).decision.error, "time_budget");
}

TEST(GlobalInitializationSearch, RejectsLikelihoodFieldAboveCellLimit) {
  PointList scan;
  for (int index = 0; index < 36; ++index) {
    const double angle = index * M_PI / 18.0;
    scan.emplace_back(std::cos(angle), std::sin(angle), 1.0f);
  }
  GlobalInitializationSearchParameters parameters;
  parameters.minimum = {-1.0, -1.0};
  parameters.maximum = {1.0, 1.0};
  parameters.maximum_range_m = 2.0;
  parameters.maximum_grid_cells = 100;
  std::atomic<bool> abort{false};
  const auto result = search_global_initialization({}, scan, parameters, abort);
  EXPECT_EQ(result.decision.error, "map_too_large");
  EXPECT_EQ(result.evaluated_candidates, 0u);
}

TEST(GlobalInitializationSearch, WindowBoundsFieldAndArea) {
  PointList scan;
  for (int index = 0; index < 36; ++index) {
    const double angle = index * M_PI / 18.0;
    scan.emplace_back(std::cos(angle), std::sin(angle), 1.0f);
  }
  GlobalInitializationSearchParameters parameters;
  parameters.minimum = {-1000.0, -1000.0};
  parameters.maximum = {1000.0, 1000.0};
  parameters.maximum_range_m = 2.0;
  GlobalInitializationGnssWindow window;
  window.antenna_position = {0.0, 0.0};
  window.radius_m = 1.0;
  parameters.gnss_window = window;
  std::atomic<bool> abort{false};
  const auto large_box = search_global_initialization({}, scan, parameters, abort);
  parameters.minimum = {-1.0, -1.0};
  parameters.maximum = {1.0, 1.0};
  const auto small_box = search_global_initialization({}, scan, parameters, abort);
  EXPECT_GT(large_box.field_cells, 0u);
  EXPECT_EQ(large_box.field_cells, small_box.field_cells);
  parameters.maximum_area_m2 = 3.0;
  EXPECT_EQ(search_global_initialization({}, scan, parameters, abort).decision.error, "map_too_large");
}

TEST(GlobalInitializationSearch, SkipsCoarsePositionsOutsideBoxAfterLeverArmCorrection) {
  const PointList map{Eigen::Vector3f(1.0f, 0.0f, 0.0f), Eigen::Vector3f(1.0f, 0.0f, 1.0f)};
  PointList scan;
  for (int index = 0; index < 36; ++index) {
    const double angle = index * M_PI / 18.0;
    scan.emplace_back(std::cos(angle), std::sin(angle), 1.0f);
  }
  GlobalInitializationSearchParameters parameters;
  parameters.minimum = {0.0, 0.0};
  parameters.maximum = {0.1, 0.1};
  parameters.maximum_range_m = 2.0;
  parameters.coarse_yaw_step_rad = 2.0 * M_PI;
  GlobalInitializationGnssWindow window;
  window.antenna_position = {0.0, 0.0};
  window.radius_m = 0.1;
  window.lever_arm = {1.0, 0.0};
  parameters.gnss_window = window;
  std::atomic<bool> abort{false};
  const auto corrected = search_global_initialization(map, scan, parameters, abort);
  EXPECT_EQ(corrected.evaluated_candidates, 0u);
  parameters.gnss_window->lever_arm.setZero();
  const auto control = search_global_initialization(map, scan, parameters, abort);
  EXPECT_GT(control.evaluated_candidates, 0u);
}

TEST(GlobalInitializationSearch, AcceptsFivePosesInAsymmetricRoom) {
  using lightnav_localization::testing::SyntheticWorld;
  SyntheticWorld world({{0.0, 0.0}, {9.0, 0.0}, {9.0, 6.0}});
  world.add_box({2.0, 0.5}, {2.8, 1.5});
  world.add_box({5.0, -1.7}, {5.5, -0.4});
  world.add_box({8.0, 3.4}, {8.5, 4.2});
  const PointList map = world.sample_map(0.15);
  // A 0.5 m sensor height exposes the floor and constrains vertical translation.
  const std::vector<Eigen::Vector3d> poses{{1.0, 0.0, 0.0}, {3.0, 0.0, M_PI / 2.0},
      {5.0, 0.0, M_PI}, {9.0, 3.0, -3.0 * M_PI / 4.0}, {9.0, 5.0, M_PI / 4.0}};
  GlobalInitializationSearchParameters parameters;
  parameters.minimum = {0.0, -1.5};
  parameters.maximum = {10.5, 6.0};
  parameters.maximum_range_m = 12.0;
  std::atomic<bool> abort{false};
  for (const auto& truth : poses) {
    SCOPED_TRACE(truth.transpose());
    Eigen::Isometry3d sensor_pose = Eigen::Isometry3d::Identity();
    sensor_pose.translation() = Eigen::Vector3d(truth.x(), truth.y(), 0.5);
    sensor_pose.linear() = Eigen::AngleAxisd(truth.z(), Eigen::Vector3d::UnitZ()).toRotationMatrix();
    const auto timed = world.synthesize_scan([&](double) { return sensor_pose; }, 0.0, 0.1, true, 360, 8);
    PointList scan;
    for (const auto& point : timed) scan.push_back(point.position + Eigen::Vector3f(0.0f, 0.0f, 0.5f));
    const auto result = search_global_initialization(map, scan, parameters, abort);
    ASSERT_TRUE(result.decision.accepted) << result.decision.error << " best="
        << result.decision.best_score << " second=" << result.decision.second_score;
    EXPECT_LE((result.pose.translation().head<2>() - truth.head<2>()).norm(), 0.1);
    const double yaw = std::atan2(result.pose.linear()(1, 0), result.pose.linear()(0, 0));
    EXPECT_LE(std::abs(std::remainder(yaw - truth.z(), 2.0 * M_PI)), M_PI / 180.0);
  }
}

TEST(GlobalInitializationSearch, RejectsInvalidParametersSeparatelyFromMapSize) {
  GlobalInitializationSearchParameters parameters;
  std::atomic<bool> abort{false};
  parameters.coarse_translation_step_m = 0.0;
  EXPECT_EQ(search_global_initialization({}, {}, parameters, abort).decision.error, "invalid_parameters");
  parameters.coarse_translation_step_m = 0.5;
  parameters.likelihood_sigma_m = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(search_global_initialization({}, {}, parameters, abort).decision.error, "invalid_parameters");
  parameters.likelihood_sigma_m = 0.3;
  parameters.maximum.x() = -1.0;
  EXPECT_EQ(search_global_initialization({}, {}, parameters, abort).decision.error, "invalid_parameters");
}

TEST(GlobalInitializationSearch, ReportsCandidateBudgetAsTimeBudget) {
  GlobalInitializationSearchParameters parameters;
  parameters.candidate_budget = 0;
  std::atomic<bool> abort{false};
  EXPECT_EQ(search_global_initialization({}, {}, parameters, abort).decision.error, "time_budget");
}

TEST(GlobalInitializationSearch, G3RejectsSquareRoom) {
  using lightnav_localization::testing::SyntheticWorld;
  SyntheticWorld world({{0.0, 0.0}, {4.0, 0.0}});
  const auto map = world.sample_map(0.15);
  Eigen::Isometry3d sensor = Eigen::Isometry3d::Identity();
  sensor.translation() = Eigen::Vector3d(1.0, 0.4, 0.5);
  const auto timed = world.synthesize_scan([&](double) { return sensor; }, 0.0, 0.1, true, 360, 8);
  PointList scan;
  for (const auto& point : timed) scan.push_back(point.position + Eigen::Vector3f(0, 0, 0.5f));
  GlobalInitializationSearchParameters parameters;
  parameters.minimum = {0.0, -2.0};
  parameters.maximum = {4.0, 2.0};
  parameters.maximum_range_m = 8.0;
  std::atomic<bool> abort{false};
  const auto result = search_global_initialization(map, scan, parameters, abort);
  EXPECT_FALSE(result.decision.accepted);
  EXPECT_EQ(result.decision.error, "ambiguous");
}

TEST(GlobalInitializationSearch, G5RejectsLongStraightCorridor) {
  using lightnav_localization::testing::SyntheticWorld;
  SyntheticWorld world({{0.0, 0.0}, {30.0, 0.0}}, false);
  const auto map = world.sample_map(0.15);
  Eigen::Isometry3d sensor = Eigen::Isometry3d::Identity();
  sensor.translation() = Eigen::Vector3d(15.0, 0.0, 0.5);
  const auto timed = world.synthesize_scan([&](double) { return sensor; }, 0.0, 0.1, true, 360, 8);
  PointList scan;
  for (const auto& point : timed) scan.push_back(point.position + Eigen::Vector3f(0, 0, 0.5f));
  GlobalInitializationSearchParameters parameters;
  parameters.minimum = {5.0, -1.0};
  parameters.maximum = {25.0, 1.0};
  parameters.maximum_range_m = 8.0;
  std::atomic<bool> abort{false};
  const auto result = search_global_initialization(map, scan, parameters, abort);
  EXPECT_FALSE(result.decision.accepted);
  EXPECT_TRUE(result.decision.error == "degenerate" || result.decision.error == "ambiguous")
      << result.decision.error;
}

TEST(GlobalInitializationSearch, G13MovedFurnitureNeverAcceptsWrongPose) {
  using lightnav_localization::testing::SyntheticWorld;
  SyntheticWorld map_world({{0.0, 0.0}, {9.0, 0.0}, {9.0, 6.0}});
  SyntheticWorld scan_world({{0.0, 0.0}, {9.0, 0.0}, {9.0, 6.0}});
  for (auto* world : {&map_world, &scan_world}) {
    world->add_box({5.0, -1.7}, {5.5, -0.4});
    world->add_box({8.0, 3.4}, {8.5, 4.2});
  }
  map_world.add_box({2.0, 0.5}, {2.8, 1.5});
  scan_world.add_box({3.0, 0.5}, {3.8, 1.5});
  const auto map = map_world.sample_map(0.15);
  Eigen::Isometry3d sensor = Eigen::Isometry3d::Identity();
  sensor.translation() = Eigen::Vector3d(1.0, 0.0, 0.5);
  const auto timed = scan_world.synthesize_scan([&](double) { return sensor; }, 0.0, 0.1, true, 360, 8);
  PointList scan;
  for (const auto& point : timed) scan.push_back(point.position + Eigen::Vector3f(0, 0, 0.5f));
  GlobalInitializationSearchParameters parameters;
  parameters.minimum = {0.0, -1.5};
  parameters.maximum = {10.5, 6.0};
  parameters.maximum_range_m = 12.0;
  std::atomic<bool> abort{false};
  const auto result = search_global_initialization(map, scan, parameters, abort);
  if (result.decision.accepted) {
    EXPECT_LE((result.pose.translation().head<2>() - Eigen::Vector2d(1.0, 0.0)).norm(), 0.1);
    EXPECT_LE(std::abs(std::atan2(result.pose.linear()(1, 0), result.pose.linear()(0, 0))), M_PI / 180.0);
  } else {
    RecordProperty("rejection_reason", result.decision.error);
    std::cout << "G13 rejection reason: " << result.decision.error << std::endl;
    EXPECT_FALSE(result.decision.error.empty());
  }
}

TEST(GlobalInitializationSearch, G4RejectsTwoIdenticalRooms) {
  using lightnav_localization::testing::SyntheticWorld;
  SyntheticWorld world({{0.0, 0.0}, {5.0, 0.0}});
  world.add_box({1.0, 0.5}, {1.7, 1.4});
  PointList map = world.sample_map(0.15);
  const PointList first_room = map;
  for (const auto& point : first_room) map.push_back(point + Eigen::Vector3f(8.0f, 0.0f, 0.0f));
  Eigen::Isometry3d sensor = Eigen::Isometry3d::Identity();
  sensor.translation() = Eigen::Vector3d(2.5, 0.0, 0.5);
  const auto timed = world.synthesize_scan([&](double) { return sensor; }, 0.0, 0.1, true, 360, 8);
  PointList scan;
  for (const auto& point : timed) scan.push_back(point.position + Eigen::Vector3f(0, 0, 0.5f));
  GlobalInitializationSearchParameters parameters;
  parameters.minimum = {0.0, -2.0};
  parameters.maximum = {13.0, 2.0};
  parameters.maximum_range_m = 6.0;
  std::atomic<bool> abort{false};
  const auto result = search_global_initialization(map, scan, parameters, abort);
  EXPECT_FALSE(result.decision.accepted);
  EXPECT_EQ(result.decision.error, "ambiguous");
}

TEST(GlobalInitializationSearch, G6WindowLimitsSearchOnLargeMap) {
  using lightnav_localization::testing::SyntheticWorld;
  SyntheticWorld world({{0.0, 0.0}, {9.0, 0.0}, {9.0, 6.0}});
  world.add_box({2.0, 0.5}, {2.8, 1.5});
  world.add_box({5.0, -1.7}, {5.5, -0.4});
  world.add_box({8.0, 3.4}, {8.5, 4.2});
  PointList map = world.sample_map(0.15);
  const PointList room = map;
  for (const auto& point : room) map.push_back(point + Eigen::Vector3f(48, 36, 0));
  Eigen::Isometry3d sensor = Eigen::Isometry3d::Identity();
  sensor.translation() = Eigen::Vector3d(1, 0, 0.5);
  const auto timed = world.synthesize_scan([&](double) { return sensor; }, 0.0, 0.1, true, 360, 8);
  PointList scan;
  for (const auto& point : timed) scan.push_back(point.position + Eigen::Vector3f(0, 0, 0.5f));
  GlobalInitializationSearchParameters parameters;
  parameters.minimum = {0, -2};
  parameters.maximum = {58.5, 42};
  parameters.maximum_range_m = 12;
  std::atomic<bool> abort{false};
  const auto whole = search_global_initialization(map, scan, parameters, abort);
  EXPECT_EQ(whole.decision.error, "map_too_large");
  EXPECT_EQ(whole.evaluated_candidates, 0u);
  parameters.minimum = {0, -1.5};
  parameters.maximum = {10.5, 6};
  const auto window = search_global_initialization(map, scan, parameters, abort);
  ASSERT_TRUE(window.decision.accepted) << window.decision.error;
  EXPECT_LE((window.pose.translation().head<2>() - Eigen::Vector2d(1, 0)).norm(), 0.1);
}

TEST(GlobalInitializationSearch, G8CorrectsAntennaLeverArm) {
  using lightnav_localization::testing::SyntheticWorld;
  SyntheticWorld world({{0.0, 0.0}, {9.0, 0.0}, {9.0, 6.0}});
  world.add_box({2.0, 0.5}, {2.8, 1.5});
  world.add_box({5.0, -1.7}, {5.5, -0.4});
  world.add_box({8.0, 3.4}, {8.5, 4.2});
  const auto map = world.sample_map(0.15);
  Eigen::Isometry3d sensor = Eigen::Isometry3d::Identity();
  sensor.translation() = Eigen::Vector3d(1.0, 0.0, 0.5);
  const auto timed = world.synthesize_scan([&](double) { return sensor; }, 0.0, 0.1, true, 360, 8);
  PointList scan;
  for (const auto& point : timed) scan.push_back(point.position + Eigen::Vector3f(0, 0, 0.5f));
  GlobalInitializationSearchParameters parameters;
  parameters.minimum = {0.0, -1.5};
  parameters.maximum = {10.5, 6.0};
  parameters.maximum_range_m = 12.0;
  GlobalInitializationGnssWindow window;
  window.antenna_position = {1.3, 0.2};
  window.lever_arm = {0.3, 0.2};
  window.radius_m = 0.25;
  parameters.gnss_window = window;
  std::atomic<bool> abort{false};
  const auto result = search_global_initialization(map, scan, parameters, abort);
  ASSERT_TRUE(result.decision.accepted) << result.decision.error;
  EXPECT_LE((result.pose.translation().head<2>() - Eigen::Vector2d(1.0, 0.0)).norm(), 0.1);
  parameters.gnss_window->lever_arm.setZero();
  EXPECT_FALSE(search_global_initialization(map, scan, parameters, abort).decision.accepted);
  parameters.gnss_window->lever_arm = -window.lever_arm;
  EXPECT_FALSE(search_global_initialization(map, scan, parameters, abort).decision.accepted);
}

TEST(GlobalInitializationSearch, G7RejectsDisplacedGnssWindow) {
  using lightnav_localization::testing::SyntheticWorld;
  SyntheticWorld world({{0.0, 0.0}, {9.0, 0.0}, {9.0, 6.0}});
  world.add_box({2.0, 0.5}, {2.8, 1.5});
  world.add_box({5.0, -1.7}, {5.5, -0.4});
  world.add_box({8.0, 3.4}, {8.5, 4.2});
  const auto map = world.sample_map(0.15);
  Eigen::Isometry3d sensor = Eigen::Isometry3d::Identity();
  sensor.translation() = Eigen::Vector3d(1.0, 0.0, 0.5);
  const auto timed = world.synthesize_scan([&](double) { return sensor; }, 0.0, 0.1, true, 360, 8);
  PointList scan;
  for (const auto& point : timed) scan.push_back(point.position + Eigen::Vector3f(0, 0, 0.5f));
  GlobalInitializationSearchParameters parameters;
  parameters.minimum = {0.0, -1.5};
  parameters.maximum = {32.0, 32.0};
  parameters.maximum_range_m = 12.0;
  GlobalInitializationGnssWindow window;
  window.antenna_position = {1.0, 0.0};
  window.radius_m = 0.5;
  parameters.gnss_window = window;
  std::atomic<bool> abort{false};
  const auto control = search_global_initialization(map, scan, parameters, abort);
  ASSERT_TRUE(control.decision.accepted) << control.decision.error;
  parameters.gnss_window->antenna_position = {31.0, 31.0};
  const auto result = search_global_initialization(map, scan, parameters, abort);
  EXPECT_FALSE(result.decision.accepted);
  EXPECT_TRUE(result.decision.error == "low_score" || result.decision.error == "outside_gnss_window")
      << result.decision.error;
}

TEST(GlobalInitializationSearch, G10MeasuresSearchOn2500SquareMetres) {
  using lightnav_localization::testing::SyntheticWorld;
  SyntheticWorld world({{0.0, 0.0}, {9.0, 0.0}, {9.0, 6.0}});
  world.add_box({2.0, 0.5}, {2.8, 1.5});
  world.add_box({5.0, -1.7}, {5.5, -0.4});
  world.add_box({8.0, 3.4}, {8.5, 4.2});
  const auto map = world.sample_map(0.15);
  Eigen::Isometry3d sensor = Eigen::Isometry3d::Identity();
  sensor.translation() = Eigen::Vector3d(1.0, 0.0, 0.5);
  const auto timed = world.synthesize_scan([&](double) { return sensor; }, 0.0, 0.1, true, 360, 8);
  PointList scan;
  for (const auto& point : timed) scan.push_back(point.position + Eigen::Vector3f(0, 0, 0.5f));
  GlobalInitializationSearchParameters parameters;
  parameters.minimum = {-20.0, -20.0};
  parameters.maximum = {30.0, 30.0};
  parameters.maximum_range_m = 12.0;
  std::atomic<bool> abort{false};
  const auto started = std::chrono::steady_clock::now();
  const auto result = search_global_initialization(map, scan, parameters, abort);
  const double total = std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
  std::cout << "G10 total=" << total << " field=" << result.field_seconds
            << " cells=" << result.field_cells << " coarse=" << result.coarse_seconds
            << " fine=" << result.fine_seconds << " verification=" << result.verification_seconds
            << " reason=" << result.decision.error << std::endl;
  for (double duration : result.target_seconds) std::cout << "G10 target=" << duration << std::endl;
  for (double duration : result.matching_seconds) std::cout << "G10 matching=" << duration << std::endl;
}

TEST(GlobalInitializationSearch, G11RejectsDuplicatedRoomWithDecoyBoxes) {
  using lightnav_localization::testing::SyntheticWorld;
  SyntheticWorld world({{0.0, 0.0}, {5.0, 0.0}});
  world.add_box({1.0, 0.5}, {1.7, 1.4});
  PointList map = world.sample_map(0.15);
  const PointList room = map;
  for (const auto& point : room) map.push_back(point + Eigen::Vector3f(8.0f, 0.0f, 0.0f));
  // Additional short obstacles create competing 2D features between the rooms.
  SyntheticWorld decoy({{16.0, 0.0}, {20.0, 0.0}});
  decoy.add_box({17.0, 0.5}, {17.7, 1.4});
  for (const auto& point : decoy.sample_map(0.15)) map.push_back(point);
  Eigen::Isometry3d sensor = Eigen::Isometry3d::Identity();
  sensor.translation() = Eigen::Vector3d(2.5, 0.0, 0.5);
  const auto timed = world.synthesize_scan([&](double) { return sensor; }, 0.0, 0.1, true, 360, 8);
  PointList scan;
  for (const auto& point : timed) scan.push_back(point.position + Eigen::Vector3f(0, 0, 0.5f));
  GlobalInitializationSearchParameters parameters;
  parameters.minimum = {0.0, -2.0};
  parameters.maximum = {20.0, 2.0};
  parameters.maximum_range_m = 6.0;
  std::atomic<bool> abort{false};
  const auto result = search_global_initialization(map, scan, parameters, abort);
  EXPECT_FALSE(result.decision.accepted);
  EXPECT_EQ(result.decision.error, "ambiguous");
}

TEST(GlobalInitializationSearch, G12RejectsRoomAbsentFromMap) {
  using lightnav_localization::testing::SyntheticWorld;
  SyntheticWorld map_world({{0.0, 0.0}, {5.0, 0.0}});
  SyntheticWorld scan_world({{0.0, 0.0}, {12.0, 0.0}}, false);
  const auto map = map_world.sample_map(0.15);
  Eigen::Isometry3d sensor = Eigen::Isometry3d::Identity();
  sensor.translation() = Eigen::Vector3d(2.5, 0.0, 0.5);
  const auto timed = scan_world.synthesize_scan([&](double) { return sensor; }, 0.0, 0.1, true, 360, 8);
  PointList scan;
  for (const auto& point : timed) scan.push_back(point.position + Eigen::Vector3f(0, 0, 0.5f));
  GlobalInitializationSearchParameters parameters;
  parameters.minimum = {0.0, -2.0};
  parameters.maximum = {5.0, 2.0};
  parameters.maximum_range_m = 8.0;
  std::atomic<bool> abort{false};
  const auto result = search_global_initialization(map, scan, parameters, abort);
  EXPECT_FALSE(result.decision.accepted);
  EXPECT_TRUE(result.decision.error == "low_score" || result.decision.error == "ambiguous")
      << result.decision.error;
}
