#include <gtest/gtest.h>

#include <chrono>
#include <cstdio>
#include <random>

#include "lightnav_localization/deskew.hpp"
#include "lightnav_localization/downsample.hpp"
#include "lightnav_localization/matcher.hpp"
#include "synthetic_world.hpp"

namespace {

using lightnav_localization::deskew_scan;
using lightnav_localization::downsample_voxel;
using lightnav_localization::match_scan;
using lightnav_localization::MatcherParameters;
using lightnav_localization::PointList;
using lightnav_localization::PoseBuffer;
using lightnav_localization::PreparedTarget;
using lightnav_localization::TimedPoint;
using lightnav_localization::testing::arc_pose;
using lightnav_localization::testing::SyntheticWorld;

// Acceptance limits of the issue table are 15 / 20 / 60 / 60 ms (TBB, 4 threads); CI allows 3x.
// This code is single-threaded, so the measured numbers are printed and the CI limits asserted.
constexpr double kDeskewCiLimitMs = 45.0;
constexpr double kDownsampleCiLimitMs = 60.0;
constexpr double kPrepareTargetCiLimitMs = 180.0;
constexpr double kMatchCiLimitMs = 180.0;
constexpr int kLidarPointCount = 69000;
constexpr int kMatchIterations = 20;
constexpr double kScanVoxelM = 0.5;
constexpr double kMapSpacingM = 0.2;
// A 4 m corridor yields few points at 0.5 m; a 0.10 m voxel gives the table's ~5k source points.
constexpr double kMatchScanVoxelM = 0.10;
constexpr std::size_t kSubmapMinPoints = 50000;  // measured: 52,020
constexpr std::size_t kSourceMinPoints = 5000;   // measured: 5,281

template <typename Function>
double milliseconds_of(Function&& function) {
  const auto begin = std::chrono::steady_clock::now();
  function();
  return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - begin).count();
}

}  // namespace

TEST(Performance, stages_stay_within_ci_limits) {
  std::mt19937 generator(42);
  std::uniform_real_distribution<float> coordinate(-30.0F, 30.0F);
  std::uniform_real_distribution<float> offset(-0.1F, 0.0F);
  std::vector<TimedPoint> lidar_points;
  for (int index = 0; index < kLidarPointCount; ++index) {
    lidar_points.push_back({{coordinate(generator), coordinate(generator), coordinate(generator) * 0.1F}, offset(generator)});
  }
  PoseBuffer odometry;
  for (double time = -0.2; time <= 0.2; time += 0.02) odometry.add(time, arc_pose(time, {0, 0}, 0.0, 0.6, 0.4));
  std::optional<PointList> corrected;
  const double deskew_ms = milliseconds_of(
      [&] { corrected = deskew_scan(lidar_points, 0.0, odometry, Eigen::Isometry3d::Identity()); });
  ASSERT_TRUE(corrected.has_value());
  PointList downsampled;
  const double downsample_ms = milliseconds_of([&] { downsampled = downsample_voxel(*corrected, kScanVoxelM); });

  // Real geometry for matching: a 100 m x 100 m L-shaped course; the scan is NOT a subset of the
  // target and the prediction is off by 0.2 m / 0.02 rad.
  const SyntheticWorld world({{0.0, 0.0}, {100.0, 0.0}, {100.0, 100.0}});
  const PointList submap = world.sample_map(kMapSpacingM);  // used as is, no downsampling
  ASSERT_GE(submap.size(), kSubmapMinPoints);
  const Eigen::Isometry3d true_pose = arc_pose(0.0, {85.0, 0.0}, 0.0, 0.0, 0.0);
  const auto scan = world.synthesize_scan([&](double) { return true_pose; }, 0.0, 0.1, true);
  PointList source;
  for (const TimedPoint& point : scan) source.push_back(point.position);
  source = downsample_voxel(source, kMatchScanVoxelM);
  ASSERT_GE(source.size(), kSourceMinPoints);
  Eigen::Isometry3d prediction = true_pose;
  prediction.translation() += Eigen::Vector3d(0.2, 0.0, 0.0);
  prediction.linear() = Eigen::AngleAxisd(0.02, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  MatcherParameters parameters;
  parameters.max_iterations = kMatchIterations;
  // The worst case in iterations is measured separately (see the PR): this scene converges in few
  // iterations, so `match` below is not the 20-iteration worst case.

  PreparedTarget target;
  const double prepare_ms = milliseconds_of([&] { target = PreparedTarget(submap); });
  lightnav_localization::MatchResult result;
  const double match_ms = milliseconds_of([&] { result = match_scan(target, source, prediction, parameters); });

  std::printf("deskew %.1f ms (69k pts), downsample %.1f ms (%zu pts), prepare_target %.1f ms (%zu pts), "
              "match %.1f ms (%zu source pts, %d iterations, inlier %.2f, converged %d)\n",
              deskew_ms, downsample_ms, downsampled.size(), prepare_ms, submap.size(), match_ms,
              source.size(), result.iterations, result.inlier_ratio, result.converged);
  EXPECT_GT(result.iterations, 0);
  EXPECT_LT(deskew_ms, kDeskewCiLimitMs);
  EXPECT_LT(downsample_ms, kDownsampleCiLimitMs);
  EXPECT_LT(prepare_ms, kPrepareTargetCiLimitMs);
  EXPECT_LT(match_ms, kMatchCiLimitMs);
}
