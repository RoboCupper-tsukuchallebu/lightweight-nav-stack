#include <gtest/gtest.h>

#include <chrono>
#include <cstdio>

#include "lightnav_localization/downsample.hpp"
#include "lightnav_localization/matcher.hpp"
#include "synthetic_world.hpp"

using namespace lightnav_localization;
using namespace lightnav_localization::testing;

namespace {
template <typename Function>
double milliseconds_of(Function&& function) {
  const auto begin = std::chrono::steady_clock::now();
  function();
  return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - begin).count();
}
// Issue table: map target (<= 1 M points) 3 s, map GICP (5 k vs <= 1 M) 100 ms; CI allows 3x.
constexpr double kBuildCiLimitMs = 9000.0;
constexpr double kMatchCiLimitMs = 300.0;
}  // namespace

TEST(PerformanceLarge, MapTargetOfAboutOneMillionPoints) {
  const SyntheticWorld world({{0, 0}, {500, 0}, {500, 300}, {300, 300}});
  const PointList map = world.sample_map(0.1);
  std::printf("map points: %zu\n", map.size());
  ASSERT_GE(map.size(), 900000u);
  ASSERT_LE(map.size(), 1100000u);

  PreparedTarget target;
  const double build_ms = milliseconds_of([&] { target = PreparedTarget(map); });
  const Eigen::Isometry3d pose = arc_pose(0.0, Eigen::Vector2d(250, 0), 0.0, 0.0, 0.0);
  const auto timed = world.synthesize_scan([&pose](double) { return pose; }, 0.0, 0.1, true, 900, 16);
  PointList scan;
  for (const TimedPoint& point : timed) scan.push_back(point.position);
  const PointList source = downsample_voxel(scan, 0.25);
  Eigen::Isometry3d prediction = pose;
  prediction.translation().y() += 0.2;
  MatchResult result;
  MatcherParameters parameters;
  const double match_ms = milliseconds_of([&] { result = match_scan(target, source, prediction, parameters); });
  std::printf("1M target build %.0f ms (CI limit %.0f), GICP %zu pts vs 1M %.1f ms (CI limit %.0f), inlier %.2f\n",
              build_ms, kBuildCiLimitMs, source.size(), match_ms, kMatchCiLimitMs, result.inlier_ratio);
  EXPECT_LT(build_ms, kBuildCiLimitMs);
  EXPECT_LT(match_ms, kMatchCiLimitMs);
  EXPECT_TRUE(result.converged);
}
