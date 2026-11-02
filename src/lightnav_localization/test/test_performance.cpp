#include <gtest/gtest.h>

#include <chrono>
#include <random>

#include "lightnav_localization/deskew.hpp"
#include "lightnav_localization/downsample.hpp"
#include "lightnav_localization/matcher.hpp"
#include "synthetic_world.hpp"

using namespace lightnav_localization;
using namespace lightnav_localization::testing;

namespace {

// Limits are the issue's CI limits (3x the acceptance limits): deskew 45 ms, downsample 60 ms,
// scan-to-submap GICP 180 ms. Measured single-threaded.
constexpr double kDeskewLimitMs = 45.0;
constexpr double kDownsampleLimitMs = 60.0;
constexpr double kMatchLimitMs = 180.0;
constexpr int kPointCount = 69000;
constexpr int kScanPointCount = 5000;
constexpr int kSubmapPointCount = 50000;

template <typename Function>
double milliseconds_of(Function&& function) {
  const auto begin = std::chrono::steady_clock::now();
  function();
  return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - begin).count();
}

}  // namespace

TEST(Performance, deskew_downsample_and_match_stay_within_limits) {
  std::mt19937 generator(42);
  std::uniform_real_distribution<float> coordinate(-30.0F, 30.0F);
  std::uniform_real_distribution<float> offset(-0.1F, 0.0F);
  std::vector<TimedPoint> scan;
  for (int index = 0; index < kPointCount; ++index) {
    scan.push_back({{coordinate(generator), coordinate(generator), coordinate(generator) * 0.1F}, offset(generator)});
  }
  PoseBuffer odometry;
  for (double time = -0.2; time <= 0.2; time += 0.02) odometry.add(time, arc_pose(time, {0, 0}, 0.0, 0.6, 0.4));

  std::optional<PointList> corrected;
  const double deskew_ms = milliseconds_of(
      [&] { corrected = deskew_scan(scan, 0.0, odometry, Eigen::Isometry3d::Identity()); });
  ASSERT_TRUE(corrected.has_value());
  PointList downsampled;
  const double downsample_ms = milliseconds_of([&] { downsampled = downsample_voxel(*corrected, 0.5); });
  // Issue sizes: about 5k scan points against a submap of up to 50k points.
  const PointList source_scan(downsampled.begin(), downsampled.begin() + kScanPointCount);
  const PointList target_submap(downsampled.begin(), downsampled.begin() + kSubmapPointCount);
  MatchResult result;
  const double match_ms = milliseconds_of([&] {
    result = match_scan(target_submap, source_scan, Eigen::Isometry3d::Identity(), MatcherParameters{});
  });

  RecordProperty("deskew_ms", std::to_string(deskew_ms));
  RecordProperty("downsample_ms", std::to_string(downsample_ms));
  RecordProperty("match_ms", std::to_string(match_ms));
  std::printf("deskew %.1f ms, downsample %.1f ms (%zu pts), match %.1f ms\n", deskew_ms,
              downsample_ms, downsampled.size(), match_ms);
  EXPECT_LT(deskew_ms, kDeskewLimitMs);
  EXPECT_LT(downsample_ms, kDownsampleLimitMs);
  EXPECT_LT(match_ms, kMatchLimitMs);
}
