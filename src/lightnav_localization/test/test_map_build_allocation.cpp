// How much memory build_map_points() requests, as a function of the keyframe count.
//
// The stop-time save of a 37 min mapping drive (1900 keyframes, 7 million points) did not finish within the
// stop limit and left no map (2026-11-25). build_map_points() reserved keyframe by keyframe, and every
// reserve copied all the points assembled so far: 15 s for that drive, growing with the square of the
// keyframe count.
//
// The requested bytes are counted instead of the time: they are the same on every machine and under any
// load. This file is a test binary of its own because it replaces the global operator new, which no other
// test should run with.

#include <gtest/gtest.h>

#include <atomic>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <new>
#include <utility>
#include <vector>

#include "lightnav_localization/map_builder.hpp"

namespace {

std::atomic<bool> is_counting_allocations{false};
std::atomic<std::size_t> allocated_byte_count{0};

}  // namespace

void* operator new(std::size_t size) {
  if (is_counting_allocations.load(std::memory_order_relaxed)) {
    allocated_byte_count.fetch_add(size, std::memory_order_relaxed);
  }
  void* const memory = std::malloc(size == 0 ? 1 : size);
  if (memory == nullptr) throw std::bad_alloc();
  return memory;
}

void operator delete(void* memory) noexcept { std::free(memory); }
void operator delete(void* memory, std::size_t /*size*/) noexcept { std::free(memory); }

namespace lightnav_localization {
namespace {

constexpr int kPointsPerKeyframe = 40;
constexpr double kMapVoxelSizeM = 0.2;

// Keyframes of a drive along a line. The points of a keyframe lie on a 0.5 m grid in front of the vehicle,
// so every one of them survives the 0.2 m voxel grid of the map.
std::vector<KeyframeCloud> make_keyframes_along_a_line(int keyframe_count,
                                                       std::vector<TrajectoryPoint>& trajectory) {
  std::vector<KeyframeCloud> keyframes;
  keyframes.reserve(static_cast<std::size_t>(keyframe_count));
  trajectory.clear();
  trajectory.reserve(static_cast<std::size_t>(keyframe_count));
  for (int keyframe_index = 0; keyframe_index < keyframe_count; ++keyframe_index) {
    KeyframeCloud keyframe;
    keyframe.node_index = keyframe_index;
    keyframe.stamp_sec = static_cast<double>(keyframe_index);
    keyframe.points_in_base.reserve(kPointsPerKeyframe);
    for (int point_index = 0; point_index < kPointsPerKeyframe; ++point_index) {
      keyframe.points_in_base.emplace_back(0.5F * static_cast<float>(point_index % 8),
                                           0.5F * static_cast<float>(point_index / 8), 1.0F);
    }
    keyframes.push_back(std::move(keyframe));
    TrajectoryPoint pose;
    pose.node_index = keyframe_index;
    pose.stamp_sec = static_cast<double>(keyframe_index);
    pose.map_to_base.translation() = Eigen::Vector3d(10.0 * keyframe_index, 0.0, 0.0);
    trajectory.push_back(pose);
  }
  return keyframes;
}

// Bytes requested through operator new while the map of `keyframe_count` keyframes is built.
std::size_t bytes_requested_to_build_map(int keyframe_count) {
  std::vector<TrajectoryPoint> trajectory;
  const std::vector<KeyframeCloud> keyframes = make_keyframes_along_a_line(keyframe_count, trajectory);
  allocated_byte_count.store(0);
  is_counting_allocations.store(true);
  const PointList map_points = build_map_points(keyframes, trajectory, kMapVoxelSizeM);
  is_counting_allocations.store(false);
  EXPECT_EQ(map_points.size(), static_cast<std::size_t>(keyframe_count) * kPointsPerKeyframe);
  return allocated_byte_count.load();
}

TEST(MapBuildAllocation, RequestedMemoryGrowsInProportionToTheKeyframeCount) {
  constexpr int kFewKeyframes = 250;
  constexpr int kManyKeyframes = 4 * kFewKeyframes;
  const std::size_t few_bytes = bytes_requested_to_build_map(kFewKeyframes);
  const std::size_t many_bytes = bytes_requested_to_build_map(kManyKeyframes);
  std::printf("[ MAP_BUILD_ALLOCATION ] keyframes=%d bytes=%zu keyframes=%d bytes=%zu ratio=%.2f\n",
              kFewKeyframes, few_bytes, kManyKeyframes, many_bytes,
              static_cast<double>(many_bytes) / static_cast<double>(few_bytes));
  ASSERT_GT(few_bytes, 0U);
  // Four times the keyframes: about 4 times the memory. Reserving keyframe by keyframe gave about 16 times.
  EXPECT_LT(many_bytes, 6 * few_bytes);
}

}  // namespace
}  // namespace lightnav_localization
