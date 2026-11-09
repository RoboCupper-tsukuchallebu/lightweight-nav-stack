#include <gtest/gtest.h>

#include <limits>

#include "lightnav_localization/visualization.hpp"

using namespace lightnav_localization;

namespace {

constexpr double kVoxelSizeM = 0.5;
constexpr std::size_t kUnlimited = 1000;

Eigen::Isometry3d pose_at(double x, double y) {
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() = Eigen::Vector3d(x, y, 0.0);
  return pose;
}

}  // namespace

TEST(VoxelMapAccumulator, KeepsOnePointPerVoxel) {
  // Arrange
  VoxelMapAccumulator accumulator(kVoxelSizeM, kUnlimited);
  const PointList points = {{0.1F, 0.1F, 0.1F}, {0.2F, 0.3F, 0.4F}, {0.6F, 0.1F, 0.1F}};

  // Act
  accumulator.add(points);

  // Assert
  ASSERT_EQ(accumulator.points().size(), 2U);
  EXPECT_FLOAT_EQ(accumulator.points()[0].x(), 0.1F);
  EXPECT_FLOAT_EQ(accumulator.points()[1].x(), 0.6F);
}

TEST(VoxelMapAccumulator, SeparatesNegativeCoordinatesFromPositiveOnes) {
  VoxelMapAccumulator accumulator(kVoxelSizeM, kUnlimited);

  accumulator.add({{-0.1F, 0.0F, 0.0F}, {0.1F, 0.0F, 0.0F}});

  EXPECT_EQ(accumulator.points().size(), 2U);
}

TEST(VoxelMapAccumulator, SkipsNonFinitePoints) {
  VoxelMapAccumulator accumulator(kVoxelSizeM, kUnlimited);
  const float not_a_number = std::numeric_limits<float>::quiet_NaN();

  accumulator.add({{not_a_number, 0.0F, 0.0F}, {1.0F, 1.0F, 1.0F}});

  EXPECT_EQ(accumulator.points().size(), 1U);
}

TEST(VoxelMapAccumulator, StopsGrowingAtMaxPoints) {
  VoxelMapAccumulator accumulator(kVoxelSizeM, 2);

  accumulator.add({{0.0F, 0.0F, 0.0F}, {1.0F, 0.0F, 0.0F}, {2.0F, 0.0F, 0.0F}});

  EXPECT_EQ(accumulator.points().size(), 2U);
}

TEST(VoxelMapAccumulator, VersionChangesOnlyWhenPointsChange) {
  VoxelMapAccumulator accumulator(kVoxelSizeM, kUnlimited);
  accumulator.add({{0.0F, 0.0F, 0.0F}});
  const std::uint64_t version_after_first_add = accumulator.version();

  accumulator.add({{0.1F, 0.1F, 0.1F}});
  EXPECT_EQ(accumulator.version(), version_after_first_add);

  accumulator.add({{5.0F, 0.0F, 0.0F}});
  EXPECT_GT(accumulator.version(), version_after_first_add);
}

TEST(PathRecorder, RecordsOnlyAfterTheMinimumStep) {
  PathRecorder recorder(0.1, kUnlimited);

  recorder.add(0.0, pose_at(0.0, 0.0));
  recorder.add(0.1, pose_at(0.05, 0.0));
  recorder.add(0.2, pose_at(0.12, 0.0));

  ASSERT_EQ(recorder.poses().size(), 2U);
  EXPECT_DOUBLE_EQ(recorder.poses().back().stamp_sec, 0.2);
}

TEST(PathRecorder, DropsTheOldestPoseBeyondMaxPoses) {
  PathRecorder recorder(0.1, 2);

  recorder.add(0.0, pose_at(0.0, 0.0));
  recorder.add(1.0, pose_at(1.0, 0.0));
  recorder.add(2.0, pose_at(2.0, 0.0));

  ASSERT_EQ(recorder.poses().size(), 2U);
  EXPECT_DOUBLE_EQ(recorder.poses().front().stamp_sec, 1.0);
}

TEST(PathRecorder, IgnoresNonFinitePoses) {
  PathRecorder recorder(0.1, kUnlimited);

  recorder.add(0.0, pose_at(std::numeric_limits<double>::quiet_NaN(), 0.0));

  EXPECT_TRUE(recorder.poses().empty());
}
