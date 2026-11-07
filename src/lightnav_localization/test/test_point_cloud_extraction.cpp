#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <vector>

#include "lightnav_localization/point_cloud_extraction.hpp"

using namespace lightnav_localization;

namespace {

void write_float(std::vector<std::uint8_t>& buffer, std::size_t offset, float value) {
  std::memcpy(buffer.data() + offset, &value, sizeof(float));
}

void write_double(std::vector<std::uint8_t>& buffer, std::size_t offset, double value) {
  std::memcpy(buffer.data() + offset, &value, sizeof(double));
}

PointCloudFieldLayout float_layout(std::uint32_t width, std::uint32_t height,
                                   std::uint32_t point_step, std::uint32_t row_step) {
  PointCloudFieldLayout layout;
  layout.width = width;
  layout.height = height;
  layout.point_step = point_step;
  layout.row_step = row_step;
  layout.x_offset = 0;
  layout.y_offset = 4;
  layout.z_offset = 8;
  layout.time_offset = 12;
  layout.x_datatype = 7;
  layout.y_datatype = 7;
  layout.z_datatype = 7;
  layout.time_datatype = 7;
  return layout;
}

}  // namespace

TEST(PointCloudExtraction, DropsNonFinitePointsInAnOrganizedCloud) {
  constexpr std::uint32_t kPointStep = 16;
  constexpr std::uint32_t kRowStep = 40;  // 2 points + 8 bytes of row padding
  std::vector<std::uint8_t> buffer(static_cast<std::size_t>(kRowStep) * 2, 0);
  write_float(buffer, 0 * kPointStep + 0, 1.0F);
  write_float(buffer, 0 * kPointStep + 4, 2.0F);
  write_float(buffer, 0 * kPointStep + 8, 3.0F);
  write_float(buffer, 0 * kPointStep + 12, 0.1F);
  write_float(buffer, 1 * kPointStep + 0, std::numeric_limits<float>::quiet_NaN());
  write_float(buffer, 1 * kPointStep + 4, 2.0F);
  write_float(buffer, 1 * kPointStep + 8, 3.0F);
  write_float(buffer, 1 * kPointStep + 12, 0.2F);
  write_float(buffer, kRowStep + 0 * kPointStep + 0, 4.0F);
  write_float(buffer, kRowStep + 0 * kPointStep + 4, 5.0F);
  write_float(buffer, kRowStep + 0 * kPointStep + 8, 6.0F);
  write_float(buffer, kRowStep + 0 * kPointStep + 12,
              std::numeric_limits<float>::infinity());
  write_float(buffer, kRowStep + 1 * kPointStep + 0,
              std::numeric_limits<float>::infinity());
  write_float(buffer, kRowStep + 1 * kPointStep + 4, 5.0F);
  write_float(buffer, kRowStep + 1 * kPointStep + 8, 6.0F);
  write_float(buffer, kRowStep + 1 * kPointStep + 12, 0.3F);

  const auto points = extract_timed_points(
      buffer.data(), buffer.size(), float_layout(2, 2, kPointStep, kRowStep));
  ASSERT_EQ(points.size(), 2U);
  EXPECT_FLOAT_EQ(points[0].position.x(), 1.0F);
  EXPECT_FLOAT_EQ(points[0].position.y(), 2.0F);
  EXPECT_FLOAT_EQ(points[0].position.z(), 3.0F);
  EXPECT_FLOAT_EQ(points[0].time_offset_sec, 0.1F);
  EXPECT_FLOAT_EQ(points[1].position.x(), 4.0F);
  EXPECT_FLOAT_EQ(points[1].time_offset_sec, 0.0F);  // non-finite time becomes 0
}

TEST(PointCloudExtraction, MissingTimeFieldDefaultsToZero) {
  constexpr std::uint32_t kPointStep = 12;
  std::vector<std::uint8_t> buffer(static_cast<std::size_t>(kPointStep) * 2, 0);
  write_float(buffer, 0, 1.0F);
  write_float(buffer, 4, 2.0F);
  write_float(buffer, 8, 3.0F);
  write_float(buffer, kPointStep + 0, 4.0F);
  write_float(buffer, kPointStep + 4, 5.0F);
  write_float(buffer, kPointStep + 8, 6.0F);
  PointCloudFieldLayout layout = float_layout(2, 1, kPointStep, kPointStep * 2);
  layout.time_offset = -1;
  layout.time_datatype = 0;
  const auto points = extract_timed_points(buffer.data(), buffer.size(), layout);
  ASSERT_EQ(points.size(), 2U);
  EXPECT_FLOAT_EQ(points[0].time_offset_sec, 0.0F);
  EXPECT_FLOAT_EQ(points[1].time_offset_sec, 0.0F);
}

TEST(PointCloudExtraction, ReadsFloat64Fields) {
  constexpr std::uint32_t kPointStep = 32;
  std::vector<std::uint8_t> buffer(kPointStep, 0);
  write_double(buffer, 0, 1.5);
  write_double(buffer, 8, 2.5);
  write_double(buffer, 16, 3.5);
  write_double(buffer, 24, 0.25);
  PointCloudFieldLayout layout = float_layout(1, 1, kPointStep, kPointStep);
  layout.x_datatype = 8;
  layout.y_datatype = 8;
  layout.z_datatype = 8;
  layout.time_datatype = 8;
  layout.time_offset = 24;
  const auto points = extract_timed_points(buffer.data(), buffer.size(), layout);
  ASSERT_EQ(points.size(), 1U);
  EXPECT_FLOAT_EQ(points[0].position.x(), 1.5F);
  EXPECT_FLOAT_EQ(points[0].time_offset_sec, 0.25F);
}

TEST(PointCloudExtraction, RejectsAnInvalidLayoutWithoutReadingOutOfBounds) {
  const std::vector<std::uint8_t> buffer(16, 0);
  PointCloudFieldLayout layout = float_layout(1, 1, 16, 16);
  layout.x_offset = 14;  // 4-byte float does not fit in the 16-byte point
  const auto points = extract_timed_points(buffer.data(), buffer.size(), layout);
  EXPECT_TRUE(points.empty());
}

TEST(PointCloudExtraction, RejectsWidthTimesPointStepOverflow) {
  const std::vector<std::uint8_t> buffer(64, 0);
  // 65536 * 65536 overflows uint32 to 0; the bounds must be checked in 64 bits.
  PointCloudFieldLayout layout = float_layout(65536, 1, 65536, 65536);
  const auto points = extract_timed_points(buffer.data(), buffer.size(), layout);
  EXPECT_TRUE(points.empty());
}

TEST(PointCloudExtraction, RejectsATruncatedLastRow) {
  constexpr std::uint32_t kPointStep = 16;
  constexpr std::uint32_t kRowStep = 32;
  // Two rows need 64 bytes; only the first row and half of the second are present.
  const std::vector<std::uint8_t> buffer(48, 0);
  const auto points = extract_timed_points(
      buffer.data(), buffer.size(), float_layout(2, 2, kPointStep, kRowStep));
  EXPECT_TRUE(points.empty());
}
