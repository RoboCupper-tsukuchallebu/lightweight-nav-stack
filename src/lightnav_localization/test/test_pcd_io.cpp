#include <gtest/gtest.h>

#include <cstdio>
#include <fstream>

#include "lightnav_localization/pcd_io.hpp"

using lightnav_localization::PcdEncoding;
using lightnav_localization::PointList;

namespace {

std::string temporary_path(const std::string& name) { return ::testing::TempDir() + name; }

PointList sample_points() { return {{1.5F, -2.25F, 3.0F}, {0.0F, 0.0F, 0.0F}, {-7.125F, 8.5F, 0.125F}}; }

void expect_equal(const PointList& expected, const PointList& actual) {
  ASSERT_EQ(expected.size(), actual.size());
  for (std::size_t index = 0; index < expected.size(); ++index) {
    EXPECT_TRUE(expected[index].isApprox(actual[index], 1e-6F));
  }
}

}  // namespace

TEST(PcdIo, round_trips_ascii) {
  const std::string path = temporary_path("ascii.pcd");
  lightnav_localization::write_pcd(path, sample_points(), PcdEncoding::kAscii);
  expect_equal(sample_points(), lightnav_localization::read_pcd(path));
}

TEST(PcdIo, round_trips_binary) {
  const std::string path = temporary_path("binary.pcd");
  lightnav_localization::write_pcd(path, sample_points(), PcdEncoding::kBinary);
  expect_equal(sample_points(), lightnav_localization::read_pcd(path));
}

TEST(PcdIo, rejects_binary_compressed) {
  const std::string path = temporary_path("compressed.pcd");
  std::ofstream(path) << "VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH 0\n"
                         "HEIGHT 1\nPOINTS 0\nDATA binary_compressed\n";
  EXPECT_THROW(lightnav_localization::read_pcd(path), std::runtime_error);
}

TEST(PcdIo, rejects_unknown_fields) {
  const std::string path = temporary_path("intensity.pcd");
  std::ofstream(path) << "VERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\n"
                         "COUNT 1 1 1 1\nWIDTH 0\nHEIGHT 1\nPOINTS 0\nDATA ascii\n";
  EXPECT_THROW(lightnav_localization::read_pcd(path), std::runtime_error);
}

TEST(PcdIo, rejects_missing_file) {
  EXPECT_THROW(lightnav_localization::read_pcd(temporary_path("does_not_exist.pcd")), std::runtime_error);
}
