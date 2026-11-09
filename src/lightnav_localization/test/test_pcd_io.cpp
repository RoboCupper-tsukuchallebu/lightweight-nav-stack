#include <gtest/gtest.h>

#include <utility>
#include <vector>

#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <system_error>
#include <fstream>
#include <stdexcept>
#include <string>

#include "lightnav_localization/pcd_io.hpp"

using lightnav_localization::PcdEncoding;
using lightnav_localization::PointList;

namespace {

// One directory per test process: fixed names under /tmp collide when several users run the tests on one machine
// (a self-hosted CI runner could not overwrite /tmp/ascii.pcd left by another user, 2026-11-10).
const std::string& temporary_directory() {
  static const std::string directory = [] {
    std::string pattern = ::testing::TempDir() + "lightnav_pcd_io_XXXXXX";
    if (mkdtemp(pattern.data()) == nullptr) {
      throw std::runtime_error("cannot create a temporary directory from " + pattern);
    }
    // Removed when the test process exits, so repeated runs do not leave directories behind.
    static const std::filesystem::path created{pattern};
    std::atexit([] {
      std::error_code ignored;
      std::filesystem::remove_all(created, ignored);
    });
    return pattern + "/";
  }();
  return directory;
}

std::string temporary_path(const std::string& name) { return temporary_directory() + name; }

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

namespace {

void write_text(const std::string& path, const std::string& text) { std::ofstream(path, std::ios::binary) << text; }

const std::string kHeaderPrefix =
    "VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n";

}  // namespace

TEST(PcdIo, rejects_each_kind_of_broken_file) {
  const std::string path = temporary_path("broken.pcd");
  const std::vector<std::pair<std::string, std::string>> broken_files = {
      {"no POINTS", kHeaderPrefix + "WIDTH 1\nHEIGHT 1\nDATA ascii\n1 2 3\n"},
      {"non-numeric POINTS", kHeaderPrefix + "WIDTH 1\nHEIGHT 1\nPOINTS abc\nDATA ascii\n1 2 3\n"},
      {"negative POINTS", kHeaderPrefix + "WIDTH 1\nHEIGHT 1\nPOINTS -1\nDATA ascii\n1 2 3\n"},
      {"huge POINTS", kHeaderPrefix + "WIDTH 4000000000\nHEIGHT 1\nPOINTS 4000000000\nDATA binary\n"},
      {"huge POINTS ascii", kHeaderPrefix + "WIDTH 4000000000\nHEIGHT 1\nPOINTS 4000000000\nDATA ascii\n1 2 3\n"},
      {"POINTS smaller than data", kHeaderPrefix + "WIDTH 1\nHEIGHT 1\nPOINTS 1\nDATA ascii\n1 2 3\n4 5 6\n"},
      {"POINTS larger than data", kHeaderPrefix + "WIDTH 2\nHEIGHT 1\nPOINTS 2\nDATA ascii\n1 2 3\n"},
      {"bad COUNT", "VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 2\nWIDTH 1\nHEIGHT 1\nPOINTS 1\nDATA ascii\n1 2 3\n"},
      {"WIDTH x HEIGHT mismatch", kHeaderPrefix + "WIDTH 3\nHEIGHT 1\nPOINTS 1\nDATA ascii\n1 2 3\n"},
      {"NaN", kHeaderPrefix + "WIDTH 1\nHEIGHT 1\nPOINTS 1\nDATA ascii\nnan 2 3\n"},
      {"non-numeric value", kHeaderPrefix + "WIDTH 1\nHEIGHT 1\nPOINTS 1\nDATA ascii\n1 abc 3\n"},
  };
  for (const auto& [description, text] : broken_files) {
    write_text(path, text);
    EXPECT_THROW(lightnav_localization::read_pcd(path), std::runtime_error) << description;
  }
}

TEST(PcdIo, rejects_binary_size_mismatch_and_non_finite_values) {
  const std::string path = temporary_path("binary_broken.pcd");
  const std::string header = kHeaderPrefix + "WIDTH 1\nHEIGHT 1\nPOINTS 1\nDATA binary\n";
  write_text(path, header + std::string(8, '\0'));  // truncated: 8 of 12 bytes
  EXPECT_THROW(lightnav_localization::read_pcd(path), std::runtime_error);
  write_text(path, header + std::string(16, '\0'));  // trailing bytes
  EXPECT_THROW(lightnav_localization::read_pcd(path), std::runtime_error);
  write_text(path, header + std::string(4, '\xff') + std::string(8, '\0'));  // float 0xffffffff is NaN
  EXPECT_THROW(lightnav_localization::read_pcd(path), std::runtime_error);
}

TEST(PcdIo, write_to_an_unwritable_location_is_an_error) {
  EXPECT_THROW(lightnav_localization::write_pcd(temporary_path("no_such_directory/map.pcd"), sample_points(),
                                                PcdEncoding::kBinary),
               std::runtime_error);
}
