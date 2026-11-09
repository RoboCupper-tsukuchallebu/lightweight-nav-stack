#include <gtest/gtest.h>

#include <cstdlib>
#include <stdexcept>
#include <string>

namespace {
// One directory per test process: fixed names under /tmp collide when several users run the tests on one machine
// (self-hosted CI, 2026-11-10).
std::string unique_temp_directory() {
  static const std::string directory = [] {
    std::string pattern = ::testing::TempDir() + "lightnav_map_metadata_XXXXXX";
    if (mkdtemp(pattern.data()) == nullptr) {
      throw std::runtime_error("cannot create a temporary directory from " + pattern);
    }
    return pattern + "/";
  }();
  return directory;
}
}  // namespace

#include <fstream>
#include <utility>
#include <vector>

#include "lightnav_localization/map_metadata.hpp"

using lightnav_localization::MapMetadata;

TEST(MapMetadata, round_trips_with_gps_origin) {
  MapMetadata metadata;
  metadata.has_gps_origin = true;
  metadata.origin_latitude_deg = 36.083412345;
  metadata.origin_longitude_deg = 140.076543210;
  metadata.origin_altitude_m = 25.5;
  metadata.created = "2026-11-02T00:00:00Z";
  metadata.keyframe_count = 1234;
  metadata.batch_optimized = true;
  const std::string path = unique_temp_directory() + "map_with_origin.yaml";

  lightnav_localization::write_map_metadata(path, metadata);
  const MapMetadata loaded = lightnav_localization::read_map_metadata(path);

  EXPECT_TRUE(loaded.has_gps_origin);
  EXPECT_DOUBLE_EQ(metadata.origin_latitude_deg, loaded.origin_latitude_deg);
  EXPECT_DOUBLE_EQ(metadata.origin_longitude_deg, loaded.origin_longitude_deg);
  EXPECT_DOUBLE_EQ(metadata.origin_altitude_m, loaded.origin_altitude_m);
  EXPECT_EQ(metadata.created, loaded.created);
  EXPECT_EQ(1234, loaded.keyframe_count);
  EXPECT_TRUE(loaded.batch_optimized);
}

TEST(MapMetadata, round_trips_without_gps_origin) {
  MapMetadata metadata;
  metadata.created = "2026-11-02T00:00:00Z";
  const std::string path = unique_temp_directory() + "map_without_origin.yaml";

  lightnav_localization::write_map_metadata(path, metadata);

  EXPECT_FALSE(lightnav_localization::read_map_metadata(path).has_gps_origin);
}

TEST(MapMetadata, missing_file_is_an_error) {
  EXPECT_THROW(lightnav_localization::read_map_metadata(unique_temp_directory() + "absent.yaml"),
               std::runtime_error);
}

namespace {

void write_yaml(const std::string& path, const std::string& origin_lat, const std::string& has_origin = "true",
                const std::string& keyframes = "3") {
  std::ofstream(path) << "has_gps_origin: " << has_origin << "\norigin_lat: " << origin_lat
                      << "\norigin_lon: 140.0\norigin_alt: 20.0\nvoxel_size_m: 0.2\ncreated: x\nkeyframe_count: "
                      << keyframes << "\nbatch_optimized: true\n";
}

}  // namespace

TEST(MapMetadata, missing_map_yaml_means_no_gps_origin) {
  const MapMetadata loaded = lightnav_localization::read_map_metadata_or_no_origin(unique_temp_directory() + "absent2.yaml");
  EXPECT_FALSE(loaded.has_gps_origin);
}

TEST(MapMetadata, existing_but_malformed_map_yaml_is_still_an_error_for_the_lenient_reader) {
  const std::string path = unique_temp_directory() + "malformed.yaml";
  write_yaml(path, "12abc");
  EXPECT_THROW(lightnav_localization::read_map_metadata_or_no_origin(path), std::runtime_error);
}

TEST(MapMetadata, rejects_invalid_values) {
  const std::string path = unique_temp_directory() + "invalid.yaml";
  const std::vector<std::pair<std::string, std::string>> bad_latitudes = {
      {"12abc", "trailing garbage"}, {"1e999", "out of range for double"}, {"91.0", "latitude range"},
      {"nan", "not finite"}};
  for (const auto& [latitude, description] : bad_latitudes) {
    write_yaml(path, latitude);
    EXPECT_THROW(lightnav_localization::read_map_metadata(path), std::runtime_error) << description;
  }
  write_yaml(path, "36.0", "True");
  EXPECT_THROW(lightnav_localization::read_map_metadata(path), std::runtime_error) << "bool spelling";
  write_yaml(path, "36.0", "true", "99999999999");
  EXPECT_THROW(lightnav_localization::read_map_metadata(path), std::runtime_error) << "int range";
}

TEST(MapMetadata, write_to_an_unwritable_location_is_an_error) {
  EXPECT_THROW(lightnav_localization::write_map_metadata(unique_temp_directory() + "no_such_directory/map.yaml", MapMetadata{}),
               std::runtime_error);
}
