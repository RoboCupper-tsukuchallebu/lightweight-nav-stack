#include <gtest/gtest.h>

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
  const std::string path = ::testing::TempDir() + "map_with_origin.yaml";

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
  const std::string path = ::testing::TempDir() + "map_without_origin.yaml";

  lightnav_localization::write_map_metadata(path, metadata);

  EXPECT_FALSE(lightnav_localization::read_map_metadata(path).has_gps_origin);
}

TEST(MapMetadata, missing_file_is_an_error) {
  EXPECT_THROW(lightnav_localization::read_map_metadata(::testing::TempDir() + "absent.yaml"),
               std::runtime_error);
}
