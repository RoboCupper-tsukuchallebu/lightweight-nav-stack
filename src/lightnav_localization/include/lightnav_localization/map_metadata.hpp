#pragma once

#include <string>

namespace lightnav_localization {

struct MapMetadata {
  bool has_gps_origin = false;
  double origin_latitude_deg = 0.0;
  double origin_longitude_deg = 0.0;
  double origin_altitude_m = 0.0;
  double voxel_size_m = 0.2;
  std::string created;
  int keyframe_count = 0;
  bool batch_optimized = false;
  int loop_closure_count = 0;
};

void write_map_metadata(const std::string& file_path, const MapMetadata& metadata);

// Strict reader. A missing file, an unknown boolean spelling, trailing garbage in a number, an
// out-of-range value or a latitude/longitude outside [-90, 90] / [-180, 180] throws
// std::runtime_error.
MapMetadata read_map_metadata(const std::string& file_path);

// Same, except that a missing file yields default metadata (has_gps_origin = false), i.e. the map
// works without GPS. A file that exists but is malformed still throws.
MapMetadata read_map_metadata_or_no_origin(const std::string& file_path);

}  // namespace lightnav_localization
