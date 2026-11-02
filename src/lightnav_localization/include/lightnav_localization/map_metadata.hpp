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
};

void write_map_metadata(const std::string& file_path, const MapMetadata& metadata);

// A missing file is an error here; callers that allow "no map.yaml" must check existence first
// and treat the map as having no GPS origin. Throws std::runtime_error.
MapMetadata read_map_metadata(const std::string& file_path);

}  // namespace lightnav_localization
