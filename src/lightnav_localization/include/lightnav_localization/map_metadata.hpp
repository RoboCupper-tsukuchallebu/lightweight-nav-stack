#pragma once

#include <optional>
#include <string>

namespace lightnav_localization {

// Which stage of Localizer::save_map() wrote the map. kUnknown: an older map.yaml without the key.
enum class MapSaveStage { kUnknown, kPreliminary, kOptimized, kFinal };

const char* to_text(MapSaveStage stage);

struct MapMetadata {
  bool has_gps_origin = false;
  double origin_latitude_deg = 0.0;
  double origin_longitude_deg = 0.0;
  double origin_altitude_m = 0.0;
  double voxel_size_m = 0.2;
  std::string created;
  int keyframe_count = 0;
  bool batch_optimized = false;
  MapSaveStage save_stage = MapSaveStage::kUnknown;  // not written when kUnknown
  // Number of loop closures of the final stage. nullopt: loop closing was not run (also for older maps);
  // 0: it ran and found none.
  std::optional<int> loop_closure_count;
};

void write_map_metadata(const std::string& file_path, const MapMetadata& metadata);

// Strict reader. A missing file, an unknown boolean spelling, trailing garbage in a number, an
// out-of-range value or a latitude/longitude outside [-90, 90] / [-180, 180] throws
// std::runtime_error.
MapMetadata read_map_metadata(const std::string& file_path);

// Same, except that a missing file yields default metadata (has_gps_origin = false), i.e. the map
// works without GPS. A file that exists but is malformed still throws.
MapMetadata read_map_metadata_or_no_origin(const std::string& file_path);

// The note a localization node reports (log WARN and LocalizationStatus.last_error) for a map that is only a
// preliminary save.
extern const char* const kPreliminaryMapNote;

// True when the file exists and says batch_optimized: false (a save cut short after its first stage).
// A missing file is false (no map.yaml is not a preliminary map); a malformed file throws like
// read_map_metadata.
bool is_preliminary_map(const std::string& file_path);

}  // namespace lightnav_localization
