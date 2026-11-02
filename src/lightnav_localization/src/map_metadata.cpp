#include "lightnav_localization/map_metadata.hpp"

#include <fstream>
#include <iomanip>
#include <map>
#include <stdexcept>

namespace lightnav_localization {
namespace {

// map.yaml is a flat "key: value" file that this package writes itself; no YAML library needed.
std::map<std::string, std::string> read_key_values(const std::string& file_path) {
  std::ifstream stream(file_path);
  if (!stream) throw std::runtime_error("map.yaml cannot be opened: " + file_path);
  std::map<std::string, std::string> values;
  std::string line;
  while (std::getline(stream, line)) {
    const auto separator = line.find(':');
    if (separator == std::string::npos || line.empty() || line[0] == '#') continue;
    const auto value_begin = line.find_first_not_of(' ', separator + 1);
    values[line.substr(0, separator)] =
        value_begin == std::string::npos ? "" : line.substr(value_begin);
  }
  return values;
}

const std::string& require(const std::map<std::string, std::string>& values, const std::string& key,
                           const std::string& file_path) {
  const auto found = values.find(key);
  if (found == values.end()) throw std::runtime_error("map.yaml (" + file_path + ") lacks key: " + key);
  return found->second;
}

}  // namespace

void write_map_metadata(const std::string& file_path, const MapMetadata& metadata) {
  std::ofstream stream(file_path);
  if (!stream) throw std::runtime_error("map.yaml cannot be opened for writing: " + file_path);
  stream << std::setprecision(12) << std::boolalpha;
  stream << "has_gps_origin: " << metadata.has_gps_origin << '\n';
  if (metadata.has_gps_origin) {
    stream << "origin_lat: " << metadata.origin_latitude_deg << '\n'
           << "origin_lon: " << metadata.origin_longitude_deg << '\n'
           << "origin_alt: " << metadata.origin_altitude_m << '\n';
  }
  stream << "voxel_size_m: " << metadata.voxel_size_m << '\n'
         << "created: " << metadata.created << '\n'
         << "keyframe_count: " << metadata.keyframe_count << '\n'
         << "batch_optimized: " << metadata.batch_optimized << '\n';
  if (!stream) throw std::runtime_error("map.yaml write failed: " + file_path);
}

MapMetadata read_map_metadata(const std::string& file_path) {
  const auto values = read_key_values(file_path);
  MapMetadata metadata;
  try {
    metadata.has_gps_origin = require(values, "has_gps_origin", file_path) == "true";
    if (metadata.has_gps_origin) {
      metadata.origin_latitude_deg = std::stod(require(values, "origin_lat", file_path));
      metadata.origin_longitude_deg = std::stod(require(values, "origin_lon", file_path));
      metadata.origin_altitude_m = std::stod(require(values, "origin_alt", file_path));
    }
    metadata.voxel_size_m = std::stod(require(values, "voxel_size_m", file_path));
    metadata.created = require(values, "created", file_path);
    metadata.keyframe_count = std::stoi(require(values, "keyframe_count", file_path));
    metadata.batch_optimized = require(values, "batch_optimized", file_path) == "true";
  } catch (const std::invalid_argument&) {
    throw std::runtime_error("map.yaml has a non-numeric value: " + file_path);
  }
  return metadata;
}

}  // namespace lightnav_localization
