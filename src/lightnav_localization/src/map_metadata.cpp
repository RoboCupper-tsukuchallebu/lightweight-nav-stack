#include "lightnav_localization/map_metadata.hpp"

#include <cmath>
#include <filesystem>
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

double parse_double(const std::string& text, const std::string& key, const std::string& file_path) {
  std::size_t consumed = 0;
  double value = 0.0;
  try {
    value = std::stod(text, &consumed);
  } catch (const std::exception&) {  // invalid_argument and out_of_range
    throw std::runtime_error("map.yaml (" + file_path + ") bad number for " + key + ": " + text);
  }
  if (consumed != text.size() || !std::isfinite(value)) {
    throw std::runtime_error("map.yaml (" + file_path + ") bad number for " + key + ": " + text);
  }
  return value;
}

int parse_int(const std::string& text, const std::string& key, const std::string& file_path) {
  const double value = parse_double(text, key, file_path);
  if (value != std::floor(value) || value < 0.0 || value > 2147483647.0) {
    throw std::runtime_error("map.yaml (" + file_path + ") bad integer for " + key + ": " + text);
  }
  return static_cast<int>(value);
}

bool parse_bool(const std::string& text, const std::string& key, const std::string& file_path) {
  if (text == "true") return true;
  if (text == "false") return false;
  throw std::runtime_error("map.yaml (" + file_path + ") " + key + " must be true or false: " + text);
}

void check_range(double value, double limit, const std::string& key, const std::string& file_path) {
  if (std::abs(value) > limit) {
    throw std::runtime_error("map.yaml (" + file_path + ") " + key + " out of range");
  }
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
  stream.flush();
  stream.close();
  if (stream.fail()) throw std::runtime_error("map.yaml write failed: " + file_path);
}

MapMetadata read_map_metadata(const std::string& file_path) {
  constexpr double kMaxLatitudeDeg = 90.0;
  constexpr double kMaxLongitudeDeg = 180.0;
  const auto values = read_key_values(file_path);
  MapMetadata metadata;
  metadata.has_gps_origin = parse_bool(require(values, "has_gps_origin", file_path), "has_gps_origin", file_path);
  if (metadata.has_gps_origin) {
    metadata.origin_latitude_deg = parse_double(require(values, "origin_lat", file_path), "origin_lat", file_path);
    metadata.origin_longitude_deg = parse_double(require(values, "origin_lon", file_path), "origin_lon", file_path);
    metadata.origin_altitude_m = parse_double(require(values, "origin_alt", file_path), "origin_alt", file_path);
    check_range(metadata.origin_latitude_deg, kMaxLatitudeDeg, "origin_lat", file_path);
    check_range(metadata.origin_longitude_deg, kMaxLongitudeDeg, "origin_lon", file_path);
  }
  metadata.voxel_size_m = parse_double(require(values, "voxel_size_m", file_path), "voxel_size_m", file_path);
  metadata.created = require(values, "created", file_path);
  metadata.keyframe_count = parse_int(require(values, "keyframe_count", file_path), "keyframe_count", file_path);
  metadata.batch_optimized = parse_bool(require(values, "batch_optimized", file_path), "batch_optimized", file_path);
  return metadata;
}

MapMetadata read_map_metadata_or_no_origin(const std::string& file_path) {
  if (!std::filesystem::exists(file_path)) return MapMetadata{};
  return read_map_metadata(file_path);
}

}  // namespace lightnav_localization
