#include "lightnav_localization/pcd_io.hpp"

#include <charconv>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace lightnav_localization {
namespace {

constexpr std::size_t kBytesPerPoint = 12;       // x y z as float32
constexpr std::size_t kMinimumAsciiPointBytes = 6;  // "0 0 0\n"

struct PcdHeader {
  std::string fields;
  std::string sizes;
  std::string types;
  std::string counts;
  std::string data_kind;
  bool has_point_count = false;
  std::size_t point_count = 0;
  bool has_width = false;
  std::size_t width = 0;
  bool has_height = false;
  std::size_t height = 1;
};

[[noreturn]] void fail(const std::string& file_path, const std::string& reason) {
  throw std::runtime_error("PCD error (" + file_path + "): " + reason);
}

std::size_t parse_unsigned(const std::string& text, const std::string& key,
                           const std::string& file_path) {
  std::size_t value = 0;
  const char* end = text.data() + text.size();
  const auto [stop, error] = std::from_chars(text.data(), end, value);
  if (text.empty() || error != std::errc() || stop != end) {
    fail(file_path, key + " is not a non-negative integer: '" + text + "'");
  }
  return value;
}

PcdHeader read_header(std::istream& stream, const std::string& file_path) {
  PcdHeader header;
  std::string line;
  while (std::getline(stream, line)) {
    std::istringstream line_stream(line);
    std::string key;
    line_stream >> key;
    std::string rest;
    std::getline(line_stream >> std::ws, rest);
    if (key == "FIELDS") header.fields = rest;
    else if (key == "SIZE") header.sizes = rest;
    else if (key == "TYPE") header.types = rest;
    else if (key == "COUNT") header.counts = rest;
    else if (key == "WIDTH") { header.width = parse_unsigned(rest, key, file_path); header.has_width = true; }
    else if (key == "HEIGHT") { header.height = parse_unsigned(rest, key, file_path); header.has_height = true; }
    else if (key == "POINTS") { header.point_count = parse_unsigned(rest, key, file_path); header.has_point_count = true; }
    else if (key == "DATA") {
      header.data_kind = rest;
      break;
    }
  }
  if (header.data_kind.empty()) fail(file_path, "missing DATA line");
  if (header.data_kind == "binary_compressed") fail(file_path, "binary_compressed is not supported");
  if (header.data_kind != "ascii" && header.data_kind != "binary") {
    fail(file_path, "unknown DATA kind: " + header.data_kind);
  }
  if (header.fields != "x y z") fail(file_path, "unsupported fields: '" + header.fields + "'");
  if (header.sizes != "4 4 4" || header.types != "F F F") fail(file_path, "x y z must be float32");
  if (!header.counts.empty() && header.counts != "1 1 1") fail(file_path, "COUNT must be 1 1 1");
  if (!header.has_point_count) fail(file_path, "missing POINTS line");
  if (header.has_width && header.width * header.height != header.point_count) {
    fail(file_path, "WIDTH x HEIGHT does not equal POINTS");
  }
  return header;
}

std::size_t remaining_bytes(std::istream& stream) {
  const auto data_begin = stream.tellg();
  stream.seekg(0, std::ios::end);
  const auto data_end = stream.tellg();
  stream.seekg(data_begin);
  return static_cast<std::size_t>(data_end - data_begin);
}

void check_finite(const Eigen::Vector3f& point, const std::string& file_path) {
  if (!point.allFinite()) fail(file_path, "point contains NaN or Inf");
}

// Flushes and closes before checking, so that errors held back by the buffer are not lost.
void finish_write(std::ofstream& stream, const std::string& file_path) {
  stream.flush();
  stream.close();
  if (stream.fail()) fail(file_path, "write failed");
}

}  // namespace

void write_pcd(const std::string& file_path, const PointList& points, PcdEncoding encoding) {
  std::ofstream stream(file_path, std::ios::binary);
  if (!stream) fail(file_path, "cannot open for writing");
  stream << "# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\n"
            "TYPE F F F\nCOUNT 1 1 1\nWIDTH "
         << points.size() << "\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS " << points.size()
         << "\nDATA " << (encoding == PcdEncoding::kAscii ? "ascii" : "binary") << "\n";
  if (encoding == PcdEncoding::kAscii) {
    stream << std::setprecision(9);
    for (const auto& point : points) stream << point.x() << ' ' << point.y() << ' ' << point.z() << '\n';
  } else {
    for (const auto& point : points) {
      stream.write(reinterpret_cast<const char*>(point.data()), kBytesPerPoint);
    }
  }
  finish_write(stream, file_path);
}

PointList read_pcd(const std::string& file_path) {
  std::ifstream stream(file_path, std::ios::binary);
  if (!stream) fail(file_path, "cannot open for reading");
  const PcdHeader header = read_header(stream, file_path);
  const std::size_t data_bytes = remaining_bytes(stream);

  // Validate the declared size against the file before allocating anything.
  const bool is_binary = header.data_kind == "binary";
  if (is_binary && header.point_count > data_bytes / kBytesPerPoint) fail(file_path, "POINTS exceeds the file size");
  if (is_binary && data_bytes != header.point_count * kBytesPerPoint) fail(file_path, "binary data size does not match POINTS");
  if (!is_binary && header.point_count > data_bytes / kMinimumAsciiPointBytes) fail(file_path, "POINTS exceeds the file size");

  PointList points(header.point_count);
  for (auto& point : points) {
    if (is_binary) {
      if (!stream.read(reinterpret_cast<char*>(point.data()), kBytesPerPoint)) fail(file_path, "truncated binary data");
    } else if (!(stream >> point.x() >> point.y() >> point.z())) {
      fail(file_path, "truncated or non-numeric ASCII data");
    }
    check_finite(point, file_path);
  }
  if (!is_binary) {
    std::string extra;
    if (stream >> extra) fail(file_path, "more ASCII data than POINTS");
  }
  return points;
}

}  // namespace lightnav_localization
