#include "lightnav_localization/pcd_io.hpp"

#include <cstdint>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace lightnav_localization {
namespace {

constexpr int kBytesPerPoint = 12;  // x y z as float32

struct PcdHeader {
  std::string fields;
  std::string sizes;
  std::string types;
  std::string data_kind;
  std::size_t point_count = 0;
};

[[noreturn]] void fail(const std::string& file_path, const std::string& reason) {
  throw std::runtime_error("PCD error (" + file_path + "): " + reason);
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
    else if (key == "POINTS") header.point_count = std::stoull(rest);
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
  return header;
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
    for (const auto& point : points) stream.write(reinterpret_cast<const char*>(point.data()), kBytesPerPoint);
  }
  if (!stream) fail(file_path, "write failed");
}

PointList read_pcd(const std::string& file_path) {
  std::ifstream stream(file_path, std::ios::binary);
  if (!stream) fail(file_path, "cannot open for reading");
  const PcdHeader header = read_header(stream, file_path);
  PointList points(header.point_count);
  if (header.data_kind == "ascii") {
    for (auto& point : points) {
      if (!(stream >> point.x() >> point.y() >> point.z())) fail(file_path, "truncated ASCII data");
    }
  } else {
    for (auto& point : points) {
      if (!stream.read(reinterpret_cast<char*>(point.data()), kBytesPerPoint)) {
        fail(file_path, "truncated binary data");
      }
    }
  }
  return points;
}

}  // namespace lightnav_localization
