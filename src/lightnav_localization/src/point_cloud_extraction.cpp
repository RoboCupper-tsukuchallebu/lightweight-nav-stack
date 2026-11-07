#include "lightnav_localization/point_cloud_extraction.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace lightnav_localization {
namespace {

enum PointFieldDatatype : int { kFloat32 = 7, kFloat64 = 8 };

double read_float_field(const std::uint8_t* point, int offset, int datatype, bool is_bigendian) {
  const std::uint8_t* source = point + offset;
  if (datatype == kFloat32) {
    float value = 0.0F;
    if (is_bigendian) {
      std::uint8_t bytes[sizeof(float)];
      for (std::size_t index = 0; index < sizeof(float); ++index) {
        bytes[index] = source[sizeof(float) - 1 - index];
      }
      std::memcpy(&value, bytes, sizeof(float));
    } else {
      std::memcpy(&value, source, sizeof(float));
    }
    return static_cast<double>(value);
  }
  double value = 0.0;
  if (is_bigendian) {
    std::uint8_t bytes[sizeof(double)];
    for (std::size_t index = 0; index < sizeof(double); ++index) {
      bytes[index] = source[sizeof(double) - 1 - index];
    }
    std::memcpy(&value, bytes, sizeof(double));
  } else {
    std::memcpy(&value, source, sizeof(double));
  }
  return value;
}

std::size_t datatype_size(int datatype) {
  switch (datatype) {
    case kFloat32: return sizeof(float);
    case kFloat64: return sizeof(double);
    default: return 0;
  }
}

bool has_room(std::uint32_t point_step, int offset, int datatype) {
  if (offset < 0) return false;
  const std::size_t size = datatype_size(datatype);
  if (size == 0) return false;
  return static_cast<std::size_t>(offset) + size <= static_cast<std::size_t>(point_step);
}

}  // namespace

bool is_supported_float_datatype(int datatype) {
  return datatype == kFloat32 || datatype == kFloat64;
}

std::vector<TimedPoint> extract_timed_points(const std::uint8_t* data, std::size_t data_size,
                                             const PointCloudFieldLayout& layout) {
  std::vector<TimedPoint> points;
  if (data == nullptr || layout.width == 0 || layout.height == 0 || layout.point_step == 0 ||
      layout.row_step < layout.width * layout.point_step ||
      !is_supported_float_datatype(layout.x_datatype) ||
      !is_supported_float_datatype(layout.y_datatype) ||
      !is_supported_float_datatype(layout.z_datatype) ||
      !has_room(layout.point_step, layout.x_offset, layout.x_datatype) ||
      !has_room(layout.point_step, layout.y_offset, layout.y_datatype) ||
      !has_room(layout.point_step, layout.z_offset, layout.z_datatype)) {
    return points;
  }
  const bool has_time = layout.time_offset >= 0 &&
                        is_supported_float_datatype(layout.time_datatype) &&
                        has_room(layout.point_step, layout.time_offset, layout.time_datatype);
  const std::size_t usable_rows = std::min<std::size_t>(layout.height,
                                                        data_size / std::max(1U, layout.row_step));
  points.reserve(static_cast<std::size_t>(layout.width) * usable_rows);
  for (std::size_t row = 0; row < usable_rows; ++row) {
    const std::uint8_t* row_data = data + row * layout.row_step;
    for (std::size_t column = 0; column < layout.width; ++column) {
      const std::uint8_t* point = row_data + column * layout.point_step;
      const double x = read_float_field(point, layout.x_offset, layout.x_datatype, layout.is_bigendian);
      const double y = read_float_field(point, layout.y_offset, layout.y_datatype, layout.is_bigendian);
      const double z = read_float_field(point, layout.z_offset, layout.z_datatype, layout.is_bigendian);
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;
      double time = 0.0;
      if (has_time) {
        time = read_float_field(point, layout.time_offset, layout.time_datatype, layout.is_bigendian);
        if (!std::isfinite(time)) time = 0.0;
      }
      TimedPoint timed_point;
      timed_point.position = Eigen::Vector3f(static_cast<float>(x), static_cast<float>(y),
                                             static_cast<float>(z));
      timed_point.time_offset_sec = static_cast<float>(time);
      points.push_back(timed_point);
    }
  }
  return points;
}

}  // namespace lightnav_localization
