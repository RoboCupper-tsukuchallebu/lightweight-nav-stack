#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "lightnav_localization/deskew.hpp"

namespace lightnav_localization {

// Layout of the PointCloud2 buffer fields this package reads. Offsets are bytes from the start of
// a point. `time_offset < 0` means that the cloud has no time field (the offset defaults to 0).
struct PointCloudFieldLayout {
  std::uint32_t width = 0;
  std::uint32_t height = 0;
  std::uint32_t point_step = 0;
  std::uint32_t row_step = 0;
  int x_offset = 0;
  int y_offset = 0;
  int z_offset = 0;
  int time_offset = -1;
  int x_datatype = 0;
  int y_datatype = 0;
  int z_datatype = 0;
  int time_datatype = 0;
  bool is_bigendian = false;
};

// Extracts TimedPoints from a PointCloud2-like buffer. Organized clouds are read row by row
// (`row_step` may exceed `width * point_step`). Points with a non-finite x, y or z are dropped;
// a non-finite time becomes 0. Returns an empty list for an invalid layout or buffer.
std::vector<TimedPoint> extract_timed_points(const std::uint8_t* data, std::size_t data_size,
                                             const PointCloudFieldLayout& layout);

// True for FLOAT32 (7) and FLOAT64 (8) PointField datatypes.
bool is_supported_float_datatype(int datatype);

}  // namespace lightnav_localization
