#pragma once

#include <Eigen/Core>
#include <string>
#include <vector>

namespace lightnav_localization {

using PointList = std::vector<Eigen::Vector3f>;

enum class PcdEncoding { kAscii, kBinary };

// Writes fields "x y z" (float32). Throws std::runtime_error on I/O failure.
void write_pcd(const std::string& file_path, const PointList& points, PcdEncoding encoding);

// Reads ASCII / binary PCD that has exactly the fields x y z (float32).
// Throws std::runtime_error for binary_compressed, unknown fields or malformed files.
PointList read_pcd(const std::string& file_path);

}  // namespace lightnav_localization
