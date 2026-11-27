#pragma once

// Support for the "same output as before the gyro" test (Issue #92 section 4.1): the outputs of a drive
// recorded with the code before the gyro existed are kept as data and compared bit for bit.

#include <Eigen/Geometry>
#include <array>
#include <cinttypes>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "lightnav_localization/state_machine.hpp"

namespace lightnav_localization::testing {

struct GoldenOutput {
  std::uint64_t hash = 0;           // of every bit of the output
  bool has_values = false;          // every 25th output also keeps its numbers (for other platforms)
  std::array<double, 13> values{};  // 3x4 pose (row major), position standard deviation
};

inline std::uint64_t hash_output(const Eigen::Isometry3d& pose, LocalizationState state, double position_std_m) {
  std::uint64_t hash = 1469598103934665603ULL;  // FNV-1a
  const auto mix = [&hash](const void* data, std::size_t size) {
    const auto* bytes = static_cast<const unsigned char*>(data);
    for (std::size_t index = 0; index < size; ++index) {
      hash ^= bytes[index];
      hash *= 1099511628211ULL;
    }
  };
  const Eigen::Matrix4d matrix = pose.matrix();
  mix(matrix.data(), sizeof(double) * 16);
  const std::int32_t state_value = static_cast<std::int32_t>(state);
  mix(&state_value, sizeof(state_value));
  mix(&position_std_m, sizeof(position_std_m));
  return hash;
}

inline GoldenOutput make_golden_output(const Eigen::Isometry3d& pose, LocalizationState state,
                                       double position_std_m, bool should_keep_values) {
  GoldenOutput output;
  output.hash = hash_output(pose, state, position_std_m);
  if (should_keep_values) {
    output.has_values = true;
    for (int row = 0; row < 3; ++row) {
      for (int column = 0; column < 4; ++column) output.values[row * 4 + column] = pose.matrix()(row, column);
    }
    output.values[12] = position_std_m;
  }
  return output;
}

inline void write_golden(const std::string& file_path, const std::vector<GoldenOutput>& outputs) {
  std::ofstream stream(file_path);
  for (const GoldenOutput& output : outputs) {
    char buffer[64];
    std::snprintf(buffer, sizeof(buffer), "%016" PRIx64, output.hash);
    stream << buffer;
    if (output.has_values) {
      for (const double value : output.values) {
        std::snprintf(buffer, sizeof(buffer), " %a", value);
        stream << buffer;
      }
    }
    stream << "\n";
  }
}

inline std::vector<GoldenOutput> read_golden(const std::string& file_path) {
  std::ifstream stream(file_path);
  std::vector<GoldenOutput> outputs;
  std::string line;
  while (std::getline(stream, line)) {
    std::istringstream line_stream(line);
    std::string hash_text;
    line_stream >> hash_text;
    if (hash_text.empty()) continue;
    GoldenOutput output;
    output.hash = std::stoull(hash_text, nullptr, 16);
    std::string token;
    std::size_t index = 0;
    while (index < output.values.size() && line_stream >> token) output.values[index++] = std::strtod(token.c_str(), nullptr);
    output.has_values = index == output.values.size();
    outputs.push_back(output);
  }
  return outputs;
}

}  // namespace lightnav_localization::testing
