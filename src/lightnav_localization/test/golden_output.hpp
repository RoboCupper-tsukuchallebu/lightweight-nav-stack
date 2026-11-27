#pragma once

// Support for the "same output as before the gyro" test (Issue #92 section 4.1): the outputs of a drive
// recorded with the code before the gyro existed (commit a1771d0) are kept as data and compared bit for
// bit. This header uses only what Localizer offered at that commit, so that it can be copied to a work
// tree of that commit to record again (see test/data/README.md).

#include <Eigen/Geometry>
#include <array>
#include <cinttypes>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <functional>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include "lightnav_localization/localizer.hpp"
#include "lightnav_localization/state_machine.hpp"
#include "synthetic_world.hpp"

namespace lightnav_localization::testing {

struct RecordedOutput {
  Eigen::Isometry3d map_to_base;
  LocalizationState state;
  double position_std_m;
};

// One output as kept in a recording: a hash of every bit (exact comparison) and the numbers
// (x, y, z, roll, pitch, yaw, position std) for platforms whose floating point rounds differently.
struct GoldenOutput {
  std::uint64_t hash = 0;
  int state = 0;
  std::array<double, 7> values{};
};

inline std::uint64_t hash_output(const RecordedOutput& output) {
  std::uint64_t hash = 1469598103934665603ULL;  // FNV-1a
  const auto mix = [&hash](const void* data, std::size_t size) {
    const auto* bytes = static_cast<const unsigned char*>(data);
    for (std::size_t index = 0; index < size; ++index) {
      hash ^= bytes[index];
      hash *= 1099511628211ULL;
    }
  };
  const Eigen::Matrix4d matrix = output.map_to_base.matrix();
  mix(matrix.data(), sizeof(double) * 16);
  const std::int32_t state_value = static_cast<std::int32_t>(output.state);
  mix(&state_value, sizeof(state_value));
  const double position_std_m = output.position_std_m;
  mix(&position_std_m, sizeof(position_std_m));
  return hash;
}

inline std::array<double, 7> output_values(const RecordedOutput& output) {
  const Eigen::Matrix3d rotation = output.map_to_base.rotation();
  const Eigen::Vector3d translation = output.map_to_base.translation();
  return {translation.x(), translation.y(), translation.z(),
          std::atan2(rotation(2, 1), rotation(2, 2)), -std::asin(rotation(2, 0)),
          std::atan2(rotation(1, 0), rotation(0, 0)), output.position_std_m};
}

inline GoldenOutput make_golden_output(const RecordedOutput& output) {
  return {hash_output(output), static_cast<int>(output.state), output_values(output)};
}

inline void write_golden(const std::string& file_path, const std::vector<RecordedOutput>& outputs) {
  std::ofstream stream(file_path);
  for (const RecordedOutput& output : outputs) {
    const GoldenOutput golden = make_golden_output(output);
    char buffer[64];
    std::snprintf(buffer, sizeof(buffer), "%016" PRIx64 " %d", golden.hash, golden.state);
    stream << buffer;
    for (const double value : golden.values) {
      std::snprintf(buffer, sizeof(buffer), " %.10g", value);
      stream << buffer;
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
    GoldenOutput output;
    if (!(line_stream >> hash_text >> output.state)) continue;
    output.hash = std::stoull(hash_text, nullptr, 16);
    for (double& value : output.values) line_stream >> value;
    outputs.push_back(output);
  }
  return outputs;
}

// ---- The recorded drive ----
// 40 m (two 20 m legs and one in-place 90 degree turn), below the 50 m that starts a prior-map rebuild on
// another thread (whose timing would change the result). Wheel scale 1.05 and 0.5 deg / 10 m yaw drift.
struct GoldenCourse {
  static constexpr double kLegLengthM = 20.0;
  static constexpr double kSpeedMps = 0.6;
  static constexpr double kTurnRateRps = 0.4;
  static double leg_duration_sec() { return kLegLengthM / kSpeedMps; }
  static double turn_duration_sec() { return (M_PI / 2) / kTurnRateRps; }
  static double total_sec() { return 2 * leg_duration_sec() + turn_duration_sec(); }
  static std::vector<Eigen::Vector2d> centerline() {
    return {Eigen::Vector2d(0, 0), Eigen::Vector2d(kLegLengthM, 0), Eigen::Vector2d(kLegLengthM, kLegLengthM)};
  }
  static Eigen::Isometry3d truth_at(double time_sec) {
    if (time_sec <= leg_duration_sec()) return arc_pose(time_sec, Eigen::Vector2d(0, 0), 0.0, kSpeedMps, 0.0);
    const double after_leg = time_sec - leg_duration_sec();
    if (after_leg <= turn_duration_sec()) {
      return arc_pose(after_leg, Eigen::Vector2d(kLegLengthM, 0), 0.0, 0.0, kTurnRateRps);
    }
    return arc_pose(after_leg - turn_duration_sec(), Eigen::Vector2d(kLegLengthM, 0), M_PI / 2, kSpeedMps, 0.0);
  }
};

// Runs the drive and returns every published output. `configure` may change the parameters, `call_wheel`
// makes the on_wheel_odom call (so that a caller can pass a gyro increment), `inspect_at_end` looks at
// the Localizer afterwards. The recording itself uses the plain two argument call.
template <typename Configure, typename CallWheel>
std::vector<RecordedOutput> drive_short_course(bool has_prior_map, const Configure& configure,
                                               const CallWheel& call_wheel,
                                               const std::function<void(Localizer&)>& inspect_at_end = {}) {
  constexpr double kStepSec = 0.02;
  constexpr double kScanDurationSec = 0.1;
  constexpr double kScanPeriodSec = 0.2;
  const SyntheticWorld world(GoldenCourse::centerline(), true);
  LocalizerParameters parameters;
  parameters.mapping = !has_prior_map;
  configure(parameters);
  Localizer localizer(parameters, has_prior_map ? world.sample_map(0.2) : PointList{}, std::nullopt);
  const auto truth_at = [](double time_sec) { return GoldenCourse::truth_at(time_sec); };
  localizer.initialize(truth_at(0.0));
  Eigen::Isometry3d odom = Eigen::Isometry3d::Identity();
  std::vector<RecordedOutput> outputs;
  const int step_count = static_cast<int>(GoldenCourse::total_sec() / kStepSec);
  const int steps_per_scan = static_cast<int>(kScanPeriodSec / kStepSec);
  for (int step = 0; step <= step_count; ++step) {
    const double time_sec = step * kStepSec;
    if (step > 0) {
      const Eigen::Isometry3d relative = truth_at(time_sec - kStepSec).inverse() * truth_at(time_sec);
      Eigen::Isometry3d drifted = relative;
      drifted.translation() *= 1.05;
      drifted.translation().z() = 0.0;
      drifted.linear() = Eigen::AngleAxisd(std::atan2(relative.rotation()(1, 0), relative.rotation()(0, 0)) +
                                               relative.translation().norm() * (0.5 * M_PI / 180.0 / 10.0),
                                           Eigen::Vector3d::UnitZ())
                             .toRotationMatrix();
      odom = odom * drifted;
    }
    if (step > 0 && step % steps_per_scan == 0 && time_sec >= kScanDurationSec) {
      localizer.on_scan(time_sec, world.synthesize_scan(truth_at, time_sec - kScanDurationSec, kScanDurationSec,
                                                        true, 360, 8));
    }
    const auto output = call_wheel(localizer, time_sec, odom);
    if (output) outputs.push_back({output->map_to_base, output->state, output->position_std_m});
  }
  if (inspect_at_end) inspect_at_end(localizer);
  return outputs;
}

// The toolchain (system, CPU, compiler) the recording was made with: floating point results differ in the
// last bits between them. CMake passes it (see CMakeLists.txt); a build without it records as "unknown".
inline std::string golden_toolchain() {
#ifdef LIGHTNAV_GOLDEN_TOOLCHAIN
  return LIGHTNAV_GOLDEN_TOOLCHAIN;
#else
  return "unknown";
#endif
}

inline std::string golden_file_name(bool has_prior_map) {
  return "golden_" + golden_toolchain() + (has_prior_map ? "_localization" : "_mapping") + "_short_course.txt";
}

// Records the drive with the plain two argument call into `directory` (both scenarios).
inline void write_golden_recordings(const std::string& directory) {
  for (const bool has_prior_map : {false, true}) {
    const std::vector<RecordedOutput> outputs = drive_short_course(
        has_prior_map, [](LocalizerParameters&) {},
        [](Localizer& localizer, double time_sec, const Eigen::Isometry3d& odom) {
          return localizer.on_wheel_odom(time_sec, odom);
        });
    write_golden(std::string(directory) + "/" + golden_file_name(has_prior_map), outputs);
  }
}

}  // namespace lightnav_localization::testing
