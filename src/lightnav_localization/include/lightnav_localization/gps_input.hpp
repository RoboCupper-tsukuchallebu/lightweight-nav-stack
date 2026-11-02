#pragma once

#include <Eigen/Geometry>
#include <memory>
#include <optional>

namespace lightnav_localization {

struct GpsOrigin {
  double latitude_deg = 0.0;
  double longitude_deg = 0.0;
  double altitude_m = 0.0;
};

// Mirrors the NavSatStatus values used by the spec.
enum class GpsFixStatus { kNoFix = -1, kFix = 0, kSbasFix = 1, kGbasFix = 2 };

struct GpsFix {
  double latitude_deg = 0.0;
  double longitude_deg = 0.0;
  double altitude_m = 0.0;
  GpsFixStatus status = GpsFixStatus::kNoFix;
  bool has_known_covariance = false;  // position_covariance_type == KNOWN
  double known_std_m = 0.0;           // horizontal std when the covariance is known
  double hdop = 0.0;                  // 0 when unknown
};

struct GpsParameters {
  double std_fix_m = 2.0;
  double std_sbas_m = 1.0;
  double std_gbas_m = 0.3;
  double max_hdop = 3.0;
  double gate_mahalanobis = 3.0;
  double altitude_sigma_scale = 3.0;
  double altitude_sigma_floor_m = 1.0;
  Eigen::Vector3d antenna_in_base = Eigen::Vector3d::Zero();
};

struct GpsMeasurement {
  Eigen::Vector3d base_position;  // antenna observation with the lever arm removed (map frame)
  Eigen::Vector3d sigma_m;
};

// Why a fix was not used. kNone means accepted.
enum class GpsRejection { kNone, kNoFix, kHdop, kGate };

struct GpsDecision {
  GpsRejection rejection = GpsRejection::kNone;
  std::optional<GpsMeasurement> measurement;
};

// Counts consecutive gate rejections while the map matcher is tracking (is_gps_conflict).
class GpsConflictCounter {
 public:
  explicit GpsConflictCounter(int threshold = 5) : threshold_(threshold) {}
  void record(bool is_rejected_by_gate) { count_ = is_rejected_by_gate ? count_ + 1 : 0; }
  bool is_conflict() const { return count_ >= threshold_; }

 private:
  int threshold_;
  int count_ = 0;
};

// map frame = ENU about `origin`.
class GpsConverter {
 public:
  GpsConverter(const GpsOrigin& origin, const GpsParameters& parameters);
  ~GpsConverter();
  GpsConverter(GpsConverter&&) noexcept;
  GpsConverter& operator=(GpsConverter&&) noexcept;

  Eigen::Vector3d to_enu(const GpsFix& fix) const;
  double horizontal_std_m(const GpsFix& fix) const;

  // Quality filter, lever-arm removal with the predicted orientation, and the Mahalanobis gate
  // against the predicted base position. `predicted_position_std_m` is the estimator's own (x, y)
  // uncertainty; it widens the gate as the spec requires when no map match is available.
  GpsDecision evaluate(const GpsFix& fix, const Eigen::Isometry3d& predicted_map_to_base,
                       double predicted_position_std_m) const;

 private:
  struct Implementation;
  std::unique_ptr<Implementation> implementation_;
  GpsParameters parameters_;
};

}  // namespace lightnav_localization
