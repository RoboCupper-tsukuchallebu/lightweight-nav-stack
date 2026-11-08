#include "lightnav_localization/gps_input.hpp"

#include <GeographicLib/LocalCartesian.hpp>
#include <algorithm>
#include <cmath>

namespace lightnav_localization {

struct GpsConverter::Implementation {
  explicit Implementation(const GpsOrigin& origin)
      : local_cartesian(origin.latitude_deg, origin.longitude_deg, origin.altitude_m) {}
  GeographicLib::LocalCartesian local_cartesian;
};

GpsConverter::GpsConverter(const GpsOrigin& origin, const GpsParameters& parameters)
    : implementation_(std::make_unique<Implementation>(origin)), parameters_(parameters) {}
GpsConverter::~GpsConverter() = default;
GpsConverter::GpsConverter(GpsConverter&&) noexcept = default;
GpsConverter& GpsConverter::operator=(GpsConverter&&) noexcept = default;

Eigen::Vector3d GpsConverter::to_enu(const GpsFix& fix) const {
  double east = 0.0;
  double north = 0.0;
  double up = 0.0;
  implementation_->local_cartesian.Forward(fix.latitude_deg, fix.longitude_deg, fix.altitude_m,
                                           east, north, up);
  return {east, north, up};
}

double GpsParameters::horizontal_std_m(const GpsFix& fix) const {
  if (fix.has_known_covariance && std::isfinite(fix.known_std_m) && fix.known_std_m > 0.0) {
    return fix.known_std_m;
  }
  switch (fix.status) {
    case GpsFixStatus::kSbasFix: return std_sbas_m;
    case GpsFixStatus::kGbasFix: return std_gbas_m;
    default: return std_fix_m;
  }
}

double GpsConverter::horizontal_std_m(const GpsFix& fix) const {
  return parameters_.horizontal_std_m(fix);
}

GpsDecision GpsConverter::evaluate(const GpsFix& fix, const Eigen::Isometry3d& predicted_map_to_base,
                                   double predicted_position_std_m) const {
  GpsDecision decision;
  if (fix.status == GpsFixStatus::kNoFix) {
    decision.rejection = GpsRejection::kNoFix;
    return decision;
  }
  if (!std::isfinite(fix.latitude_deg) || !std::isfinite(fix.longitude_deg) ||
      !std::isfinite(fix.altitude_m) || std::abs(fix.latitude_deg) > 90.0 ||
      std::abs(fix.longitude_deg) > 180.0 || !std::isfinite(fix.hdop)) {
    decision.rejection = GpsRejection::kInvalid;
    return decision;
  }
  if (fix.hdop > parameters_.max_hdop) {
    decision.rejection = GpsRejection::kHdop;
    return decision;
  }
  const double horizontal_std = horizontal_std_m(fix);
  const double altitude_std =
      std::max(parameters_.altitude_sigma_floor_m, parameters_.altitude_sigma_scale * horizontal_std);
  const Eigen::Vector3d sigma(horizontal_std, horizontal_std, altitude_std);
  const Eigen::Vector3d base_position =
      to_enu(fix) - predicted_map_to_base.linear() * parameters_.antenna_in_base;
  const Eigen::Vector3d difference = base_position - predicted_map_to_base.translation();
  // Horizontal gate only: the vertical estimate is weak by design (planar constraint).
  const double combined_std = std::hypot(horizontal_std, predicted_position_std_m);
  const bool trusted_fix = fix.has_known_covariance && std::isfinite(fix.known_std_m) &&
                           fix.known_std_m > 0.0 &&
                           fix.known_std_m <= parameters_.gps_trusted_std_m;
  const double horizontal_distance = difference.head<2>().norm();
  const bool absolute_rejection = horizontal_distance > parameters_.gps_absolute_gate_m;
  const bool mahalanobis_rejection = horizontal_distance / combined_std > parameters_.gate_mahalanobis;
  decision.gate_would_reject = mahalanobis_rejection || (trusted_fix && absolute_rejection);
  // Keep the conflict diagnostic independent of the recovery allowance.
  const double trusted_gate_distance_m = std::max(
      parameters_.gps_absolute_gate_m, parameters_.gate_mahalanobis * predicted_position_std_m);
  const bool reject_measurement = trusted_fix
      ? horizontal_distance > trusted_gate_distance_m
      : mahalanobis_rejection;
  if (reject_measurement) {
    decision.rejection = GpsRejection::kGate;
    return decision;
  }
  decision.measurement = GpsMeasurement{base_position, sigma};
  return decision;
}

}  // namespace lightnav_localization
