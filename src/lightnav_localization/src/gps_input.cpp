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

double GpsConverter::horizontal_std_m(const GpsFix& fix) const {
  // A receiver can report a covariance of 0 or NaN: fall back to the status table then.
  if (fix.has_known_covariance && std::isfinite(fix.known_std_m) && fix.known_std_m > 0.0) {
    return fix.known_std_m;
  }
  switch (fix.status) {
    case GpsFixStatus::kSbasFix: return parameters_.std_sbas_m;
    case GpsFixStatus::kGbasFix: return parameters_.std_gbas_m;
    default: return parameters_.std_fix_m;
  }
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
  if (difference.head<2>().norm() / combined_std > parameters_.gate_mahalanobis) {
    decision.rejection = GpsRejection::kGate;
    return decision;
  }
  decision.measurement = GpsMeasurement{base_position, sigma};
  return decision;
}

}  // namespace lightnav_localization
