#pragma once

#include <Eigen/Geometry>
#include <cmath>
#include <iterator>
#include <map>
#include <optional>

namespace lightnav_localization {

// Time-indexed Isometry3d with linear (translation) / slerp (rotation) interpolation.
class PoseBuffer {
 public:
  void add(double stamp_sec, const Eigen::Isometry3d& pose) { poses_[stamp_sec] = pose; }

  // Drops poses older than `stamp_sec`, keeping the newest pose at or before it so that
  // interpolation at `stamp_sec` still works.
  void prune_before(double stamp_sec) {
    auto keep_from = poses_.upper_bound(stamp_sec);
    if (keep_from == poses_.begin()) return;
    poses_.erase(poses_.begin(), std::prev(keep_from));
  }

  // True when `stamp_sec` is inside the buffered range. A scan stamped at its end (or start) may
  // arrive before the wheel odometry that covers its last point; callers wait until this is true.
  bool covers(double stamp_sec) const {
    return !poses_.empty() && stamp_sec <= poses_.rbegin()->first;
  }

  std::size_t size() const { return poses_.size(); }

  // Returns nullopt when stamp_sec is NaN or outside [oldest, newest].
  std::optional<Eigen::Isometry3d> interpolate(double stamp_sec) const {
    if (std::isnan(stamp_sec) || poses_.empty() || stamp_sec < poses_.begin()->first || stamp_sec > poses_.rbegin()->first) {
      return std::nullopt;
    }
    const auto upper = poses_.lower_bound(stamp_sec);
    if (upper->first == stamp_sec || upper == poses_.begin()) {
      return upper->second;
    }
    const auto lower = std::prev(upper);
    const double ratio = (stamp_sec - lower->first) / (upper->first - lower->first);
    Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
    result.linear() = Eigen::Quaterniond(lower->second.rotation())
                          .slerp(ratio, Eigen::Quaterniond(upper->second.rotation()))
                          .toRotationMatrix();
    result.translation() =
        (1.0 - ratio) * lower->second.translation() + ratio * upper->second.translation();
    return result;
  }

 private:
  std::map<double, Eigen::Isometry3d> poses_;
};

}  // namespace lightnav_localization
