#pragma once

namespace lightnav_localization {

// Whether the gyro yaw increments may replace the wheel yaw increments (Issue #92, section 3.3).
//   kUnverified           right after configuration; the gyro is not used until it agrees with LiDAR / wheels
//   kActive               verified; used for yaw replacement and rotation factors
//   kSuspended            wheels and LiDAR together outvoted the gyro; not used until verified again
//   kDisabledInconsistent repeatedly inconsistent; not used until the node is reconfigured
// This header is intentionally free of ROS and GTSAM.
enum class GyroConsistencyState { kUnverified, kActive, kSuspended, kDisabledInconsistent };

}  // namespace lightnav_localization
