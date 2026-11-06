#pragma once

#include "lightnav_localization/pcd_io.hpp"

namespace lightnav_localization {

// Voxel grid downsampling (one averaged point per occupied voxel).
PointList downsample_voxel(const PointList& points, double voxel_size_m);

}  // namespace lightnav_localization
