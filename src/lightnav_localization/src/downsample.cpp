#include "lightnav_localization/downsample.hpp"

#include <small_gicp/points/point_cloud.hpp>
#include <small_gicp/util/downsampling.hpp>

namespace lightnav_localization {

PointList downsample_voxel(const PointList& points, double voxel_size_m) {
  const small_gicp::PointCloud input(points);
  const auto sampled = small_gicp::voxelgrid_sampling(input, voxel_size_m);
  PointList result;
  result.reserve(sampled->size());
  for (std::size_t index = 0; index < sampled->size(); ++index) {
    result.push_back(sampled->point(index).head<3>().cast<float>());
  }
  return result;
}

}  // namespace lightnav_localization
