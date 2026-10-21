#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <type_traits>
#include <unordered_map>
#include <vector>

namespace sa_livo {
namespace voxel_detail {
struct Key {
  int64_t x, y, z;
  bool operator==(const Key& other) const {
    return x == other.x && y == other.y && z == other.z;
  }
};
struct Hash {
  size_t operator()(const Key& key) const {
    size_t value = 0;
    for (int64_t coordinate : {key.x, key.y, key.z})
      value ^= std::hash<int64_t>{}(coordinate) + 0x9e3779b97f4a7c15ULL +
               (value << 6) + (value >> 2);
    return value;
  }
};
struct Centroid {
  double x = 0, y = 0, z = 0, intensity = 0;
  size_t count = 0;
};
} // namespace voxel_detail

// Store occupied cells only: a large bounding box must not disable downsampling.
// First-seen cell order makes output independent of unordered_map iteration order.
template <typename PointT>
pcl::PointCloud<PointT> sparseVoxelFilter(const pcl::PointCloud<PointT>& input, double leaf) {
  static_assert(std::is_same_v<PointT, pcl::PointXYZ> || std::is_same_v<PointT, pcl::PointXYZI>,
                "This filter preserves XYZ and optional intensity fields");
  if (!std::isfinite(leaf) || leaf <= 0) throw std::invalid_argument("Invalid voxel size");
  std::unordered_map<voxel_detail::Key, size_t, voxel_detail::Hash> cells;
  std::vector<voxel_detail::Centroid> sums;
  for (const auto& point : input) {
    if (!point.getVector3fMap().allFinite()) continue;
    const double x = std::floor(double(point.x) / leaf);
    const double y = std::floor(double(point.y) / leaf);
    const double z = std::floor(double(point.z) / leaf);
    const double bound = std::ldexp(1.0, 63);
    if (x < -bound || x >= bound || y < -bound || y >= bound || z < -bound || z >= bound)
      throw std::invalid_argument("Point exceeds 64-bit voxel coordinate range");
    voxel_detail::Key key{int64_t(x), int64_t(y), int64_t(z)};
    auto found = cells.emplace(key, sums.size());
    if (found.second) sums.emplace_back();
    auto& sum = sums[found.first->second];
    sum.x += point.x; sum.y += point.y; sum.z += point.z; ++sum.count;
    if constexpr (std::is_same_v<PointT, pcl::PointXYZI>) sum.intensity += point.intensity;
  }
  pcl::PointCloud<PointT> output;
  output.header = input.header;
  output.sensor_origin_ = input.sensor_origin_;
  output.sensor_orientation_ = input.sensor_orientation_;
  output.reserve(sums.size());
  for (const auto& sum : sums) {
    PointT point;
    point.x = sum.x / sum.count; point.y = sum.y / sum.count; point.z = sum.z / sum.count;
    if constexpr (std::is_same_v<PointT, pcl::PointXYZI>) point.intensity = sum.intensity / sum.count;
    output.push_back(point);
  }
  output.is_dense = true;
  return output;
}
} // namespace sa_livo
