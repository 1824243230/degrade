#include "sparse_voxel_filter.h"
#include <cassert>
#include <limits>

int main() {
  pcl::PointCloud<pcl::PointXYZI> cloud;
  auto add = [&](float x, float y, float z, float intensity) {
    pcl::PointXYZI point; point.x=x; point.y=y; point.z=z; point.intensity=intensity;
    cloud.push_back(point);
  };
  add(.1f,.1f,.1f,2); add(.3f,.3f,.3f,4);
  // The bounding grid exceeds PCL VoxelGrid's 32-bit limit, even with few points.
  add(10000.1f,10000.1f,10000.1f,6); add(10000.3f,10000.3f,10000.3f,8);
  add(-.1f,-.1f,-.1f,10); add(-.3f,-.3f,-.3f,12);
  add(std::numeric_limits<float>::quiet_NaN(),0,0,0);
  const auto filtered=sa_livo::sparseVoxelFilter(cloud,1.0);
  assert(filtered.size()==3 && filtered.is_dense);
  assert(std::abs(filtered[0].x-.2)<1e-6 && std::abs(filtered[0].intensity-3)<1e-6);
  assert(std::abs(filtered[1].x-10000.2)<.002 && std::abs(filtered[1].intensity-7)<1e-6);
  assert(std::abs(filtered[2].x+.2)<1e-6 && std::abs(filtered[2].intensity-11)<1e-6);
  pcl::PointCloud<pcl::PointXYZ> xyz;
  for(const auto& point:filtered) xyz.emplace_back(point.x,point.y,point.z);
  assert(sa_livo::sparseVoxelFilter(xyz,1.0).size()==3);
  assert(sa_livo::sparseVoxelFilter(pcl::PointCloud<pcl::PointXYZ>{},1.0).empty());
  bool rejected=false;
  try {sa_livo::sparseVoxelFilter(xyz,0);} catch(const std::invalid_argument&){rejected=true;}
  assert(rejected);
}
