#pragma once
#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <string>
#include <iosfwd>
#include <vector>

namespace sa_loop {
using Pose = Eigen::Isometry3d;
using Matrix6 = Eigen::Matrix<double, 6, 6>;
using Cloud = pcl::PointCloud<pcl::PointXYZI>;
using Planes = pcl::PointCloud<pcl::PointXYZINormal>;
struct GraphNode {
  double stamp = 0;
  Pose raw = Pose::Identity(), optimized = Pose::Identity();
};
struct GraphEdge {
  // Measurement maps coordinates in node b to coordinates in node a.
  int a = -1, b = -1;
  Pose measurement = Pose::Identity();
  Matrix6 information = Matrix6::Identity(); // [rotation radians; translation meters]
  bool loop = false;
};
class PoseGraph {
public:
  std::vector<GraphNode> nodes;
  std::vector<GraphEdge> edges;
  bool optimize(int iterations = 50);
  Pose correctionAt(double stamp) const;
};
struct VerificationConfig {
  double radius = .3, min_overlap = .35, max_rmse = .15;
  double min_eigen_ratio = 1e-4, rotation_length = 2.0;
  int min_plane_matches = 6;
};
struct Verification {
  bool accepted = false;
  std::string reason = "empty_cloud";
  double overlap_forward = 0, overlap_reverse = 0, rmse = 0, eigen_ratio = 0;
  int plane_matches = 0;
  Matrix6 information = Matrix6::Zero();
};
Verification verifyRegistration(const Cloud &source, const Cloud &target,
    const Planes &source_planes, const Planes &target_planes, const Pose &target_from_source,
    const VerificationConfig &cfg);
Cloud::Ptr voxelFilter(const Cloud &cloud, double leaf);
bool validPose(const Pose &pose);
void writePose(std::ostream &out, double stamp, const Pose &pose);
} // namespace sa_loop
