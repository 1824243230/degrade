#pragma once
#include "loop_geometry.h"
#include <ros/node_handle.h>
#include <memory>

namespace sa_loop {
struct FrontendSnapshot {
  double stamp=0;
  Pose pose=Pose::Identity();
  Matrix6 covariance=Matrix6::Identity();
  Cloud::ConstPtr cloud; // points in IMU coordinates at stamp; never mutated by worker
  double visual_quality=0;
  int visual_observations=0;
};
struct LoopConfig {
  bool enabled=false, offline=false;
  std::string output_dir, btc_config;
  std::string odom_frame="camera_init", map_frame="map";
  int queue_limit=32, min_triangles=6;
  double submap_seconds=1, submap_distance=2, submap_angle_deg=20, voxel=.2;
  double exclusion_seconds=30, min_btc_score=.5;
  double consistency_distance=.5, consistency_angle_deg=5;
  VerificationConfig verification;
};
LoopConfig loadLoopConfig(const ros::NodeHandle &node,const std::string &output_dir,bool offline);
class LoopClosureBackend {
public:
  LoopClosureBackend(const LoopConfig &,const ros::NodeHandle &);
  ~LoopClosureBackend();
  void submit(const FrontendSnapshot &snapshot);
  void finish(); // drain, optimize pending accepted constraints, export; propagates worker errors
private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
} // namespace sa_loop
