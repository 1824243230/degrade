// SPDX-License-Identifier: GPL-2.0-only
#include "localization_state.h"
#include "voxel_map.h"
#include "IMU_Processing.h"
#include "sa_vio.h"
#include <cassert>
#include <iostream>

void immutableMap(bool adaptive) {
  VoxelMapConfig config{};config.adaptive_support_en_=adaptive;
  config.max_voxel_size_=0.5;config.max_points_num_=50;config.max_layer_=1;
  config.layer_init_num_={5,5,5,5,5};config.planner_threshold_=0.01;
  config.planarity_ratio_th_=0.05;config.min_eigen_abs_=0.01;config.freeze_min_points_=15;
  config.scale1_radius_=2;config.chi2_thres_=9;config.max_shell_=6;config.multi_scale_en_=true;
  std::unordered_map<VOXEL_LOCATION,VoxelOctoTree*> empty;
  VoxelMapManager map(config,empty);
  std::vector<V3D> points;
  for(int i=0;i<40;++i)for(int j=0;j<40;++j)points.emplace_back(i*0.08,j*0.08,0);
  map.loadPriorMap(points,0.05);
  const auto count=adaptive ? map.adaptive_map_->size() : map.voxel_map_.size();assert(count>0);
  const auto cells=adaptive ? map.adaptive_map_->cells() : AdaptiveVoxelGrid::Map{};
  std::vector<pointWithVar> changed(1);changed[0].point_w=V3D(100,100,100);changed[0].var=M3D::Identity();
  map.UpdateVoxelMap(changed);map.state_.pos_end=V3D(1000,1000,1000);map.mapSliding();map.resetTracking();
  assert(count==(adaptive ? map.adaptive_map_->size() : map.voxel_map_.size()));
  if(adaptive)for(const auto& entry:cells){const auto& after=map.adaptive_map_->cells().at(entry.first);assert(entry.second.n==after.n);assert(entry.second.s1==after.s1);}
}
int main(){
  sa_localization::Config config;config.reset_exposure_std=0.7;config.reset_velocity_std=0.13;
  sa_localization::Recovery recovery;recovery.stamp=10;
  recovery.match.T_map_lidar=Eigen::Translation3d(3,4,2)*Eigen::AngleAxisd(0.5,V3D(1,2,3).normalized());
  recovery.mean_gyro=V3D(.001,-.002,.003);
  Eigen::Isometry3d extrinsic=Eigen::Translation3d(.1,.2,-.3)*Eigen::AngleAxisd(.2,V3D::UnitX());
  const V3D gravity=V3D(1,2,-9).normalized()*9.81;
  for(bool right:{false,true}) {
    g_right_invariant_en=right;
    const auto state=makeRecoveredState(recovery,extrinsic,gravity,V3D::Zero(),config);
    assert((state.rot_end*extrinsic.linear()-recovery.match.T_map_lidar.linear()).norm()<1e-10);
    assert((state.rot_end*extrinsic.translation()+state.pos_end-recovery.match.T_map_lidar.translation()).norm()<1e-10);
    assert(state.gravity==gravity);assert(state.vel_end.isZero());
    assert(std::abs(state.cov(StateIndex::Exposure,StateIndex::Exposure)-.49)<1e-12);
    assert(std::abs(state.cov(StateIndex::Velocity,StateIndex::Velocity)-.13*.13)<1e-12);
    assert(state.cov.llt().info()==Eigen::Success);
    sensor_msgs::Imu::Ptr imu(new sensor_msgs::Imu);
    const V3D acceleration=state.rot_end.transpose()*(-gravity);
    imu->linear_acceleration.x=acceleration.x();imu->linear_acceleration.y=acceleration.y();imu->linear_acceleration.z=acceleration.z();
    imu->angular_velocity.x=state.bias_g.x();imu->angular_velocity.y=state.bias_g.y();imu->angular_velocity.z=state.bias_g.z();
    imu->header.stamp=sec2Stamp(10);
    ImuProcess process;process.set_extrinsic(V3D::Zero(),M3D::Identity());
    process.resetForRelocalization(10,imu,state,acceleration);
    assert(!process.imu_need_init);assert(process.imu_time_init);assert(std::abs(process.IMU_mean_acc_norm-9.81)<1e-6);
    LidarMeasureGroup group;group.lio_vio_flg=LIO;group.last_lio_update_time=10;
    group.lidar_frame_beg_time=10;group.lidar_frame_end_time=10.1;
    PointType point;point.x=1;point.y=2;point.z=3;point.curvature=100;
    group.lidar->push_back(point);group.pcl_proc_cur->push_back(point);
    MeasureGroup measurement;measurement.lio_time=10.1;
    for(int i=1;i<=20;++i){sensor_msgs::Imu::Ptr sample(new sensor_msgs::Imu(*imu));sample->header.stamp=sec2Stamp(10+i*.005);measurement.imu.push_back(sample);}
    group.measures.push_back(measurement);
    StatesGroup propagated=state;PointCloudXYZI::Ptr output(new PointCloudXYZI);
    process.Process2(group,propagated,output);
    assert((propagated.rot_end-state.rot_end).norm()<1e-8);assert((propagated.pos_end-state.pos_end).norm()<1e-8);
  }
  immutableMap(true);immutableMap(false);
  std::cout<<"state, IMU boundary and immutable-map tests passed"<<std::endl;
}
