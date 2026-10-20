// SPDX-License-Identifier: GPL-2.0-only
#include "preprocess.h"
#include "IMU_Processing.h"
#include <cassert>
#include <limits>

int main() {
  Preprocess pre;
  pre.set(false, VELO16, 2.0, 1);
  pre.N_SCANS=16;
  pre.max_range=150;
  pcl::PointCloud<velodyne_ros::Point> input;
  for (const float time : {.10f, 0.f, .05f}) {
    velodyne_ros::Point p{}; p.x=3; p.y=1; p.z=0; p.ring=3; p.time=time;
    input.push_back(p);
  }
  auto bad=input[0]; bad.x=std::numeric_limits<float>::quiet_NaN(); input.push_back(bad);
  bad=input[0]; bad.x=151; input.push_back(bad);
  bad=input[0]; bad.x=0; bad.y=0; input.push_back(bad);
  bad=input[0]; bad.ring=64; input.push_back(bad);
  bad=input[0]; bad.time=-.1f; input.push_back(bad);
  sensor_msgs::PointCloud2::Ptr message(new sensor_msgs::PointCloud2);
  pcl::toROSMsg(input,*message);
  PointCloudXYZI::Ptr output(new PointCloudXYZI);
  pre.process(message,output);
  assert(output->size()==3);
  assert(std::abs(output->front().curvature)<1e-5);
  assert(std::abs((*output)[1].curvature-50)<1e-4);
  assert(std::abs(output->back().curvature-100)<1e-4);

  // The configured local gravity must survive initialization and propagation.
  G_m_s2=9.7946;
  ImuProcess imu;
  imu.set_imu_init_frame_num(2);
  imu.set_extrinsic(V3D::Zero(),M3D::Identity());
  LidarMeasureGroup group; group.lio_vio_flg=LIO;
  group.last_lio_update_time=10; group.lidar_frame_beg_time=10;
  MeasureGroup batch; batch.lio_time=10.1;
  for(int i=0;i<3;++i) {
    sensor_msgs::Imu::Ptr sample(new sensor_msgs::Imu);
    sample->header.stamp=sec2Stamp(10+i*.005);
    sample->linear_acceleration.z=9.7946;
    imu.cache_init_imu(sample); batch.imu.push_back(sample);
  }
  group.measures.push_back(batch);
  StatesGroup state;
  imu.Process2(group,state,output);
  assert(!imu.imu_need_init);
  assert(std::abs(state.gravity.norm()-9.7946)<1e-8);
  group.measures.clear(); batch.imu.clear(); batch.lio_time=10.2;
  group.lidar_frame_beg_time=10.1; group.lidar_frame_end_time=10.2;
  PointType point{}; point.x=3; point.curvature=100;
  group.lidar->push_back(point); group.pcl_proc_cur->push_back(point);
  for(int i=1;i<=20;++i) {
    sensor_msgs::Imu::Ptr sample(new sensor_msgs::Imu);
    sample->header.stamp=sec2Stamp(10.1+i*.005); sample->linear_acceleration.z=9.7946;
    batch.imu.push_back(sample);
  }
  group.measures.push_back(batch); imu.Process2(group,state,output);
  assert(state.pos_end.norm()<1e-8); assert(state.vel_end.norm()<1e-8);

  // A moving start must not seed gravity/bias; only a full later stable window does.
  ImuProcess stationary;
  stationary.set_imu_init_frame_num(30);
  stationary.set_stationary_init(true,1.0,0.05,0.10);
  StatesGroup stable_state;
  LidarMeasureGroup stable_group; stable_group.lio_vio_flg=LIO;
  for (int frame=0;frame<30 && stationary.imu_need_init;++frame) {
    MeasureGroup samples; samples.lio_time=20+(frame+1)*.1;
    for (int j=0;j<20;++j) {
      sensor_msgs::Imu::Ptr sample(new sensor_msgs::Imu);
      sample->header.stamp=sec2Stamp(20+frame*.1+j*.005);
      sample->linear_acceleration.z=9.7946;
      sample->angular_velocity.z=frame<12 ? .2 : .001;
      samples.imu.push_back(sample);
    }
    stable_group.measures.clear(); stable_group.measures.push_back(samples);
    stationary.Process2(stable_group,stable_state,output);
    if(frame<21) assert(stationary.imu_need_init);
  }
  assert(!stationary.imu_need_init);
  assert(std::abs(stable_state.gravity.norm()-9.7946)<1e-8);
  assert(std::abs(stable_state.bias_g.z()-.001)<1e-9);
}
