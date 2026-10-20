// SPDX-License-Identifier: GPL-2.0-only
#include "localization.h"
#include <kiss_matcher/KISSMatcher.hpp>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <cassert>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <thread>

using namespace sa_localization;
Cloud scene() {
  Cloud cloud;
  for(int i=0;i<64;++i) for(int j=0;j<42;++j) {
    const double x=(i-32)*0.15,y=(j-21)*0.15;
    cloud.push_back(pcl::PointXYZ(x,y,-1.7+0.04*std::sin(x*1.7)));
    cloud.push_back(pcl::PointXYZ(5.2+0.03*std::sin(y*2),x*0.7,(j-21)*0.10));
    if(i<45) cloud.push_back(pcl::PointXYZ(x,-4.3,(j-21)*0.11+0.02*std::sin(x)));
  }
  for(int i=0;i<24;++i) for(int j=0;j<24;++j) {
    const double a=i*0.11,b=j*0.13;
    cloud.push_back(pcl::PointXYZ(-1.3+std::cos(a)*std::sin(b),1.7+std::sin(a)*std::sin(b),0.4+std::cos(b)));
  }
  return cloud;
}
MapMetadata metadata() {
  MapMetadata m; m.frame_id="map";m.version="synthetic-v1";m.calibration_id="default";m.gravity=Eigen::Vector3d(0,0,-9.81);return m;
}
Quality healthy(double stamp) {
  Quality q;q.stamp=stamp;q.points=1000;q.matches=800;q.raw_rms=q.gated_rms=0.01;return q;
}
Packet packet(double stamp,const Cloud& cloud) {
  Packet p;p.stamp=stamp;p.cloud=cloud;
  for(int i=0;i<20;++i) p.imu.push_back({stamp-0.095+i*0.005,Eigen::Vector3d(0,0,9.81),Eigen::Vector3d::Zero()});
  return p;
}
void qualityTests() {
  Config c;TrackingQualityMonitor monitor(c);
  State state=State::TRACKING;
  for(int i=0;i<30;++i) {auto q=healthy(1+i*0.1);q.visual_available=true;q.visual_q=0;q.visual_observations=0;state=monitor.update(q,state);assert(state==State::TRACKING);}
  monitor.reset();state=State::TRACKING;
  for(int i=0;i<4;++i) {auto q=healthy(5+i*0.1);q.matches=20;q.visual_available=true;q.visual_q=1;q.visual_observations=100;state=monitor.update(q,state);assert(state!=State::LOST);}
  for(int i=0;i<10;++i) state=monitor.update(healthy(5.4+i*0.1),state);
  assert(state==State::TRACKING);
  monitor.reset();state=State::TRACKING;
  for(int i=0;i<7;++i) {auto q=healthy(10+i*0.1);q.matches=0;q.raw_rms=1;q.solved=false;state=monitor.update(q,state);}
  assert(state==State::LOST);
  monitor.reset();state=State::RECOVERING;
  for(int i=0;i<10;++i) {state=monitor.update(healthy(20+i*0.1),state);assert(state==State::RECOVERING);}
  state=monitor.update(healthy(21),state);assert(state==State::TRACKING);
  auto q=healthy(22);q.finite=false;assert(monitor.update(q,state)==State::LOST);
  monitor.reset();state=State::TRACKING;
  for(int i=0;i<22;++i) {auto low=healthy(30+i*0.1);low.matches=20;low.visual_available=true;low.visual_q=1;state=monitor.update(low,state);}
  assert(state==State::LOST);
}
void mapTests(const Cloud& cloud,const std::filesystem::path& dir) {
  Config c;c.map_path=(dir/"map.pcd").string();c.metadata_path=c.map_path+".yaml";
  pcl::io::savePCDFileBinary(c.map_path,cloud); metadata().save(c.metadata_path);
  const auto map=PriorMapManager::load(c);assert(map->cloud().size()>100);assert(map->metadata().version=="synthetic-v1");
  for(bool rgb:{false,true}) {
    if(rgb) {
      pcl::PointCloud<pcl::PointXYZRGB> colored;
      for(const auto& p:cloud){pcl::PointXYZRGB point;point.x=p.x;point.y=p.y;point.z=p.z;point.r=20;point.g=40;point.b=60;colored.push_back(point);}
      pcl::io::savePCDFileBinary(c.map_path,colored);
    } else {
      pcl::PointCloud<pcl::PointXYZI> intensity;
      for(const auto& p:cloud){pcl::PointXYZI point;point.x=p.x;point.y=p.y;point.z=p.z;point.intensity=42;intensity.push_back(point);}
      pcl::io::savePCDFileBinary(c.map_path,intensity);
    }
    assert(PriorMapManager::load(c)->cloud().size()==map->cloud().size());
  }
  {
    std::ofstream file(c.map_path);
    file<<"VERSION .7\nFIELDS x y z\nSIZE 8 8 8\nTYPE F F F\nCOUNT 1 1 1\nWIDTH "<<cloud.size()<<"\nHEIGHT 1\nPOINTS "<<cloud.size()<<"\nDATA ascii\n";
    file.precision(10);
    for(const auto& p:cloud)file<<p.x<<' '<<p.y<<' '<<p.z<<'\n';
  }
  assert(PriorMapManager::load(c)->cloud().size()==map->cloud().size());
  Cloud with_nan=cloud;with_nan.push_back(pcl::PointXYZ(std::numeric_limits<float>::quiet_NaN(),0,0));
  assert(PriorMapManager(c,metadata(),with_nan).cloud().size()==map->cloud().size());
  c.calibration_id="wrong";
  bool caught=false;try{PriorMapManager::load(c);}catch(const std::exception&){caught=true;}assert(caught);
  c.calibration_id="default";
  std::ofstream(c.metadata_path)<<"units: cm\n";
  caught=false;try{PriorMapManager::load(c);}catch(const std::exception&){caught=true;}assert(caught);
  caught=false;try{PriorMapManager(c,metadata(),Cloud());}catch(const std::exception&){caught=true;}assert(caught);
  c.tile_stride=41;caught=false;try{c.validate();}catch(const std::exception&){caught=true;}assert(caught);
}
void registrationTests(const Cloud& cloud) {
  Config config;config.match_timeout=30;
  auto map=std::make_shared<PriorMapManager>(config,metadata(),cloud);
  Eigen::Isometry3d truth=Eigen::Isometry3d::Identity();
  truth.linear()=(Eigen::AngleAxisd(0.61,Eigen::Vector3d::UnitZ())*Eigen::AngleAxisd(0.09,Eigen::Vector3d::UnitX())).toRotationMatrix();
  truth.translation()=Eigen::Vector3d(2,-1,0.5);
  Cloud source;pcl::transformPointCloud(cloud,source,truth.inverse().matrix().cast<float>());
  std::atomic<bool> cancel(false);KissRelocalizer matcher(config);
  const auto result=matcher.match(source,*map,truth.linear().transpose()*metadata().gravity,cancel);
  std::cout<<"global: "<<result.success<<" "<<result.reason<<" rms="<<result.rmse<<" overlap="<<result.overlap<<std::endl;
  assert(result.success);
  assert((truth.translation()-result.T_map_lidar.translation()).norm()<0.03);
  assert(rotationDistance(truth.linear(),result.T_map_lidar.linear())<0.01);
  assert(!matcher.match(Cloud(),*map,metadata().gravity,cancel).success);
  Cloud unrelated=source;for(auto& point:unrelated){point.x+=50;point.y+=50;point.z+=50;}
  assert(!matcher.refine(unrelated,map->cloud(),Eigen::Isometry3d::Identity()).success);
  cancel=true;assert(!matcher.match(source,*map,metadata().gravity,cancel).success);cancel=false;
  Cloud plane;for(int i=0;i<50;++i)for(int j=0;j<50;++j)plane.push_back(pcl::PointXYZ(i*0.15,j*0.15,0));
  assert(!matcher.refine(plane,plane,Eigen::Isometry3d::Identity()).success);
  Cloud repeated=cloud;
  for(const auto& p:cloud)repeated.push_back(pcl::PointXYZ(p.x+24,p.y,p.z));
  PriorMapManager ambiguous_map(config,metadata(),repeated);
  const auto ambiguous=matcher.match(cloud,ambiguous_map,metadata().gravity,cancel);
  std::cout<<"repeated rooms: "<<ambiguous.reason<<std::endl;
  assert(!ambiguous.success);
  // Verify the upstream solver cannot leak a previous valid solution.
  kiss_matcher::KISSMatcher core(0.3f);
  Eigen::Matrix<double,3,Eigen::Dynamic> a(3,100),b(3,100);
  for(int i=0;i<100;++i){a.col(i)=cloud[i*30].getVector3fMap().cast<double>();b.col(i)=truth*a.col(i);}
  assert(core.solve(a,b).valid);
  a.resize(3,0);b.resize(3,0);assert(!core.solve(a,b).valid);
}
void lifecycleTests(const Cloud& cloud) {
  Config config;config.stationary_seconds=0.3;config.stationary_frames=3;
  auto map=std::make_shared<PriorMapManager>(config,metadata(),cloud);
  std::atomic<int> calls(0);
  auto job=[&](const Cloud&,const Eigen::Vector3d&,const std::atomic<bool>& cancel){
    ++calls;MatchResult result;result.success=!cancel.load();result.reason="synthetic worker";return result;
  };
  RelocalizationManager manager(config,map,Eigen::Matrix3d::Identity(),job);
  double stamp=100;
  bool applied=false;
  for(int i=0;i<25 && !applied;++i){applied=manager.ingest(packet(stamp,cloud));stamp+=0.1;std::this_thread::sleep_for(std::chrono::milliseconds(1));}
  assert(applied);assert(calls==1);assert(manager.state()==State::VERIFYING);assert(!manager.valid());
  manager.committed(stamp);
  for(int i=0;i<12;++i) manager.updateQuality(healthy(stamp+i*0.1));
  assert(manager.valid());
  auto bad=healthy(stamp+2);bad.finite=false;manager.updateQuality(bad);assert(!manager.processesTracking());
  manager.request("manual retry");assert(manager.attempts()==0);assert(!manager.valid());
  // An in-flight result must not survive a sensor epoch change.
  std::atomic<bool> release(false);
  RelocalizationManager delayed(config,map,Eigen::Matrix3d::Identity(),[&](const Cloud&,const Eigen::Vector3d&,const std::atomic<bool>&){
    while(!release)std::this_thread::sleep_for(std::chrono::milliseconds(1));MatchResult r;r.success=true;return r;
  });
  for(int i=0;i<10 && !delayed.workerBusy();++i) delayed.ingest(packet(200+i*0.1,cloud));
  assert(delayed.workerBusy());const auto generation=delayed.attemptId();
  delayed.clockReset();release=true;
  for(int i=0;i<100 && delayed.workerBusy();++i){delayed.poll();std::this_thread::sleep_for(std::chrono::milliseconds(1));}
  assert(!delayed.workerBusy());assert(delayed.attemptId()>generation);assert(delayed.state()!=State::VERIFYING);assert(!delayed.valid());
  // Repeated resets reuse the same immutable prior map and never reuse a previous task result.
  stamp=300;
  for(int cycle=0;cycle<50;++cycle) {
    manager.request("repeated recovery regression");
    bool restored=false;
    for(int i=0;i<25 && !restored;++i){restored=manager.ingest(packet(stamp,cloud));stamp+=.1;std::this_thread::sleep_for(std::chrono::milliseconds(1));}
    assert(restored);manager.committed(stamp);
    for(int i=0;i<12;++i)manager.updateQuality(healthy(stamp+i*.1));
    assert(manager.valid());stamp+=2;
  }
  assert(map->metadata().version=="synthetic-v1");
  Config failed_config=config;failed_config.retry_delay=.001;
  RelocalizationManager failing(failed_config,map,Eigen::Matrix3d::Identity(),[](const Cloud&,const Eigen::Vector3d&,const std::atomic<bool>&){
    MatchResult result;result.reason="intentional failure";return result;
  });
  for(int i=0;i<100 && failing.state()!=State::FAILED;++i){failing.ingest(packet(1000+i*.1,cloud));std::this_thread::sleep_for(std::chrono::milliseconds(2));}
  assert(failing.state()==State::FAILED);assert(failing.attempts()==failed_config.max_attempts);
  Config timeout_config=config;timeout_config.match_timeout=.02;timeout_config.max_attempts=1;
  release=false;
  RelocalizationManager timeout(timeout_config,map,Eigen::Matrix3d::Identity(),[&](const Cloud&,const Eigen::Vector3d&,const std::atomic<bool>&){
    while(!release)std::this_thread::sleep_for(std::chrono::milliseconds(1));MatchResult r;r.success=true;return r;
  });
  for(int i=0;i<10 && !timeout.workerBusy();++i)timeout.ingest(packet(1100+i*.1,cloud));
  assert(timeout.workerBusy());std::this_thread::sleep_for(std::chrono::milliseconds(30));timeout.poll();
  assert(timeout.state()==State::FAILED);release=true;
  while(timeout.workerBusy()){timeout.poll();std::this_thread::sleep_for(std::chrono::milliseconds(1));}
  assert(timeout.state()==State::FAILED);
}
int main(){
  char temporary[]="/tmp/sa_livo_localization_XXXXXX";const char* dir=mkdtemp(temporary);assert(dir);
  const auto cloud=scene();qualityTests();mapTests(cloud,dir);registrationTests(cloud);lifecycleTests(cloud);
  std::filesystem::remove_all(dir);std::cout<<"localization tests passed"<<std::endl;
}
