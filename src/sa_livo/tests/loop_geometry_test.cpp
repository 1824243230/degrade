#include "loop_geometry.h"
#include <btc.h>
#include <pcl/io/pcd_io.h>
#include <cassert>
#include <random>
#include <iostream>
#include "loop_test_room.h"

int main(int argc,char **argv) {
  using namespace sa_loop;
  std::mt19937 random(42);std::uniform_real_distribution<double> distribution(-4,4);
  Cloud source,target;Planes sp,tp;
  Pose T=Pose::Identity();T.linear()=Eigen::AngleAxisd(.4,Eigen::Vector3d(.3,.5,.8).normalized()).toRotationMatrix();T.translation()<<1,-.8,.3;
  for(int i=0;i<300;++i) {
    Eigen::Vector3d p(distribution(random),distribution(random),distribution(random));
    Eigen::Vector3d n(distribution(random),distribution(random),distribution(random));n.normalize();
    pcl::PointXYZI point;point.x=p.x();point.y=p.y();point.z=p.z();point.intensity=1;source.push_back(point);
    Eigen::Vector3d v=T*p;point.x=v.x();point.y=v.y();point.z=v.z();target.push_back(point);
    pcl::PointXYZINormal plane;plane.x=p.x();plane.y=p.y();plane.z=p.z();plane.normal_x=n.x();plane.normal_y=n.y();plane.normal_z=n.z();sp.push_back(plane);
    n=T.linear()*n;plane.x=v.x();plane.y=v.y();plane.z=v.z();plane.normal_x=n.x();plane.normal_y=n.y();plane.normal_z=n.z();tp.push_back(plane);
  }
  VerificationConfig cfg;auto verified=verifyRegistration(source,target,sp,tp,T,cfg);
  assert(verified.accepted && verified.rmse<1e-5 && verified.overlap_forward>.99);
  assert(!verifyRegistration(source,target,sp,tp,T.inverse(),cfg).accepted);
  assert(!verifyRegistration(Cloud{},target,sp,tp,T,cfg).accepted);
  Planes planar=sp;
  for(auto &p:planar){p.normal_x=0;p.normal_y=0;p.normal_z=1;}
  auto degenerate=verifyRegistration(source,source,planar,planar,Pose::Identity(),cfg);
  assert(!degenerate.accepted && degenerate.reason=="unobservable");
  PoseGraph graph;
  const Eigen::Vector3d positions[]={{0,0,0},{2,0,0},{2,2,0},{0,2,0},{0,0,0}};
  for(int i=0;i<5;++i){Pose pose=Pose::Identity();pose.translation()=positions[i]+Eigen::Vector3d(.2*i,.05*i,0);graph.nodes.push_back({double(i),pose,pose});
    if(i){GraphEdge e;e.a=i-1;e.b=i;e.measurement=graph.nodes[i-1].raw.inverse()*pose;e.information=Matrix6::Identity()*100;graph.edges.push_back(e);}}
  assert(graph.optimize());
  for(const auto &n:graph.nodes)assert((n.raw.matrix()-n.optimized.matrix()).norm()<1e-6);
  assert((graph.correctionAt(2.3).matrix()-Pose::Identity().matrix()).norm()<1e-6);
  GraphEdge loop;loop.a=0;loop.b=4;loop.measurement=Pose::Identity();loop.information=Matrix6::Identity()*10000;loop.loop=true;graph.edges.push_back(loop);
  assert(graph.optimize());assert(graph.nodes.back().optimized.translation().norm()<.05);
  assert(graph.nodes.front().optimized.translation().norm()<1e-8);
  const Pose before=graph.nodes.back().optimized;
  graph.edges.back().a=99;assert(!graph.optimize());assert((before.matrix()-graph.nodes.back().optimized.matrix()).norm()==0);
  assert(validPose(graph.correctionAt(2.5)));
  // Exercise BTC's real alignment direction, not just graph residual construction.
  ConfigSetting settings;settings.normal_threshold_=.2;settings.dis_threshold_=.5;
  BtcDescManager manager(settings);
  std::pair<Eigen::Vector3d,Eigen::Matrix3d> guess{T.translation()+Eigen::Vector3d(.01,-.02,.01),T.linear()};
  assert(manager.PlaneGeomrtricIcp(sp.makeShared(),tp.makeShared(),guess));
  assert((guess.first-T.translation()).norm()<1e-4);
  std::vector<BTC> descriptors;manager.GenerateBtcDescs(Cloud::Ptr(new Cloud),0,descriptors);
  std::pair<int,double> candidate;std::vector<std::pair<BTC,BTC>> matches;
  manager.SearchLoop(descriptors,candidate,guess,matches);assert(candidate.first==-1);
  assert(!manager.PlaneGeomrtricIcp(Planes::Ptr(new Planes),tp.makeShared(),guess));
  {
    ConfigSetting config;
    config.plane_merge_normal_thre_=.1;config.plane_merge_dis_thre_=.3;
    BtcDescManager sparse(config);Cloud::Ptr points(new Cloud);
    // Nonempty, well-separated points create no valid plane voxels.
    for(int k=0;k<4;++k){pcl::PointXYZI p;p.x=k*2;p.y=k*3;p.z=k;points->push_back(p);}
    sparse.GenerateBtcDescs(points,0,descriptors);assert(sparse.plane_cloud_vec_[0]->empty());
  }
  {
    // A reproducible room with pillars: exercise actual descriptor generation,
    // insertion and positive retrieval, including temporal exclusion.
    Cloud::Ptr room=loopTestRoom();
    std::string file=std::string(ROOT_DIR)+"../btc_descriptor/config/config_indoor.yaml";
    ConfigSetting config;load_config_setting(file,config);config.skip_near_num_=0;
    BtcDescManager tested(config);tested.min_time_separation_=30;tested.frame_times_={100,101,140};
    std::vector<BTC> first,near,far;tested.GenerateBtcDescs(room,0,first);assert(first.size()>6);tested.AddBtcDescs(first);
    tested.GenerateBtcDescs(room,1,near);tested.SearchLoop(near,candidate,guess,matches);assert(candidate.first==-1);
    tested.GenerateBtcDescs(room,2,far);tested.SearchLoop(far,candidate,guess,matches);
    assert(candidate.first==0 && candidate.second>.5 && matches.size()>=6);
    assert(guess.first.norm()<1e-4 && (guess.second-Eigen::Matrix3d::Identity()).norm()<1e-4);
  }
  if(argc>1) {
    Cloud::Ptr real(new Cloud);assert(pcl::io::loadPCDFile(argv[1],*real)==0);
    std::string file=std::string(ROOT_DIR)+"../btc_descriptor/config/config_indoor.yaml";
    ConfigSetting config;load_config_setting(file,config);BtcDescManager tested(config);
    tested.print_debug_info_=true;tested.GenerateBtcDescs(real,0,descriptors);
    std::cout<<"Generated "<<descriptors.size()<<" BTC descriptors from supplied submap\n";
  }
  std::cout<<"Loop geometry, observability, BTC direction, and graph rollback passed\n";
}
