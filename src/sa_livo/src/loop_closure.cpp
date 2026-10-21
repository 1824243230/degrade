#include "loop_closure.h"
#include "sparse_voxel_filter.h"
#include <btc.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/String.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2_ros/transform_broadcaster.h>
#include <condition_variable>
#include <deque>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <thread>
#include <chrono>

namespace sa_loop {
namespace {
using Clock=std::chrono::steady_clock;
double milliseconds(Clock::time_point t){return std::chrono::duration<double,std::milli>(Clock::now()-t).count();}
ros::Time stamp(double t){ros::Time value;value.fromSec(t);return value;}
geometry_msgs::Pose poseMessage(const Pose &p) {
  geometry_msgs::Pose m;const Eigen::Quaterniond q(p.linear());
  m.position.x=p.translation().x();m.position.y=p.translation().y();m.position.z=p.translation().z();
  m.orientation.x=q.x();m.orientation.y=q.y();m.orientation.z=q.z();m.orientation.w=q.w();return m;
}
void require(bool condition,const std::string &message){if(!condition) throw std::invalid_argument(message);}
}
LoopConfig loadLoopConfig(const ros::NodeHandle &node,const std::string &out,bool offline) {
  LoopConfig c;c.output_dir=out;c.offline=offline;
  node.param("loop_closure/enabled",c.enabled,false);
  node.param("loop_closure/btc_config",c.btc_config,std::string());
  node.param("loop_closure/queue_limit",c.queue_limit,32);
  node.param("loop_closure/submap_seconds",c.submap_seconds,1.0);
  node.param("loop_closure/submap_distance",c.submap_distance,2.0);
  node.param("loop_closure/submap_angle_deg",c.submap_angle_deg,20.0);
  node.param("loop_closure/voxel",c.voxel,.2);
  node.param("loop_closure/exclusion_seconds",c.exclusion_seconds,30.0);
  node.param("loop_closure/min_btc_score",c.min_btc_score,.5);
  node.param("loop_closure/min_triangles",c.min_triangles,6);
  node.param("loop_closure/consistency_distance",c.consistency_distance,.5);
  node.param("loop_closure/consistency_angle_deg",c.consistency_angle_deg,5.0);
  node.param("loop_closure/radius",c.verification.radius,.3);
  node.param("loop_closure/min_overlap",c.verification.min_overlap,.35);
  node.param("loop_closure/max_rmse",c.verification.max_rmse,.15);
  node.param("loop_closure/min_eigen_ratio",c.verification.min_eigen_ratio,1e-4);
  require(c.queue_limit>0 && c.queue_limit<=4096,"Invalid loop queue_limit");
  for(double v:{c.submap_seconds,c.submap_distance,c.submap_angle_deg,c.voxel,
      c.exclusion_seconds,c.consistency_distance,c.consistency_angle_deg,
      c.verification.radius,c.verification.max_rmse,c.verification.rotation_length})
    require(std::isfinite(v)&&v>0,"Loop parameters must be finite and positive");
  require(c.min_triangles>=3 && c.min_btc_score>=0 && c.min_btc_score<=1 &&
      c.verification.min_overlap>0 && c.verification.min_overlap<=1 &&
      c.verification.min_eigen_ratio>0 && c.verification.min_eigen_ratio<1,"Invalid loop validation threshold");
  if(c.enabled) require(!c.btc_config.empty(),"loop_closure/btc_config is required");
  return c;
}
struct LoopClosureBackend::Impl {
  LoopConfig cfg;
  BtcDescManager btc;
  PoseGraph graph;
  struct Frame {double time;Pose raw;};
  std::vector<Frame> frames;
  std::vector<std::string> clouds;
  Cloud submap;
  Pose anchor=Pose::Identity();Matrix6 anchor_cov=Matrix6::Identity(),previous_cov=Matrix6::Identity();
  double anchor_time=-1,last_time=-1;
  bool submap_dropped=false;
  struct Pending {int source=-1,target=-1;Pose correction=Pose::Identity();GraphEdge edge;
    double score=0;size_t triangles=0;Verification verification;};
  Pending pending;
  size_t accepted=0,rejected=0,skipped=0,cloud_jobs=0,max_queue=0;
  std::mutex mutex;std::condition_variable condition;
  std::deque<FrontendSnapshot> queue;
  bool stopping=false,finished=false;
  std::exception_ptr failure;
  std::thread worker;
  std::ofstream loops,timings,online;
  ros::Publisher path_pub,map_pub,odom_pub,status_pub,markers_pub;
  tf2_ros::TransformBroadcaster tf;
  Impl(const LoopConfig &c,const ros::NodeHandle &node):cfg(c) {
    require(cfg.enabled,"Backend constructed while disabled");
    std::string file=cfg.btc_config;ConfigSetting settings;load_config_setting(file,settings);
    settings.skip_near_num_=0;settings.icp_threshold_=cfg.min_btc_score;
    btc.config_setting_=settings;btc.min_time_separation_=cfg.exclusion_seconds;
    std::filesystem::create_directories(cfg.output_dir+"/submaps");
    loops.open(cfg.output_dir+"/loops.csv");timings.open(cfg.output_dir+"/backend_timing.csv");
    online.open(cfg.output_dir+"/online_trajectory.tum");
    if(!loops || !timings || !online) throw std::runtime_error("Cannot open backend outputs");
    loops<<"source,target,timestamp,btc_score,triangles,overlap_forward,overlap_reverse,rmse,eigen_ratio,accepted,reason\n";
    timings<<"submap,timestamp,extraction_ms,query_ms,verification_ms,optimization_ms,queue,skipped_frames\n";
    ros::NodeHandle nh(node);
    path_pub=nh.advertise<nav_msgs::Path>("/loop_closure/path",1,true);
    map_pub=nh.advertise<sensor_msgs::PointCloud2>("/loop_closure/map",1,true);
    odom_pub=nh.advertise<nav_msgs::Odometry>("/loop_closure/odometry",10);
    status_pub=nh.advertise<std_msgs::String>("/loop_closure/status",1,true);
    markers_pub=nh.advertise<visualization_msgs::MarkerArray>("/loop_closure/markers",1,true);
    worker=std::thread([this]{run();});
  }
  std::string cloudPath(int id) const {return cfg.output_dir+"/submaps/"+std::to_string(id)+".pcd";}
  Cloud::Ptr loadCloud(int id) const {
    Cloud::Ptr cloud(new Cloud);
    if(pcl::io::loadPCDFile(clouds.at(id),*cloud)<0) throw std::runtime_error("Cannot read submap");
    return cloud;
  }
  void logLoop(int i,int j,double score,size_t triangles,const Verification &v,bool pass,const std::string &reason) {
    loops<<std::setprecision(12)<<i<<','<<j<<','<<graph.nodes.at(i).stamp<<','<<score<<','<<triangles<<','
        <<v.overlap_forward<<','<<v.overlap_reverse<<','<<v.rmse<<','<<v.eigen_ratio<<','<<pass<<','<<reason<<'\n';
    if(!pass) ++rejected;
  }
  void flush() {
    if(anchor_time<0) return;
    const int id=graph.nodes.size();
    Pose optimized=graph.nodes.empty()?anchor:graph.nodes.back().optimized*graph.nodes.back().raw.inverse()*anchor;
    graph.nodes.push_back({anchor_time,anchor,optimized});
    if(id) {
      GraphEdge edge;edge.a=id-1;edge.b=id;edge.measurement=graph.nodes[id-1].raw.inverse()*anchor;
      // Marginal pose covariances are not independent relative-pose covariance.
      // Use conservative diagonal uncertainty floors and sum both marginals.
      edge.information.setZero();
      for(int k=0;k<6;++k) {
        const double floor=k<3 ? std::pow(2*M_PI/180,2) : .01;
        edge.information(k,k)=1/std::max(floor,std::abs(anchor_cov(k,k))+std::abs(previous_cov(k,k)));
      }
      graph.edges.push_back(edge);
    }
    previous_cov=anchor_cov;
    auto cloud=voxelFilter(submap,cfg.voxel);
    clouds.push_back(cloudPath(id));
    // PCL cannot write an empty PCD. A valid zero-point header is written explicitly.
    if(cloud->empty()) {
      std::ofstream out(clouds.back());out<<"# .PCD v0.7\nVERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\nWIDTH 0\nHEIGHT 1\nPOINTS 0\nDATA ascii\n";
    } else if(pcl::io::savePCDFileBinary(clouds.back(),*cloud)<0) throw std::runtime_error("Cannot write submap");
    btc.frame_times_.push_back(anchor_time);
    std::vector<BTC> descriptors;
    auto start=Clock::now();
    // Partial data under realtime overflow are kept for mapping, excluded from retrieval.
    Cloud::Ptr description_cloud=submap_dropped ? Cloud::Ptr(new Cloud) : cloud;
    btc.GenerateBtcDescs(description_cloud,id,descriptors);
    double extraction=milliseconds(start);start=Clock::now();
    std::pair<int,double> candidate(-1,0);
    std::pair<Eigen::Vector3d,Eigen::Matrix3d> transform;
    std::vector<std::pair<BTC,BTC>> matches;
    btc.SearchLoop(descriptors,candidate,transform,matches);
    double query=milliseconds(start),verification_ms=0,optimization_ms=0;
    bool did_optimize=false;
    Verification v;
    if(candidate.first>=0) {
      int j=candidate.first;start=Clock::now();bool icp_ok=true;
      for(int k=0;k<3 && icp_ok;++k)
        icp_ok=btc.PlaneGeomrtricIcp(btc.plane_cloud_vec_[id],btc.plane_cloud_vec_[j],transform);
      Pose T=Pose::Identity();T.linear()=transform.second;T.translation()=transform.first;
      if(matches.size()<size_t(cfg.min_triangles)) v.reason="triangles";
      else if(!icp_ok) v.reason="icp_failed";
      else v=verifyRegistration(*cloud,*loadCloud(j),*btc.plane_cloud_vec_[id],*btc.plane_cloud_vec_[j],T,cfg.verification);
      verification_ms=milliseconds(start);
      if(v.accepted) {
        Pose correction=graph.nodes[j].raw*T*anchor.inverse();
        bool consistent=pending.source==id-1 && pending.target>=0 &&
            (correction.translation()-pending.correction.translation()).norm()<=cfg.consistency_distance &&
            Eigen::AngleAxisd(correction.linear()*pending.correction.linear().transpose()).angle()<=cfg.consistency_angle_deg*M_PI/180;
        GraphEdge edge;edge.a=j;edge.b=id;edge.measurement=T;edge.information=v.information;edge.loop=true;
        if(consistent) {
          const size_t old_size=graph.edges.size();
          if(!hasLoop(pending.edge.a,pending.edge.b)) graph.edges.push_back(pending.edge);
          if(!hasLoop(edge.a,edge.b)) graph.edges.push_back(edge);
          start=Clock::now();did_optimize=graph.optimize();optimization_ms=milliseconds(start);
          if(did_optimize) {
            accepted+=graph.edges.size()-old_size;
            logLoop(id,j,candidate.second,matches.size(),v,true,"accepted");
            if(graph.edges.size()-old_size>1)
              logLoop(pending.source,pending.target,pending.score,pending.triangles,pending.verification,true,"confirmed_by_next_submap");
          } else {
            graph.edges.resize(old_size);logLoop(id,j,candidate.second,matches.size(),v,false,"optimization_failed");
          }
        } else logLoop(id,j,candidate.second,matches.size(),v,false,"pending_consistency");
        pending.source=id;pending.target=j;pending.correction=correction;pending.edge=edge;
        pending.score=candidate.second;pending.triangles=matches.size();pending.verification=v;
      } else { pending=Pending{};logLoop(id,j,candidate.second,matches.size(),v,false,v.reason); }
    } else {
      pending=Pending{};logLoop(id,-1,0,0,v,false,submap_dropped?"realtime_overflow":"no_candidate");
    }
    btc.AddBtcDescs(descriptors);
    size_t queued;
    {std::lock_guard<std::mutex> lock(mutex);queued=queue.size();}
    timings<<id<<','<<std::setprecision(12)<<anchor_time<<','<<extraction<<','<<query<<','<<verification_ms<<','<<optimization_ms<<','<<queued<<','<<skipped<<'\n';
    if(did_optimize) publishGraph(true);
    publishStatus(queued,false);
    submap.clear();anchor_time=-1;submap_dropped=false;
  }
  bool hasLoop(int a,int b) const {
    return std::any_of(graph.edges.begin(),graph.edges.end(),[=](const GraphEdge &e){return e.loop&&e.a==a&&e.b==b;});
  }
  void consume(const FrontendSnapshot &s) {
    if(s.stamp<=last_time || !std::isfinite(s.stamp) || !validPose(s.pose))
      throw std::runtime_error("Invalid/nonmonotonic frontend snapshot");
    last_time=s.stamp;frames.push_back({s.stamp,s.pose});
    if(anchor_time<0) {anchor=s.pose;anchor_time=s.stamp;anchor_cov=s.covariance;}
    if(s.cloud) {
      Pose relative=anchor.inverse()*s.pose;
      for(const auto &p:*s.cloud) {
        pcl::PointXYZI q=p;Eigen::Vector3d v=relative*p.getVector3fMap().cast<double>();
        if(!v.allFinite()) continue;q.x=v.x();q.y=v.y();q.z=v.z();submap.push_back(q);
      }
    } else {submap_dropped=true;++skipped;}
    if(s.stamp-anchor_time>=cfg.submap_seconds ||
        (s.pose.translation()-anchor.translation()).norm()>=cfg.submap_distance ||
        Eigen::AngleAxisd(anchor.linear().transpose()*s.pose.linear()).angle()>=cfg.submap_angle_deg*M_PI/180) flush();
    Pose correction=graph.nodes.empty()?Pose::Identity():graph.nodes.back().optimized*graph.nodes.back().raw.inverse();
    Pose corrected=correction*s.pose;writePose(online,s.stamp,corrected);
    nav_msgs::Odometry odom;odom.header.stamp=stamp(s.stamp);odom.header.frame_id=cfg.map_frame;
    odom.child_frame_id="aft_mapped";odom.pose.pose=poseMessage(corrected);odom_pub.publish(odom);
    geometry_msgs::TransformStamped t;t.header=odom.header;t.child_frame_id=cfg.odom_frame;
    auto p=poseMessage(correction);t.transform.translation.x=p.position.x;t.transform.translation.y=p.position.y;
    t.transform.translation.z=p.position.z;t.transform.rotation=p.orientation;tf.sendTransform(t);
  }
  void publishStatus(size_t queued,bool done) {
    size_t maxq;{std::lock_guard<std::mutex> lock(mutex);maxq=max_queue;}
    std_msgs::String msg;std::ostringstream s;
    s<<"{\"submaps\":"<<graph.nodes.size()<<",\"loop_edges\":"<<accepted<<",\"nonaccepted_events\":"<<rejected
      <<",\"queue\":"<<queued<<",\"max_queue\":"<<maxq<<",\"skipped_frames\":"<<skipped<<",\"finished\":"<<(done?"true":"false")<<"}";
    msg.data=s.str();status_pub.publish(msg);
  }
  void publishGraph(bool with_map) {
    nav_msgs::Path path;path.header.frame_id=cfg.map_frame;path.header.stamp=stamp(std::max(0.0,last_time));
    for(const auto &n:graph.nodes) {geometry_msgs::PoseStamped p;p.header=path.header;p.header.stamp=stamp(n.stamp);p.pose=poseMessage(n.optimized);path.poses.push_back(p);}
    path_pub.publish(path);
    visualization_msgs::MarkerArray markers;
    visualization_msgs::Marker lines;lines.header=path.header;lines.ns="accepted_loops";lines.id=0;
    lines.type=visualization_msgs::Marker::LINE_LIST;lines.action=visualization_msgs::Marker::ADD;
    lines.pose.orientation.w=1;lines.scale.x=.08;lines.color.r=1;lines.color.a=1;
    for(const auto &e:graph.edges) if(e.loop) {lines.points.push_back(poseMessage(graph.nodes[e.a].optimized).position);lines.points.push_back(poseMessage(graph.nodes[e.b].optimized).position);}
    markers.markers.push_back(lines);markers_pub.publish(markers);
    if(with_map) {
      Cloud world;
      for(size_t i=0;i<clouds.size();++i) {
        if(btc.plane_cloud_vec_[i]->empty() && std::filesystem::file_size(clouds[i])<256) continue;
        auto local=loadCloud(i);
        for(const auto &p:*local) {pcl::PointXYZI q=p;Eigen::Vector3d v=graph.nodes[i].optimized*p.getVector3fMap().cast<double>();q.x=v.x();q.y=v.y();q.z=v.z();world.push_back(q);}
      }
      Cloud::Ptr map(new Cloud(sa_livo::sparseVoxelFilter(world,cfg.voxel)));
      if(!map->empty() && pcl::io::savePCDFileBinary(cfg.output_dir+"/optimized_map.pcd",*map)<0) throw std::runtime_error("Cannot save optimized map");
      sensor_msgs::PointCloud2 msg;pcl::toROSMsg(*map,msg);msg.header=path.header;map_pub.publish(msg);
    }
  }
  void exportResults() {
    std::ofstream trajectory(cfg.output_dir+"/optimized_trajectory.tum"),nodes(cfg.output_dir+"/submaps.csv"),edges(cfg.output_dir+"/graph_edges.csv");
    if(!trajectory || !nodes || !edges) throw std::runtime_error("Cannot export graph");
    for(const auto &f:frames) writePose(trajectory,f.time,graph.correctionAt(f.time)*f.raw);
    nodes<<"id,timestamp,raw_x,raw_y,raw_z,optimized_x,optimized_y,optimized_z\n";
    for(size_t i=0;i<graph.nodes.size();++i){const auto &n=graph.nodes[i];nodes<<std::setprecision(12)<<i<<','<<n.stamp;for(int k=0;k<3;++k)nodes<<','<<n.raw.translation()(k);for(int k=0;k<3;++k)nodes<<','<<n.optimized.translation()(k);nodes<<'\n';}
    edges<<"a,b,loop,tx,ty,tz,qx,qy,qz,qw\n";
    for(const auto &e:graph.edges){Eigen::Quaterniond q(e.measurement.linear());edges<<e.a<<','<<e.b<<','<<e.loop;for(int k=0;k<3;++k)edges<<','<<e.measurement.translation()(k);edges<<','<<q.x()<<','<<q.y()<<','<<q.z()<<','<<q.w()<<'\n';}
    publishGraph(true);publishStatus(0,true);
    std::ofstream summary(cfg.output_dir+"/backend_summary.json");
    summary<<"{\"submaps\":"<<graph.nodes.size()<<",\"loop_edges\":"<<accepted<<",\"skipped_frames\":"<<skipped<<",\"max_queue\":"<<max_queue<<",\"frames\":"<<frames.size()<<",\"finished\":true}\n";
  }
  void run() {
    try {
      while(true) {
        FrontendSnapshot snapshot;
        {std::unique_lock<std::mutex> lock(mutex);condition.wait(lock,[this]{return stopping||!queue.empty();});
          if(queue.empty()) break;snapshot=std::move(queue.front());queue.pop_front();
          if(snapshot.cloud) --cloud_jobs;condition.notify_all();}
        consume(snapshot);
      }
      flush();exportResults();
    } catch(...) {std::lock_guard<std::mutex> lock(mutex);failure=std::current_exception();stopping=true;condition.notify_all();}
  }
};
LoopClosureBackend::LoopClosureBackend(const LoopConfig &c,const ros::NodeHandle &n):impl_(new Impl(c,n)){}
LoopClosureBackend::~LoopClosureBackend(){try{finish();}catch(const std::exception &e){ROS_ERROR("Loop backend finalization: %s",e.what());}}
void LoopClosureBackend::submit(const FrontendSnapshot &input) {
  auto &p=*impl_;std::unique_lock<std::mutex> lock(p.mutex);
  if(p.cfg.offline) p.condition.wait(lock,[&]{return p.cloud_jobs<size_t(p.cfg.queue_limit)||p.stopping;});
  if(p.failure) std::rethrow_exception(p.failure);
  if(p.stopping) throw std::runtime_error("Submit to stopped loop backend");
  FrontendSnapshot copy=input;
  if(p.cloud_jobs>=size_t(p.cfg.queue_limit)) copy.cloud.reset();
  if(copy.cloud) ++p.cloud_jobs;
  p.queue.push_back(std::move(copy));p.max_queue=std::max(p.max_queue,p.queue.size());p.condition.notify_all();
}
void LoopClosureBackend::finish() {
  auto &p=*impl_;
  {std::lock_guard<std::mutex> lock(p.mutex);p.stopping=true;p.condition.notify_all();}
  if(p.worker.joinable()) p.worker.join();
  if(p.failure) std::rethrow_exception(p.failure);
}
} // namespace sa_loop
