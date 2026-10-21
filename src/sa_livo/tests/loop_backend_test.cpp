#include "loop_closure.h"
#include <ros/ros.h>
#include <filesystem>
#include <fstream>
#include <cassert>
#include <unistd.h>
#include <iostream>
#include <gtest/gtest.h>
#include "loop_test_room.h"
TEST(LoopBackend, DrainsAndPreservesPoses) {
  ros::NodeHandle nh;
  using namespace sa_loop;
  std::string out="/tmp/salivo-loop-backend-"+std::to_string(getpid());
  LoopConfig c;c.enabled=true;c.offline=true;c.queue_limit=1;c.output_dir=out;
  c.btc_config=std::string(ROOT_DIR)+"../btc_descriptor/config/config_indoor.yaml";
  c.exclusion_seconds=1000;
  {
    LoopClosureBackend backend(c,nh);
    for(int i=0;i<20;++i){FrontendSnapshot s;s.stamp=100+i*.2;s.pose.translation()<<i*.1,0,0;
      Cloud::Ptr cloud(new Cloud);s.cloud=cloud; // featureless data are a supported no-loop case
      backend.submit(s);assert(s.cloud && s.cloud->empty());}
    backend.finish();backend.finish();
  }
  auto lines=[](const std::string &file){std::ifstream in(file);assert(in);size_t count=0;std::string line;while(std::getline(in,line))++count;return count;};
  assert(lines(out+"/optimized_trajectory.tum")==20);
  assert(lines(out+"/online_trajectory.tum")==20);
  assert(lines(out+"/submaps.csv")>2);
  std::ifstream trajectory(out+"/optimized_trajectory.tum");double t,x,y,z,qx,qy,qz,qw;int i=0;
  while(trajectory>>t>>x>>y>>z>>qx>>qy>>qz>>qw){assert(std::abs(x-i*.1)<1e-8);assert(std::abs(t-(100+i*.2))<1e-8);++i;}
  assert(i==20);
  std::filesystem::remove_all(out);
  std::cout<<"Backend drained bounded queue, preserved raw poses, and finalized twice\n";
}
TEST(LoopBackend, AcceptsConsistentLoopAndOptimizes) {
  ros::NodeHandle nh;using namespace sa_loop;
  std::string out="/tmp/salivo-loop-positive-"+std::to_string(getpid());
  LoopConfig c;c.enabled=true;c.offline=true;c.queue_limit=1;c.output_dir=out;
  c.btc_config=std::string(ROOT_DIR)+"../btc_descriptor/config/config_indoor.yaml";
  c.exclusion_seconds=2;
  // The axis-aligned synthetic planes need >=10 samples in a 0.5 m BTC
  // voxel; preserve their known 0.1 m spacing. Validation gates stay unchanged.
  c.voxel=.1;
  Cloud::Ptr room=loopTestRoom();
  {
    LoopClosureBackend backend(c,nh);
    for(int i=0;i<16;++i) {
      FrontendSnapshot s;s.stamp=100+i*1.1;s.cloud=room;
      s.pose.translation()<<i*.01,0,0;s.covariance*=.01;
      backend.submit(s);
    }
    backend.finish();
  }
  std::ifstream edge_file(out+"/graph_edges.csv");std::string line;bool accepted=false;
  while(std::getline(edge_file,line)){
    std::istringstream row(line);std::string a,b,loop;
    std::getline(row,a,',');std::getline(row,b,',');std::getline(row,loop,',');
    if(loop=="1")accepted=true;
  }
  ASSERT_TRUE(accepted);
  std::ifstream poses(out+"/optimized_trajectory.tum");double t,x,y,z,qx,qy,qz,qw;double last_x=0;
  while(poses>>t>>x>>y>>z>>qx>>qy>>qz>>qw)last_x=x;
  EXPECT_LT(std::abs(last_x),.08);
  std::filesystem::remove_all(out);
}
TEST(LoopBackend, RealtimeOverflowKeepsPoseMetadata) {
  ros::NodeHandle nh;using namespace sa_loop;
  std::string out="/tmp/salivo-loop-overflow-"+std::to_string(getpid());
  LoopConfig c;c.enabled=true;c.offline=false;c.queue_limit=1;c.output_dir=out;
  c.btc_config=std::string(ROOT_DIR)+"../btc_descriptor/config/config_indoor.yaml";
  Cloud::Ptr room=loopTestRoom();
  {
    LoopClosureBackend backend(c,nh);
    for(int i=0;i<200;++i){FrontendSnapshot s;s.stamp=100+i*.1;s.cloud=room;s.pose.translation().x()=i*.01;backend.submit(s);}
    backend.finish();
  }
  std::ifstream summary(out+"/backend_summary.json");
  std::string text((std::istreambuf_iterator<char>(summary)),std::istreambuf_iterator<char>());
  auto key=text.find("\"skipped_frames\":");ASSERT_NE(key,std::string::npos);
  EXPECT_GT(std::stoi(text.substr(key+17)),0);
  EXPECT_NE(text.find("\"frames\":200"),std::string::npos);
  std::ifstream trajectory(out+"/optimized_trajectory.tum");std::string line;size_t count=0;
  while(std::getline(trajectory,line))++count;EXPECT_EQ(count,200u);
  std::filesystem::remove_all(out);
}
int main(int argc,char **argv) {
  testing::InitGoogleTest(&argc,argv);
  ros::init(argc,argv,"loop_backend_test");
  return RUN_ALL_TESTS();
}
