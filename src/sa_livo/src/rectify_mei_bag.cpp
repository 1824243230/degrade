// Offline normalization. Uses the same OpenCV 4.2 ABI as the LIVO frontend.
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <livox_ros_driver2/CustomMsg.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/ccalib/omnidir.hpp>
#include <opencv2/imgcodecs.hpp>
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <iostream>
#include <array>

int main(int argc,char **argv) {
  ros::init(argc,argv,"rectify_mei_bag",ros::init_options::AnonymousName|ros::init_options::NoSigintHandler);
  ros::Time::init();
  if(argc!=4){std::cerr<<"Usage: rectify_mei_bag INPUT.bag OUTPUT.bag preparation.yaml\n";return 2;}
  std::filesystem::path target(argv[2]),temporary(std::string(argv[2])+".partial");
  if(std::filesystem::exists(target)||std::filesystem::exists(temporary)){std::cerr<<"Output exists\n";return 1;}
  try {
    auto cfg=YAML::LoadFile(argv[3]);
    if(!cfg["calibration_confirmed"].as<bool>()) throw std::runtime_error("Sensor calibration must be confirmed");
    const auto camera=cfg["mei"],virt=cfg["virtual_camera"];
    int width=camera["width"].as<int>(),height=camera["height"].as<int>();
    int out_width=virt["width"].as<int>(),out_height=virt["height"].as<int>();
    auto vec=[](YAML::Node n){std::vector<double> v=n.as<std::vector<double>>();cv::Mat m(v,true);return m.clone();};
    cv::Mat K=vec(camera["K"]).reshape(1,3).clone(),D=vec(camera["D"]),xi=vec(camera["xi"]);
    cv::Mat P=vec(virt["K"]).reshape(1,3).clone(),mapx,mapy;
    cv::omnidir::initUndistortRectifyMap(K,D,xi,cv::Mat::eye(3,3,CV_64F),P,
        cv::Size(out_width,out_height),CV_32FC1,mapx,mapy,cv::omnidir::RECTIFY_PERSPECTIVE);
    cv::Mat mask(out_height,out_width,CV_8U,cv::Scalar(0));
    for(int y=0;y<out_height;++y) for(int x=0;x<out_width;++x) {
      float u=mapx.at<float>(y,x),v=mapy.at<float>(y,x);
      if(std::isfinite(u)&&std::isfinite(v)&&u>=0&&v>=0&&u<width-1&&v<height-1) mask.at<uint8_t>(y,x)=255;
    }
    if(!cv::imwrite((target.parent_path()/"valid_mask.png").string(),mask)) throw std::runtime_error("Cannot write valid mask");
    std::string lidar_topic=cfg["lidar_topic"].as<std::string>(),imu_topic=cfg["imu_topic"].as<std::string>(),image_topic=cfg["image_topic"].as<std::string>();
    double scale=cfg["imu_acc_scale"].as<double>();
    if(!std::isfinite(scale)||scale<=0) throw std::runtime_error("Invalid IMU scale");
    double rate=cfg["image_rate_hz"].as<double>();
    if(!std::isfinite(rate)||rate<=0) throw std::runtime_error("Invalid image rate");
    rosbag::Bag input(argv[1],rosbag::bagmode::Read),output(temporary.string(),rosbag::bagmode::Write);
    output.setCompression(rosbag::compression::LZ4);
    rosbag::View view(input,rosbag::TopicQuery({lidar_topic,imu_topic,image_topic}));
    size_t lidars=0,imus=0,images=0;long long slot=-1;std::array<double,3> last{{-1,-1,-1}};
    for(const auto &record:view) {
      if(record.getTopic()==imu_topic) {
        auto m=record.instantiate<sensor_msgs::Imu>();if(!m) throw std::runtime_error("Wrong IMU type");
        if(m->header.stamp.toSec()<=last[0]) throw std::runtime_error("Nonmonotonic IMU");last[0]=m->header.stamp.toSec();
        auto normalized=*m;normalized.linear_acceleration.x*=scale;normalized.linear_acceleration.y*=scale;normalized.linear_acceleration.z*=scale;
        for(double &v:normalized.linear_acceleration_covariance) if(v>=0) v*=scale*scale;
        output.write("/imu/data",m->header.stamp,normalized);++imus;
      } else if(record.getTopic()==lidar_topic) {
        if(record.getMD5Sum()==ros::message_traits::MD5Sum<livox_ros_driver2::CustomMsg>::value()) {
          // ROS1 instantiate checks MD5; v1/v2 wire definitions are identical.
          auto m=record.instantiate<livox_ros_driver2::CustomMsg>();if(!m) throw std::runtime_error("Wrong Livox type");
          if(m->header.stamp.toSec()<=last[1]) throw std::runtime_error("Nonmonotonic LiDAR");last[1]=m->header.stamp.toSec();
          auto normalized=*m;normalized.timebase=m->header.stamp.toNSec();
          output.write("/livox/lidar",m->header.stamp,normalized);
        } else {
          auto m=record.instantiate<sensor_msgs::PointCloud2>();if(!m) throw std::runtime_error("Expected decoded PointCloud2 or Livox CustomMsg");
          if(m->header.stamp.toSec()<=last[1]) throw std::runtime_error("Nonmonotonic LiDAR");last[1]=m->header.stamp.toSec();
          output.write("/velodyne_points",m->header.stamp,*m);
        }
        ++lidars;
      } else {
        std_msgs::Header header;cv::Mat image;
        if(auto m=record.instantiate<sensor_msgs::Image>()) {header=m->header;image=cv_bridge::toCvCopy(m,"bgr8")->image;}
        else if(auto m=record.instantiate<sensor_msgs::CompressedImage>()) {header=m->header;image=cv::imdecode(m->data,cv::IMREAD_COLOR);}
        else throw std::runtime_error("Wrong image type");
        if(image.empty()||image.cols!=width||image.rows!=height) throw std::runtime_error("Image dimensions do not match MEI calibration");
        double t=header.stamp.toSec();if(t<=last[2]) throw std::runtime_error("Nonmonotonic image");last[2]=t;
        const long long current=std::floor(t*rate+1e-6);if(current<=slot) continue;slot=current;
        cv::Mat rectified;cv::remap(image,rectified,mapx,mapy,cv::INTER_LINEAR,cv::BORDER_CONSTANT);
        auto m=cv_bridge::CvImage(header,"bgr8",rectified).toImageMsg();
        output.write("/camera_1/image_rect",header.stamp,*m);++images;
        if(images==1 || images%700==0) cv::imwrite((target.parent_path()/("rectified_"+std::to_string(images)+".png")).string(),rectified);
      }
    }
    if(!lidars||!imus||!images) throw std::runtime_error("Missing sensor stream");
    output.close();input.close();std::filesystem::rename(temporary,target);
    std::cout<<"Normalized lidar="<<lidars<<" imu="<<imus<<" images="<<images<<"\n";
  } catch(const std::exception &e) {std::cerr<<e.what()<<'\n';std::filesystem::remove(temporary);return 1;}
  return 0;
}
