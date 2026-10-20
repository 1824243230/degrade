// SPDX-License-Identifier: GPL-2.0-only
// Offline VLP-16 decoding using the installed ROS1 Velodyne calibration/decoder.
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Imu.h>
#include <velodyne_pointcloud/rawdata.h>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>

class XYZIRT : public velodyne_rawdata::DataContainerBase {
public:
  explicit XYZIRT(unsigned int scans)
    : DataContainerBase(150.0, 2.0, "", "", 0, 1, true, scans, 6,
        "x", 1, sensor_msgs::PointField::FLOAT32,
        "y", 1, sensor_msgs::PointField::FLOAT32,
        "z", 1, sensor_msgs::PointField::FLOAT32,
        "intensity", 1, sensor_msgs::PointField::FLOAT32,
        "ring", 1, sensor_msgs::PointField::UINT16,
        "time", 1, sensor_msgs::PointField::FLOAT32) {}

  void newLine() override {}
  void addPoint(float x, float y, float z, uint16_t ring, uint16_t,
                float distance, float intensity, float time) override {
    if (!pointInRange(distance) || !std::isfinite(x) || !std::isfinite(y) ||
        !std::isfinite(z) || !std::isfinite(time) || time < 0 || ring >= 16) return;
    const size_t offset = cloud.width * cloud.point_step;
    if (offset + cloud.point_step > cloud.data.size())
      throw std::runtime_error("Decoder exceeded its packet point allocation");
    const float xyz_i[] = {x, y, z, intensity};
    std::memcpy(cloud.data.data()+offset, xyz_i, sizeof(xyz_i));
    std::memcpy(cloud.data.data()+offset+16, &ring, sizeof(ring));
    std::memcpy(cloud.data.data()+offset+18, &time, sizeof(time));
    ++cloud.width;
    min_time = std::min(min_time, double(time));
    max_time = std::max(max_time, double(time));
  }
  double min_time=std::numeric_limits<double>::infinity(), max_time=0;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "decode_velodyne_bag", ros::init_options::AnonymousName | ros::init_options::NoSigintHandler);
  ros::Time::init();  // Standalone rosbag I/O; no NodeHandle or ROS master needed.
  if (argc != 4) {
    std::cerr << "Usage: decode_velodyne_bag INPUT.bag OUTPUT.bag VLP16db.yaml\n";
    return 2;
  }
  const std::filesystem::path output(argv[2]), temporary(output.string()+".partial");
  if (std::filesystem::exists(output) || std::filesystem::exists(temporary)) {
    std::cerr << "Output already exists; choose a new output path\n";
    return 1;
  }
  try {
    velodyne_rawdata::RawData decoder;
    if (decoder.setupOffline(argv[3], "VLP16", 150.0, 2.0) != 0)
      throw std::runtime_error("Cannot initialize VLP16 calibration");
    // setupOffline does not execute the nodelet's dynamic-reconfigure callback.
    decoder.setParameters(2.0, 150.0, 0.0, 2.0*M_PI);
    XYZIRT points(decoder.scansPerPacket());
    rosbag::Bag input(argv[1], rosbag::bagmode::Read), result(temporary.string(), rosbag::bagmode::Write);
    result.setCompression(rosbag::compression::LZ4);
    rosbag::View records(input, rosbag::TopicQuery(std::vector<std::string>{"/velodyne_packets", "/imu/data"}));
    size_t scans=0, imus=0, count=0;
    double last_scan=-1, last_imu=-1, first=std::numeric_limits<double>::infinity(), last=0;
    for (const auto &record : records) {
      if (record.getTopic()=="/imu/data") {
        const auto imu=record.instantiate<sensor_msgs::Imu>();
        if (!imu) throw std::runtime_error("Unexpected IMU message type");
        const double stamp=imu->header.stamp.toSec();
        if (stamp<=last_imu) throw std::runtime_error("Non-increasing IMU header time");
        last_imu=stamp; first=std::min(first,stamp); last=std::max(last,stamp);
        result.write("/handsfree/imu", imu->header.stamp, *imu);
        ++imus;
      } else {
        const auto scan=record.instantiate<velodyne_msgs::VelodyneScan>();
        if (!scan || scan->packets.empty()) throw std::runtime_error("Missing Velodyne packets");
        const double stamp=scan->header.stamp.toSec();
        if (stamp<=last_scan) throw std::runtime_error("Non-increasing scan header time");
        last_scan=stamp; first=std::min(first,stamp); last=std::max(last,stamp);
        points.setup(scan);
        for (const auto &packet : scan->packets) {
          if (packet.data[1205]!=0x22) throw std::runtime_error("This converter requires VLP16 packets (model 0x22)");
          decoder.unpack(packet, points, scan->header.stamp);
        }
        auto cloud=points.finishCloud();
        cloud.header=scan->header;
        if (cloud.width<2) throw std::runtime_error("Decoded empty scan");
        result.write("/velodyne_points", cloud.header.stamp, cloud);
        count+=cloud.width;
        if (++scans % 250 == 0) std::cout << "decoded " << scans << " scans\n";
      }
    }
    if (!scans || !imus || points.max_time>0.3) throw std::runtime_error("Invalid decoded stream counts/timing");
    result.close(); input.close();
    std::filesystem::rename(temporary, output);
    std::ofstream stats(output.string()+".json");
    stats << std::setprecision(16) << "{\n  \"scans\": " << scans << ",\n  \"imus\": " << imus
      << ",\n  \"points\": " << count << ",\n  \"start\": " << first << ",\n  \"end\": " << last
      << ",\n  \"min_point_time_s\": " << points.min_time << ",\n  \"max_point_time_s\": " << points.max_time
      << ",\n  \"model\": \"VLP16\",\n  \"range_m\": [2, 150],\n  \"images_used\": 0,\n"
      << "  \"record_time\": \"sensor header time; original record epoch is different\"\n}\n";
    if (!stats) throw std::runtime_error("Cannot write conversion statistics");
    std::cout << "Decoded scans=" << scans << " imu=" << imus << " points=" << count << '\n';
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    std::filesystem::remove(temporary);
    return 1;
  }
  return 0;
}
