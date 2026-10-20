// SPDX-License-Identifier: GPL-2.0-only
#include "localization.h"
#include <pcl/PCLPointCloud2.h>
#include <pcl/common/common.h>
#include <pcl/conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <map>
#include <cstring>
#include <stdexcept>

namespace sa_localization {
void Config::validate() const {
  const double positive[] = {map_sigma, kiss_resolution, fine_resolution, query_radius,
    tile_size, tile_stride, correspondence_distance, rmse_max, gravity_max_deg, observability_ratio,
    verify_translation, verify_rotation_deg, ambiguity_overlap, ambiguity_rmse, match_timeout,
    retry_delay, stationary_seconds, stationary_translation, stationary_rotation_deg, gyro_max,
    gyro_std_max, acc_std_max, sensor_timeout, bad_seconds, no_observation_seconds, low_lidar_seconds,
    good_seconds, recovering_seconds, tracking_min_ratio, good_min_ratio, bad_rms, visual_q_min,
    max_position_variance, max_rotation_variance, reset_rotation_std_deg, reset_position_std,
    reset_velocity_std, reset_exposure_std, reset_gyro_bias_std, reset_acc_bias_std, reset_gravity_std};
  for (double value : positive)
    if (!std::isfinite(value) || value <= 0) throw std::invalid_argument("localization parameters must be finite and positive");
  if (kiss_resolution < 0.005 || tile_size < tile_stride + 2 * query_radius || tile_size/tile_stride>4 ||
      overlap_min <= 0 || overlap_min > 1 || !std::isfinite(overlap_min) ||
      good_min_ratio > 1 || tracking_min_ratio > good_min_ratio || threads < 1 ||
      min_inliers < 3 || min_query_points < min_inliers || max_attempts < 1 ||
      stationary_frames < 2 || recovering_frames < 1 || max_lidar_buffer < 2 || max_imu_buffer < 2 ||
      tracking_min_matches < 1 || good_min_matches < tracking_min_matches ||
      frame_id.empty() || calibration_id.empty())
    throw std::invalid_argument("invalid localization configuration (including tile coverage)");
}

MapMetadata MapMetadata::load(const std::string& path) {
  const auto yaml = YAML::LoadFile(path);
  if (yaml["units"].as<std::string>() != "m") throw std::invalid_argument("map units must be m");
  MapMetadata result;
  result.frame_id = yaml["frame_id"].as<std::string>();
  result.version = yaml["version"].as<std::string>();
  result.calibration_id = yaml["calibration_id"].as<std::string>();
  const auto gravity = yaml["gravity"].as<std::vector<double>>();
  if (gravity.size() != 3) throw std::invalid_argument("map gravity requires three components");
  result.gravity = Eigen::Vector3d(gravity[0], gravity[1], gravity[2]);
  if (!result.gravity.allFinite() || std::abs(result.gravity.norm() - 9.81) > 0.5 ||
      result.frame_id.empty() || result.version.empty() || result.calibration_id.empty())
    throw std::invalid_argument("invalid map metadata");
  return result;
}
void MapMetadata::save(const std::string& path) const {
  YAML::Emitter yaml;
  yaml << YAML::BeginMap << YAML::Key << "units" << YAML::Value << "m"
       << YAML::Key << "frame_id" << YAML::Value << frame_id
       << YAML::Key << "version" << YAML::Value << version
       << YAML::Key << "calibration_id" << YAML::Value << calibration_id
       << YAML::Key << "gravity" << YAML::Value << YAML::Flow << YAML::BeginSeq
       << gravity.x() << gravity.y() << gravity.z() << YAML::EndSeq << YAML::EndMap;
  std::ofstream stream(path);
  if (!stream || !(stream << yaml.c_str() << '\n')) throw std::runtime_error("cannot save map metadata: " + path);
}
Cloud filteredCloud(const Cloud& input, double resolution, double radius) {
  Cloud::Ptr finite(new Cloud);
  finite->reserve(input.size());
  for (const auto& point : input) {
    const auto p = point.getVector3fMap();
    if (p.allFinite() && (radius <= 0 || p.squaredNorm() <= radius * radius)) finite->push_back(point);
  }
  if (finite->empty()) return *finite;
  pcl::VoxelGrid<pcl::PointXYZ> filter;
  filter.setLeafSize(resolution, resolution, resolution);
  filter.setInputCloud(finite);
  Cloud result;
  filter.filter(result);
  return result;
}
PriorMapManager::PriorMapManager(const Config& config, const MapMetadata& metadata, const Cloud& input)
    : metadata_(metadata) {
  config.validate();
  if(metadata.version.empty() || !metadata.gravity.allFinite() || std::abs(metadata.gravity.norm()-9.81)>0.5)
    throw std::invalid_argument("invalid prior map gravity/version");
  if (metadata.frame_id != config.frame_id || metadata.calibration_id != config.calibration_id)
    throw std::invalid_argument("map frame/calibration_id does not match localization configuration");
  cloud_.reset(new Cloud(filteredCloud(input, config.fine_resolution)));
  if (cloud_->size() < size_t(config.min_query_points)) throw std::invalid_argument("prior map is empty or too small");
  pcl::PointXYZ low, high;
  pcl::getMinMax3D(*cloud_, low, high);
  const Eigen::Vector3d min = low.getVector3fMap().cast<double>();
  const Eigen::Vector3d max = high.getVector3fMap().cast<double>();
  if ((max - min).maxCoeff() <= config.tile_size) {
    tiles_.push_back(Cloud::ConstPtr(new Cloud(filteredCloud(*cloud_, config.kiss_resolution))));
    return;
  }
  // A grid cell is covered by every centre within query_radius of its boundary.
  // Size >= stride + 2*query_radius guarantees coverage of a query-sized sphere.
  using Key = std::tuple<int, int, int>;
  std::map<Key, Cloud::Ptr> buckets;
  const double half = config.tile_size * 0.5;
  const Cloud coarse=filteredCloud(*cloud_,config.kiss_resolution);
  for (const auto& point : coarse) {
    const Eigen::Vector3d p = point.getVector3fMap().cast<double>();
    Eigen::Array3i first = ((p.array() - half - min.array()) / config.tile_stride).ceil().cast<int>();
    Eigen::Array3i last = ((p.array() + half - min.array()) / config.tile_stride).floor().cast<int>();
    for (int x = first.x(); x <= last.x(); ++x)
      for (int y = first.y(); y <= last.y(); ++y)
        for (int z = first.z(); z <= last.z(); ++z) {
          auto& bucket = buckets[Key(x,y,z)];
          if (!bucket) bucket.reset(new Cloud);
          bucket->push_back(point);
        }
  }
  for (const auto& bucket : buckets) {
    auto tile = filteredCloud(*bucket.second, config.kiss_resolution);
    if (tile.size() >= size_t(config.min_query_points)) tiles_.push_back(Cloud::ConstPtr(new Cloud(std::move(tile))));
  }
  if (tiles_.empty()) throw std::invalid_argument("no usable map tiles");
}
std::shared_ptr<PriorMapManager> PriorMapManager::load(const Config& config) {
  pcl::PCLPointCloud2 blob;
  if (pcl::io::loadPCDFile(config.map_path, blob) < 0) throw std::runtime_error("cannot load prior map: " + config.map_path);
  for (const std::string name : {"x", "y", "z"}) {
    bool found = false;
    for (const auto& field : blob.fields) found |= field.name == name && field.count == 1 &&
      (field.datatype == pcl::PCLPointField::FLOAT32 || field.datatype == pcl::PCLPointField::FLOAT64);
    if (!found) throw std::invalid_argument("PCD requires floating point x/y/z fields");
  }
  Cloud cloud;
  // Decode XYZ explicitly: PCL's typed conversion does not convert FLOAT64 fields to FLOAT32.
  const pcl::PCLPointField* fields[3]={nullptr,nullptr,nullptr};
  for(const auto& field:blob.fields) {
    if(field.name=="x")fields[0]=&field;
    if(field.name=="y")fields[1]=&field;
    if(field.name=="z")fields[2]=&field;
  }
  if(blob.is_bigendian)throw std::invalid_argument("big endian PCD is not supported");
  cloud.reserve(size_t(blob.width)*blob.height);
  for(size_t row=0;row<blob.height;++row)for(size_t col=0;col<blob.width;++col) {
    pcl::PointXYZ point;
    float* values[3]={&point.x,&point.y,&point.z};
    for(int i=0;i<3;++i) {
      const size_t offset=row*blob.row_step+col*blob.point_step+fields[i]->offset;
      const size_t bytes=fields[i]->datatype==pcl::PCLPointField::FLOAT64 ? 8 : 4;
      if(offset+bytes>blob.data.size())throw std::invalid_argument("truncated PCD data");
      if(bytes==8){double value;std::memcpy(&value,blob.data.data()+offset,8);*values[i]=float(value);}
      else std::memcpy(values[i],blob.data.data()+offset,4);
    }
    cloud.push_back(point);
  }
  return std::make_shared<PriorMapManager>(config, MapMetadata::load(config.metadata_path), cloud);
}
double rotationDistance(const Eigen::Matrix3d& a, const Eigen::Matrix3d& b) {
  return Eigen::AngleAxisd(a.transpose() * b).angle();
}
bool validTransform(const Eigen::Isometry3d& t) {
  return t.matrix().allFinite() && std::abs(t.linear().determinant() - 1.0) < 1e-4 &&
    (t.linear().transpose() * t.linear() - Eigen::Matrix3d::Identity()).norm() < 1e-4;
}
} // namespace sa_localization
