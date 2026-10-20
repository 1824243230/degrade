// SPDX-License-Identifier: GPL-2.0-only
#include "LIVMapper.h"
#include "localization_state.h"
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Cholesky>
#include <algorithm>
#include <filesystem>

void LIVMapper::readLocalizationParameters()
{
#define LOC_PARAM(field) node.param("localization/" #field,localization_config_.field,localization_config_.field)
  LOC_PARAM(enabled); LOC_PARAM(map_path); LOC_PARAM(metadata_path); LOC_PARAM(calibration_id); LOC_PARAM(frame_id);
  LOC_PARAM(map_sigma); LOC_PARAM(kiss_resolution); LOC_PARAM(fine_resolution); LOC_PARAM(query_radius);
  LOC_PARAM(tile_size); LOC_PARAM(tile_stride); LOC_PARAM(min_inliers); LOC_PARAM(min_query_points); LOC_PARAM(threads);
  LOC_PARAM(correspondence_distance); LOC_PARAM(overlap_min); LOC_PARAM(rmse_max); LOC_PARAM(gravity_max_deg);
  LOC_PARAM(observability_ratio); LOC_PARAM(verify_translation); LOC_PARAM(verify_rotation_deg);
  LOC_PARAM(ambiguity_overlap); LOC_PARAM(ambiguity_rmse); LOC_PARAM(match_timeout); LOC_PARAM(retry_delay); LOC_PARAM(max_attempts);
  LOC_PARAM(stationary_seconds); LOC_PARAM(stationary_frames); LOC_PARAM(stationary_translation); LOC_PARAM(stationary_rotation_deg);
  LOC_PARAM(gyro_max); LOC_PARAM(gyro_std_max); LOC_PARAM(acc_std_max); LOC_PARAM(sensor_timeout);
  LOC_PARAM(bad_seconds); LOC_PARAM(no_observation_seconds); LOC_PARAM(low_lidar_seconds); LOC_PARAM(good_seconds);
  LOC_PARAM(recovering_seconds); LOC_PARAM(recovering_frames); LOC_PARAM(tracking_min_matches); LOC_PARAM(good_min_matches);
  LOC_PARAM(tracking_min_ratio); LOC_PARAM(good_min_ratio); LOC_PARAM(bad_rms); LOC_PARAM(visual_q_min);
  LOC_PARAM(max_position_variance); LOC_PARAM(max_rotation_variance); LOC_PARAM(reset_rotation_std_deg);
  LOC_PARAM(reset_position_std); LOC_PARAM(reset_velocity_std); LOC_PARAM(reset_exposure_std);
  LOC_PARAM(reset_gyro_bias_std); LOC_PARAM(reset_acc_bias_std); LOC_PARAM(reset_gravity_std);
  LOC_PARAM(max_lidar_buffer); LOC_PARAM(max_imu_buffer);
#undef LOC_PARAM
  if(!localization_config_.enabled) return;
  localization_config_.validate();
  if(!lidar_en || !imu_en) throw std::invalid_argument("fixed-map localization requires LiDAR and IMU");
  if(localization_config_.map_path.empty() || localization_config_.metadata_path.empty())
    throw std::invalid_argument("localization/map_path and metadata_path are required");
  if(lag_smooth_>0) throw std::invalid_argument("fixed-map localization requires lio/lag_smooth=0");
  world_frame_=localization_config_.frame_id;
  pcd_save_en=false;
  velocity_init_en_=false;
  gravity_align_en=false;
}

void LIVMapper::initializeLocalization()
{
  if(!localization_config_.enabled) return;
  prior_map_=sa_localization::PriorMapManager::load(localization_config_);
  std::vector<V3D> points;
  points.reserve(prior_map_->cloud().size());
  for(const auto& p:prior_map_->cloud()) points.push_back(p.getVector3fMap().cast<double>());
  voxelmap_manager->loadPriorMap(points,localization_config_.map_sigma);
  voxelmap_manager->world_frame_=world_frame_;
  lidar_map_inited=true;
  localization_.reset(new sa_localization::RelocalizationManager(localization_config_,prior_map_,extR));
  localization_base_seq_=seq_name;
  localization_status_pub_=nh.advertise<sa_livo::LocalizationStatus>("/localization/status",10,true);
  localization_pose_pub_=nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/localization/pose",10);
  prior_map_pub_=nh.advertise<sensor_msgs::PointCloud2>("/localization/prior_map",1,true);
  relocalize_service_=nh.advertiseService("/localization/relocalize",&LIVMapper::requestRelocalization,this);
  sensor_msgs::PointCloud2 message;
  pcl::toROSMsg(prior_map_->cloud(),message);
  message.header.frame_id=world_frame_;
  message.header.stamp=ros::Time::now();
  prior_map_pub_.publish(message);
  publishLocalizationStatus(true);
  ROS_INFO("[ LOCALIZATION ] loaded %zu map points, %zu search tiles, version=%s",
    points.size(),prior_map_->tiles().size(),prior_map_->metadata().version.c_str());
}

bool LIVMapper::requestRelocalization(std_srvs::Trigger::Request&,std_srvs::Trigger::Response& response)
{
  localization_->request("manual relocalization request");
  LidarMeasures=LidarMeasureGroup(); lidar_pushed=false;
  response.success=true; response.message="request accepted; completion is reported on /localization/status";
  publishLocalizationStatus(true);
  return true;
}

void LIVMapper::publishLocalizationStatus(bool force)
{
  if(!localization_) return;
  const double wall=ros::WallTime::now().toSec();
  if(!force && wall-localization_status_wall_<0.2) return;
  localization_status_wall_=wall;
  sa_livo::LocalizationStatus message;
  message.header.stamp=ros::Time::now(); message.header.frame_id=world_frame_;
  message.state=static_cast<uint8_t>(localization_->state());
  message.state_name=sa_localization::stateName(localization_->state());
  message.valid=localization_->valid(); message.stop_required=!message.valid;
  message.reason=localization_->reason(); message.map_version=prior_map_->metadata().version;
  message.attempt_id=localization_->attemptId(); message.attempts=localization_->attempts();
  const auto& q=localization_->quality();
  message.source_stamp=sec2Stamp(std::max(0.,q.stamp));
  message.points=std::max(q.points,0); message.matches=std::max(q.matches,0); message.match_ratio=q.ratio();
  message.raw_residual_rms=q.raw_rms; message.gated_residual_rms=q.gated_rms;
  message.visual_quality=q.visual_q; message.visual_observations=std::max(q.visual_observations,0);
  message.position_variance=q.position_variance; message.rotation_variance=q.rotation_variance;
  message.correction_translation=q.correction_translation; message.correction_rotation=q.correction_rotation;
  message.degenerate=q.degenerate; message.information_eigenvalues=q.eigenvalues;
  localization_status_pub_.publish(message);
}

bool LIVMapper::processLocalization()
{
  if(!localization_) return false;
  const double ros_now=ros::Time::now().toSec();
  if(localization_ros_time_>=0 && ros_now<localization_ros_time_) localization_clock_reset_=true;
  localization_ros_time_=ros_now;
  if(localization_clock_reset_.exchange(false)) {
    std::scoped_lock lock(mtx_buffer,mtx_buffer_imu_prop);
    localization_->clockReset();
    lid_raw_data_buffer.clear(); lid_header_time_buffer.clear(); imu_buffer.clear(); prop_imu_buffer.clear();
    img_buffer.clear(); img_time_buffer.clear(); img2_buffer.clear(); img2_time_buffer.clear();
    LidarMeasures=LidarMeasureGroup(); lidar_pushed=false; localization_boundary_.reset();
    last_timestamp_lidar=last_timestamp_imu=last_timestamp_img=last_timestamp_img2=-1;
    first_lidar_header_time_=last_img_kept_time_=-1;
    ekf_finish_once=false;
  }
  localization_->poll();
  if(localization_buffer_overrun_.exchange(false)) {
    localization_->sensorWait("sensor queue overrun; discarded integration continuity");
    LidarMeasures=LidarMeasureGroup();lidar_pushed=false;
  }
  direct_localization_pause_=localization_->state()==sa_localization::State::RELOCALIZING;
  double lidar_stamp,imu_stamp;
  {
    std::lock_guard<std::mutex> lock(mtx_buffer);
    lidar_stamp=last_timestamp_lidar; imu_stamp=last_timestamp_imu;
  }
  const double now=direct_bag_path_.empty() ? ros::Time::now().toSec() : std::max(lidar_stamp,imu_stamp);
  if(lidar_stamp>=0 && imu_stamp>=0 && now>0 &&
      (now-lidar_stamp>localization_config_.sensor_timeout || now-imu_stamp>localization_config_.sensor_timeout)) {
    localization_->sensorWait("LiDAR or IMU stream is stale");
    LidarMeasures=LidarMeasureGroup(); lidar_pushed=false;
  }
  if(direct_bag_path_.empty() && localization_->processesTracking() && localization_->quality().stamp>0 &&
      now-localization_->quality().stamp>2*localization_config_.sensor_timeout) {
    localization_->sensorWait("no synchronized tracking updates despite incoming sensors");
    LidarMeasures=LidarMeasureGroup();lidar_pushed=false;
  }
  publishLocalizationStatus();
  if(localization_->processesTracking()) return false;
  sa_localization::Packet packet;
  {
    std::lock_guard<std::mutex> lock(mtx_buffer);
    if(lid_raw_data_buffer.empty() || imu_buffer.empty()) return true;
    const auto cloud=lid_raw_data_buffer.front();
    if(!cloud || cloud->size()<2) {
      lid_raw_data_buffer.pop_front(); lid_header_time_buffer.pop_front(); sig_buffer.notify_all(); return true;
    }
    double duration=0;
    for(const auto& p:*cloud) if(std::isfinite(p.curvature)) duration=std::max(duration,double(p.curvature)/1000.);
    packet.stamp=lid_header_time_buffer.front()+duration;
    if(imu_stamp<packet.stamp) return true;
    if(localization_boundary_) {
      const auto& m=*localization_boundary_;
      packet.imu.push_back({stamp2Sec(m.header.stamp),V3D(m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z),
        V3D(m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z)});
    }
    while(!imu_buffer.empty() && stamp2Sec(imu_buffer.front()->header.stamp)<=packet.stamp) {
      localization_boundary_=imu_buffer.front(); imu_buffer.pop_front();
      const auto& m=*localization_boundary_;
      packet.imu.push_back({stamp2Sec(m.header.stamp),V3D(m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z),
        V3D(m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z)});
    }
    packet.cloud.reserve(cloud->size());
    for(const auto& p:*cloud) packet.cloud.push_back(pcl::PointXYZ(p.x,p.y,p.z));
    lid_raw_data_buffer.pop_front(); lid_header_time_buffer.pop_front();
    while(!img_time_buffer.empty() && img_time_buffer.front()+exposure_time_init<=packet.stamp) { img_time_buffer.pop_front(); img_buffer.pop_front(); }
    while(!img2_time_buffer.empty() && img2_time_buffer.front()<=packet.stamp) { img2_time_buffer.pop_front(); img2_buffer.pop_front(); }
    sig_buffer.notify_all();
  }
  bool gap=packet.imu.empty();
  for(size_t i=1;i<packet.imu.size();++i) gap|=packet.imu[i].stamp-packet.imu[i-1].stamp>localization_config_.sensor_timeout;
  if(gap || packet.stamp-packet.imu.back().stamp>localization_config_.sensor_timeout) {
    localization_->sensorWait("missing IMU coverage for relocalization scan"); return true;
  }
  if(localization_->ingest(packet)) applyRelocalization(localization_->recovery());
  direct_localization_pause_=localization_->state()==sa_localization::State::RELOCALIZING;
  publishLocalizationStatus();
  return true;
}

void LIVMapper::applyRelocalization(const sa_localization::Recovery& recovery)
{
  if(!localization_boundary_) throw std::runtime_error("relocalization has no boundary IMU");
  Eigen::Isometry3d extrinsic=Eigen::Isometry3d::Identity(); extrinsic.linear()=extR; extrinsic.translation()=extT;
  const StatesGroup restored=makeRecoveredState(recovery,extrinsic,prior_map_->metadata().gravity,trusted_acc_bias_,localization_config_);
  std::scoped_lock lock(mtx_buffer,mtx_buffer_imu_prop);
  if(localization_clock_reset_) return;
  _state=state_propagat=latest_ekf_state=imu_propagate=restored;
  voxelmap_manager->resetTracking(); voxelmap_manager->state_=restored;
  p_imu->resetForRelocalization(recovery.stamp,localization_boundary_,restored,recovery.mean_acc);
  latest_ekf_time=recovery.stamp; last_imu_prop_dt_=0; ekf_finish_once=true; state_update_flg=true; new_imu=false;
  prop_imu_buffer.clear();
  LidarMeasures=LidarMeasureGroup(); LidarMeasures.last_lio_update_time=recovery.stamp; lidar_pushed=false;
  while(!lid_header_time_buffer.empty() && lid_header_time_buffer.front()<recovery.stamp) { lid_header_time_buffer.pop_front(); lid_raw_data_buffer.pop_front(); }
  while(!imu_buffer.empty() && stamp2Sec(imu_buffer.front()->header.stamp)<=recovery.stamp) imu_buffer.pop_front();
  while(!img_time_buffer.empty() && img_time_buffer.front()+exposure_time_init<=recovery.stamp) { img_time_buffer.pop_front(); img_buffer.pop_front(); }
  while(!img2_time_buffer.empty() && img2_time_buffer.front()<=recovery.stamp) { img2_time_buffer.pop_front(); img2_buffer.pop_front(); }
  if(sa_vio) sa_vio->reset(); if(sa_vio2) sa_vio2->reset(); if(vio_manager) vio_manager->resetTracking();
  _pv_list.clear(); visual_sub_map->clear(); feats_undistort->clear(); feats_down_body->clear(); feats_down_world->clear();
  pcl_w_wait_pub->clear(); pcl_wait_pub->clear(); pcl_rgb_wait_pub->clear();
  lag_poses_.clear(); lag_rel0_.clear(); ba_window_.clear(); ba_rel0_.clear(); ba_color_queue_.clear(); ba_ok_=false;
  pending_color_frame_=ColorFrameSnapshot(); have_prev_color_pose_=false; prev_color_t_=-1; prev_color_image_.release();
  have_last_good_att_=false; zupt_since_=-1; vinit_t0_=vinit_t1_=-1; velocity_init_done_=true;
  map_insert_counter_=0; gravity_align_finished=true; lidar_map_inited=true;
  path.poses.clear(); traj_lines_.clear(); traj_times_.clear();
  seq_name=localization_base_seq_+"_segment_"+std::to_string(++localization_segment_);
  localization_->committed(recovery.stamp);
  sig_buffer.notify_all();
  ROS_INFO("[ LOCALIZATION ] applied recovery at %.6f, RMSE %.3f overlap %.3f",recovery.stamp,recovery.match.rmse,recovery.match.overlap);
  publishLocalizationStatus(true);
}

sa_localization::Quality LIVMapper::getTrackingQuality(bool visual) const
{
  sa_localization::Quality q;
  q.stamp=measurementStamp().toSec(); q.points=feats_down_size;
  q.matches=voxelmap_manager->effct_feat_num_; q.degenerate=voxelmap_manager->degenerate_update_;
  q.eigenvalues=voxelmap_manager->fused_last_.eigenvals;
  q.visual_available=visual && sa_vio && sa_vio->windowSize()>=2;
  if(sa_vio) { q.visual_q=sa_vio->lastQuality(); q.visual_observations=sa_vio->numValidObs(); }
  q.position_variance=_state.cov.block<3,3>(StateIndex::Position,StateIndex::Position).trace();
  q.rotation_variance=_state.cov.block<3,3>(StateIndex::Rotation,StateIndex::Rotation).trace();
  q.finite=_state.rot_end.allFinite() && _state.pos_end.allFinite() && _state.vel_end.allFinite() &&
    _state.bias_g.allFinite() && _state.bias_a.allFinite() && _state.gravity.allFinite() &&
    std::isfinite(_state.inv_expo_time) && _state.cov.allFinite();
  if(q.finite) {
    Eigen::LDLT<Matrix<double,DIM_STATE,DIM_STATE>> factor(0.5*(_state.cov+_state.cov.transpose()));
    q.finite=factor.info()==Eigen::Success && factor.isPositive() &&
      std::abs(_state.rot_end.determinant()-1)<1e-3;
  }
  q.solved=!q.degenerate && (baseline_lio_state_estimation_ || voxelmap_manager->fused_last_.ok);
  q.correction_translation=(_state.pos_end-state_propagat.pos_end).norm();
  q.correction_rotation=sa_localization::rotationDistance(state_propagat.rot_end,_state.rot_end);
  double raw2=0,gated2=0; int n=0,ng=0;
  const auto& planes=voxelmap_manager->plane_cache_;
  for(size_t i=0;i<planes.size() && i<feats_down_body->size();++i) {
    if(!planes[i].valid) continue;
    const auto& p=(*feats_down_body)[i];
    const V3D world=_state.rot_end*(extR*V3D(p.x,p.y,p.z)+extT)+_state.pos_end;
    const double residual=planes[i].normal.dot(world)+planes[i].d;
    if(!std::isfinite(residual)) continue;
    raw2+=residual*residual; ++n;
    if(planes[i].sigma2>0 && residual*residual/planes[i].sigma2<=voxelmap_manager->config_setting_.chi2_thres_) { gated2+=residual*residual; ++ng; }
  }
  if(n) q.raw_rms=std::sqrt(raw2/n);
  if(ng) q.gated_rms=std::sqrt(gated2/ng);
  else if(baseline_lio_state_estimation_) q.gated_rms=q.raw_rms;
  return q;
}

bool LIVMapper::checkLocalizationQuality(bool visual)
{
  if(!localization_) return true;
  localization_->updateQuality(getTrackingQuality(visual));
  if(localization_->valid()) {
    if(_state.bias_a.allFinite() && _state.bias_a.norm()<2.) trusted_acc_bias_=_state.bias_a;
    localization_->setRecoveryHint(_state.pos_end);
  }
  if(!localization_->processesTracking()) { LidarMeasures=LidarMeasureGroup(); lidar_pushed=false; }
  publishLocalizationStatus(true);
  return localization_->valid() || localization_->state()==sa_localization::State::RECOVERING;
}

void LIVMapper::saveMapMetadata(const std::string& pcd_path) const
{
  if(!_state.gravity.allFinite() || std::abs(_state.gravity.norm()-G_m_s2)>0.5) {
    ROS_WARN("Map saved without metadata: gravity was not initialized"); return;
  }
  sa_localization::MapMetadata metadata;
  // map is a name for the saved world coordinates; no numerical rotation is applied.
  metadata.frame_id=localization_config_.frame_id;
  metadata.gravity=_state.gravity; metadata.calibration_id=localization_config_.calibration_id;
  metadata.version=seq_name+"_"+std::to_string(ros::WallTime::now().toNSec());
  metadata.save(pcd_path+".yaml");
}
