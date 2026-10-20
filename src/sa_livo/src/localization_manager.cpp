// SPDX-License-Identifier: GPL-2.0-only
#include "localization.h"
#include <pcl/common/transforms.h>
#include <algorithm>

namespace sa_localization {
const char* stateName(State state) {
  static const char* names[] = {"WAIT_MAP", "WAIT_SENSOR", "WAIT_STATIONARY", "RELOCALIZING", "VERIFYING",
    "RECOVERING", "TRACKING", "DEGRADED", "LOST", "FAILED"};
  return names[static_cast<unsigned>(state)];
}
void TrackingQualityMonitor::reset() {
  last_=bad_since_=empty_since_=low_since_=good_since_=-1;
  good_frames_=0;
  reason_.clear();
}
State TrackingQualityMonitor::update(const Quality& q, State current) {
  if (!q.finite || !std::isfinite(q.position_variance) || !std::isfinite(q.rotation_variance) ||
      !std::isfinite(q.stamp) || (last_>=0 && q.stamp<last_)) { reason_="non-finite state or timestamp rollback"; return State::LOST; }
  if (q.stamp == last_) return current;
  last_=q.stamp;
  const bool low = q.matches < config_.tracking_min_matches || q.ratio() < config_.tracking_min_ratio;
  const bool visual_low = q.visual_available && q.visual_q < config_.visual_q_min;
  const bool residual_bad = !std::isfinite(q.raw_rms) || q.raw_rms > config_.bad_rms;
  const bool uncertain = q.position_variance > config_.max_position_variance || q.rotation_variance > config_.max_rotation_variance;
  const int signals = int(low) + int(residual_bad) + int(visual_low) + int(!q.solved || uncertain);
  const bool empty = q.matches==0 && (!q.visual_available || q.visual_observations==0 || visual_low);
  auto timer = [&](bool active,double& since) { if (!active) since=-1; else if(since<0) since=q.stamp; };
  timer(signals>=2,bad_since_); timer(empty,empty_since_); timer(low,low_since_);
  const bool good = q.solved && !uncertain && q.matches>=config_.good_min_matches &&
    q.ratio()>=config_.good_min_ratio && std::isfinite(q.raw_rms) && q.raw_rms<=config_.rmse_max;
  timer(good,good_since_);
  good_frames_ = good ? good_frames_+1 : 0;
  auto elapsed = [&](double since,double duration) { return since>=0 && q.stamp-since+1e-9>=duration; };
  if (elapsed(bad_since_,config_.bad_seconds) || elapsed(empty_since_,config_.no_observation_seconds) || elapsed(low_since_,config_.low_lidar_seconds)) {
    reason_="persistent loss of map tracking"; return State::LOST;
  }
  if (current==State::RECOVERING) {
    if (!good) { reason_="recovery observation failed"; return State::LOST; }
    if (good_frames_>=config_.recovering_frames && elapsed(good_since_,config_.recovering_seconds)) {
      reason_="tracking recovered"; return State::TRACKING;
    }
    return State::RECOVERING;
  }
  if (signals>=2 || low) { reason_="suspect map tracking"; return State::DEGRADED; }
  if (current==State::DEGRADED && !elapsed(good_since_,config_.good_seconds)) return current;
  reason_="tracking";
  return State::TRACKING;
}

RelocalizationManager::RelocalizationManager(const Config& config, std::shared_ptr<const PriorMapManager> map,
    const Eigen::Matrix3d& rotation, MatchFunction match_function)
    : config_(config),map_(std::move(map)),R_imu_lidar_(rotation),matcher_(config),
      match_function_(std::move(match_function)),monitor_(config) {
  config_.validate();
  retry_after_=std::chrono::steady_clock::now();
}
RelocalizationManager::~RelocalizationManager() {
  if(cancel_) cancel_->store(true);
  if(future_.valid()) future_.wait();
}
bool RelocalizationManager::processesTracking() const {
  return state_==State::TRACKING || state_==State::DEGRADED || state_==State::RECOVERING;
}
void RelocalizationManager::clearStationary() {
  anchor_.reset(); collected_.reset(); anchor_stamp_=-1; collected_frames_=verified_frames_=0; imu_window_.clear();
}
void RelocalizationManager::invalidate() {
  ++generation_;
  if(cancel_) cancel_->store(true);
  clearStationary();
}
void RelocalizationManager::request(const std::string& reason) {
  invalidate(); monitor_.reset(); attempts_=0;
  state_=State::WAIT_STATIONARY; reason_=reason; retry_after_=std::chrono::steady_clock::now();
}
void RelocalizationManager::clockReset() {
  request("timestamp rollback: discarded old task and history"); last_stamp_=-1;
}
void RelocalizationManager::sensorWait(const std::string& reason) {
  if(state_==State::WAIT_SENSOR || state_==State::FAILED) return;
  invalidate(); monitor_.reset(); state_=State::WAIT_SENSOR; reason_=reason;
}
void RelocalizationManager::failed(const std::string& reason) {
  invalidate(); reason_=reason;
  state_=attempts_>=config_.max_attempts ? State::FAILED : State::WAIT_STATIONARY;
  retry_after_=std::chrono::steady_clock::now()+std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(config_.retry_delay));
}
void RelocalizationManager::poll() {
  if(!future_.valid()) return;
  if(future_.wait_for(std::chrono::seconds(0))==std::future_status::ready) {
    MatchResult result;
    try { result=future_.get(); } catch(const std::exception& e) { result.reason=std::string("matcher exception: ")+e.what(); }
    if(launched_generation_!=generation_ || state_!=State::RELOCALIZING) return;
    if(std::chrono::duration<double>(std::chrono::steady_clock::now()-launched_at_).count()>config_.match_timeout) { failed("matching task expired"); return; }
    if(!result.success) { failed(result.reason); return; }
    candidate_=result; state_=State::VERIFYING; reason_="waiting for independent scans"; verified_frames_=0; verify_stamp_=launch_stamp_;
  } else if(state_==State::RELOCALIZING && std::chrono::duration<double>(std::chrono::steady_clock::now()-launched_at_).count()>config_.match_timeout) {
    failed("matching task timed out; waiting for worker to finish");
  }
}
bool RelocalizationManager::stationary(const Packet& packet,Eigen::Isometry3d& transform) {
  for(const auto& imu:packet.imu) {
    if(!imu.acceleration.allFinite() || !imu.gyro.allFinite()) return false;
    if(imu_window_.empty() || imu.stamp>imu_window_.back().stamp) imu_window_.push_back(imu);
  }
  while(!imu_window_.empty() && imu_window_.front().stamp<packet.stamp-config_.stationary_seconds) imu_window_.pop_front();
  if(imu_window_.size()<3) return false;
  mean_acc_.setZero(); mean_gyro_.setZero();
  for(const auto& imu:imu_window_) { mean_acc_+=imu.acceleration; mean_gyro_+=imu.gyro; }
  mean_acc_/=imu_window_.size(); mean_gyro_/=imu_window_.size();
  double avar=0,gvar=0;
  for(const auto& imu:imu_window_) { avar+=(imu.acceleration-mean_acc_).squaredNorm(); gvar+=(imu.gyro-mean_gyro_).squaredNorm(); }
  if(mean_acc_.norm()<1e-3 || mean_gyro_.norm()>config_.gyro_max ||
      std::sqrt(avar/imu_window_.size())>config_.acc_std_max || std::sqrt(gvar/imu_window_.size())>config_.gyro_std_max) return false;
  if(!anchor_) {
    anchor_.reset(new Cloud(filteredCloud(packet.cloud,config_.fine_resolution,config_.query_radius)));
    if(anchor_->size()<size_t(config_.min_query_points)) { anchor_.reset(); return false; }
    collected_.reset(new Cloud(*anchor_)); anchor_stamp_=packet.stamp; collected_frames_=1;
    transform.setIdentity(); return true;
  }
  return matcher_.stationaryTransform(packet.cloud,*anchor_,transform);
}
bool RelocalizationManager::ingest(const Packet& input) {
  poll();
  if(input.stamp<last_stamp_) clockReset();
  if(input.stamp==last_stamp_ || state_==State::FAILED || processesTracking()) return false;
  last_stamp_=input.stamp;
  if(state_==State::WAIT_SENSOR || state_==State::LOST) { state_=State::WAIT_STATIONARY; reason_="waiting for stationary LiDAR and IMU"; }
  Packet packet=input;
  packet.cloud=filteredCloud(input.cloud,config_.fine_resolution,config_.query_radius);
  Eigen::Isometry3d relative=Eigen::Isometry3d::Identity();
  if(!stationary(packet,relative)) {
    const bool matching=state_==State::RELOCALIZING || state_==State::VERIFYING;
    invalidate();
    // Keep current IMUs for the next packet rather than requiring callback batching to provide a full window.
    for(const auto& imu:packet.imu) imu_window_.push_back(imu);
    if(matching) { state_=State::WAIT_STATIONARY; reason_="motion invalidated matching task"; }
    return false;
  }
  if(state_==State::VERIFYING) {
    if(packet.stamp<=verify_stamp_) return false;
    auto result=matcher_.refine(packet.cloud,map_->cloud(),candidate_.T_map_lidar*relative);
    const Eigen::Vector3d gravity_lidar=R_imu_lidar_.transpose()*(-mean_acc_.normalized());
    const double cosine=std::max(-1.,std::min(1.,(result.T_map_lidar.linear()*gravity_lidar).dot(map_->metadata().gravity.normalized())));
    if(!result.success || std::acos(cosine)>config_.gravity_max_deg*kPi/180. ||
      (verified_frames_>0 && ((result.T_map_lidar.translation()-previous_verification_.T_map_lidar.translation()).norm()>config_.verify_translation ||
       rotationDistance(result.T_map_lidar.linear(),previous_verification_.T_map_lidar.linear())>config_.verify_rotation_deg*kPi/180.))) {
      failed("independent scan verification failed: "+result.reason); return false;
    }
    previous_verification_=result; verify_stamp_=packet.stamp;
    if(++verified_frames_<2) return false;
    recovery_={result,packet.stamp,mean_acc_,mean_gyro_};
    return true;
  }
  if(state_!=State::WAIT_STATIONARY) return false;
  if(collected_frames_<config_.stationary_frames || packet.stamp-anchor_stamp_<config_.stationary_seconds) {
    if(packet.stamp>anchor_stamp_) {
      Cloud transformed;
      pcl::transformPointCloud(packet.cloud,transformed,relative.matrix().cast<float>());
      *collected_+=transformed;
      *collected_=filteredCloud(*collected_,config_.fine_resolution);
      ++collected_frames_;
    }
    return false;
  }
  if(future_.valid() || std::chrono::steady_clock::now()<retry_after_) return false;
  ++attempts_; ++generation_; launched_generation_=generation_; launch_stamp_=packet.stamp;
  cancel_=std::make_shared<std::atomic<bool>>(false); launched_at_=std::chrono::steady_clock::now();
  auto source=std::make_shared<const Cloud>(*collected_); auto map=map_; auto token=cancel_; auto config=config_;
  auto function=match_function_; const Eigen::Vector3d gravity=relative.linear()*R_imu_lidar_.transpose()*(-mean_acc_.normalized());
  const Eigen::Vector3d hint=hint_; const bool have_hint=have_hint_;
  future_=std::async(std::launch::async,[source,map,token,config,function,gravity,hint,have_hint]() {
    if(function) return function(*source,gravity,*token);
    return KissRelocalizer(config).match(*source,*map,gravity,*token,have_hint ? &hint : nullptr);
  });
  state_=State::RELOCALIZING; reason_="searching complete prior map";
  return false;
}
void RelocalizationManager::committed(double stamp) {
  state_=State::RECOVERING; reason_="observing recovered LiDAR tracking"; monitor_.reset(); clearStationary();
  quality_=Quality();quality_.stamp=stamp;
}
void RelocalizationManager::updateQuality(const Quality& q) {
  quality_=q;
  if(!processesTracking()) return;
  const auto next=monitor_.update(q,state_);
  state_=next; reason_=monitor_.reason();
  if(next==State::LOST) { invalidate(); attempts_=0; retry_after_=std::chrono::steady_clock::now(); }
  if(next==State::TRACKING) {
    attempts_=0;
    // The mapper supplies a hint from its last trusted pose, see setRecoveryHint.
  }
}
} // namespace sa_localization
