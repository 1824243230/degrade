#include "LIVMapper.h"
#include <opencv2/imgcodecs.hpp>
#include <Eigen/Eigenvalues>
#include <filesystem>
#include <iomanip>

void LIVMapper::initializeLoopClosure()
{
  if(evaluation_output_dir_.empty())
    evaluation_output_dir_=std::string(ROOT_DIR)+"Log/result/"+seq_name;
  if(!image_valid_mask_.empty()) {
    cv::Mat mask=cv::imread(image_valid_mask_,cv::IMREAD_GRAYSCALE);
    if(mask.empty() || !sa_vio) throw std::runtime_error("Invalid image valid mask");
    sa_vio->setValidMask(mask);
  }
  auto cfg=sa_loop::loadLoopConfig(node,evaluation_output_dir_,!direct_bag_path_.empty());
  if(cfg.enabled && (slam_mode_!=LIVO || localization_ || lag_smooth_>0 || sa_ba_en_ ||
      use_baseline_vio_frontend_ || use_dual_event_))
    throw std::invalid_argument("BTC requires single-joint SA-LIVO mapping without fixed-map/lag/BA diagnostic modes");
  if(evaluation_enabled_ || cfg.enabled) {
    std::filesystem::create_directories(evaluation_output_dir_);
    if(std::filesystem::exists(evaluation_output_dir_+"/raw_trajectory.tum"))
      throw std::runtime_error("Evaluation output exists; choose a new output directory");
    livo_trajectory_stream_.open(evaluation_output_dir_+"/raw_trajectory.tum");
    livo_quality_stream_.open(evaluation_output_dir_+"/livo_quality.csv");
    if(!livo_trajectory_stream_ || !livo_quality_stream_) throw std::runtime_error("Cannot open LIVO evaluation output");
    livo_quality_stream_<<"timestamp,points,matches,raw_rms,gated_rms,degenerate,map_blocked,finite,frontend_ms,visual_valid,visual_rejected,visual_rms,q,visual_information_norm";
    for(const auto *prefix:{"lidar_lambda","joint_lambda","gate","lidar_scaled_lambda"})
      for(int k=0;k<6;++k) livo_quality_stream_<<','<<prefix<<k;
    livo_quality_stream_<<'\n';
  }
  if(cfg.enabled) loop_backend_.reset(new sa_loop::LoopClosureBackend(cfg,nh));
}

void LIVMapper::recordLivoFrame(const InfoForm6 &visual,double quality,double elapsed_ms)
{
  if(!livo_trajectory_stream_.is_open() && !loop_backend_) return;
  sa_loop::FrontendSnapshot snapshot;
  snapshot.stamp=LidarMeasures.measures.back().vio_time;
  snapshot.pose.linear()=_state.rot_end;snapshot.pose.translation()=_state.pos_end;
  snapshot.covariance=_state.cov.block<6,6>(0,0);
  snapshot.visual_quality=quality;snapshot.visual_observations=sa_vio->numValidObs();
  if(livo_trajectory_stream_.is_open()) {
    sa_loop::writePose(livo_trajectory_stream_,snapshot.stamp,snapshot.pose);
    auto quality_state=getTrackingQuality(true);
    livo_quality_stream_<<std::setprecision(12)<<snapshot.stamp<<','<<quality_state.points<<','<<quality_state.matches<<','
      <<quality_state.raw_rms<<','<<quality_state.gated_rms<<','<<voxelmap_manager->degenerate_update_<<','
      <<voxelmap_manager->map_update_blocked_<<','<<quality_state.finite<<','<<elapsed_ms<<','
      <<sa_vio->numValidObs()<<','<<sa_vio->numRejectedObs()<<','<<sa_vio->lastRms()<<','<<quality<<','<<visual.Lambda.norm();
    Eigen::SelfAdjointEigenSolver<sa_loop::Matrix6> es(voxelmap_manager->lidar_information_last_);
    for(int k=0;k<6;++k) livo_quality_stream_<<','<<(es.info()==Eigen::Success?es.eigenvalues()(k):0);
    const auto &f=voxelmap_manager->fused_last_;
    for(int k=0;k<6;++k) livo_quality_stream_<<','<<(int(f.eigenvals.size())>k?f.eigenvals[k]:0);
    for(int k=0;k<6;++k) livo_quality_stream_<<','<<(int(f.gate_weights.size())>k?f.gate_weights[k]:0);
    sa_loop::Matrix6 scale=sa_loop::Matrix6::Identity();scale.topLeftCorner<3,3>()/=2.0;
    Eigen::SelfAdjointEigenSolver<sa_loop::Matrix6> scaled(scale*voxelmap_manager->lidar_information_last_*scale);
    for(int k=0;k<6;++k) livo_quality_stream_<<','<<(scaled.info()==Eigen::Success?scaled.eigenvalues()(k):0);
    livo_quality_stream_<<'\n';
  }
  if(loop_backend_) {
    sa_loop::Cloud::Ptr cloud(new sa_loop::Cloud);
    cloud->reserve(feats_undistort->size());
    // Descriptor density is independent of the front-end's coarser matching grid.
    for(const auto &p:*feats_undistort) {
      Eigen::Vector3d v=extR*p.getVector3fMap().cast<double>()+extT;
      if(!v.allFinite()) continue;
      pcl::PointXYZI point;point.x=v.x();point.y=v.y();point.z=v.z();point.intensity=p.intensity;cloud->push_back(point);
    }
    snapshot.cloud=cloud;loop_backend_->submit(snapshot);
  }
}
