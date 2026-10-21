// SPDX-License-Identifier: GPL-2.0-only
#include "localization.h"
#include <kiss_matcher/KISSMatcher.hpp>
#include <small_gicp/registration/registration_helper.hpp>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <tbb/global_control.h>
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <numeric>
#include <sstream>

namespace sa_localization {
namespace {
Points points(const Cloud& cloud) {
  Points result;
  result.reserve(cloud.size());
  for (const auto& p : cloud) result.push_back(p.getVector3fMap());
  return result;
}
bool observable(const Eigen::Matrix<double,6,6>& h, double scale, double threshold) {
  Eigen::Matrix<double,6,6> d = Eigen::Matrix<double,6,6>::Identity();
  d.topLeftCorner<3,3>() /= std::max(1.0, scale);
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double,6,6>> eig(d * h * d);
  return eig.info() == Eigen::Success && eig.eigenvalues().allFinite() &&
    eig.eigenvalues()[5] > 1e-9 && eig.eigenvalues()[0] / eig.eigenvalues()[5] >= threshold;
}
}
MatchResult KissRelocalizer::refine(const Cloud& input, const Cloud& target,
                                   const Eigen::Isometry3d& guess, bool check_observability) const {
  MatchResult out;
  out.reason = "insufficient point support";
  if (!validTransform(guess)) { out.reason = "invalid initial transform"; return out; }
  const Cloud source = filteredCloud(input, config_.fine_resolution);
  if (source.size() < size_t(config_.min_query_points) || target.size() < size_t(config_.min_query_points)) return out;
  small_gicp::RegistrationSetting setting;
  setting.type = small_gicp::RegistrationSetting::GICP;
  setting.num_threads = config_.threads;
  setting.max_iterations = 40;
  setting.downsampling_resolution = config_.fine_resolution;
  setting.max_correspondence_distance = std::max(1.0, config_.correspondence_distance * 2);
  const auto fit = small_gicp::align(points(target), points(source), guess, setting);
  out.T_map_lidar = fit.T_target_source;
  if (!fit.converged || !validTransform(out.T_map_lidar)) { out.reason = "GICP did not converge"; return out; }
  pcl::KdTreeFLANN<pcl::PointXYZ> tree;
  tree.setInputCloud(target.makeShared());
  double squared_error = 0, scale2 = 0;
  Eigen::Matrix<double,6,6> plane_information = Eigen::Matrix<double,6,6>::Zero();
  std::vector<int> indices(12);
  std::vector<float> distances(12);
  for (const auto& point : source) {
    const Eigen::Vector3d p_source = point.getVector3fMap().cast<double>();
    const Eigen::Vector3d rotated = out.T_map_lidar.linear() * p_source;
    const Eigen::Vector3d p = rotated + out.T_map_lidar.translation();
    pcl::PointXYZ query(p.x(), p.y(), p.z());
    const int n = tree.nearestKSearch(query, 12, indices, distances);
    if (n == 0 || distances[0] > config_.correspondence_distance * config_.correspondence_distance) continue;
    ++out.inliers;
    squared_error += distances[0];
    scale2 += p_source.squaredNorm();
    if (check_observability && n >= 6) {
      Eigen::Vector3d mean = Eigen::Vector3d::Zero();
      for (int j=0;j<n;++j) mean += target[indices[j]].getVector3fMap().cast<double>();
      mean /= n;
      Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
      for (int j=0;j<n;++j) {
        const Eigen::Vector3d d = target[indices[j]].getVector3fMap().cast<double>() - mean;
        covariance += d * d.transpose();
      }
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(covariance);
      if (eig.info() != Eigen::Success || eig.eigenvalues().sum() < 1e-9 ||
          eig.eigenvalues()[0] > 0.1 * eig.eigenvalues().sum()) continue;
      const Eigen::Vector3d normal = eig.eigenvectors().col(0);
      Eigen::Matrix<double,6,1> jacobian;
      jacobian << rotated.cross(normal), normal;
      plane_information += jacobian * jacobian.transpose();
    }
  }
  out.overlap = double(out.inliers) / source.size();
  out.rmse = out.inliers ? std::sqrt(squared_error / out.inliers) : std::numeric_limits<double>::infinity();
  if (out.inliers < config_.min_inliers || out.overlap < config_.overlap_min || out.rmse > config_.rmse_max) {
    out.reason = "independent geometric validation failed";
    return out;
  }
  const double scale = std::sqrt(scale2 / std::max(out.inliers, 1));
  if (check_observability && (!observable(fit.H, scale, config_.observability_ratio) ||
      !observable(plane_information, scale, config_.observability_ratio))) {
    out.reason = "geometrically unobservable pose";
    return out;
  }
  out.success = true;
  out.reason = "verified";
  return out;
}

bool KissRelocalizer::stationaryTransform(const Cloud& source, const Cloud& anchor,
                                         Eigen::Isometry3d& transform) const {
  auto fit = refine(source, anchor, Eigen::Isometry3d::Identity());
  transform = fit.T_map_lidar;
  return fit.success && transform.translation().norm() <= config_.stationary_translation &&
    rotationDistance(Eigen::Matrix3d::Identity(), transform.linear()) <= config_.stationary_rotation_deg * kPi / 180.;
}

MatchResult KissRelocalizer::match(const Cloud& source, const PriorMapManager& map,
                                  const Eigen::Vector3d& gravity_lidar, const std::atomic<bool>& cancel,
                                  const Eigen::Vector3d* hint) const {
  MatchResult failure;
  failure.reason = "no acceptable global candidate";
  const auto start = std::chrono::steady_clock::now();
  auto expired = [&]() { return cancel.load() || std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count() > config_.match_timeout; };
  tbb::global_control thread_limit(tbb::global_control::max_allowed_parallelism, config_.threads);
  const Points source_points = points(filteredCloud(source, config_.kiss_resolution));
  if (source_points.size() < size_t(config_.min_query_points)) { failure.reason = "insufficient query points"; return failure; }
  std::vector<size_t> order(map.tiles().size());
  std::iota(order.begin(), order.end(), 0);
  if (hint) std::stable_sort(order.begin(), order.end(), [&](size_t a, size_t b) {
    auto center = [&](size_t i) {
      Eigen::Vector3d c = Eigen::Vector3d::Zero();
      for (const auto& p : *map.tiles()[i]) c += p.getVector3fMap().cast<double>();
      return Eigen::Vector3d(c / map.tiles()[i]->size());
    };
    return (center(a)-*hint).squaredNorm() < (center(b)-*hint).squaredNorm();
  });
  std::vector<MatchResult> candidates;
  size_t correspondence_candidates=0, solved_candidates=0, gravity_candidates=0;
  size_t best_correspondences=0, best_solver_inliers=0, best_inliers=0;
  double best_overlap=0;
  std::string last_refinement_failure;
  struct Search { Cloud::ConstPtr target; int depth; };
  std::vector<Search> searches;
  for(size_t index:order) searches.push_back({map.tiles()[index],0});
  for (size_t job=0;job<searches.size();++job) {
    const auto tile=searches[job].target;
    const int depth=searches[job].depth;
    if (expired()) { failure.reason = "global search cancelled or timed out"; return failure; }
    // A new solver per tile avoids a stale valid solution after too few correspondences.
    kiss_matcher::KISSMatcher matcher(float(config_.kiss_resolution));
    auto matched = matcher.match(source_points, points(*tile));
    const auto& source_matches = std::get<0>(matched);
    const auto& target_matches = std::get<1>(matched);
    best_correspondences=std::max(best_correspondences,source_matches.size());
    if (source_matches.size() < size_t(config_.min_inliers)) continue;
    ++correspondence_candidates;
    Eigen::Matrix<double,3,Eigen::Dynamic> s(3,source_matches.size()), t(3,target_matches.size());
    for (size_t i=0;i<source_matches.size();++i) { s.col(i)=source_matches[i].cast<double>(); t.col(i)=target_matches[i].cast<double>(); }
    matcher.resetSolver();
    const auto solution = matcher.solve(s,t);
    best_solver_inliers=std::max(best_solver_inliers,matcher.getNumFinalInliers());
    if (!solution.valid || matcher.getNumFinalInliers() < size_t(config_.min_inliers)) continue;
    ++solved_candidates;
    Eigen::Isometry3d guess = Eigen::Isometry3d::Identity();
    guess.linear() = solution.rotation;
    guess.translation() = solution.translation;
    if (!validTransform(guess) || !gravity_lidar.allFinite() || gravity_lidar.norm() < 1e-6) continue;
    const double alignment = std::max(-1.0, std::min(1.0,
      (guess.linear() * gravity_lidar.normalized()).dot(map.metadata().gravity.normalized())));
    if (std::acos(alignment) > config_.gravity_max_deg * kPi / 180.) continue;
    ++gravity_candidates;
    auto candidate = refine(source, map.cloud(), guess);
    best_inliers=std::max(best_inliers,size_t(candidate.inliers));
    best_overlap=std::max(best_overlap,candidate.overlap);
    if (!candidate.success) { last_refinement_failure=candidate.reason; continue; }
    const double final_alignment = std::max(-1.0, std::min(1.0,
      (candidate.T_map_lidar.linear()*gravity_lidar.normalized()).dot(map.metadata().gravity.normalized())));
    if (std::acos(final_alignment) > config_.gravity_max_deg * kPi / 180.) continue;
    bool duplicate = false;
    for (auto& existing : candidates) {
      if ((existing.T_map_lidar.translation()-candidate.T_map_lidar.translation()).norm() <= config_.verify_translation &&
          rotationDistance(existing.T_map_lidar.linear(),candidate.T_map_lidar.linear()) <= config_.verify_rotation_deg*kPi/180.) {
        if (candidate.rmse < existing.rmse) existing = candidate;
        duplicate = true;
        break;
      }
    }
    if (!duplicate) candidates.push_back(candidate);
    // A small map can contain two identical rooms inside a single tile. Search
    // again after masking the accepted footprint, rather than treating one
    // solver output as proof that the global pose is unique.
    Cloud::Ptr aligned(new Cloud);
    pcl::transformPointCloud(source,*aligned,candidate.T_map_lidar.matrix().cast<float>());
    pcl::KdTreeFLANN<pcl::PointXYZ> footprint;
    footprint.setInputCloud(aligned);
    Cloud::Ptr remaining(new Cloud);
    std::vector<int> nearest(1);std::vector<float> squared_distance(1);
    const double mask_distance=std::max(config_.correspondence_distance,config_.kiss_resolution*2);
    for(const auto& point:*tile)
      if(footprint.nearestKSearch(point,1,nearest,squared_distance)==0 || squared_distance[0]>mask_distance*mask_distance)
        remaining->push_back(point);
    if(remaining->size()>=size_t(config_.min_query_points) && tile->size()-remaining->size()>=size_t(config_.min_inliers)) {
      if(depth>=7) { failure.reason="too many global hypotheses to establish uniqueness";return failure; }
      searches.push_back({remaining,depth+1});
    }
  }
  if (expired()) { failure.reason = "global search cancelled or timed out"; return failure; }
  if (candidates.empty()) {
    std::ostringstream reason;
    reason<<failure.reason<<": tiles="<<searches.size()<<", query="<<source_points.size()
      <<", best_correspondences="<<best_correspondences<<", correspondence_candidates="<<correspondence_candidates
      <<", solved="<<solved_candidates<<", gravity_pass="<<gravity_candidates
      <<", best_solver_inliers="<<best_solver_inliers
      <<", best_inliers="<<best_inliers<<", best_overlap="<<best_overlap;
    if(!last_refinement_failure.empty()) reason<<", refinement="<<last_refinement_failure;
    failure.reason=reason.str();return failure;
  }
  std::sort(candidates.begin(),candidates.end(),[](const MatchResult& a,const MatchResult& b) {
    return a.overlap > b.overlap || (a.overlap == b.overlap && a.rmse < b.rmse);
  });
  for (size_t i=1;i<candidates.size();++i)
    if (std::abs(candidates[0].overlap-candidates[i].overlap) < config_.ambiguity_overlap &&
        std::abs(candidates[0].rmse-candidates[i].rmse) < config_.ambiguity_rmse) {
      failure.reason = "ambiguous global candidates";
      return failure;
    }
  return candidates.front();
}
} // namespace sa_localization
