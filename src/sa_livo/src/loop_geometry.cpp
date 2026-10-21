#include "loop_geometry.h"
#include <ceres/ceres.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <iomanip>
#include <limits>

namespace sa_loop {
namespace {
struct PoseParameters {
  double q[4], t[3]; // Eigen quaternion: x y z w
  explicit PoseParameters(const Pose &p) {
    Eigen::Map<Eigen::Quaterniond> rotation(q);
    rotation = Eigen::Quaterniond(p.linear());
    Eigen::Map<Eigen::Vector3d> translation(t);
    translation = p.translation();
  }
  Pose pose() const {
    Pose p = Pose::Identity();
    p.linear() = Eigen::Map<const Eigen::Quaterniond>(q).normalized().toRotationMatrix();
    p.translation() = Eigen::Map<const Eigen::Vector3d>(t);
    return p;
  }
};
struct EdgeError {
  Eigen::Quaterniond rotation;
  Eigen::Vector3d translation;
  Matrix6 square_root;
  explicit EdgeError(const GraphEdge &edge)
      : rotation(edge.measurement.linear()), translation(edge.measurement.translation()) {
    Eigen::SelfAdjointEigenSolver<Matrix6> es(edge.information);
    square_root = es.eigenvalues().cwiseMax(0).cwiseSqrt().asDiagonal() * es.eigenvectors().transpose();
  }
  template<typename T> bool operator()(const T *qa, const T *ta, const T *qb, const T *tb, T *r) const {
    const Eigen::Map<const Eigen::Quaternion<T>> a(qa), b(qb);
    const Eigen::Map<const Eigen::Matrix<T,3,1>> pa(ta), pb(tb);
    const Eigen::Quaternion<T> relative = a.conjugate() * b;
    Eigen::Quaternion<T> error = relative * rotation.cast<T>().conjugate();
    if (error.w() < T(0)) error.coeffs() = -error.coeffs();
    Eigen::Matrix<T,6,1> e;
    e.template head<3>() = T(2) * error.vec();
    e.template tail<3>() = a.conjugate() * (pb-pa) - translation.cast<T>();
    Eigen::Map<Eigen::Matrix<T,6,1>> residual(r);
    residual = square_root.cast<T>() * e;
    return true;
  }
};
struct Overlap { double fraction = 0, sum_sq = 0; size_t count = 0; };
Overlap overlap(const Cloud &source, const Cloud &target, const Pose &transform, double radius) {
  Overlap result;
  if (source.empty() || target.empty()) return result;
  pcl::KdTreeFLANN<pcl::PointXYZI> tree;
  tree.setInputCloud(target.makeShared());
  std::vector<int> ids(1); std::vector<float> distances(1);
  for (const auto &p : source) {
    const Eigen::Vector3d v = transform * p.getVector3fMap().cast<double>();
    pcl::PointXYZI q; q.x=v.x();q.y=v.y();q.z=v.z();
    if (tree.nearestKSearch(q, 1, ids, distances) == 1 && distances[0] <= radius*radius) {
      ++result.count; result.sum_sq += distances[0];
    }
  }
  result.fraction = double(result.count)/source.size();
  return result;
}
}
bool validPose(const Pose &p) {
  return p.matrix().allFinite() && std::abs(p.linear().determinant()-1)<1e-5 &&
      (p.linear().transpose()*p.linear()-Eigen::Matrix3d::Identity()).norm()<1e-5;
}
bool PoseGraph::optimize(int iterations) {
  if (nodes.empty()) return true;
  std::vector<PoseParameters> parameters;
  parameters.reserve(nodes.size());
  for (const auto &node : nodes) {
    if (!validPose(node.optimized)) return false;
    parameters.emplace_back(node.optimized);
  }
  ceres::Problem problem;
  for (auto &p : parameters) {
    problem.AddParameterBlock(p.q,4,new ceres::EigenQuaternionManifold);
    problem.AddParameterBlock(p.t,3);
  }
  for (const auto &edge : edges) {
    if (edge.a<0 || edge.b<0 || edge.a>=int(nodes.size()) || edge.b>=int(nodes.size()) ||
        !validPose(edge.measurement) || !edge.information.allFinite()) return false;
    Eigen::SelfAdjointEigenSolver<Matrix6> es(edge.information);
    if (es.info()!=Eigen::Success || es.eigenvalues().minCoeff() < -1e-8) return false;
    auto *cost = new ceres::AutoDiffCostFunction<EdgeError,6,4,3,4,3>(new EdgeError(edge));
    problem.AddResidualBlock(cost,edge.loop ? new ceres::HuberLoss(1.0) : nullptr,
        parameters[edge.a].q,parameters[edge.a].t,parameters[edge.b].q,parameters[edge.b].t);
  }
  problem.SetParameterBlockConstant(parameters[0].q);
  problem.SetParameterBlockConstant(parameters[0].t);
  ceres::Solver::Options options;
  options.linear_solver_type=ceres::SPARSE_NORMAL_CHOLESKY;
  options.sparse_linear_algebra_library_type=ceres::EIGEN_SPARSE;
  options.max_num_iterations=iterations; options.num_threads=1;
  ceres::Solver::Summary summary; ceres::Solve(options,&problem,&summary);
  if (!summary.IsSolutionUsable() || !std::isfinite(summary.final_cost) ||
      summary.final_cost > summary.initial_cost + 1e-8) return false;
  for (const auto &p : parameters) if (!validPose(p.pose())) return false;
  for (size_t i=0;i<nodes.size();++i) nodes[i].optimized=parameters[i].pose();
  return true;
}
Pose PoseGraph::correctionAt(double stamp) const {
  if (nodes.empty()) return Pose::Identity();
  auto after=std::upper_bound(nodes.begin(),nodes.end(),stamp,
      [](double t,const GraphNode &n){return t<n.stamp;});
  if (after==nodes.begin()) return after->optimized*after->raw.inverse();
  const auto &before=*(after-1);
  Pose c0=before.optimized*before.raw.inverse();
  if (after==nodes.end()) return c0;
  Pose c1=after->optimized*after->raw.inverse();
  double alpha=std::clamp((stamp-before.stamp)/(after->stamp-before.stamp),0.0,1.0);
  // SE(3) exponential interpolation; Sophus is kept out of this public API.
  Eigen::AngleAxisd aa(c0.linear().transpose()*c1.linear());
  Eigen::Vector3d w=aa.axis()*aa.angle();
  Eigen::Matrix3d W;
  W << 0,-w.z(),w.y(),w.z(),0,-w.x(),-w.y(),w.x(),0;
  const double theta=w.norm();
  Eigen::Matrix3d V=Eigen::Matrix3d::Identity();
  if(theta>1e-8) V+=(1-std::cos(theta))/(theta*theta)*W+(theta-std::sin(theta))/(theta*theta*theta)*W*W;
  else V+=.5*W+W*W/6;
  const Eigen::Vector3d v=V.fullPivLu().solve(c0.linear().transpose()*(c1.translation()-c0.translation()));
  Eigen::Matrix3d A=alpha*W, Va=Eigen::Matrix3d::Identity();
  double angle=alpha*theta;
  if(angle>1e-8) Va+=(1-std::cos(angle))/(angle*angle)*A+(angle-std::sin(angle))/(angle*angle*angle)*A*A;
  else Va+=.5*A+A*A/6;
  Pose increment=Pose::Identity();
  increment.linear()=Eigen::Quaterniond::Identity().slerp(alpha,Eigen::Quaterniond(c0.linear().transpose()*c1.linear())).toRotationMatrix();
  increment.translation()=Va*(alpha*v);
  return c0*increment;
}
Cloud::Ptr voxelFilter(const Cloud &cloud,double leaf) {
  Cloud::Ptr clean(new Cloud), result(new Cloud);
  for(const auto &p:cloud) if(p.getVector3fMap().allFinite()) clean->push_back(p);
  if(clean->empty()) return result;
  pcl::VoxelGrid<pcl::PointXYZI> filter; filter.setLeafSize(leaf,leaf,leaf);
  filter.setInputCloud(clean); filter.filter(*result); return result;
}
Verification verifyRegistration(const Cloud &source,const Cloud &target,
    const Planes &sp,const Planes &tp,const Pose &T,const VerificationConfig &cfg) {
  Verification result;
  if(source.empty() || target.empty() || sp.empty() || tp.empty()) return result;
  if(!validPose(T)) { result.reason="invalid_transform";return result; }
  auto a=overlap(source,target,T,cfg.radius),b=overlap(target,source,T.inverse(),cfg.radius);
  result.overlap_forward=a.fraction;result.overlap_reverse=b.fraction;
  result.rmse=a.count+b.count ? std::sqrt((a.sum_sq+b.sum_sq)/(a.count+b.count)) : 0;
  if(std::min(a.fraction,b.fraction)<cfg.min_overlap) {result.reason="overlap";return result;}
  if(result.rmse>cfg.max_rmse) {result.reason="rmse";return result;}
  pcl::KdTreeFLANN<pcl::PointXYZINormal> tree;tree.setInputCloud(tp.makeShared());
  std::vector<int> ids(1);std::vector<float> distances(1);
  Matrix6 H=Matrix6::Zero();
  for(const auto &p:sp) {
    Eigen::Vector3d rotated=T.linear()*p.getVector3fMap().cast<double>();
    Eigen::Vector3d v=rotated+T.translation();
    pcl::PointXYZINormal q;q.x=v.x();q.y=v.y();q.z=v.z();
    if(tree.nearestKSearch(q,1,ids,distances)!=1 || distances[0]>cfg.radius*cfg.radius) continue;
    const auto &target_plane=tp[ids[0]];
    Eigen::Vector3d n=target_plane.getNormalVector3fMap().cast<double>();
    Eigen::Vector3d ns=T.linear()*p.getNormalVector3fMap().cast<double>();
    if(!n.allFinite() || n.norm()<.5 || !ns.allFinite() || ns.norm()<.5) continue;
    n.normalize();ns.normalize();
    if(std::abs(n.dot(ns))<.98) continue;
    Eigen::Matrix<double,6,1> h;h<<rotated.cross(n),n;H+=h*h.transpose();++result.plane_matches;
  }
  if(result.plane_matches<cfg.min_plane_matches) {result.reason="plane_matches";return result;}
  Matrix6 scale=Matrix6::Identity();scale.topLeftCorner<3,3>()/=cfg.rotation_length;
  Eigen::SelfAdjointEigenSolver<Matrix6> es(scale*H*scale);
  if(es.info()!=Eigen::Success || es.eigenvalues()(5)<=0) {result.reason="hessian";return result;}
  result.eigen_ratio=std::max(0.0,es.eigenvalues()(0))/es.eigenvalues()(5);
  if(result.eigen_ratio<cfg.min_eigen_ratio) {result.reason="unobservable";return result;}
  Eigen::SelfAdjointEigenSolver<Matrix6> raw(H/std::max(.0025,result.rmse*result.rmse));
  result.information=raw.eigenvectors()*raw.eigenvalues().cwiseMax(0).cwiseMin(1e6).asDiagonal()*raw.eigenvectors().transpose();
  result.accepted=true;result.reason="verified";return result;
}
void writePose(std::ostream &out,double stamp,const Pose &p) {
  const Eigen::Quaterniond q(p.linear());
  out<<std::fixed<<std::setprecision(9)<<stamp<<' '<<p.translation().transpose()<<' '
      <<q.x()<<' '<<q.y()<<' '<<q.z()<<' '<<q.w()<<'\n';
}
} // namespace sa_loop
