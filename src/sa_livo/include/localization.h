// SPDX-License-Identifier: GPL-2.0-only
#pragma once

#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <atomic>
#include <deque>
#include <functional>
#include <future>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace sa_localization {
using Cloud = pcl::PointCloud<pcl::PointXYZ>;
using Points = std::vector<Eigen::Vector3f>;
constexpr double kPi = 3.14159265358979323846;

enum class State : uint8_t { WAIT_MAP, WAIT_SENSOR, WAIT_STATIONARY, RELOCALIZING,
  VERIFYING, RECOVERING, TRACKING, DEGRADED, LOST, FAILED };
const char* stateName(State state);

struct Config {
  bool enabled = false;
  std::string map_path, metadata_path, calibration_id = "default", frame_id = "map";
  double map_sigma = 0.05, kiss_resolution = 0.3, fine_resolution = 0.15;
  double query_radius = 20.0, tile_size = 80.0, tile_stride = 40.0;
  int min_inliers = 30, min_query_points = 100, threads = 2;
  double correspondence_distance = 0.5, overlap_min = 0.6, rmse_max = 0.2;
  double gravity_max_deg = 10.0, observability_ratio = 1e-5;
  double verify_translation = 0.2, verify_rotation_deg = 3.0;
  double ambiguity_overlap = 0.05, ambiguity_rmse = 0.05;
  double match_timeout = 30.0, retry_delay = 5.0;
  int max_attempts = 3;
  double stationary_seconds = 1.0, stationary_translation = 0.05, stationary_rotation_deg = 1.0;
  double gyro_max = 0.05, gyro_std_max = 0.01, acc_std_max = 0.15;
  int stationary_frames = 5;
  double sensor_timeout = 0.5, bad_seconds = 0.5, no_observation_seconds = 0.3;
  double low_lidar_seconds = 2.0, good_seconds = 0.5, recovering_seconds = 1.0;
  int recovering_frames = 10, tracking_min_matches = 30, good_min_matches = 50;
  double tracking_min_ratio = 0.1, good_min_ratio = 0.2, bad_rms = 0.3;
  double visual_q_min = 0.05, max_position_variance = 3.0, max_rotation_variance = 0.25;
  double reset_rotation_std_deg = 5.0, reset_position_std = 0.2, reset_velocity_std = 0.1;
  double reset_exposure_std = 0.1, reset_gyro_bias_std = 0.01, reset_acc_bias_std = 0.1;
  double reset_gravity_std = 0.1;
  int max_lidar_buffer = 30, max_imu_buffer = 6000;
  void validate() const;
};

struct MapMetadata {
  std::string frame_id, version, calibration_id;
  Eigen::Vector3d gravity = Eigen::Vector3d::Zero();
  static MapMetadata load(const std::string& path);
  void save(const std::string& path) const;
};

class PriorMapManager {
public:
  PriorMapManager(const Config& config, const MapMetadata& metadata, const Cloud& cloud);
  static std::shared_ptr<PriorMapManager> load(const Config& config);
  const Cloud& cloud() const { return *cloud_; }
  const MapMetadata& metadata() const { return metadata_; }
  const std::vector<Cloud::ConstPtr>& tiles() const { return tiles_; }
private:
  MapMetadata metadata_;
  Cloud::ConstPtr cloud_;
  std::vector<Cloud::ConstPtr> tiles_;
};

struct Quality {
  double stamp = 0.0;
  int points = 0, matches = 0, visual_observations = 0;
  double raw_rms = std::numeric_limits<double>::infinity();
  double gated_rms = std::numeric_limits<double>::infinity();
  double visual_q = 0.0, position_variance = 0.0, rotation_variance = 0.0;
  double correction_translation = 0.0, correction_rotation = 0.0;
  std::vector<double> eigenvalues;
  bool finite = true, solved = true, degenerate = false, visual_available = false;
  double ratio() const { return points > 0 ? double(matches) / points : 0.0; }
};

class TrackingQualityMonitor {
public:
  explicit TrackingQualityMonitor(const Config& config) : config_(config) {}
  void reset();
  State update(const Quality& quality, State current);
  const std::string& reason() const { return reason_; }
private:
  Config config_;
  double last_ = -1, bad_since_ = -1, empty_since_ = -1, low_since_ = -1, good_since_ = -1;
  int good_frames_ = 0;
  std::string reason_;
};

struct ImuSample {
  double stamp = 0.0;
  Eigen::Vector3d acceleration = Eigen::Vector3d::Zero(), gyro = Eigen::Vector3d::Zero();
};
struct Packet { double stamp = 0.0; Cloud cloud; std::vector<ImuSample> imu; };
struct MatchResult {
  bool success = false;
  std::string reason;
  Eigen::Isometry3d T_map_lidar = Eigen::Isometry3d::Identity();
  double overlap = 0.0, rmse = std::numeric_limits<double>::infinity();
  int inliers = 0;
};
struct Recovery {
  MatchResult match;
  double stamp = 0.0;
  Eigen::Vector3d mean_acc = Eigen::Vector3d::Zero(), mean_gyro = Eigen::Vector3d::Zero();
};

class KissRelocalizer {
public:
  explicit KissRelocalizer(const Config& config) : config_(config) {}
  MatchResult match(const Cloud& source, const PriorMapManager& map,
                    const Eigen::Vector3d& gravity_lidar, const std::atomic<bool>& cancel,
                    const Eigen::Vector3d* hint = nullptr) const;
  MatchResult refine(const Cloud& source, const Cloud& target, const Eigen::Isometry3d& guess,
                     bool check_observability = true) const;
  bool stationaryTransform(const Cloud& source, const Cloud& anchor, Eigen::Isometry3d& T_anchor_source) const;
private:
  Config config_;
};

class RelocalizationManager {
public:
  using MatchFunction = std::function<MatchResult(const Cloud&, const Eigen::Vector3d&, const std::atomic<bool>&)>;
  RelocalizationManager(const Config& config, std::shared_ptr<const PriorMapManager> map,
                        const Eigen::Matrix3d& R_imu_lidar, MatchFunction matcher = {});
  ~RelocalizationManager();
  bool ingest(const Packet& packet); // true only for a fresh, independently verified recovery
  void poll();
  void request(const std::string& reason);
  void sensorWait(const std::string& reason);
  void clockReset();
  void committed(double stamp);
  void updateQuality(const Quality& quality);
  bool processesTracking() const;
  bool valid() const { return state_ == State::TRACKING; }
  bool workerBusy() const { return future_.valid(); }
  State state() const { return state_; }
  const std::string& reason() const { return reason_; }
  uint64_t attemptId() const { return generation_; }
  int attempts() const { return attempts_; }
  const Recovery& recovery() const { return recovery_; }
  const Quality& quality() const { return quality_; }
  void setRecoveryHint(const Eigen::Vector3d& position) { hint_=position; have_hint_=position.allFinite(); }
private:
  void invalidate();
  void failed(const std::string& reason);
  void clearStationary();
  bool stationary(const Packet& packet, Eigen::Isometry3d& T_anchor_current);
  Config config_;
  std::shared_ptr<const PriorMapManager> map_;
  Eigen::Matrix3d R_imu_lidar_;
  KissRelocalizer matcher_;
  MatchFunction match_function_;
  TrackingQualityMonitor monitor_;
  State state_ = State::WAIT_SENSOR;
  std::string reason_ = "waiting for LiDAR and IMU";
  Quality quality_;
  Cloud::Ptr anchor_, collected_;
  std::deque<ImuSample> imu_window_;
  Eigen::Vector3d mean_acc_ = Eigen::Vector3d::Zero(), mean_gyro_ = Eigen::Vector3d::Zero();
  double anchor_stamp_ = -1, last_stamp_ = -1, launch_stamp_ = -1, verify_stamp_ = -1;
  int collected_frames_ = 0, verified_frames_ = 0, attempts_ = 0;
  uint64_t generation_ = 0, launched_generation_ = 0;
  std::shared_ptr<std::atomic<bool>> cancel_;
  std::future<MatchResult> future_;
  std::chrono::steady_clock::time_point launched_at_, retry_after_;
  MatchResult candidate_, previous_verification_;
  Recovery recovery_;
  Eigen::Vector3d hint_ = Eigen::Vector3d::Zero();
  bool have_hint_ = false;
};

Cloud filteredCloud(const Cloud& cloud, double resolution, double radius = 0.0);
double rotationDistance(const Eigen::Matrix3d& a, const Eigen::Matrix3d& b);
bool validTransform(const Eigen::Isometry3d& transform);
} // namespace sa_localization
