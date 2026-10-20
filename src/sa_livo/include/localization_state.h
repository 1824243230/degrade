// SPDX-License-Identifier: GPL-2.0-only
#pragma once
#include "common_lib.h"
#include "localization.h"

inline StatesGroup makeRecoveredState(const sa_localization::Recovery& recovery,
    const Eigen::Isometry3d& T_imu_lidar, const Eigen::Vector3d& gravity,
    const Eigen::Vector3d& trusted_acc_bias, const sa_localization::Config& config) {
  const auto pose = recovery.match.T_map_lidar * T_imu_lidar.inverse();
  StatesGroup state;
  state.rot_end=pose.linear(); state.pos_end=pose.translation(); state.vel_end.setZero();
  state.bias_g=recovery.mean_gyro;
  state.bias_a=trusted_acc_bias;
  state.gravity=gravity; state.inv_expo_time=1.0;
  state.cov.setZero();
  auto block=[&](int index,double sigma) { state.cov.block<3,3>(index,index).diagonal().setConstant(sigma*sigma); };
  block(StateIndex::Rotation, config.reset_rotation_std_deg*sa_localization::kPi/180.);
  block(StateIndex::Position,config.reset_position_std);
  state.cov(StateIndex::Exposure,StateIndex::Exposure)=config.reset_exposure_std*config.reset_exposure_std;
  block(StateIndex::Velocity,config.reset_velocity_std);
  block(StateIndex::GyroBias,config.reset_gyro_bias_std);
  block(StateIndex::AccBias,config.reset_acc_bias_std);
  block(StateIndex::Gravity,config.reset_gravity_std);
  return state;
}
