#ifndef UNITREE_CONTROLLER__TYPES_HPP_
#define UNITREE_CONTROLLER__TYPES_HPP_

#include "Eigen/Core"
#include "Eigen/Geometry"

#include "unitree_legged_sdk/comm.h"

namespace unitree_controller
{

using Vector19d = Eigen::Matrix<double, 19, 1>;
using Vector18d = Eigen::Matrix<double, 18, 1>;
using Vector12d = Eigen::Matrix<double, 12, 1>;
using Vector4d  = Eigen::Matrix<double, 4, 1>;
using Vector3d  = Eigen::Matrix<double, 3, 1>;
using Quaterniond = Eigen::Quaterniond;

struct UnitreeStates
{
  // joint states
  Vector12d qJ = Vector12d::Zero();
  Vector12d dqJ = Vector12d::Zero();
  Vector12d tauJ = Vector12d::Zero();

  // Imu states
  Quaterniond imu_orientation      = Quaterniond::Identity();
  Vector3d imu_angular_velocity    = Vector3d::Zero();
  Vector3d imu_linear_acceleration = Vector3d::Zero();

  // force sensor states
  Vector4d foot_force_sensor = Vector4d::Zero();

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct UnitreeCommands
{
  // joint commands
  Vector12d qJ_cmd   = Vector12d::Constant(UNITREE_LEGGED_SDK::PosStopF);
  Vector12d dqJ_cmd  = Vector12d::Constant(UNITREE_LEGGED_SDK::VelStopF);
  Vector12d tauJ_cmd = Vector12d::Zero();
  Vector12d Kp_cmd   = Vector12d::Zero();
  Vector12d Kd_cmd   = Vector12d::Zero();

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} // namespace unitree_controller

#endif  // UNITREE_CONTROLLER__TYPES_HPP_