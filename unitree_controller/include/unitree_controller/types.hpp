#ifndef UNITREE_CONTROLLER__TYPES_HPP_
#define UNITREE_CONTROLLER__TYPES_HPP_

#include <string>
#include <stdexcept>

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

/**
 * States of the robot.
 */
struct UnitreeStates
{
  /**
   * Joint positions measurements.
   */
  Vector12d qJ = Vector12d::Zero();

  /**
   * Joint velocities measurements.
   */
  Vector12d dqJ = Vector12d::Zero();

  /**
   * Joint torques measurements.
   */
  Vector12d tauJ = Vector12d::Zero();

  /**
   * Imu orientation estimation.
   */
  Quaterniond imu_orientation = Quaterniond::Identity();

  /**
   * Imu angular valocity measurements (gyro sensor).
   */
  Vector3d imu_angular_velocity = Vector3d::Zero();

  /**
   * Imu linear acceleration measurements (accelerometer).
   */
  Vector3d imu_linear_acceleration = Vector3d::Zero();

  /**
   * Force sensor measurements.
   */
  Vector4d foot_force_sensor = Vector4d::Zero();

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/**
 * Commands to the robot.
 */
struct UnitreeCommands
{
  /**
   * Joint position commands.
   */
  Vector12d qJ_cmd = Vector12d::Constant(UNITREE_LEGGED_SDK::PosStopF);

  /**
   * Joint velocity commands.
   */
  Vector12d dqJ_cmd = Vector12d::Constant(UNITREE_LEGGED_SDK::VelStopF);

  /**
   * Joint torque commands.
   */
  Vector12d tauJ_cmd = Vector12d::Zero();

  /**
   * Position gain commands.
   */
  Vector12d Kp_cmd = Vector12d::Zero();

  /**
   * Velocity gain commands.
   */
  Vector12d Kd_cmd = Vector12d::Zero();

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

enum class ControlMode {
  ZeroTorque,
  StandingUp,
  Control,
  SittingDown,
};

std::string FromControlModeToString(const ControlMode control_mode) {
  switch (control_mode)
  {
  case ControlMode::ZeroTorque:
    return "ZeroTorque"
    break;
  case ControlMode::StandingUp:
    return "StandingUp"
    break;
  case ControlMode::Control:
    return "Control"
    break;
  case ControlMode::SittingDown:
    return "SittingDown"
    break;
  default:
    throw std::runtime_error("Invalid ControlMode");
    return "";
    break;
  }
}

ControlMode FromStringToControlMode(const std::string& control_mode) {
  if (control_mode == "ZeroTorque" || control_mode == "zero_torque") {
    return ControlMode::ZeroTorque;
  }
  else if (control_mode == "StandingUp" || control_mode == "standing_up") {
    return ControlMode::StandingUp;
  }
  else if (control_mode == "Control" || control_mode == "control") {
    return ControlMode::Control;
  }
  else if (control_mode == "SittingDown" || control_mode == "sitting_down") {
    return ControlMode::SittingDown;
  }
  else {
    throw std::runtime_error("Invalid ControlMode");
    return ControlMode::ZeroTorque:
  }
}

} // namespace unitree_controller

#endif  // UNITREE_CONTROLLER__TYPES_HPP_