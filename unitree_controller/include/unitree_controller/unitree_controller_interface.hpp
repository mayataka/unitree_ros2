#ifndef UNITREE_CONTROLLER__UNITREE_CONTROLLER_INTERFACE_HPP_
#define UNITREE_CONTROLLER__UNITREE_CONTROLLER_INTERFACE_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/node_options.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/service.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "realtime_tools/realtime_publisher.h"
#include "realtime_tools/realtime_buffer.h"
#include "geometry_msgs/msg/twist.hpp"

#include "Eigen/Core"
#include "Eigen/Geometry"

#include "unitree_legged_sdk/comm.h"
#include "unitree_hardware/hardware_interface_type_values.hpp"

#include "unitree_controller/visibility_control.h"


namespace unitree_controller
{

using namespace std::chrono_literals;  // NOLINT

class UnitreeControllerInterface : public controller_interface::ControllerInterface
{
protected:
  using Vector19d = Eigen::Matrix<double, 19, 1>;
  using Vector18d = Eigen::Matrix<double, 18, 1>;
  using Vector12d = Eigen::Matrix<double, 12, 1>;
  using Vector4d  = Eigen::Matrix<double, 4, 1>;
  using Vector3d  = Eigen::Matrix<double, 3, 1>;
  using Quaterniond = Eigen::Quaterniond;

  virtual controller_interface::CallbackReturn on_init_impl() = 0;

  virtual controller_interface::CallbackReturn on_configure_impl(
    const rclcpp_lifecycle::State & previous_state) = 0;

  virtual controller_interface::return_type update_impl(
    const rclcpp::Time & time, const rclcpp::Duration & period) = 0;

  virtual bool reset_impl() = 0;

  // get the update rate (Hz)
  double getUpdateRate() const { return update_rate_; }

  // get the update period (ms)
  double getUpdatePeriod() const { return update_period_; }

  const Vector12d& getJointPositions() const { return qJ_; }

  const Vector12d& getJointVelocities() const { return dqJ_; }

  const Vector12d& getJointTorques() const { return tauJ_; }

  const Quaterniond& getImuOrientation() const { return imu_quat_; }

  const Vector3d& getImuAngularVelocity() const { return imu_ang_vel_; }

  const Vector3d& getImuLinearAcceleration() const { return imu_lin_acc_; }

  const Vector4d& getFootForceSensors() const { return f_; }

  Vector12d& getJointPositionCommands() { return qJ_cmd_; }

  Vector12d& getJointVelocityCommands() { return dqJ_cmd_; }

  Vector12d& getJointTorqueCommands() { return tauJ_cmd_; }

  Vector12d& getJointPositionGainCommands() { return Kp_cmd_; }

  Vector12d& getJointVelocityGainCommands() { return Kd_cmd_; }

public:
  UNITREE_CONTROLLER_PUBLIC 
  UnitreeControllerInterface();

  UNITREE_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_init() override;

  UNITREE_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  UNITREE_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  UNITREE_CONTROLLER_PUBLIC
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  UNITREE_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  UNITREE_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  UNITREE_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  UNITREE_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  UNITREE_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

  UNITREE_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;

  UNITREE_CONTROLLER_PUBLIC
  bool reset();

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  // hardware interfaces
  std::vector<std::string> joint_names_, sensor_names_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> 
      qJ_interface_, dqJ_interface_, tauJ_interface_, 
      imu_orientation_interface_, imu_angular_velocity_interface_, 
      imu_linear_acceleration_interface_, foot_force_sensor_interface_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> 
      qJ_cmd_interface_, dqJ_cmd_interface_, tauJ_cmd_interface_, 
      Kp_cmd_interface_, Kd_cmd_interface_;

  // update rate (Hz)
  double update_rate_;

  // update period (ms)
  double update_period_;

  // joint states
  Vector12d qJ_, dqJ_, tauJ_;

  // imu states
  Quaterniond imu_quat_;
  Vector3d imu_ang_vel_, imu_lin_acc_;

  // foot force sensor states
  Vector4d f_;

  // join commands
  Vector12d qJ_cmd_, dqJ_cmd_, tauJ_cmd_, Kp_cmd_, Kd_cmd_;
};

}  // namespace unitree_controller

#endif // UNITREE_CONTROLLER__UNITREE_CONTROLLER_INTERFACE_HPP_