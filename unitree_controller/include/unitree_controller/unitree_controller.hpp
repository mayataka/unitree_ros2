#ifndef UNITREE_CONTROLLER__UNITREE_CONTROLLER_HPP_
#define UNITREE_CONTROLLER__UNITREE_CONTROLLER_HPP_

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

// #include "unitree_msgs/msg/state.hpp"
// #include "unitree_msgs/srv/reset_state_estimation.hpp"
// #include "unitree_msgs/srv/set_control_mode.hpp"

#include "unitree_legged_sdk/comm.h"
#include "unitree_hardware/hardware_interface_type_values.hpp"

#include "unitree_controller/visibility_control.h"


namespace unitree_controller
{

using namespace std::chrono_literals;  // NOLINT

// enum class ControlMode {
// ZeroTorque  = 0,
// StandingUp  = 1,
// Idling      = 2,
// Control     = 3,
// SittingDown = 4,
// };

class UnitreeController : public controller_interface::ControllerInterface
{
public:
  using Vector19d = Eigen::Matrix<double, 19, 1>;
  using Vector18d = Eigen::Matrix<double, 18, 1>;
  using Vector12d = Eigen::Matrix<double, 12, 1>;
  using Vector4d  = Eigen::Matrix<double, 4, 1>;
  using Vector3d  = Eigen::Matrix<double, 3, 1>;
  using Quaterniond = Eigen::Quaterniond;

  UNITREE_CONTROLLER_PUBLIC 
  UnitreeController();

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

protected:
  // hardware interfaces
  std::vector<std::string> joint_names_, sensor_names_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> 
      qJ_interface_, dqJ_interface_, tauJ_interface_, 
      imu_orientation_interface_, imu_angular_velocity_interface_, 
      imu_linear_acceleration_interface_, foot_force_sensor_interface_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> 
      qJ_cmd_interface_, dqJ_cmd_interface_, tauJ_cmd_interface_, 
      Kp_cmd_interface_, Kd_cmd_interface_;

  // // state publisher
  // using StatePublisher = rclcpp::Publisher<unitree_msgs::msg::State>;
  // using RealtimeStatePublisher = realtime_tools::RealtimePublisher<unitree_msgs::msg::State>;
  // std::shared_ptr<StatePublisher> state_publisher_;
  // std::shared_ptr<RealtimeStatePublisher> realtime_state_publisher_;

  // sampling time 
  double sampling_time_;

  // joint states
  Vector12d qJ_, dqJ_, tauJ_;

  // imu states
  Vector4d imu_quat_;
  Vector3d imu_ang_vel_, imu_lin_acc_;

  // foot force sensor states
  Vector4d f_;

  // join commands
  Vector12d qJ_cmd_, dqJ_cmd_, tauJ_cmd_, Kp_cmd_, Kd_cmd_;

  void auto_declare_params();
};

}  // namespace unitree_controller

#endif // UNITREE_CONTROLLER__UNITREE_CONTROLLER_HPP_