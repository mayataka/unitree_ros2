#ifndef UNITREE_CONTROLLER__UNITREE_CONTROLLER_INTERFACE_HPP_
#define UNITREE_CONTROLLER__UNITREE_CONTROLLER_INTERFACE_HPP_

#include <chrono>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/node_options.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/service.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "lifecycle_msgs/msg/state.hpp"

#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "unitree_legged_sdk/comm.h"
#include "unitree_hardware/hardware_interface_type_values.hpp"

#include "unitree_controller/types.hpp"
#include "unitree_controller/visibility_control.h"


namespace unitree_controller
{

using namespace std::chrono_literals;  // NOLINT

class UnitreeControllerInterface : public controller_interface::ControllerInterface
{
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

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

protected:
  /**
   * Derived controllers have to declare parameters in this method.
   * Error handling does not have to be done. It is done in `on_init`-method of this class.
   */
  virtual void declare_parameters() = 0;

  /**
   * Derived controllers have to read parameters in this method.  The method is called from 
   * `on_configure`-method of this class.
   *
   * It is expected that error handling of exceptions is done.
   *
   * \returns controller_interface::CallbackReturn::SUCCESS if parameters are successfully read and their values are
   * allowed, controller_interface::CallbackReturn::ERROR otherwise.
   */
  virtual controller_interface::CallbackReturn read_parameters() = 0;

  /**
   * Derived controllers have to return joint names that are retrived from ros parameters.
   * The variable is then used to propagate the state and command interface configuration 
   * to controller manager. 
   */
  virtual std::vector<std::string> get_joint_names() const = 0;

  /**
   * Derived controllers have to return sensor names that are retrived from ros parameters.
   * The variable is then used to propagate the state and command interface configuration 
   * to controller manager. 
   */
  virtual std::vector<std::string> get_sensor_names() const = 0;

  /**
   * Derived controllers have to implement control update in this method. The result must be set in `commands`.
   *
   * \param[in] time The time at the start of this control loop iteration
   * \param[in] period The measured time taken by the last control loop iteration
   * \param[in] states States of the robot 
   * \param[in] commands Commands to the robot
   * \returns controller_interface::return_type::OK if control update is successfully done, 
   * controller_interface::return_type::ERROR otherwise.
   */
  virtual controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period,
    const UnitreeStates & states, UnitreeCommands & commands) = 0;

private:
  // hardware interfaces 
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> 
      qJ_interface_, dqJ_interface_, tauJ_interface_, 
      imu_orientation_interface_, imu_angular_velocity_interface_, 
      imu_linear_acceleration_interface_, foot_force_sensor_interface_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> 
      qJ_cmd_interface_, dqJ_cmd_interface_, tauJ_cmd_interface_, 
      Kp_cmd_interface_, Kd_cmd_interface_;

  // states and commands
  UnitreeStates states_;
  UnitreeCommands commands_;
};

}  // namespace unitree_controller

#endif // UNITREE_CONTROLLER__UNITREE_CONTROLLER_INTERFACE_HPP_