#ifndef UNITREE_HARDWARE__UNITREE_HARDWARE_HPP_
#define UNITREE_HARDWARE__UNITREE_HARDWARE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <array>

#include "rclcpp/macros.hpp"

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "unitree_hardware/hardware_interface_type_values.hpp"
#include "unitree_hardware/visibility_control.h"

#include "unitree_legged_sdk/comm.h"
#include "unitree_legged_sdk/safety.h"
#include "unitree_legged_sdk/udp.h"

namespace unitree_hardware
{
class UnitreeHardware final : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(UnitreeHardware);

  UNITREE_HARDWARE_PUBLIC
  UnitreeHardware();

  UNITREE_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  UNITREE_HARDWARE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  UNITREE_HARDWARE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  UNITREE_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  UNITREE_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  UNITREE_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  UNITREE_HARDWARE_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  UNITREE_HARDWARE_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  UNITREE_LEGGED_SDK::UDP udp_;
  UNITREE_LEGGED_SDK::Safety safety_;
  UNITREE_LEGGED_SDK::LowState state_;
  UNITREE_LEGGED_SDK::LowCmd command_;

  std::vector<double> qJ_, dqJ_, tauJ_, 
                      imu_quaternion_, imu_gyroscope_, imu_accelerometer_, foot_force_sensor_;
  std::vector<double> qJ_cmd_, dqJ_cmd_, tauJ_cmd_, Kp_cmd_, Kd_cmd_;

  static constexpr std::array<int, 12> joints_ = {UNITREE_LEGGED_SDK::FL_0, 
                                                  UNITREE_LEGGED_SDK::FL_1, 
                                                  UNITREE_LEGGED_SDK::FL_2,
                                                  UNITREE_LEGGED_SDK::FR_0, 
                                                  UNITREE_LEGGED_SDK::FR_1, 
                                                  UNITREE_LEGGED_SDK::FR_2, 
                                                  UNITREE_LEGGED_SDK::RL_0, 
                                                  UNITREE_LEGGED_SDK::RL_1, 
                                                  UNITREE_LEGGED_SDK::RL_2, 
                                                  UNITREE_LEGGED_SDK::RR_0, 
                                                  UNITREE_LEGGED_SDK::RR_1, 
                                                  UNITREE_LEGGED_SDK::RR_2};

  static constexpr std::array<int, 4> feet_ = {UNITREE_LEGGED_SDK::FL_,
                                               UNITREE_LEGGED_SDK::FR_,
                                               UNITREE_LEGGED_SDK::RL_,
                                               UNITREE_LEGGED_SDK::RR_};

};

}  // namespace unitree_hardware

#endif  // UNITREE_HARDWARE__UNITREE_HARDWARE_HPP_