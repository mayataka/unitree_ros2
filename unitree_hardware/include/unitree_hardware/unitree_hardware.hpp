#ifndef UNITREE_HARDWARE__UNITREE_HARDWARE_HPP_
#define UNITREE_HARDWARE__UNITREE_HARDWARE_HPP_

#include <cstdint>
#include <memory>
#include <string>
#include <vector>
#include <array>

#include "rclcpp/macros.hpp"

#include "hardware_interface/base_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_status_values.hpp"
#include "unitree_hardware/visibility_control.h"

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "unitree_legged_sdk/udp.h"
#include "unitree_legged_sdk/safety.h"

namespace unitree_hardware
{
class UnitreeHardware final 
: public hardware_interface::BaseInterface<hardware_interface::SystemInterface>
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(UnitreeHardware);

  UNITREE_HARDWARE_PUBLIC
  hardware_interface::return_type configure(const hardware_interface::HardwareInfo & info) override;

  UNITREE_HARDWARE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  UNITREE_HARDWARE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  UNITREE_HARDWARE_PUBLIC
  hardware_interface::return_type start() override;

  UNITREE_HARDWARE_PUBLIC
  hardware_interface::return_type stop() override;

  UNITREE_HARDWARE_PUBLIC
  hardware_interface::return_type read() override;

  UNITREE_HARDWARE_PUBLIC
  hardware_interface::return_type write() override;

private:
  std::shared_ptr<UNITREE_LEGGED_SDK::UDP> udp_;
  std::shared_ptr<UNITREE_LEGGED_SDK::Safety> safety_;
  UNITREE_LEGGED_SDK::LowState state_;
  UNITREE_LEGGED_SDK::LowCmd cmd_;

  std::array<double, 12> qJ_;
  std::array<double, 12> dqJ_;
  std::array<double, 12> ddqJ_;
  std::array<double, 12> tauJ_;
  std::array<double, 4> imu_quaternion_;
  std::array<double, 3> imu_gyroscope_;
  std::array<double, 3> imu_accelerometer_;
  std::array<double, 4> foot_force_sensor_;

  std::array<double, 12> qJ_cmd_;
  std::array<double, 12> dqJ_cmd_;
  std::array<double, 12> tauJ_cmd_;
  std::array<double, 12> Kp_cmd_;
  std::array<double, 12> Kd_cmd_;

  std::array<int, 12> joints_ = {UNITREE_LEGGED_SDK::FL_0, 
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
  std::array<int, 4> feet_ = {UNITREE_LEGGED_SDK::FL_,
                              UNITREE_LEGGED_SDK::FR_,
                              UNITREE_LEGGED_SDK::RL_,
                              UNITREE_LEGGED_SDK::RR_};

};

}  // namespace unitree_hardware

#endif  // UNITREE_HARDWARE__UNITREE_HARDWARE_HPP_ 