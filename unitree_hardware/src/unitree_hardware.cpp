#include "unitree_hardware/unitree_hardware.hpp"

#include <chrono>
#include <cmath>
#include <limits>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace unitree_hardware
{
hardware_interface::return_type UnitreeHardware::configure(
  const hardware_interface::HardwareInfo & info)
{
  if (configure_default(info) != hardware_interface::return_type::OK)
  {
    return hardware_interface::return_type::ERROR;
  }
  udp_ = std::make_shared<UNITREE_LEGGED_SDK::UDP>(UNITREE_LEGGED_SDK::LOWLEVEL);
  safety_ = std::make_shared<UNITREE_LEGGED_SDK::Safety>(UNITREE_LEGGED_SDK::LeggedType::A1);
  state_ = UNITREE_LEGGED_SDK::LowState();
  cmd_ = UNITREE_LEGGED_SDK::LowCmd();
  cmd_.levelFlag = UNITREE_LEGGED_SDK::LOWLEVEL;
  udp_->InitCmdData(cmd_);
  // Joint states
  qJ_.fill(std::numeric_limits<double>::quiet_NaN());
  dqJ_.fill(std::numeric_limits<double>::quiet_NaN());
  tauJ_.fill(std::numeric_limits<double>::quiet_NaN());
  // Imu states
  imu_quaternion_.fill(std::numeric_limits<double>::quiet_NaN());
  imu_gyroscope_.fill(std::numeric_limits<double>::quiet_NaN());
  imu_accelerometer_.fill(std::numeric_limits<double>::quiet_NaN());
  // Foot force sensor states 
  foot_force_sensor_.fill(std::numeric_limits<double>::quiet_NaN());
  // Joint commands
  qJ_cmd_.fill(std::numeric_limits<double>::quiet_NaN());
  dqJ_cmd_.fill(std::numeric_limits<double>::quiet_NaN());
  tauJ_cmd_.fill(std::numeric_limits<double>::quiet_NaN());
  Kp_cmd_.fill(std::numeric_limits<double>::quiet_NaN());
  Kd_cmd_.fill(std::numeric_limits<double>::quiet_NaN());

  // check joint state and command interfaces
  if (info_.joints.size() != 12)
  {
    RCLCPP_FATAL(
      rclcpp::get_logger("UnitreeHardware"),
      "Number of joint is %d. 12 expected.", info_.joints.size());
    return hardware_interface::return_type::ERROR;
  }
  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // check joint state interfaces
    if (joint.state_interfaces.size() != 3)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("UnitreeHardware"),
        "Joint '%s'has %d state interfaces. 3 expected.", joint.name.c_str(), joint.state_interfaces.size());
      return hardware_interface::return_type::ERROR;
    }
    if (!(joint.state_interfaces[0].name == hardware_interface::HW_IF_POSITION ||
          joint.state_interfaces[0].name == hardware_interface::HW_IF_VELOCITY ||
          joint.state_interfaces[0].name == hardware_interface::HW_IF_EFFORT))
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("UnitreeHardware"),
        "Joint '%s' has %s state interfaces. Expected %s, %s, %s, or %s.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION,
        hardware_interface::HW_IF_VELOCITY, hardware_interface::HW_IF_EFFORT);
      return hardware_interface::return_type::ERROR;
    }
    // check joint command interfaces
    if (joint.command_interfaces.size() != 3)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("UnitreeHardware"),
        "Joint '%s' has %d command interfaces. 3 expected.", joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::return_type::ERROR;
    }

    if (!(joint.command_interfaces[0].name == hardware_interface::HW_IF_POSITION ||
          joint.command_interfaces[0].name == hardware_interface::HW_IF_VELOCITY ||
          joint.command_interfaces[0].name == hardware_interface::HW_IF_EFFORT))
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("UnitreeHardware"),
        "Joint '%s' has %s command interface. Expected %s, %s, or %s.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION,
        hardware_interface::HW_IF_VELOCITY, hardware_interface::HW_IF_EFFORT);

      return hardware_interface::return_type::ERROR;
    }
  }

  // check Imu state interfaces
  if (info_.sensors[0].state_interfaces.size() != 10)
  {
    RCLCPP_FATAL(
      rclcpp::get_logger("UnitreeHardware"),
      "Sensor[0] (should be Imu) has %d state interfaces. 10 expected.", info_.sensors[0].state_interfaces.size());
    return hardware_interface::return_type::ERROR;
  }
  if (!(info_.sensors[0].state_interfaces[0].name == "orientation.x"))
  {
    RCLCPP_FATAL(
      rclcpp::get_logger("UnitreeHardware"),
      "Sensor[0] (should be Imu) has %s state interface at state_interfaces[0]. Expected orientation.x",
      info_.sensors[0].state_interfaces[0].name.c_str());
    return hardware_interface::return_type::ERROR;
  }
  if (!(info_.sensors[0].state_interfaces[1].name == "orientation.y"))
  {
    RCLCPP_FATAL(
      rclcpp::get_logger("UnitreeHardware"),
      "Sensor[0] (should be Imu) has %s state interface at state_interfaces[1]. Expected orientation.y",
      info_.sensors[0].state_interfaces[1].name.c_str());
    return hardware_interface::return_type::ERROR;
  }
  if (!(info_.sensors[0].state_interfaces[2].name == "orientation.z"))
  {
    RCLCPP_FATAL(
      rclcpp::get_logger("UnitreeHardware"),
      "Sensor[0] (should be Imu) has %s state interface at state_interfaces[2]. Expected orientation.z",
      info_.sensors[0].state_interfaces[2].name.c_str());
    return hardware_interface::return_type::ERROR;
  }
  if (!(info_.sensors[0].state_interfaces[3].name == "orientation.w"))
  {
    RCLCPP_FATAL(
      rclcpp::get_logger("UnitreeHardware"),
      "Sensor[0] (should be Imu) has %s state interface at state_interfaces[3]. Expected orientation.w",
      info_.sensors[0].state_interfaces[3].name.c_str());
    return hardware_interface::return_type::ERROR;
  }

  if (!(info_.sensors[0].state_interfaces[4].name == "angular_velocity.x"))
  {
    RCLCPP_FATAL(
      rclcpp::get_logger("UnitreeHardware"),
      "Sensor[0] (should be Imu) has %s state interface at state_interfaces[4]. Expected angular_velocity.x",
      info_.sensors[0].state_interfaces[4].name.c_str());
    return hardware_interface::return_type::ERROR;
  }
  if (!(info_.sensors[0].state_interfaces[5].name == "angular_velocity.y"))
  {
    RCLCPP_FATAL(
      rclcpp::get_logger("UnitreeHardware"),
      "Sensor[0] (should be Imu) has %s state interface at state_interfaces[5]. Expected angular_velocity.y",
      info_.sensors[0].state_interfaces[5].name.c_str());
    return hardware_interface::return_type::ERROR;
  }
  if (!(info_.sensors[0].state_interfaces[6].name == "angular_velocity.z"))
  {
    RCLCPP_FATAL(
      rclcpp::get_logger("UnitreeHardware"),
      "Sensor[0] (should be Imu) has %s state interface at state_interfaces[6]. Expected angular_velocity.z",
      info_.sensors[0].state_interfaces[6].name.c_str());
    return hardware_interface::return_type::ERROR;
  }
  if (!(info_.sensors[0].state_interfaces[7].name == "linear_acceleration.x"))
  {
    RCLCPP_FATAL(
      rclcpp::get_logger("UnitreeHardware"),
      "Sensor[0] (should be Imu) has %s state interface at state_interfaces[7]. Expected linear_acceleration.x",
      info_.sensors[0].state_interfaces[7].name.c_str());
    return hardware_interface::return_type::ERROR;
  }
  if (!(info_.sensors[0].state_interfaces[8].name == "linear_acceleration.y"))
  {
    RCLCPP_FATAL(
      rclcpp::get_logger("UnitreeHardware"),
      "Sensor[0] (should be Imu) has %s state interface at state_interfaces[8]. Expected linear_acceleration.y",
      info_.sensors[0].state_interfaces[8].name.c_str());
    return hardware_interface::return_type::ERROR;
  }
  if (!(info_.sensors[0].state_interfaces[9].name == "linear_acceleration.z"))
  {
    RCLCPP_FATAL(
      rclcpp::get_logger("UnitreeHardware"),
      "Sensor[0] (should be Imu) has %s state interface at state_interfaces[9]. Expected linear_acceleration.z",
      info_.sensors[0].state_interfaces[9].name.c_str());
    return hardware_interface::return_type::ERROR;
  }

  // check foot force sensor interfaces
  for (std::size_t i = 0; i < 4; i++)
  {
    if (info_.sensors[i+1].state_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("UnitreeHardware"),
        "Sensor[%d] (should be foot force sensor) has %d state interfaces. 1 expected.", i+1, info_.sensors[0].state_interfaces.size());
      return hardware_interface::return_type::ERROR;
    }
    if (!(info_.sensors[i+1].state_interfaces[0].name == "force.z"))
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("UnitreeHardware"),
        "Sensor[%d] (should be foot force sensor) has %s state interface. Expected force.z", 
        i+1, info_.sensors[i+1].state_interfaces[0].name.c_str());
      return hardware_interface::return_type::ERROR;
    }
  }

  status_ = hardware_interface::status::CONFIGURED;
  return hardware_interface::return_type::OK;
}

std::vector<hardware_interface::StateInterface>
UnitreeHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  // Joint state
  for (std::size_t i = 0; i < 12; i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &qJ_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &dqJ_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &tauJ_[i]));
  }
  // Imu state
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    info_.sensors[0].name, info_.sensors[0].state_interfaces[0].name, &imu_quaternion_[0]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    info_.sensors[0].name, info_.sensors[0].state_interfaces[1].name, &imu_quaternion_[1]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    info_.sensors[0].name, info_.sensors[0].state_interfaces[2].name, &imu_quaternion_[2]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    info_.sensors[0].name, info_.sensors[0].state_interfaces[3].name, &imu_quaternion_[3]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    info_.sensors[0].name, info_.sensors[0].state_interfaces[4].name, &imu_gyroscope_[0]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    info_.sensors[0].name, info_.sensors[0].state_interfaces[5].name, &imu_gyroscope_[1]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    info_.sensors[0].name, info_.sensors[0].state_interfaces[6].name, &imu_gyroscope_[2]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    info_.sensors[0].name, info_.sensors[0].state_interfaces[7].name, &imu_accelerometer_[0]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    info_.sensors[0].name, info_.sensors[0].state_interfaces[8].name, &imu_accelerometer_[1]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    info_.sensors[0].name, info_.sensors[0].state_interfaces[9].name, &imu_accelerometer_[2]));
  // Foot force sensors 
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    info_.sensors[1].name, info_.sensors[1].state_interfaces[0].name, &foot_force_sensor_[0]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    info_.sensors[2].name, info_.sensors[2].state_interfaces[0].name, &foot_force_sensor_[1]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    info_.sensors[3].name, info_.sensors[3].state_interfaces[0].name, &foot_force_sensor_[2]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    info_.sensors[4].name, info_.sensors[4].state_interfaces[0].name, &foot_force_sensor_[3]));
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
UnitreeHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (std::size_t i = 0; i < 12; i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &qJ_cmd_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &dqJ_cmd_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &tauJ_cmd_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, "Kp", &Kp_cmd_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, "Kd", &Kd_cmd_[i]));
  }
  return command_interfaces;
}

hardware_interface::return_type UnitreeHardware::start()
{
  RCLCPP_INFO(
    rclcpp::get_logger("UnitreeHardware"), "Starting... please wait...");

  // Set some default values
  for (std::size_t i = 0; i < 12; i++)
  {
    if (std::isnan(qJ_[i]))
    {
      qJ_[i] = 0;
    }
    if (std::isnan(dqJ_[i]))
    {
      dqJ_[i] = 0;
    }
    if (std::isnan(tauJ_[i]))
    {
      tauJ_[i] = 0;
    }
    if (std::isnan(qJ_cmd_[i]))
    {
      qJ_cmd_[i] = 0;
    }
    if (std::isnan(dqJ_cmd_[i]))
    {
      dqJ_cmd_[i] = 0;
    }
    if (std::isnan(tauJ_cmd_[i]))
    {
      tauJ_cmd_[i] = 0;
    }
    if (std::isnan(Kp_cmd_[i]))
    {
      Kp_cmd_[i] = 0;
    }
    if (std::isnan(Kd_cmd_[i]))
    {
      Kd_cmd_[i] = 0;
    }
  }
  status_ = hardware_interface::status::STARTED;

  RCLCPP_INFO(
    rclcpp::get_logger("UnitreeHardware"), "System successfully started!");
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type UnitreeHardware::stop()
{
  RCLCPP_INFO(
    rclcpp::get_logger("UnitreeHardware"), "Stopping... please wait...");
  status_ = hardware_interface::status::STOPPED;
  RCLCPP_INFO(
    rclcpp::get_logger("UnitreeHardware"), "System successfully stopped!");

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type UnitreeHardware::read()
{
  udp_->Recv();
  udp_->GetRecv(state_);
  // Joint state
  for (std::size_t i = 0; i < 12; i++)
  {
    qJ_[i]   = static_cast<double>(state_.motorState[joints_[i]].q);
    dqJ_[i]  = static_cast<double>(state_.motorState[joints_[i]].dq);
    tauJ_[i] = static_cast<double>(state_.motorState[joints_[i]].tauEst);
  }
  // Imu state
  imu_quaternion_[0] = static_cast<double>(state_.imu.quaternion[1]); // x
  imu_quaternion_[1] = static_cast<double>(state_.imu.quaternion[2]); // y
  imu_quaternion_[2] = static_cast<double>(state_.imu.quaternion[3]); // z 
  imu_quaternion_[3] = static_cast<double>(state_.imu.quaternion[0]); // w
  for (std::size_t i = 0; i < 3; i++)
  {
    imu_gyroscope_[i] = static_cast<double>(state_.imu.gyroscope[i]);
  }
  for (std::size_t i = 0; i < 3; i++)
  {
    imu_accelerometer_[i] = static_cast<double>(state_.imu.accelerometer[i]);
  }
  // Foot force sensor
  for (std::size_t i = 0; i < 4; i++)
  {
    foot_force_sensor_[i] = static_cast<double>(state_.footForce[feet_[i]]);
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type UnitreeHardware::write()
{
  for (std::size_t i = 0; i < 12; i++)
  {
    cmd_.motorCmd[joints_[i]].q   = static_cast<float>(qJ_cmd_[i]);
    cmd_.motorCmd[joints_[i]].dq  = static_cast<float>(dqJ_cmd_[i]);
    cmd_.motorCmd[joints_[i]].tau = static_cast<float>(tauJ_cmd_[i]);
    cmd_.motorCmd[joints_[i]].Kp  = static_cast<float>(Kp_cmd_[i]);
    cmd_.motorCmd[joints_[i]].Kd  = static_cast<float>(Kd_cmd_[i]);
  }
  return hardware_interface::return_type::OK;
}

}  // namespace unitree_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  unitree_hardware::UnitreeHardware,
  hardware_interface::SystemInterface)