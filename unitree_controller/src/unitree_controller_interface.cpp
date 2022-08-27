#include "unitree_controller/unitree_controller_interface.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "controller_interface/helpers.hpp"


namespace unitree_controller
{

UnitreeControllerInterface::UnitreeControllerInterface()
: controller_interface::ControllerInterface(), 
  joint_names_({}), 
  sensor_names_({}), 
  qJ_interface_(), 
  dqJ_interface_(), 
  tauJ_interface_(), 
  imu_orientation_interface_(), 
  imu_angular_velocity_interface_(), 
  imu_linear_acceleration_interface_(), 
  foot_force_sensor_interface_(),
  qJ_cmd_interface_(), 
  dqJ_cmd_interface_(), 
  tauJ_cmd_interface_(), 
  Kp_cmd_interface_(), 
  Kd_cmd_interface_(),
  update_rate_(0),
  update_period_(0),
  qJ_(Vector12d::Zero()),
  dqJ_(Vector12d::Zero()),
  tauJ_(Vector12d::Zero()),
  imu_quat_(Quaterniond::Identity()),
  imu_ang_vel_(Vector3d::Zero()),
  imu_lin_acc_(Vector3d::Zero()),
  f_(Vector4d::Zero()),
  qJ_cmd_(Vector12d::Zero()),
  dqJ_cmd_(Vector12d::Zero()),
  tauJ_cmd_(Vector12d::Zero()),
  Kp_cmd_(Vector12d::Zero()),
  Kd_cmd_(Vector12d::Zero())
{
}

controller_interface::CallbackReturn UnitreeControllerInterface::on_init()
{
  try 
  {
    // node parameters
    auto_declare<int>("update_rate", 400); // Hz
    // interfaces
    auto_declare<std::vector<std::string>>("joints", joint_names_);
    auto_declare<std::vector<std::string>>("sensors", sensor_names_);
  }
  catch (const std::exception & e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  // implementation in the derived class
  if (on_init_impl() == controller_interface::CallbackReturn::ERROR) 
  {
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration 
UnitreeControllerInterface::command_interface_configuration() const
{
  const std::vector<std::string> joint_command_interface_types = { hardware_interface::HW_IF_POSITION,
                                                                   hardware_interface::HW_IF_VELOCITY,
                                                                   hardware_interface::HW_IF_EFFORT,
                                                                   unitree_hardware::HW_IF_POSITION_GAIN,
                                                                   unitree_hardware::HW_IF_VELOCITY_GAIN};
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  conf.names.reserve(joint_names_.size() * joint_command_interface_types.size()); // Joint commands
  for (const auto & joint_name : joint_names_)
  {
    for (const auto & interface_type : joint_command_interface_types)
    {
      conf.names.push_back(joint_name + "/" + interface_type);
    }
  }
  return conf;
}

controller_interface::InterfaceConfiguration
UnitreeControllerInterface::state_interface_configuration() const
{
  const std::vector<std::string> joint_state_interface_types = { hardware_interface::HW_IF_POSITION,
                                                                 hardware_interface::HW_IF_VELOCITY,
                                                                 hardware_interface::HW_IF_EFFORT};
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  conf.names.reserve(joint_names_.size() * joint_state_interface_types.size()  // Joint states
                      + 4 + 3 + 3  // Imu states (quat + gyro + acc)
                      + 4); // Foot force sensors 
  // Joint state
  for (const auto & joint_name : joint_names_)
  {
    for (const auto & interface_type : joint_state_interface_types)
    {
      conf.names.push_back(joint_name + "/" + interface_type);
    }
  }
  // Imu state
  conf.names.push_back(sensor_names_[0] + "/" + "orientation.x");
  conf.names.push_back(sensor_names_[0] + "/" + "orientation.y");
  conf.names.push_back(sensor_names_[0] + "/" + "orientation.z");
  conf.names.push_back(sensor_names_[0] + "/" + "orientation.w");
  conf.names.push_back(sensor_names_[0] + "/" + "angular_velocity.x");
  conf.names.push_back(sensor_names_[0] + "/" + "angular_velocity.y");
  conf.names.push_back(sensor_names_[0] + "/" + "angular_velocity.z");
  conf.names.push_back(sensor_names_[0] + "/" + "linear_acceleration.x");
  conf.names.push_back(sensor_names_[0] + "/" + "linear_acceleration.y");
  conf.names.push_back(sensor_names_[0] + "/" + "linear_acceleration.z");
  // Foot force sensor states
  conf.names.push_back(sensor_names_[1] + "/" + "force.z");
  conf.names.push_back(sensor_names_[2] + "/" + "force.z");
  conf.names.push_back(sensor_names_[3] + "/" + "force.z");
  conf.names.push_back(sensor_names_[4] + "/" + "force.z");
  return conf;
}

controller_interface::return_type UnitreeControllerInterface::update(
    const rclcpp::Time & time, const rclcpp::Duration & period) 
{
  if (get_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
  {
    return controller_interface::return_type::OK;
  }
  // get joint state
  for (std::size_t i = 0 ; i < 12; ++i)
  {
    qJ_.coeffRef(i)   = qJ_interface_[i].get().get_value();
    dqJ_.coeffRef(i)  = dqJ_interface_[i].get().get_value();
    tauJ_.coeffRef(i) = tauJ_interface_[i].get().get_value();
  }
  // get Imu state
  imu_quat_.x() = imu_orientation_interface_[0].get().get_value();
  imu_quat_.y() = imu_orientation_interface_[1].get().get_value();
  imu_quat_.z() = imu_orientation_interface_[2].get().get_value();
  imu_quat_.w() = imu_orientation_interface_[3].get().get_value();
  for (std::size_t i = 0 ; i < 3; ++i) 
  {
    imu_ang_vel_.coeffRef(i) = imu_angular_velocity_interface_[i].get().get_value();
  }
  for (std::size_t i = 0 ; i < 3; ++i) 
  {
    imu_lin_acc_.coeffRef(i) = imu_linear_acceleration_interface_[i].get().get_value();
  }
  // get foor force sensor states 
  for (std::size_t i = 0 ; i < 4; ++i) 
  {
    f_.coeffRef(i) = foot_force_sensor_interface_[i].get().get_value();
  }

  // implementation in the derived class
  const auto return_value = update_impl(time, period);

  // set joint commands
  for (std::size_t i = 0 ; i < 12; ++i)
  {
    qJ_cmd_interface_[i].get().set_value(qJ_cmd_.coeff(i));
    dqJ_cmd_interface_[i].get().set_value(dqJ_cmd_.coeff(i));
    tauJ_cmd_interface_[i].get().set_value(tauJ_cmd_.coeff(i));
    Kp_cmd_interface_[i].get().set_value(Kp_cmd_.coeff(i));
    Kd_cmd_interface_[i].get().set_value(Kd_cmd_.coeff(i));
  }

  return return_value;
}

controller_interface::CallbackReturn
UnitreeControllerInterface::on_configure(const rclcpp_lifecycle::State & previous_state)
{
  auto logger = get_node()->get_logger();

  // update parameters
  joint_names_ = get_node()->get_parameter("joints").as_string_array();
  sensor_names_ = get_node()->get_parameter("sensors").as_string_array();

  if (!reset())
  {
    return controller_interface::CallbackReturn::FAILURE;
  }

  if (joint_names_.empty())
  {
    RCLCPP_ERROR(logger, "'joints' parameter is empty.");
    return controller_interface::CallbackReturn::FAILURE;
  }
  if (sensor_names_.empty())
  {
    RCLCPP_ERROR(logger, "'sensors' parameter is empty.");
    return controller_interface::CallbackReturn::FAILURE;
  }

  // Node parameter
  update_rate_ = static_cast<double>(get_node()->get_parameter("update_rate").get_value<int>());
  RCLCPP_INFO(logger, "Controller will be updated at %.2f Hz.", update_rate_);
  if (update_rate_ > 0.0)
  {
    update_period_ = 1.0 / update_rate_; // seconds
  }
  else
  {
    RCLCPP_ERROR(logger, "'update_rate' must be positive, got %lf.", update_rate_);
    return controller_interface::CallbackReturn::ERROR;
  }

  // implementation in the derived class
  const auto return_value = on_configure_impl(previous_state);

  return return_value;
}

controller_interface::CallbackReturn
UnitreeControllerInterface::on_activate(const rclcpp_lifecycle::State &)
{
  // Joint state interfaces
  qJ_interface_.clear();
  dqJ_interface_.clear();
  tauJ_interface_.clear();
  if (!controller_interface::get_ordered_interfaces(state_interfaces_, joint_names_, 
                                                    hardware_interface::HW_IF_POSITION, qJ_interface_)) 
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Expected %zu qJ_interface, got %zu.", 
      12, qJ_interface_.size());
    return controller_interface::CallbackReturn::ERROR;
  }
  if (!controller_interface::get_ordered_interfaces(state_interfaces_, joint_names_, 
                                                    hardware_interface::HW_IF_VELOCITY, dqJ_interface_)) 
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Expected %zu dqJ_interface, got %zu.", 
      12, dqJ_interface_.size());
    return controller_interface::CallbackReturn::ERROR;
  }
  if (!controller_interface::get_ordered_interfaces(state_interfaces_, joint_names_, 
                                                    hardware_interface::HW_IF_EFFORT, tauJ_interface_))
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Expected %zu tauJ_interface, got %zu.", 
      12, tauJ_interface_.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  // Imu state interfaces
  imu_orientation_interface_.clear();
  imu_angular_velocity_interface_.clear();
  imu_linear_acceleration_interface_.clear();
  controller_interface::get_ordered_interfaces(state_interfaces_, {sensor_names_[0]}, 
                                               "orientation.x", imu_orientation_interface_);
  controller_interface::get_ordered_interfaces(state_interfaces_, {sensor_names_[0]}, 
                                               "orientation.y", imu_orientation_interface_);
  controller_interface::get_ordered_interfaces(state_interfaces_, {sensor_names_[0]}, 
                                               "orientation.z", imu_orientation_interface_);
  controller_interface::get_ordered_interfaces(state_interfaces_, {sensor_names_[0]}, 
                                               "orientation.w", imu_orientation_interface_);
  if (imu_orientation_interface_.size() != 4) 
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Expected %zu imu_orientation_interface, got %zu.", 
      4, imu_orientation_interface_.size());
    return controller_interface::CallbackReturn::ERROR;
  }
  controller_interface::get_ordered_interfaces(state_interfaces_, {sensor_names_[0]}, 
                                               "angular_velocity.x", imu_angular_velocity_interface_);
  controller_interface::get_ordered_interfaces(state_interfaces_, {sensor_names_[0]}, 
                                               "angular_velocity.y", imu_angular_velocity_interface_);
  controller_interface::get_ordered_interfaces(state_interfaces_, {sensor_names_[0]}, 
                                               "angular_velocity.z", imu_angular_velocity_interface_);
  if (imu_angular_velocity_interface_.size() != 3) 
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Expected %zu imu_angular_velocity_interface, got %zu.", 
      3, imu_orientation_interface_.size());
    return controller_interface::CallbackReturn::ERROR;
  }
  controller_interface::get_ordered_interfaces(state_interfaces_, {sensor_names_[0]}, 
                                               "linear_acceleration.x", imu_linear_acceleration_interface_);
  controller_interface::get_ordered_interfaces(state_interfaces_, {sensor_names_[0]}, 
                                               "linear_acceleration.y", imu_linear_acceleration_interface_);
  controller_interface::get_ordered_interfaces(state_interfaces_, {sensor_names_[0]}, 
                                               "linear_acceleration.z", imu_linear_acceleration_interface_);
  if (imu_linear_acceleration_interface_.size() != 3) 
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Expected %zu imu_linear_acceleration_interface, got %zu.", 
      3, imu_linear_acceleration_interface_.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  // Foot force sensor interfaces
  foot_force_sensor_interface_.clear();
  for (std::size_t i = 0 ; i < 4; ++i) 
  {
    controller_interface::get_ordered_interfaces(state_interfaces_, {sensor_names_[i+1]}, 
                                                 "force.z", foot_force_sensor_interface_);
  }
  if (foot_force_sensor_interface_.size() != 4) 
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Expected %zu foot_force_interface, got %zu.", 
      4, foot_force_sensor_interface_.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  // Joint command interfaces
  qJ_cmd_interface_.clear();
  dqJ_cmd_interface_.clear();
  tauJ_cmd_interface_.clear();
  Kp_cmd_interface_.clear();
  Kd_cmd_interface_.clear();
  if (!controller_interface::get_ordered_interfaces(command_interfaces_, joint_names_, 
                                                    hardware_interface::HW_IF_POSITION, qJ_cmd_interface_))
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Expected %zu qJ_cmd_interface, got %zu.", 
      12, qJ_cmd_interface_.size());
    return controller_interface::CallbackReturn::ERROR;
  }
  if (!controller_interface::get_ordered_interfaces(command_interfaces_, joint_names_, 
                                                    hardware_interface::HW_IF_VELOCITY, dqJ_cmd_interface_))
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Expected %zu dqJ_cmd_interface, got %zu.", 
      12, dqJ_cmd_interface_.size());
    return controller_interface::CallbackReturn::ERROR;
  }
  if (!controller_interface::get_ordered_interfaces(command_interfaces_, joint_names_, 
                                                    hardware_interface::HW_IF_EFFORT, tauJ_cmd_interface_)) 
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Expected %zu tauJ_cmd_interface, got %zu.", 
      12, tauJ_cmd_interface_.size());
    return controller_interface::CallbackReturn::ERROR;
  }
  if (!controller_interface::get_ordered_interfaces(command_interfaces_, joint_names_, 
                                                    "Kp", Kp_cmd_interface_)) 
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Expected %zu Kp_cmd_interface, got %zu.", 
      12, Kp_cmd_interface_.size());
    return controller_interface::CallbackReturn::ERROR;
  }
  if (!controller_interface::get_ordered_interfaces(command_interfaces_, joint_names_, 
                                                    "Kd", Kd_cmd_interface_)) 
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Expected %zu Kp_cmd_interface, got %zu.", 
      12, Kd_cmd_interface_.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  // get initial joint state
  for (std::size_t i = 0 ; i < 12; ++i)
  {
    qJ_.coeffRef(i)   = qJ_interface_[i].get().get_value();
    dqJ_.coeffRef(i)  = dqJ_interface_[i].get().get_value();
    tauJ_.coeffRef(i) = tauJ_interface_[i].get().get_value();
  }

  // get Imu state
  imu_quat_.x() = imu_orientation_interface_[0].get().get_value();
  imu_quat_.y() = imu_orientation_interface_[1].get().get_value();
  imu_quat_.z() = imu_orientation_interface_[2].get().get_value();
  imu_quat_.w() = imu_orientation_interface_[3].get().get_value();
  for (std::size_t i = 0 ; i < 3; ++i) 
  {
    imu_ang_vel_.coeffRef(i) = imu_angular_velocity_interface_[i].get().get_value();
  }
  for (std::size_t i = 0 ; i < 3; ++i) 
  {
    imu_lin_acc_.coeffRef(i) = imu_linear_acceleration_interface_[i].get().get_value();
  }

  // get foot force sensor values
  for (std::size_t i = 0 ; i < 4; ++i) 
  {
    f_.coeffRef(i) = foot_force_sensor_interface_[i].get().get_value();
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
UnitreeControllerInterface::on_deactivate(const rclcpp_lifecycle::State &)
{
  for (std::size_t i = 0; i < 12; ++i) 
  {
    qJ_cmd_interface_[i].get().set_value(UNITREE_LEGGED_SDK::PosStopF);
    dqJ_cmd_interface_[i].get().set_value(UNITREE_LEGGED_SDK::VelStopF);
    tauJ_cmd_interface_[i].get().set_value(0.0);
    Kp_cmd_interface_[i].get().set_value(0.0);
    Kd_cmd_interface_[i].get().set_value(0.0);
  }

  qJ_interface_.clear();
  dqJ_interface_.clear();
  tauJ_interface_.clear();
  imu_orientation_interface_.clear();
  imu_angular_velocity_interface_.clear();
  imu_linear_acceleration_interface_.clear();
  foot_force_sensor_interface_.clear();

  qJ_cmd_interface_.clear();
  dqJ_cmd_interface_.clear();
  tauJ_cmd_interface_.clear();
  Kp_cmd_interface_.clear();
  Kd_cmd_interface_.clear();

  release_interfaces();

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
UnitreeControllerInterface::on_cleanup(const rclcpp_lifecycle::State &)
{
  // TODO 
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
UnitreeControllerInterface::on_error(const rclcpp_lifecycle::State &)
{
  if (!reset())
  {
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

bool UnitreeControllerInterface::reset()
{
  // implementation in the derived class
  return reset_impl();
}

controller_interface::CallbackReturn
UnitreeControllerInterface::on_shutdown(const rclcpp_lifecycle::State &)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

}  // namespace unitree_controller