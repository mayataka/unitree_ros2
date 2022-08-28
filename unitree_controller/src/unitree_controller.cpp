#include "unitree_controller/unitree_controller.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"

namespace unitree_controller
{

UnitreeController::UnitreeController()
: UnitreeControllerBase(), 
  joint_names_({}), 
  sensor_names_({}),
  control_rate_(0), 
  control_period_(0),
  state_estimator_() {}

void UnitreeController::declare_parameters() 
{
  // interfaces
  auto_declare<std::vector<std::string>>("joints", joint_names_);
  auto_declare<std::vector<std::string>>("sensors", sensor_names_);
  // node parameters
  auto_declare<int>("control_rate", 400);
}

controller_interface::CallbackReturn UnitreeController::read_parameters() 
{
  // interfaces
  joint_names_ = get_node()->get_parameter("joints").as_string_array();
  sensor_names_ = get_node()->get_parameter("sensors").as_string_array();
  // node parameters
  control_rate_  = static_cast<double>(get_node()->get_parameter("control_rate").get_value<int>());

  if (joint_names_.size() != 12)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "'joints' parameter has wrong size");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (sensor_names_.size() != 5)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "'sensors' parameter has wrong size");
    return controller_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(get_node()->get_logger(), "Controller will be updated at %.2f Hz.", control_rate_);
  if (control_rate_ > 0.0)
  {
    control_period_ = 1.0 / control_rate_; // seconds
  }
  else
  {
    RCLCPP_ERROR(get_node()->get_logger(), "'control_rate_' must be positive, got %lf.", control_rate_);
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

std::vector<std::string> UnitreeController::get_joint_names() const {
  return joint_names_;  
}

std::vector<std::string> UnitreeController::get_sensor_names() const {
  return sensor_names_;  
}

controller_interface::return_type UnitreeController::update(
    const rclcpp::Time & time, const rclcpp::Duration & period,
    const UnitreeStates & states, UnitreeCommands & commands) 
{
  RCLCPP_INFO(get_node()->get_logger(), "derived time: %.2f [s]", time.seconds());
  return controller_interface::return_type::OK;
}

} // namespace unitree_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  unitree_controller::UnitreeController, 
  controller_interface::ControllerInterface)