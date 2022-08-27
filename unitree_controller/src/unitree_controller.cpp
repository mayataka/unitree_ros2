#include "unitree_controller/unitree_controller.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"

namespace unitree_controller
{

UnitreeController::UnitreeController()
: UnitreeControllerBase(), update_rate_(0), update_period_(0) {}

void UnitreeController::declare_parameters() 
{
  // interfaces
  auto_declare<std::vector<std::string>>("joints", joint_names_);
  auto_declare<std::vector<std::string>>("sensors", sensor_names_);
  // node parameters
  auto_declare<int>("update_rate", 400);
}

controller_interface::CallbackReturn UnitreeController::read_parameters() 
{
  // interfaces
  joint_names_ = get_node()->get_parameter("joints").as_string_array();
  sensor_names_ = get_node()->get_parameter("sensors").as_string_array();
  // node parameters
  update_rate_  = static_cast<double>(get_node()->get_parameter("update_rate").get_value<int>());

  if (joint_names_.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "'joints' parameter was empty");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (sensor_names_.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "'sensors' parameter was empty");
    return controller_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(get_node()->get_logger(), "Controller will be updated at %.2f Hz.", update_rate_);
  if (update_rate_ > 0.0)
  {
    update_period_ = 1.0 / update_rate_; // seconds
  }
  else
  {
    RCLCPP_ERROR(get_node()->get_logger(), "'update_rate' must be positive, got %lf.", update_rate_);
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
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