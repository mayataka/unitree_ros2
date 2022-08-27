#include "unitree_controller/unitree_controller.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"

namespace unitree_controller
{

UnitreeController::UnitreeController()
: UnitreeControllerBase() {}

void UnitreeController::declare_parameters() 
{
  // // node parameters
  // auto_declare<int>("update_rate", 400);
}

controller_interface::CallbackReturn UnitreeController::read_parameters() 
{
  // // interfaces
  // joint_names_ = get_node()->get_parameter("joints").as_string_array();
  // sensor_names_ = get_node()->get_parameter("sensors").as_string_array();
  // // node parameters
  // update_rate_  = static_cast<double>(get_node()->get_parameter("update_rate").get_value<int>());

  // const auto logger = get_node()->get_logger();

  // RCLCPP_INFO(logger, "Controller will be updated at %.2f Hz.", update_rate_);
  // if (update_rate_ > 0.0)
  // {
  //   update_period_ = 1.0 / update_rate_; // seconds
  // }
  // else
  // {
  //   RCLCPP_ERROR(logger, "'update_rate' must be positive, got %lf.", update_rate_);
  //   return controller_interface::CallbackReturn::ERROR;
  // }
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