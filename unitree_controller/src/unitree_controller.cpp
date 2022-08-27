#include "unitree_controller/unitree_controller.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"

namespace unitree_controller
{

controller_interface::CallbackReturn UnitreeController::on_init_impl()
{
  // try 
  // {
  // }
  // catch (const std::exception & e) {
  //   fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
  //   return controller_interface::CallbackReturn::ERROR;
  // }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn UnitreeController::on_configure_impl(
  const rclcpp_lifecycle::State & previous_state) {
  // 
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type UnitreeController::update_impl(
    const rclcpp::Time & time, const rclcpp::Duration & period) 
{
  auto logger = get_node()->get_logger();

  const auto& imu_quat = getImuOrientation();
  RCLCPP_INFO(logger, "Quat:  %lf, %lf, %lf, %lf.", imu_quat.x(), imu_quat.y(), imu_quat.z(), imu_quat.w());

  getJointPositionCommands().setZero();
  getJointVelocityCommands().setZero();
  getJointTorqueCommands().setZero();
  getJointPositionGainCommands().setZero();
  getJointVelocityGainCommands().setZero();

  return controller_interface::return_type::OK;
}

bool UnitreeController::reset_impl()
{
  return true;
}

}  // namespace unitree_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  unitree_controller::UnitreeController, 
  controller_interface::ControllerInterface)