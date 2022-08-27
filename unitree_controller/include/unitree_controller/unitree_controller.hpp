#ifndef UNITREE_CONTROLLER__UNITREE_CONTROLLER_HPP_
#define UNITREE_CONTROLLER__UNITREE_CONTROLLER_HPP_

#include "Eigen/Core"
#include "Eigen/Geometry"

#include "unitree_controller/unitree_controller_interface.hpp"


namespace unitree_controller
{

using namespace std::chrono_literals;  // NOLINT

class UnitreeController : public UnitreeControllerInterface
{
private:
  controller_interface::CallbackReturn on_init_impl() override;

  controller_interface::CallbackReturn on_configure_impl(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update_impl(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  bool reset_impl() override;
};

}  // namespace unitree_controller

#endif // UNITREE_CONTROLLER__UNITREE_CONTROLLER_HPP_