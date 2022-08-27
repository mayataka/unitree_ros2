#ifndef UNITREE_CONTROLLER__UNITREE_CONTROLLER_HPP_
#define UNITREE_CONTROLLER__UNITREE_CONTROLLER_HPP_

#include "unitree_controller/unitree_controller_base.hpp"
#include "unitree_controller/types.hpp"
#include "unitree_controller/visibility_control.h"


namespace unitree_controller
{

using namespace std::chrono_literals;  // NOLINT

class UnitreeController : public UnitreeControllerBase
{
public:
  UNITREE_CONTROLLER_PUBLIC 
  UnitreeController();

protected:
  void declare_parameters() override;

  controller_interface::CallbackReturn read_parameters() override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period,
    const UnitreeStates & states, UnitreeCommands & commands) override;
};

} // namespace unitree_controller

#endif // UNITREE_CONTROLLER__UNITREE_CONTROLLER_HPP_