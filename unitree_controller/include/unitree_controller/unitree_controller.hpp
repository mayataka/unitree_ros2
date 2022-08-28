#ifndef UNITREE_CONTROLLER__UNITREE_CONTROLLER_HPP_
#define UNITREE_CONTROLLER__UNITREE_CONTROLLER_HPP_

#include "unitree_controller/unitree_controller_base.hpp"
#include "unitree_controller/types.hpp"
#include "unitree_controller/visibility_control.h"

#include "legged_state_estimator/legged_state_estimator.hpp"


namespace unitree_controller
{

using namespace std::chrono_literals;  // NOLINT

class UnitreeController : public UnitreeControllerBase
{
public:
  UNITREE_CONTROLLER_PUBLIC 
  UnitreeController();

private:
  void declare_parameters() override;

  controller_interface::CallbackReturn read_parameters() override;

  std::vector<std::string> get_joint_names() const override;

  std::vector<std::string> get_sensor_names() const override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period,
    const UnitreeStates & states, UnitreeCommands & commands) override;

  // interfaces
  std::vector<std::string> joint_names_, sensor_names_;

  // node parameters
  double control_rate_, control_period_;

  legged_state_estimator::LeggedStateEstimator state_estimator_;

};

} // namespace unitree_controller

#endif // UNITREE_CONTROLLER__UNITREE_CONTROLLER_HPP_