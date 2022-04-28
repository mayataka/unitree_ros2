#ifndef UNITREE_GAZEBO__UNITREE_GAZEBO_SYSTEM_HPP_
#define UNITREE_GAZEBO__UNITREE_GAZEBO_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>

#include "angles/angles.h"

#include "gazebo_ros2_control/gazebo_system_interface.hpp"

#include "std_msgs/msg/bool.hpp"


namespace unitree_gazebo
{
// Forward declaration
class UnitreeGazeboSystemPrivate;

// These class must inherit `gazebo_ros2_control::GazeboSystemInterface` which implements a
// simulated `ros2_control` `hardware_interface::SystemInterface`.

class UnitreeGazeboSystem : public gazebo_ros2_control::GazeboSystemInterface
{
public:
  // Documentation Inherited
  hardware_interface::return_type configure(const hardware_interface::HardwareInfo & system_info)
  override;

  // Documentation Inherited
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  // Documentation Inherited
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  // Documentation Inherited
  hardware_interface::return_type start() override;

  // Documentation Inherited
  hardware_interface::return_type stop() override;

  // Documentation Inherited
  hardware_interface::return_type read() override;

  // Documentation Inherited
  hardware_interface::return_type write() override;

  // Documentation Inherited
  bool initSim(
    rclcpp::Node::SharedPtr & model_nh,
    gazebo::physics::ModelPtr parent_model,
    const hardware_interface::HardwareInfo & hardware_info,
    sdf::ElementPtr sdf) override;

private:
  void registerJoints(
    const hardware_interface::HardwareInfo & hardware_info,
    gazebo::physics::ModelPtr parent_model);

  void registerSensors(
    const hardware_interface::HardwareInfo & hardware_info,
    gazebo::physics::ModelPtr parent_model);

  /// \brief Private data class
  std::unique_ptr<UnitreeGazeboSystemPrivate> dataPtr;
};

}  // namespace unitree_gazebo

#endif  // UNITREE_GAZEBO__UNITREE_GAZEBO_SYSTEM_HPP_