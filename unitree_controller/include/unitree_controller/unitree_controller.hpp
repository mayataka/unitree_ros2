#ifndef UNITREE_CONTROLLER__UNITREE_CONTROLLER_HPP_
#define UNITREE_CONTROLLER__UNITREE_CONTROLLER_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/node_options.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/service.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "realtime_tools/realtime_publisher.h"
#include "realtime_tools/realtime_buffer.h"
#include "geometry_msgs/msg/twist.hpp"

#include "unitree_msgs/msg/state.hpp"
#include "unitree_msgs/srv/reset_state_estimation.hpp"
#include "unitree_msgs/srv/set_control_mode.hpp"
#include "unitree_controller/visibility_control.h"

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "inekf/state_estimator.hpp"
#include "unitree_controller/whole_body_controller.hpp"


namespace unitree_controller
{

using namespace std::chrono_literals;  // NOLINT

enum class ControlMode {
ZeroTorque  = 0,
StandingUp  = 1,
Idling      = 2,
Control     = 3,
SittingDown = 4,
};

class UnitreeController : public controller_interface::ControllerInterface
{
public:
  using Vector19d = Eigen::Matrix<double, 19, 1>;
  using Vector18d = Eigen::Matrix<double, 18, 1>;
  using Vector12d = Eigen::Matrix<double, 12, 1>;
  using Vector4d  = Eigen::Matrix<double, 4, 1>;
  using Vector3d  = Eigen::Matrix<double, 3, 1>;
  using Quaterniond = Eigen::Quaterniond;

  UNITREE_CONTROLLER_PUBLIC 
  UnitreeController();

  UNITREE_CONTROLLER_PUBLIC
  controller_interface::return_type init(const std::string & controller_name) override;

  UNITREE_CONTROLLER_PUBLIC
  controller_interface::return_type init(const std::string & controller_name, 
                                         rclcpp::NodeOptions & node_options) override;

  UNITREE_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  UNITREE_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  UNITREE_CONTROLLER_PUBLIC
  controller_interface::return_type update() override;

  UNITREE_CONTROLLER_PUBLIC
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  UNITREE_CONTROLLER_PUBLIC
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  UNITREE_CONTROLLER_PUBLIC
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  UNITREE_CONTROLLER_PUBLIC
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  UNITREE_CONTROLLER_PUBLIC
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

  UNITREE_CONTROLLER_PUBLIC
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;

  UNITREE_CONTROLLER_PUBLIC
  bool reset();

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

protected:
  // hardware interfaces
  std::vector<std::string> joint_names_, sensor_names_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> 
      qJ_interface_, dqJ_interface_, tauJ_interface_, 
      imu_orientation_interface_, imu_angular_velocity_interface_, 
      imu_linear_acceleration_interface_, foot_force_sensor_interface_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> 
      qJ_cmd_interface_, dqJ_cmd_interface_, tauJ_cmd_interface_, 
      Kp_cmd_interface_, Kd_cmd_interface_;

  // state publisher
  using StatePublisher = rclcpp::Publisher<unitree_msgs::msg::State>;
  using RealtimeStatePublisher = realtime_tools::RealtimePublisher<unitree_msgs::msg::State>;
  std::shared_ptr<StatePublisher> state_publisher_;
  std::shared_ptr<RealtimeStatePublisher> realtime_state_publisher_;

  // command subscriber
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_subscription_;
  void velocityCommandSubscriptionCallback(
      const geometry_msgs::msg::Twist::SharedPtr msg);
  Vector3d linear_vel_cmd_, angular_vel_cmd_;
  realtime_tools::RealtimeBuffer<Vector3d> linear_vel_cmd_rt_, angular_vel_cmd_rt_;

  // reset state esitmation service
  rclcpp::Service<unitree_msgs::srv::ResetStateEstimation>::SharedPtr reset_state_estimation_server_;
  void resetStateEstimationCallback(
      const std::shared_ptr<unitree_msgs::srv::ResetStateEstimation::Request> request,
      std::shared_ptr<unitree_msgs::srv::ResetStateEstimation::Response> response);
  Vector3d base_pos_init_;
  realtime_tools::RealtimeBuffer<Vector3d> base_pos_init_rt_;
  Quaterniond base_quat_init_;
  realtime_tools::RealtimeBuffer<Quaterniond> base_quat_init_rt_;
  bool reset_state_estimation_;
  realtime_tools::RealtimeBuffer<bool> reset_state_estimation_rt_;

  // set control mode service
  ControlMode current_control_mode_, previous_control_mode_;
  rclcpp::Service<unitree_msgs::srv::SetControlMode>::SharedPtr set_control_mode_server_;
  void setControlModeCallback(
      const std::shared_ptr<unitree_msgs::srv::SetControlMode::Request> request,
      std::shared_ptr<unitree_msgs::srv::SetControlMode::Response> response);
  ControlMode request_control_mode_;
  realtime_tools::RealtimeBuffer<ControlMode> request_control_mode_rt_;
  bool set_control_mode_;
  realtime_tools::RealtimeBuffer<bool> set_control_mode_rt_;
  rclcpp::Duration min_zero_torque_duration_, min_standing_up_duration_,
                   min_idling_duration_, min_control_duration_, 
                   min_sitting_down_duration_;
  rclcpp::Time last_set_control_mode_time_;

  // gains
  double kp_standing_up_, kd_standing_up_;
  double kp_idling_, kd_idling_;
  double kp_control_, kd_control_;
  double kp_sitting_down_, kd_sitting_down_;

  double dt_;
  Vector19d q_est_;
  Vector18d v_est_;
  Vector12d qJ_, dqJ_, tauJ_, qJ_cmd_, dqJ_cmd_, tauJ_cmd_, Kp_cmd_, Kd_cmd_;
  Quaterniond quat_;
  Vector3d imu_ang_vel_, imu_lin_acc_;
  std::vector<double> foot_force_sensor_;

  Vector19d q_standing_ = (Vector19d() << 0, 0, 0.3181, 0, 0, 0, 1, 
                                          0.0,  0.67, -1.3, 
                                          0.0,  0.67, -1.3, 
                                          0.0,  0.67, -1.3, 
                                          0.0,  0.67, -1.3).finished();
  Vector19d q_sitting_down_ = (Vector19d() << 0, 0, 0.2, 0, 0, 0, 1, 
                                              0.0,  1.0, -2.5, 
                                              0.0,  1.0, -2.5, 
                                              0.0,  1.0, -2.5, 
                                              0.0,  1.0, -2.5).finished();

  std::shared_ptr<inekf::StateEstimator> state_estimator_;
  bool enable_state_estimation_;
  std::shared_ptr<WholeBodyController> whole_body_controller_;

  void auto_declare_params();

};
}  // namespace unitree_controller

#endif // UNITREE_CONTROLLER__UNITREE_CONTROLLER_HPP_