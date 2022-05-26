#ifndef UNITREE_CONTROLLER__MPC_GRF_PLANNER_HPP_
#define UNITREE_CONTROLLER__MPC_GRF_PLANNER_HPP_

#include <memory>
#include <string>
#include <vector>
#include <limit>
#include <optional>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "srbd_mpc/mpc.hpp"
#include "srbd_mpc/state_equation.hpp"
#include "srbd_mpc/cost_function.hpp"
#include "srbd_mpc/friction_cone.hpp"
#include "srbd_mpc/robot_state.hpp"
#include "srbd_mpc/contact_schedule.hpp"
#include "srbd_mpc/gait_command.hpp"


namespace unitree_controller
{

class MpcGrfPlanner 
{
public:
  using Vector19d = Eigen::Matrix<double, 19, 1>;
  using Vector18d = Eigen::Matrix<double, 18, 1>;
  using Vector12d = Eigen::Matrix<double, 12, 1>;
  using Vector6d  = Eigen::Matrix<double, 6, 1>;
  using Vector4d  = Eigen::Matrix<double, 4, 1>;
  using Vector3d  = Eigen::Matrix<double, 3, 1>;
  using Matrix6d  = Eigen::Matrix<double, 6, 6>;
  using Matrix3d  = Eigen::Matrix<double, 3, 3>;
  using Quaterniond = Eigen::Quaterniond;

  MpcGrfPlanner(const std::string& urdf_file_name,
                const double dt, const double friction_coefficient=0.5, 
                const double min_normal_force=0.0, 
                const double max_normal_force=std::numeric_limits<double>::infinity());

  std::optional<std::string> init(const double t, const Vector19d& q, 
                                  const Vector18d& v=Vector18d::Zero(), 
                                  const std::shared_ptr<FootStepPlanner>& foot_step_planner,
                                  const bool verbose=false);

  bool solveMPC(const double t, const Vector19d& q, const Vector18d& v,
                const Vector3d& vcom_cmd, const double yaw_rate_cmd,
                const std::shared_ptr<FootStepPlanner>& foot_step_planner);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  srbd_mpc::MPC mpc_;
  srbd_mpc::RobotState robot_state_;
  srbd_mpc::ContactSchedule contact_schedule_;
  srbd_mpc::GaitCommand gait_command_;
};

}  // namespace unitree_controller

#endif // UNITREE_CONTROLLER__MPC_GRF_PLANNER_HPP_