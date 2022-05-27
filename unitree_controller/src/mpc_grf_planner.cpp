#include "unitree_controller/mpc_grf_planner.hpp"


namespace unitree_controller
{

MpcGrfPlanner::MpcGrfPlanner(const std::string& urdf_file_name, const double dt, 
                             const double friction_coefficient, 
                             const double min_normal_force, 
                             const double max_normal_force)
{
  const Matrix6d Qqq = Vector6d::Constant(10.0).asDiagonal();
  const Matrix6d Qvv = Vector6d::Constant(0.1).asDiagonal();
  const Matrix3d Quu = Vector6d::Constant(0.00001).asDiagonal();
  auto state_equation = StateEquation(dt);
  auto cost_function = CostFunction(dt, Qqq, Qvv, Quu);
  auto friction_cone = FrictionCone(friction_coefficient, min_normal_force, max_normal_force);
  mpc_ = srbdmpc::MPC(state_equation, cost_function, friction_cone);
}


std::optional<std::string> MpcGrfPlanner::init(
    const double t, const Vector19d& q, const Vector18d& v, 
    const std::shared_ptr<FootStepPlanner>& foot_step_planner, 
    const bool verbose) {
  contact_schedule_ = srbdmpc::ContactSchedule();
  robot_state_ = srbdmpc::RobotState(urdf_file_name, {"FL_foot", "RL_foot", "FR_foot", "RR_foot"});
  gait_command_  = srbdmpc::GaitCommand();
  mpc_.init(contact_schedule_);
}


bool MpcGrfPlanner::solveMPC(const double t, const Vector19d& q, const Vector18d& v,
                             const Vector3d& vcom_cmd, const double yaw_rate_cmd,
                             const std::shared_ptr<FootStepPlanner>& foot_step_planner) {
  robot_state_.update(q, v);
  contact_schedule_ = 
  gait_command_.v = v_com_cmd;
  gait_command_.yaw_rate = yaw_rate_cmd;
  mpc_.solve(robot_state_, contact_schedule_, gait_command_);
}

}  // namespace unitree_controller
