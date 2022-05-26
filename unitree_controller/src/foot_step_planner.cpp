#include "srbd_mpc/gait_command.hpp"
quedrupedal gaits

namespace unitree_controller
{

class FootStepPlanner 
public:
  using Vector19d = Eigen::Matrix<double, 19, 1>;
  using Vector18d = Eigen::Matrix<double, 18, 1>;
  using Vector12d = Eigen::Matrix<double, 12, 1>;
  using Vector6d  = Eigen::Matrix<double, 6, 1>;
  using Vector4d  = Eigen::Matrix<double, 4, 1>;
  using Vector3d  = Eigen::Matrix<double, 3, 1>;
  using Quaterniond = Eigen::Quaterniond;

  FootStepPlanner();

  void setParameters(const double t_swing, const double t_stance, 
                     const double gain);

  void init(const Vector19d& q, const Vector18d& v=Vector18d::Zero(), 
                                  const bool verbose=false);

  bool solveMPC(const double t, const Vector19d& q, const Vector18d& v,
                const Vector3d& vcom_cmd, const double yaw_rate_cmd);

  const Vector12d& tauJCmd() const { return tau_; }
  const Vector12d& qJCmd() const { return qJ_cmd_; }
  const Vector12d& dqJCmd() const { return dqJ_cmd_; }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  tsid::robots::RobotWrapper robot_;
  pinocchio::Data robot_data_;
  tsid::InverseDynamicsFormulationAccForce id_formulaiton_;
  tsid::contacts::ContactPoint contact_LF_, contact_LH_, contact_RF_, contact_RH_;
  tsid::tasks::TaskActuationBounds task_actuation_bounds_;
  tsid::tasks::TaskJointPosVelAccBounds task_joint_bounds_;
  tsid::tasks::TaskJointPosture task_joint_posture_;
  tsid::tasks::TaskComEquality task_com_;
  tsid::tasks::TaskAMEquality task_am_;
  tsid::tasks::TaskSE3Equality task_LF_foot_, task_LH_foot_, 
                               task_RF_foot_, task_RH_foot_;
  tsid::trajectories::TrajectoryEuclidianConstant qJ_traj_, com_traj_, am_traj_;
  tsid::trajectories::TrajectorySE3Constant LF_foot_traj_, LH_foot_traj_,
                                            RF_foot_traj_, RH_foot_traj_;
  tsid::trajectories::TrajectorySample qJ_ref_, com_ref_, am_ref_,
                                       LF_foot_ref_, LH_foot_ref_,
                                       RF_foot_ref_, RH_foot_ref_;
  pinocchio::SE3 LF_foot_pos_ref_, LH_foot_pos_ref_,
                 RF_foot_pos_ref_, RH_foot_pos_ref_;
  Vector6d LF_foot_vel_ref_, LF_foot_acc_ref_,
           LH_foot_vel_ref_, LH_foot_acc_ref_,
           RF_foot_vel_ref_, RF_foot_acc_ref_,
           RH_foot_vel_ref_, RH_foot_acc_ref_;
  bool is_LF_foot_contact_active_, is_LF_foot_contact_active_prev_,
       is_LH_foot_contact_active_, is_LH_foot_contact_active_prev_,
       is_RF_foot_contact_active_, is_RF_foot_contact_active_prev_,
       is_RH_foot_contact_active_, is_RH_foot_contact_active_prev_;
  tsid::solvers::SolverHQuadProgFast qp_solver_;
  Vector12d tau_, qJ_cmd_, dqJ_cmd_;
  Vector18d a_;
  double dt_;
};

}  // namespace unitree_controller

#endif // UNITREE_CONTROLLER__FOOT_STEP_PLANNER_HPP_