#ifndef UNITREE_CONTROLLER__WHOLE_BODY_CONTROLLER_HPP_
#define UNITREE_CONTROLLER__WHOLE_BODY_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>
#include <optional>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics-derivatives.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/frames-derivatives.hpp"
#include "pinocchio/algorithm/center-of-mass.hpp"
#include "tsid/robots/robot-wrapper.hpp"
#include "tsid/formulations/inverse-dynamics-formulation-acc-force.hpp"
#include "tsid/contacts/contact-point.hpp"
#include "tsid/tasks/task-actuation-bounds.hpp"
#include "tsid/tasks/task-joint-posVelAcc-bounds.hpp"
#include "tsid/tasks/task-joint-posture.hpp"
#include "tsid/tasks/task-se3-equality.hpp"
#include "tsid/tasks/task-com-equality.hpp"
#include "tsid/tasks/task-angular-momentum-equality.hpp"
#include "tsid/trajectories/trajectory-euclidian.hpp"
#include "tsid/trajectories/trajectory-se3.hpp"
#include "tsid/solvers/solver-HQP-eiquadprog-fast.hpp"
#include "tsid/solvers/utils.hpp"


namespace unitree_controller
{

class WholeBodyController 
{
public:
  using Vector19d = Eigen::Matrix<double, 19, 1>;
  using Vector18d = Eigen::Matrix<double, 18, 1>;
  using Vector12d = Eigen::Matrix<double, 12, 1>;
  using Vector6d  = Eigen::Matrix<double, 6, 1>;
  using Vector4d  = Eigen::Matrix<double, 4, 1>;
  using Vector3d  = Eigen::Matrix<double, 3, 1>;
  using Quaterniond = Eigen::Quaterniond;

  WholeBodyController(const std::string& urdf_file_name,
                      const std::string& pkg_dir, const double dt,
                      const double friction_coefficient=0.5,
                      const double min_normal_force=0.0,
                      const double max_normal_force=1.0e08);

  void setJointPostureRef(const Vector12d& qJ, 
                          const Vector12d& dqJ=Vector12d::Zero(),
                          const Vector12d& ddqJ=Vector12d::Zero());

  void setCoMRef(const Vector3d& com_pos_ref, 
                 const Vector3d& com_vel_ref=Vector3d::Zero(),
                 const Vector3d& com_acc_ref=Vector3d::Zero());

  void setAMRef(const Vector3d& am_ref);

  void setLFFootRef(const Vector3d& pos_ref,
                    const Vector3d& vel_ref=Vector3d::Zero(),
                    const Vector3d& acc_ref=Vector3d::Zero());

  void setLFFootRef(const pinocchio::SE3& pos_ref,
                    const Vector6d& vel_ref=Vector6d::Zero(),
                    const Vector6d& acc_ref=Vector6d::Zero());

  void setLHFootRef(const Vector3d& pos_ref,
                    const Vector3d& vel_ref=Vector3d::Zero(),
                    const Vector3d& acc_ref=Vector3d::Zero());

  void setLHFootRef(const pinocchio::SE3& pos_ref,
                    const Vector6d& vel_ref=Vector6d::Zero(),
                    const Vector6d& acc_ref=Vector6d::Zero());

  void setRFFootRef(const Vector3d& pos_ref,
                    const Vector3d& vel_ref=Vector3d::Zero(),
                    const Vector3d& acc_ref=Vector3d::Zero());

  void setRFFootRef(const pinocchio::SE3& pos_ref,
                    const Vector6d& vel_ref=Vector6d::Zero(),
                    const Vector6d& acc_ref=Vector6d::Zero());

  void setRHFootRef(const Vector3d& pos_ref,
                    const Vector3d& vel_ref=Vector3d::Zero(),
                    const Vector3d& acc_ref=Vector3d::Zero());

  void setRHFootRef(const pinocchio::SE3& pos_ref,
                    const Vector6d& vel_ref=Vector6d::Zero(),
                    const Vector6d& acc_ref=Vector6d::Zero());

  void enableLFFootContact() { is_LF_foot_contact_active_ = true; }
  void disableLFFootContact() { is_LF_foot_contact_active_ = false; }

  void enableLHFootContact() { is_LH_foot_contact_active_ = true; }
  void disableLHFootContact() { is_LH_foot_contact_active_ = false; }

  void enableRFFootContact() { is_RF_foot_contact_active_ = true; }
  void disableRFFootContact() { is_RF_foot_contact_active_ = false; }

  void enableRHFootContact() { is_RH_foot_contact_active_ = true; }
  void disableRHFootContact() { is_RH_foot_contact_active_ = false; }

  std::optional<std::string> init(const Vector19d& q, 
                                  const Vector18d& v=Vector18d::Zero(), 
                                  const bool verbose=false);

  bool solveQP(const double t, const Vector19d& q, const Vector18d& v);

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

#endif // UNITREE_CONTROLLER__WHOLE_BODY_CONTROLLER_HPP_