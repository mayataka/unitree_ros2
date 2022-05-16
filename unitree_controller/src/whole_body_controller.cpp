#include "unitree_controller/whole_body_controller.hpp"

#include <cassert>

namespace unitree_controller
{

WholeBodyController::WholeBodyController(const std::string& urdf_file_name,
                                         const std::string& pkg_dir,
                                         const double dt,
                                         const double friction_coefficient,
                                         const double min_normal_force,
                                         const double max_normal_force)
  : robot_(tsid::robots::RobotWrapper(urdf_file_name, {pkg_dir}, 
                                      pinocchio::JointModelFreeFlyer())),
    robot_data_(pinocchio::Data(robot_.model())),
    id_formulaiton_("inverse-dynamics", robot_),
    contact_LF_("contact_LF", robot_, "FL_foot", (Vector3d() << 0, 0, 1).finished(), 
                friction_coefficient, min_normal_force, max_normal_force), 
    contact_LH_("contact_LH", robot_, "RL_foot", (Vector3d() << 0, 0, 1).finished(), 
                friction_coefficient, min_normal_force, max_normal_force), 
    contact_RF_("contact_RF", robot_, "FR_foot", (Vector3d() << 0, 0, 1).finished(), 
                friction_coefficient, min_normal_force, max_normal_force), 
    contact_RH_("contact_RH", robot_, "RR_foot", (Vector3d() << 0, 0, 1).finished(),
                friction_coefficient, min_normal_force, max_normal_force), 
    task_actuation_bounds_("task_actuation_bounds", robot_),
    task_joint_bounds_("task_joint_bounds", robot_, dt, false),
    task_joint_posture_("task_joint_posture", robot_),
    task_com_("task_com_", robot_),
    task_am_("task_am_", robot_),
    task_LF_foot_("task_LF_foot", robot_, "FL_foot"), 
    task_LH_foot_("task_LH_foot", robot_, "RL_foot"), 
    task_RF_foot_("task_RF_foot", robot_, "FR_foot"), 
    task_RH_foot_("task_RH_foot", robot_, "RR_foot"),
    qJ_traj_("qJ_traj"),
    com_traj_("com_traj"),
    am_traj_("am_traj"),
    LF_foot_traj_("LF_foot_traj"),
    LH_foot_traj_("LH_foot_traj"),
    RF_foot_traj_("RF_foot_traj"),
    RH_foot_traj_("RH_foot_traj"),
    qJ_ref_(12),
    com_ref_(3),
    am_ref_(3),
    LF_foot_ref_(LF_foot_traj_.computeNext()),
    LH_foot_ref_(LH_foot_traj_.computeNext()),
    RF_foot_ref_(RF_foot_traj_.computeNext()),
    RH_foot_ref_(RH_foot_traj_.computeNext()),
    LF_foot_pos_ref_(pinocchio::SE3::Identity()),
    LH_foot_pos_ref_(pinocchio::SE3::Identity()),
    RF_foot_pos_ref_(pinocchio::SE3::Identity()),
    RH_foot_pos_ref_(pinocchio::SE3::Identity()),
    LF_foot_vel_ref_(Vector6d::Zero()), 
    LF_foot_acc_ref_(Vector6d::Zero()),
    LH_foot_vel_ref_(Vector6d::Zero()), 
    LH_foot_acc_ref_(Vector6d::Zero()),
    RF_foot_vel_ref_(Vector6d::Zero()), 
    RF_foot_acc_ref_(Vector6d::Zero()),
    RH_foot_vel_ref_(Vector6d::Zero()), 
    RH_foot_acc_ref_(Vector6d::Zero()),
    qp_solver_("qp-solver"),
    tau_(Vector12d::Zero()), 
    qJ_cmd_(Vector12d::Zero()), 
    dqJ_cmd_(Vector12d::Zero()),
    a_(Vector18d::Zero()),
    dt_(dt)
{
  // contacts (hard or soft tasks)
  const double force_regularization_weight = 1.0e-05;
  const double contact_weight = 1.0e05;
  const double Kp_contact = 10.0;
  const double Kd_contact = 2.0 * std::sqrt(Kp_contact);
  contact_LF_.Kp(Vector3d::Constant(Kp_contact));
  contact_LF_.Kd(Vector3d::Constant(Kd_contact));
  contact_LH_.Kp(Vector3d::Constant(Kp_contact));
  contact_LH_.Kd(Vector3d::Constant(Kd_contact));
  contact_RF_.Kp(Vector3d::Constant(Kp_contact));
  contact_RF_.Kd(Vector3d::Constant(Kd_contact));
  contact_RH_.Kp(Vector3d::Constant(Kp_contact));
  contact_RH_.Kd(Vector3d::Constant(Kd_contact));
  contact_LF_.useLocalFrame(false);
  contact_LH_.useLocalFrame(false);
  contact_RF_.useLocalFrame(false);
  contact_RH_.useLocalFrame(false);
  id_formulaiton_.addRigidContact(contact_LF_, force_regularization_weight, contact_weight, 0); 
  id_formulaiton_.addRigidContact(contact_LH_, force_regularization_weight, contact_weight, 0); 
  id_formulaiton_.addRigidContact(contact_RF_, force_regularization_weight, contact_weight, 0); 
  id_formulaiton_.addRigidContact(contact_RH_, force_regularization_weight, contact_weight, 0); 
  // Foot tasks
  const double Kp_foot_task = 10.0;
  const double Kd_foot_task = 2.0 * std::sqrt(Kp_foot_task);
  const double foot_task_weight = 1.0;
  task_LF_foot_.Kp(Vector6d::Constant(Kp_foot_task));
  task_LF_foot_.Kd(Vector6d::Constant(Kd_foot_task));
  task_LH_foot_.Kp(Vector6d::Constant(Kp_foot_task));
  task_LH_foot_.Kd(Vector6d::Constant(Kd_foot_task));
  task_RF_foot_.Kp(Vector6d::Constant(Kp_foot_task));
  task_RF_foot_.Kd(Vector6d::Constant(Kd_foot_task));
  task_RH_foot_.Kp(Vector6d::Constant(Kp_foot_task));
  task_RH_foot_.Kd(Vector6d::Constant(Kd_foot_task));
  task_LF_foot_.setReference(LF_foot_ref_);
  task_LH_foot_.setReference(LH_foot_ref_);
  task_RF_foot_.setReference(RF_foot_ref_);
  task_RH_foot_.setReference(RH_foot_ref_);
  id_formulaiton_.addMotionTask(task_LF_foot_, foot_task_weight, 1);
  id_formulaiton_.addMotionTask(task_LH_foot_, foot_task_weight, 1);
  id_formulaiton_.addMotionTask(task_RF_foot_, foot_task_weight, 1);
  id_formulaiton_.addMotionTask(task_RH_foot_, foot_task_weight, 1);
  // com task
  const double Kp_com_task = 10.0;
  const double Kd_com_task = 2.0 * std::sqrt(Kp_com_task);
  const double com_task_weight = 1.0;
  task_com_.setReference(com_ref_);
  task_com_.Kp(Vector3d::Constant(Kp_com_task));
  task_com_.Kd(Vector3d::Constant(Kd_com_task));
  id_formulaiton_.addMotionTask(task_com_, com_task_weight, 1);
  // angular momentum
  const double Kp_am_task = 10.0;
  const double Kd_am_task = 2.0 * std::sqrt(Kp_am_task);
  const double am_task_weight = 1.0;
  task_am_.setReference(am_ref_);
  task_am_.Kp((Vector3d() << Kp_am_task, Kp_am_task, 0.).finished());
  task_am_.Kd((Vector3d() << Kd_am_task, Kd_am_task, 0.).finished());
  id_formulaiton_.addMotionTask(task_am_, am_task_weight, 1);
  // joint posture
  const double Kp_joint_posture_task = 1.0;
  const double Kd_joint_posture_task = 2.0 * std::sqrt(Kp_joint_posture_task);
  const double joint_posture_task_weight = 1.0e-03;
  task_joint_posture_.setReference(qJ_ref_);
  task_joint_posture_.Kp(Vector12d::Constant(Kp_joint_posture_task));
  task_joint_posture_.Kd(Vector12d::Constant(Kd_joint_posture_task));
  id_formulaiton_.addMotionTask(task_joint_posture_, joint_posture_task_weight, 1);
  // joint bounds (hard tasks)
  task_actuation_bounds_.setBounds(-robot_.model().effortLimit.tail(12), 
                                   robot_.model().effortLimit.tail(12));
  id_formulaiton_.addActuationTask(task_actuation_bounds_, 1.0, 0);
  // TODO: add joint position constraints the below does not work well
  // task_joint_bounds_.setImposeBounds(true, true, true, false);
  // task_joint_bounds_.setPositionBounds(robot_.model().lowerPositionLimit.tail(12),
  //                                      robot_.model().upperPositionLimit.tail(12));
  task_joint_bounds_.setImposeBounds(false, true, false, false);
  task_joint_bounds_.setVelocityBounds(robot_.model().velocityLimit.tail(12));
  id_formulaiton_.addMotionTask(task_joint_bounds_, 1.0, 0);
}


void WholeBodyController::setJointPostureRef(const Vector12d& qJ, 
                                             const Vector12d& dqJ,
                                             const Vector12d& ddqJ)
{
  qJ_ref_.setValue(qJ);
  qJ_ref_.setDerivative(dqJ);
  qJ_ref_.setSecondDerivative(ddqJ);
  task_joint_posture_.setReference(qJ_ref_);
}


void WholeBodyController::setCoMRef(const Vector3d& com_pos_ref,
                                    const Vector3d& com_vel_ref,
                                    const Vector3d& com_acc_ref)
{
  com_ref_.setValue(com_pos_ref);
  com_ref_.setDerivative(com_vel_ref);
  com_ref_.setSecondDerivative(com_acc_ref);
  task_com_.setReference(com_ref_);
}


void WholeBodyController::setAMRef(const Vector3d& am_ref)
{
  am_ref_.setValue(am_ref);
  task_am_.setReference(am_ref_);
}


void WholeBodyController::setLFFootRef(const Vector3d& pos_ref,
                                       const Vector3d& vel_ref,
                                       const Vector3d& acc_ref)
{
  LF_foot_pos_ref_.translation() = pos_ref;
  LF_foot_vel_ref_.template tail<3>() = vel_ref;
  LF_foot_acc_ref_.template tail<3>() = acc_ref;
  setLFFootRef(LF_foot_pos_ref_, LF_foot_vel_ref_, LF_foot_acc_ref_);
}


void WholeBodyController::setLFFootRef(const pinocchio::SE3& pos_ref,
                                       const Vector6d& vel_ref,
                                       const Vector6d& acc_ref)
{
  LF_foot_traj_.setReference(pos_ref);
  LF_foot_ref_.setValue(LF_foot_traj_.computeNext().getValue());
  LF_foot_ref_.setDerivative(acc_ref);
  task_LF_foot_.setReference(LF_foot_ref_);
}


void WholeBodyController::setLHFootRef(const Vector3d& pos_ref,
                                       const Vector3d& vel_ref,
                                       const Vector3d& acc_ref)
{
  LH_foot_pos_ref_.translation() = pos_ref;
  LH_foot_vel_ref_.template tail<3>() = vel_ref;
  LH_foot_acc_ref_.template tail<3>() = acc_ref;
  setLHFootRef(LH_foot_pos_ref_, LH_foot_vel_ref_, LH_foot_acc_ref_);
}


void WholeBodyController::setLHFootRef(const pinocchio::SE3& pos_ref,
                                       const Vector6d& vel_ref,
                                       const Vector6d& acc_ref)
{
  LH_foot_traj_.setReference(pos_ref);
  LH_foot_ref_.setValue(LH_foot_traj_.computeNext().getValue());
  LH_foot_ref_.setDerivative(vel_ref);
  LH_foot_ref_.setSecondDerivative(acc_ref);
  task_LH_foot_.setReference(LH_foot_ref_);
}


void WholeBodyController::setRFFootRef(const Vector3d& pos_ref,
                                       const Vector3d& vel_ref,
                                       const Vector3d& acc_ref)
{
  RF_foot_pos_ref_.translation() = pos_ref;
  RF_foot_vel_ref_.template tail<3>() = vel_ref;
  RF_foot_acc_ref_.template tail<3>() = acc_ref;
  setRFFootRef(RF_foot_pos_ref_, RF_foot_vel_ref_, RF_foot_acc_ref_);
}


void WholeBodyController::setRFFootRef(const pinocchio::SE3& pos_ref,
                                       const Vector6d& vel_ref,
                                       const Vector6d& acc_ref)
{
  RF_foot_traj_.setReference(pos_ref);
  RF_foot_ref_.setValue(RF_foot_traj_.computeNext().getValue());
  RF_foot_ref_.setDerivative(vel_ref);
  RF_foot_ref_.setSecondDerivative(acc_ref);
  task_RF_foot_.setReference(RF_foot_ref_);
}


void WholeBodyController::setRHFootRef(const Vector3d& pos_ref,
                                       const Vector3d& vel_ref,
                                       const Vector3d& acc_ref)
{
  RH_foot_pos_ref_.translation() = pos_ref;
  RH_foot_vel_ref_.template tail<3>() = vel_ref;
  RH_foot_acc_ref_.template tail<3>() = acc_ref;
  setRHFootRef(RH_foot_pos_ref_, RH_foot_vel_ref_, RH_foot_acc_ref_);
}


void WholeBodyController::setRHFootRef(const pinocchio::SE3& pos_ref,
                                       const Vector6d& vel_ref,
                                       const Vector6d& acc_ref)
{
  RH_foot_traj_.setReference(pos_ref);
  RH_foot_ref_.setValue(RH_foot_traj_.computeNext().getValue());
  RH_foot_ref_.setDerivative(vel_ref);
  RH_foot_ref_.setSecondDerivative(acc_ref);
  task_RH_foot_.setReference(RH_foot_ref_);
}


std::optional<std::string> WholeBodyController::init(const Vector19d& q, 
                                                     const Vector18d& v, 
                                                     const bool verbose) 
{
  setJointPostureRef(q.template tail<12>());
  robot_.computeAllTerms(robot_data_, q, v);
  setCoMRef(robot_data_.com[0]);
  setLFFootRef(robot_.framePosition(robot_data_, robot_.model().getFrameId("FL_foot")));
  setLHFootRef(robot_.framePosition(robot_data_, robot_.model().getFrameId("RL_foot")));
  setRFFootRef(robot_.framePosition(robot_data_, robot_.model().getFrameId("FR_foot")));
  setRHFootRef(robot_.framePosition(robot_data_, robot_.model().getFrameId("RR_foot")));
  contact_LF_.setReference(robot_.framePosition(robot_data_, robot_.model().getFrameId("FL_foot")));
  contact_LH_.setReference(robot_.framePosition(robot_data_, robot_.model().getFrameId("RL_foot")));
  contact_RF_.setReference(robot_.framePosition(robot_data_, robot_.model().getFrameId("FR_foot")));
  contact_RH_.setReference(robot_.framePosition(robot_data_, robot_.model().getFrameId("RR_foot")));
  const auto& qp_data = id_formulaiton_.computeProblemData(0.0, q, v);
  qp_solver_.resize(id_formulaiton_.nVar(), id_formulaiton_.nEq(), id_formulaiton_.nIn());
  if (verbose) {
    return tsid::solvers::HQPDataToString(qp_data);
  }
  else {
    return {};
  }
}


bool WholeBodyController::solveQP(const double t, const Vector19d& q, const Vector18d& v) 
{
  // robot_.computeAllterms(robot_data_, q, v);
  // contact_LF_.setReference(robot_.framePosition(robot_data_, robot_.model().getFrameId("FL_foot")));
  // contact_LH_.setReference(robot_.framePosition(robot_data_, robot_.model().getFrameId("RL_foot")));
  // contact_RF_.setReference(robot_.framePosition(robot_data_, robot_.model().getFrameId("FR_foot")));
  // contact_RH_.setReference(robot_.framePosition(robot_data_, robot_.model().getFrameId("RR_foot")));
  const auto& qp_data = id_formulaiton_.computeProblemData(t, q, v);
  const auto& qp_solution = qp_solver_.solve(qp_data);
  tau_ = id_formulaiton_.getActuatorForces(qp_solution);
  a_ = id_formulaiton_.getAccelerations(qp_solution);
  qJ_cmd_ = q.template tail<12>() + dt_ * v.template tail<12>() 
                                  + 0.5 * (dt_*dt_) * a_.template tail<12>();
  dqJ_cmd_ = v.template tail<12>() + dt_ * a_.template tail<12>();
  return (qp_solution.status == tsid::solvers::HQPStatus::HQP_STATUS_OPTIMAL);
}

}  // namespace unitree_controller