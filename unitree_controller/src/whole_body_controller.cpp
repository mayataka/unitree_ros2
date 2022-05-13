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
    qJ_traj_("qJ_traj"),
    task_joint_pos_("task_joint_pos", robot_),
    task_com_pos_("task_com_pos", robot_),
    qp_solver_("qp-solver"),
    dt_(dt)
{
  const double force_regularization_weight = 1.0e-05;
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
  id_formulaiton_.addRigidContact(contact_LF_, force_regularization_weight); 
  id_formulaiton_.addRigidContact(contact_LH_, force_regularization_weight); 
  id_formulaiton_.addRigidContact(contact_RF_, force_regularization_weight); 
  id_formulaiton_.addRigidContact(contact_RH_, force_regularization_weight); 
  task_actuation_bounds_.setBounds(-robot_.model().effortLimit.tail(12), 
                                   robot_.model().effortLimit.tail(12));
  task_joint_bounds_.setImposeBounds(true, true, false, false);
  task_joint_bounds_.setPositionBounds(robot_.model().lowerPositionLimit.tail(12),
                                       robot_.model().upperPositionLimit.tail(12));
  task_joint_bounds_.setVelocityBounds(robot_.model().velocityLimit.tail(12));
  id_formulaiton_.addActuationTask(task_actuation_bounds_, 1.0, 0);
  id_formulaiton_.addMotionTask(task_joint_bounds_, 1.0, 0);

  pinocchio::urdf::buildModel(urdf_file_name, 
                              pinocchio::JointModelFreeFlyer(), pin_model_);
  pin_data_ = pinocchio::Data(pin_model_);
}


void WholeBodyController::setJointPositionTask(const Vector12d& qJ, 
                                               const double weight)
{
  const double Kp_joint_position_task = 10.0;
  const double Kd_joint_position_task = 2.0 * std::sqrt(Kp_joint_position_task);
  qJ_traj_.setReference(qJ);
  task_joint_pos_.setReference(qJ_traj_(0.));
  task_joint_pos_.Kp(Vector12d::Constant(Kp_joint_position_task));
  task_joint_pos_.Kd(Vector12d::Constant(Kd_joint_position_task));
  id_formulaiton_.addMotionTask(task_joint_pos_, weight, 1);
}


void WholeBodyController::setCoMTask(const Vector3d& com) 
{
  // id_formulation_.
}


void WholeBodyController::setupQP(const double t, const Vector19d& q, const Vector18d& v) 
{
  id_formulaiton_.computeProblemData(t, q, v);
  qp_solver_.resize(id_formulaiton_.nVar(), id_formulaiton_.nEq(), id_formulaiton_.nIn());
}


void WholeBodyController::solveQP(const double t, const Vector19d& q, const Vector18d& v) 
{
  pinocchio::framesForwardKinematics(pin_model_, pin_data_, q);
  contact_LF_.setReference(robot_.framePosition(pin_data_, pin_model_.getFrameId("FL_foot")));
  contact_LH_.setReference(robot_.framePosition(pin_data_, pin_model_.getFrameId("RL_foot")));
  contact_RF_.setReference(robot_.framePosition(pin_data_, pin_model_.getFrameId("FR_foot")));
  contact_RH_.setReference(robot_.framePosition(pin_data_, pin_model_.getFrameId("RR_foot")));
  const auto& qp_data = id_formulaiton_.computeProblemData(t, q, v);
  const auto& qp_solution = qp_solver_.solve(qp_data);
  tau_ = id_formulaiton_.getActuatorForces(qp_solution);
  a_ = id_formulaiton_.getAccelerations(qp_solution);
  qJ_cmd_ = q.template tail<12>() + dt_ * v.template tail<12>() 
                                  + 0.5 * (dt_*dt_) * a_.template tail<12>();
  dqJ_cmd_ = v.template tail<12>() + dt_ * a_.template tail<12>();
}

}  // namespace unitree_controller