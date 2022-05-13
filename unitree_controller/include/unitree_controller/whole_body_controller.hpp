#ifndef UNITREE_CONTROLLER__WHOLE_BODY_CONTROLLER_HPP_
#define UNITREE_CONTROLLER__WHOLE_BODY_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics-derivatives.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/frames-derivatives.hpp"
#include "tsid/robots/robot-wrapper.hpp"
#include "tsid/formulations/inverse-dynamics-formulation-acc-force.hpp"
#include "tsid/contacts/contact-point.hpp"
#include "tsid/trajectories/trajectory-euclidian.hpp"
#include "tsid/trajectories/trajectory-se3.hpp"
#include "tsid/tasks/task-actuation-bounds.hpp"
#include "tsid/tasks/task-joint-posVelAcc-bounds.hpp"
#include "tsid/tasks/task-joint-posture.hpp"
#include "tsid/tasks/task-com-equality.hpp"
#include "tsid/solvers/solver-HQP-eiquadprog-fast.hpp"


namespace unitree_controller
{

class WholeBodyController 
{
public:
  using Vector19d = Eigen::Matrix<double, 19, 1>;
  using Vector18d = Eigen::Matrix<double, 18, 1>;
  using Vector12d = Eigen::Matrix<double, 12, 1>;
  using Vector4d  = Eigen::Matrix<double, 4, 1>;
  using Vector3d  = Eigen::Matrix<double, 3, 1>;
  using Quaterniond = Eigen::Quaterniond;

  WholeBodyController(const std::string& urdf_file_name,
                      const std::string& pkg_dir, const double dt,
                      const double friction_coefficient=0.6,
                      const double min_normal_force=0.1,
                      const double max_normal_force=100.0);

  void setContactStatus(const std::vector<bool>& is_contact_active);

  void setJointPositionTask(const Vector12d& qJ, const double weight);

  void setCoMTask(const Vector3d& com_pos);

  void setupQP(const double t, const Vector19d& q, const Vector18d& v);

  void solveQP(const double t, const Vector19d& q, const Vector18d& v);

  const Vector12d& tauJCmd() const { return tau_; }
  const Vector12d& qJCmd() const { return qJ_cmd_; }
  const Vector12d& dqJCmd() const { return dqJ_cmd_; }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  pinocchio::Model pin_model_;
  pinocchio::Data pin_data_;
  tsid::robots::RobotWrapper robot_;
  tsid::InverseDynamicsFormulationAccForce id_formulaiton_;
  tsid::contacts::ContactPoint contact_LF_, contact_LH_, contact_RF_, contact_RH_;
  tsid::tasks::TaskActuationBounds task_actuation_bounds_;
  tsid::tasks::TaskJointPosVelAccBounds task_joint_bounds_;
  tsid::trajectories::TrajectoryEuclidianConstant qJ_traj_;
  tsid::tasks::TaskJointPosture task_joint_pos_;
  tsid::tasks::TaskComEquality task_com_pos_;
  tsid::solvers::SolverHQuadProgFast qp_solver_;
  Vector12d tau_, qJ_cmd_, dqJ_cmd_;
  Vector18d a_;
  double dt_;
};

}  // namespace unitree_controller

#endif // UNITREE_CONTROLLER__WHOLE_BODY_CONTROLLER_HPP_