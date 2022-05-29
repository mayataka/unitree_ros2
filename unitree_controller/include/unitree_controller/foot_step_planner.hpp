#ifndef UNITREE_CONTROLLER__FOOT_STEP_PLANNER_HPP_
#define UNITREE_CONTROLLER__FOOT_STEP_PLANNER_HPP_

#include <memory>
#include <string>
#include <vector>
#include <limit>
#include <optional>

#include "Eigen/Core"
#include "Eigen/Geometry"

#include "srbd-mpc/mpc.hpp"
#include "srbd-mpc/state_equation.hpp"
#include "srbd-mpc/cost_function.hpp"
#include "srbd-mpc/friction_cone.hpp"
#include "srbd-mpc/robot_state.hpp"
#include "srbd-mpc/contact_schedule.hpp"
#include "srbd-mpc/gait_command.hpp"


namespace unitree_controller
{

class FootStepPlanner 
public:
  using Vector3d = Eigen::Matrix<double, 3, 1>;
  using Matrix3d = Eigen::Matrix<double, 3, 3>;
  using Quaterniond = Eigen::Quaterniond;

  FootStepPlanner() = default;

  ~FootStepPlanner() = default;

  void setParameters(const double t_swing, const double t_stance, 
                     const double t_swing_start, const double gain);

  void plan(const Vector3d& pos, const Matrix3d& R, const Vector3d& v, 
            const Vector3d& v_command, const double yaw_rate_command, 
            const bool verbose=false);

  const std::vector<bool>& getContactStatus(const double t) const;

  const std::vector<Eigen::Vector3d>& getContactPositions(const double t) const;

private:
  double t_swing_, t_stance_, t_swing_start_, gain_;
  std::vector<bool> contact_status_;
  std::vector<Eigen::Vector3d> contact_positions_;

};

}  // namespace unitree_controller

#endif // UNITREE_CONTROLLER__FOOT_STEP_PLANNER_HPP_