#include "unitree_controller/foot_step_planner.hpp"

namespace unitree_controller
{

void FootStepPlanner::setParameters(const double t_swing, const double t_stance, 
                                    const double t_swing_start, const double gain)
{
  t_swing_ = t_swing;
  t_stance_ = t_stance;
  t_swing_start_ = t_swing_start;
  gain_ = gain;
}


void FootStepPlanner::plan(const Vector3d& pos, const Matrix3d& R, const Vector3d& v, 
                           const Vector3d& v_command, const double yaw_rate_command, 
                           const bool verbose)
{

}


}  // namespace unitree_controller