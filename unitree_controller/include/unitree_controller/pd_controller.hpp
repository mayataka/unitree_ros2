#ifndef UNITREE_CONTROLLER__PD_CONTROLLER_HPP_
#define UNITREE_CONTROLLER__PD_CONTROLLER_HPP_

#include "unitree_controller/types.hpp"

namespace unitree_controller
{

class PDController {
public:
  ///
  /// @brief Construct a PD controller. 
  /// @param[in] qJ Joint position command. 
  /// @param[in] dqJ Joint velocity command. 
  /// @param[in] tauJ Torque command. 
  /// @param[in] Kp Position gain. Must be positive.
  /// @param[in] Kp Velocity gain. Must be positive.
  ///
  PDController(const Vector12d& qJ, const Vector12d& dqJ, const Vector12d& tauJ, 
               const Vector12d& Kp, const Vector12d& Kd);

  ///
  /// @brief Default constructor. 
  ///
  PDController() = default;

  ///
  /// @brief Default destructor. 
  ///
  ~PDController() = default;

  ///
  /// @brief Gets the read joint positions. 
  ///
  const Vector12d& qJ_cmd() const { return qJ_cmd_; }

  ///
  /// @brief Gets the read joint velocities. 
  ///
  const Vector12d& dqJ_cmd() const { return dqJ_cmd_; }

  ///
  /// @brief Gets the read joint torques. 
  ///
  const Vector12d& tauJ_cmd() const { return tauJ_cmd_; }

  ///
  /// @brief Gets the read joint positions. 
  ///
  const Vector12d& Kp_cmd() const { return Kp_cmd_; }

  ///
  /// @brief Gets the read joint velocities. 
  ///
  const Vector12d& Kd_cmd() const { return Kd_cmd_; }

  ///
  /// @brief Creates a zero-torque controller. 
  ///
  static PDController ZeroTorqueController();

  ///
  /// @brief Creates a standing-up controller. 
  ///
  static PDController StandingUpController();

  ///
  /// @brief Creates a sitting-down controller. 
  ///
  static PDController SittingDownController();

private:
  Vector12d qJ_cmd_, dqJ_cmd_, tauJ_cmd_, Kp_cmd_, Kd_cmd_;
};

} // namespace unitree_controller

#endif // UNITREE_CONTROLLER__UNITREE_CONTROLLER_HPP_