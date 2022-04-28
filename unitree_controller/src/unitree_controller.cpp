#include "unitree_controller/unitree_controller.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"


namespace unitree_controller
{

UnitreeController::UnitreeController()
: controller_interface::ControllerInterface(), joint_names_({}), sensor_names_({})
{
  dt_ = 0;
  q_est_.setZero();
  v_est_.setZero();
  qJ_.setZero();
  dqJ_.setZero();
  ddqJ_.setZero();
  tauJ_.setZero();
  qJ_cmd_.setZero();
  dqJ_cmd_.setZero();
  tauJ_cmd_.setZero();
  Kp_cmd_.setZero();
  Kd_cmd_.setZero();
  quat_.setIdentity();
  imu_ang_vel_.setZero();
  imu_lin_acc_.setZero();
  foot_force_sensor_ = {0., 0., 0., 0.};
}

controller_interface::return_type UnitreeController::init(
  const std::string & controller_name)
{
  // initialize lifecycle node
  const auto ret = ControllerInterface::init(controller_name);
  if (ret != controller_interface::return_type::OK)
  {
    return ret;
  }
  auto_declare_params();

  return controller_interface::return_type::OK;
}

controller_interface::return_type UnitreeController::init(
  const std::string & controller_name, rclcpp::NodeOptions& node_options)
{
  // initialize lifecycle node
  const auto ret = ControllerInterface::init(controller_name, node_options);
  if (ret != controller_interface::return_type::OK)
  {
    return ret;
  }
  auto_declare_params();

  return controller_interface::return_type::OK;
}

void UnitreeController::auto_declare_params()
{
  // node parameters
  auto_declare<std::vector<std::string>>("joints", joint_names_);
  auto_declare<std::vector<std::string>>("sensors", sensor_names_);
  auto_declare<double>("sampling_rate", 400.0);
}

controller_interface::InterfaceConfiguration 
UnitreeController::command_interface_configuration() const
{
  const std::vector<std::string> joint_command_interface_types = { hardware_interface::HW_IF_POSITION,
                                                                   hardware_interface::HW_IF_VELOCITY,
                                                                   hardware_interface::HW_IF_EFFORT,
                                                                   "Kp", "Kd"};
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  conf.names.reserve(joint_names_.size() * joint_command_interface_types.size()); // Joint commands
  for (const auto & joint_name : joint_names_)
  {
    for (const auto & interface_type : joint_command_interface_types)
    {
      conf.names.push_back(joint_name + "/" + interface_type);
    }
  }
  return conf;
}

controller_interface::InterfaceConfiguration
UnitreeController::state_interface_configuration() const
{
  const std::vector<std::string> joint_state_interface_types = { hardware_interface::HW_IF_POSITION,
                                                                 hardware_interface::HW_IF_VELOCITY,
                                                                 hardware_interface::HW_IF_EFFORT};
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  conf.names.reserve(joint_names_.size() * joint_state_interface_types.size()  // Joint states
                      + 4 + 3 + 3); // Imu states
                      // + 4); // TODO: Foot force sensors
  // Joint state
  for (const auto & joint_name : joint_names_)
  {
    for (const auto & interface_type : joint_state_interface_types)
    {
      conf.names.push_back(joint_name + "/" + interface_type);
    }
  }
  // Imu state
  conf.names.push_back(sensor_names_[0] + "/" + "orientation.x");
  conf.names.push_back(sensor_names_[0] + "/" + "orientation.y");
  conf.names.push_back(sensor_names_[0] + "/" + "orientation.z");
  conf.names.push_back(sensor_names_[0] + "/" + "orientation.w");
  conf.names.push_back(sensor_names_[0] + "/" + "angular_velocity.x");
  conf.names.push_back(sensor_names_[0] + "/" + "angular_velocity.y");
  conf.names.push_back(sensor_names_[0] + "/" + "angular_velocity.z");
  conf.names.push_back(sensor_names_[0] + "/" + "linear_acceleration.x");
  conf.names.push_back(sensor_names_[0] + "/" + "linear_acceleration.y");
  conf.names.push_back(sensor_names_[0] + "/" + "linear_acceleration.z");
  // // TODO: Foot force sensors 
  // conf.names.push_back(sensor_names_[1] + "/" + "force.z");
  // conf.names.push_back(sensor_names_[2] + "/" + "force.z");
  // conf.names.push_back(sensor_names_[3] + "/" + "force.z");
  // conf.names.push_back(sensor_names_[4] + "/" + "force.z");
  return conf;
}

controller_interface::return_type UnitreeController::update()
{
  if (get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
  {
    return controller_interface::return_type::OK;
  }
  // get joint state
  for (std::size_t i = 0 ; i < 12; ++i)
  {
    qJ_.coeffRef(i)   = qJ_interface_[i].get().get_value();
    dqJ_.coeffRef(i)  = dqJ_interface_[i].get().get_value();
    tauJ_.coeffRef(i) = tauJ_interface_[i].get().get_value();
  }
  // get Imu state
  quat_.x() = imu_orientation_interface_[0].get().get_value();
  quat_.y() = imu_orientation_interface_[1].get().get_value();
  quat_.z() = imu_orientation_interface_[2].get().get_value();
  quat_.w() = imu_orientation_interface_[3].get().get_value();
  for (std::size_t i = 0 ; i < 3; ++i) 
  {
    imu_ang_vel_.coeffRef(i) = imu_angular_velocity_interface_[i].get().get_value();
  }
  for (std::size_t i = 0 ; i < 3; ++i) 
  {
    imu_lin_acc_.coeffRef(i) = imu_linear_acceleration_interface_[i].get().get_value();
  }
  // // TODO: get foot force sensor values
  // for (std::size_t i = 0 ; i < 4; ++i) 
  // {
  //   foot_force_sensor_[i] = foot_force_sensor_interface_[i].get().get_value();
  // }

  state_estimator_->update(imu_ang_vel_, imu_lin_acc_, qJ_, dqJ_, ddqJ_, tauJ_, foot_force_sensor_);
  q_est_.template head<3>()     = state_estimator_->getBasePositionEstimate();
  q_est_.template segment<4>(3) = state_estimator_->getBaseQuaternionEstimate();
  q_est_.template tail<12>()    = qJ_;
  v_est_.template head<3>()     = state_estimator_->getBaseLinearVelocityEstimateLocal();
  v_est_.template segment<3>(3) = imu_ang_vel_;
  v_est_.template tail<12>()    = state_estimator_->getJointVelocityEstimate();

  // 
  // control update here...
  //

  // set joint commands
  for (std::size_t i = 0 ; i < 12; ++i)
  {
    qJ_cmd_interface_[i].get().set_value(qJ_cmd_.coeff(i));
    dqJ_cmd_interface_[i].get().set_value(dqJ_cmd_.coeff(i));
    tauJ_cmd_interface_[i].get().set_value(tauJ_cmd_.coeff(i));
    Kp_cmd_interface_[i].get().set_value(Kp_cmd_.coeff(i));
    Kd_cmd_interface_[i].get().set_value(Kd_cmd_.coeff(i));
  }

  if (realtime_state_publisher_->trylock())
  {
    auto& state = realtime_state_publisher_->msg_;
    state.t = node_->now().seconds();
    Eigen::Map<Vector19d>(&(state.q[0])) = q_est_; 
    Eigen::Map<Vector18d>(&(state.v[0])) = v_est_; 
    Eigen::Map<Vector12d>(&(state.tau[0])) = tauJ_cmd_; 
    realtime_state_publisher_->unlockAndPublish();
  }

  return controller_interface::return_type::OK;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
UnitreeController::on_configure(const rclcpp_lifecycle::State &)
{
  auto logger = node_->get_logger();

  // update parameters
  joint_names_ = node_->get_parameter("joints").as_string_array();
  sensor_names_ = node_->get_parameter("sensors").as_string_array();

  if (!reset())
  {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
  }

  if (joint_names_.empty())
  {
    RCLCPP_WARN(logger, "'joints' parameter is empty.");
  }

  // Node parameter
  const double control_rate = node_->get_parameter("control_rate").get_value<double>();
  RCLCPP_INFO(logger, "Controller will be updated at %.2f Hz.", control_rate);
  if (control_rate > 0.0)
  {
    dt_ = 1.0 / control_rate; // seconds
  }
  else
  {
    RCLCPP_ERROR(logger, "control_rate must be positive, got %lf.", control_rate);
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }

  // init state publisher 
//   const double state_publish_rate = node_->get_parameter("state_publish_rate").get_value<double>();
  RCLCPP_INFO(logger, "Controller state will be published at %.2f Hz.", control_rate);
  state_publisher_ = node_->create_publisher<unitree_msgs::msg::State>("~/state", rclcpp::SystemDefaultsQoS());
  realtime_state_publisher_ = std::make_shared<RealtimeStatePublisher>(state_publisher_);

  // init InEKF
  const std::string urdf_pkg = ament_index_cpp::get_package_share_directory("a1_description");
  const std::string urdf = urdf_pkg + "/urdf/a1.urdf";

  auto state_estimator_settings = inekf::StateEstimatorSettings::UnitreeA1(urdf, dt_);
  state_estimator_ = std::make_shared<inekf::StateEstimator>(state_estimator_settings);
  state_estimator_->init({0, 0, 0.318}, {0, 0, 0, 1}, Eigen::Vector3d::Zero(), 
                         Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

// Fill ordered_interfaces with references to the matching interfaces
// in the same order as in joint_names
template <typename T>
bool get_ordered_interfaces(
  std::vector<T> & unordered_interfaces, const std::vector<std::string> & joint_names,
  const std::string & interface_type, std::vector<std::reference_wrapper<T>> & ordered_interfaces)
{
  for (const auto & joint_name : joint_names)
  {
    for (auto & interface : unordered_interfaces)
    {
      if (
        (interface.get_name() == joint_name) && (interface.get_interface_name() == interface_type))
      {
        ordered_interfaces.emplace_back(std::ref(interface));
      }
    }
  }

  return joint_names.size() == ordered_interfaces.size();
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
UnitreeController::on_activate(const rclcpp_lifecycle::State &)
{
  // Joint state interfaces
  qJ_interface_.clear();
  dqJ_interface_.clear();
  tauJ_interface_.clear();
  if (!get_ordered_interfaces(state_interfaces_, joint_names_, 
                              hardware_interface::HW_IF_POSITION, qJ_interface_)) 
  {
    RCLCPP_ERROR(
      node_->get_logger(), "Expected %zu qJ_interface, got %zu.", 
      12, qJ_interface_.size());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }
  if (!get_ordered_interfaces(state_interfaces_, joint_names_, 
                              hardware_interface::HW_IF_VELOCITY, dqJ_interface_)) 
  {
    RCLCPP_ERROR(
      node_->get_logger(), "Expected %zu dqJ_interface, got %zu.", 
      12, dqJ_interface_.size());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }
  if (!get_ordered_interfaces(state_interfaces_, joint_names_, 
                              hardware_interface::HW_IF_EFFORT, tauJ_interface_))
  {
    RCLCPP_ERROR(
      node_->get_logger(), "Expected %zu tauJ_interface, got %zu.", 
      12, tauJ_interface_.size());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }

  // Imu state interfaces
  imu_orientation_interface_.clear();
  imu_angular_velocity_interface_.clear();
  imu_linear_acceleration_interface_.clear();
  get_ordered_interfaces(state_interfaces_, {sensor_names_[0]}, 
                         "orientation.x", imu_orientation_interface_);
  get_ordered_interfaces(state_interfaces_, {sensor_names_[0]}, 
                         "orientation.y", imu_orientation_interface_);
  get_ordered_interfaces(state_interfaces_, {sensor_names_[0]}, 
                         "orientation.z", imu_orientation_interface_);
  get_ordered_interfaces(state_interfaces_, {sensor_names_[0]}, 
                         "orientation.w", imu_orientation_interface_);
  if (imu_orientation_interface_.size() != 4) 
  {
    RCLCPP_ERROR(
      node_->get_logger(), "Expected %zu imu_orientation_interface, got %zu.", 
      4, imu_orientation_interface_.size());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }
  get_ordered_interfaces(state_interfaces_, {sensor_names_[0]}, 
                         "angular_velocity.x", imu_angular_velocity_interface_);
  get_ordered_interfaces(state_interfaces_, {sensor_names_[0]}, 
                         "angular_velocity.y", imu_angular_velocity_interface_);
  get_ordered_interfaces(state_interfaces_, {sensor_names_[0]}, 
                         "angular_velocity.z", imu_angular_velocity_interface_);
  if (imu_angular_velocity_interface_.size() != 3) 
  {
    RCLCPP_ERROR(
      node_->get_logger(), "Expected %zu imu_angular_velocity_interface, got %zu.", 
      3, imu_orientation_interface_.size());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }
  get_ordered_interfaces(state_interfaces_, {sensor_names_[0]}, 
                         "linear_acceleration.x", imu_linear_acceleration_interface_);
  get_ordered_interfaces(state_interfaces_, {sensor_names_[0]}, 
                         "linear_acceleration.y", imu_linear_acceleration_interface_);
  get_ordered_interfaces(state_interfaces_, {sensor_names_[0]}, 
                         "linear_acceleration.z", imu_linear_acceleration_interface_);
  if (imu_linear_acceleration_interface_.size() != 3) 
  {
    RCLCPP_ERROR(
      node_->get_logger(), "Expected %zu imu_linear_acceleration_interface, got %zu.", 
      3, imu_linear_acceleration_interface_.size());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }

  // // TODO: Foot force sensor interfaces
  // foot_force_sensor_interface_.clear();
  // for (std::size_t i = 0 ; i < 4; ++i) 
  // {
  //   get_ordered_interfaces(state_interfaces_, {sensor_names_[i+1]}, 
  //                          "force.z", foot_force_sensor_interface_);
  // }
  // if (foot_force_sensor_interface_.size() != 4) 
  // {
  //   RCLCPP_ERROR(
  //     node_->get_logger(), "Expected %zu foot_force_interface, got %zu.", 
  //     4, foot_force_sensor_interface_.size());
  //   return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  // }

  // Joint command interfaces
  qJ_cmd_interface_.clear();
  dqJ_cmd_interface_.clear();
  tauJ_cmd_interface_.clear();
  Kp_cmd_interface_.clear();
  Kd_cmd_interface_.clear();
  if (!get_ordered_interfaces(command_interfaces_, joint_names_, 
                              hardware_interface::HW_IF_POSITION, qJ_cmd_interface_))
  {
    RCLCPP_ERROR(
      node_->get_logger(), "Expected %zu qJ_cmd_interface, got %zu.", 
      12, qJ_cmd_interface_.size());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }
  if (!get_ordered_interfaces(command_interfaces_, joint_names_, 
                              hardware_interface::HW_IF_VELOCITY, dqJ_cmd_interface_))
  {
    RCLCPP_ERROR(
      node_->get_logger(), "Expected %zu dqJ_cmd_interface, got %zu.", 
      12, dqJ_cmd_interface_.size());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }
  if (!get_ordered_interfaces(command_interfaces_, joint_names_, 
                              hardware_interface::HW_IF_EFFORT, tauJ_cmd_interface_)) 
  {
    RCLCPP_ERROR(
      node_->get_logger(), "Expected %zu tauJ_cmd_interface, got %zu.", 
      12, tauJ_cmd_interface_.size());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }
  if (!get_ordered_interfaces(command_interfaces_, joint_names_, 
                              "Kp", Kp_cmd_interface_)) 
  {
    RCLCPP_ERROR(
      node_->get_logger(), "Expected %zu Kp_cmd_interface, got %zu.", 
      12, Kp_cmd_interface_.size());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }
  if (!get_ordered_interfaces(command_interfaces_, joint_names_, 
                              "Kd", Kd_cmd_interface_)) 
  {
    RCLCPP_ERROR(
      node_->get_logger(), "Expected %zu Kp_cmd_interface, got %zu.", 
      12, Kd_cmd_interface_.size());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }

  // get initial joint state
  for (std::size_t i = 0 ; i < 12; ++i)
  {
    qJ_.coeffRef(i)   = qJ_interface_[i].get().get_value();
    dqJ_.coeffRef(i)  = dqJ_interface_[i].get().get_value();
    tauJ_.coeffRef(i) = tauJ_interface_[i].get().get_value();
  }
  // get Imu state
  quat_.x() = imu_orientation_interface_[0].get().get_value();
  quat_.y() = imu_orientation_interface_[1].get().get_value();
  quat_.z() = imu_orientation_interface_[2].get().get_value();
  quat_.w() = imu_orientation_interface_[3].get().get_value();
  for (std::size_t i = 0 ; i < 3; ++i) 
  {
    imu_ang_vel_.coeffRef(i) = imu_angular_velocity_interface_[i].get().get_value();
  }
  for (std::size_t i = 0 ; i < 3; ++i) 
  {
    imu_lin_acc_.coeffRef(i) = imu_linear_acceleration_interface_[i].get().get_value();
  }
  // // TODO: get foot force sensor values
  // for (std::size_t i = 0 ; i < 4; ++i) 
  // {
  //   foot_force_sensor_[i] = foot_force_sensor_interface_[i].get().get_value();
  // }

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
UnitreeController::on_deactivate(const rclcpp_lifecycle::State &)
{
  constexpr double kPosStopF = (2.146E+9f);
  constexpr double kVelStopF = (16000.0f);
  for (std::size_t i = 0; i < 12; ++i) 
  {
    qJ_cmd_interface_[i].get().set_value(kPosStopF);
    dqJ_cmd_interface_[i].get().set_value(kVelStopF);
    tauJ_cmd_interface_[i].get().set_value(0.0);
    Kp_cmd_interface_[i].get().set_value(0.0);
    Kd_cmd_interface_[i].get().set_value(0.0);
  }

  qJ_interface_.clear();
  dqJ_interface_.clear();
  tauJ_interface_.clear();
  imu_orientation_interface_.clear();
  imu_angular_velocity_interface_.clear();
  imu_linear_acceleration_interface_.clear();
  foot_force_sensor_interface_.clear();
  qJ_cmd_interface_.clear();
  dqJ_cmd_interface_.clear();
  tauJ_cmd_interface_.clear();
  Kp_cmd_interface_.clear();
  Kd_cmd_interface_.clear();

  release_interfaces();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
UnitreeController::on_cleanup(const rclcpp_lifecycle::State &)
{
  // TODO 
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
UnitreeController::on_error(const rclcpp_lifecycle::State &)
{
  if (!reset())
  {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

bool UnitreeController::reset()
{
  return true;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
UnitreeController::on_shutdown(const rclcpp_lifecycle::State &)
{
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

}  // namespace unitree_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  unitree_controller::UnitreeController, 
  controller_interface::ControllerInterface)