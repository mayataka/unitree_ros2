#include "unitree_controller/unitree_controller.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"

namespace unitree_controller
{

UnitreeController::UnitreeController()
: UnitreeControllerInterface(), 
  joint_names_({}), 
  sensor_names_({}),
  control_rate_(0), 
  control_period_(0),
  state_estimator_() {}

void UnitreeController::declare_parameters() 
{
  // interfaces
  auto_declare<std::vector<std::string>>("joints", joint_names_);
  auto_declare<std::vector<std::string>>("sensors", sensor_names_);
  // node parameters
  auto_declare<int>("control_rate", 400);
  // state estimator settings
  auto_declare<std::string>("state_estimator_settings.urdf_path", "");
  auto_declare<std::string>("state_estimator_settings.imu_frame", "imu_link");
  auto_declare<std::vector<std::string>>("state_estimator_settings.contact_frames", 
                                         {"FL_foot", "FR_foot", "RL_foot", "RR_foot"});
  auto_declare<std::vector<double>>("state_estimator_settings.contact_estimator_settings.beta0", 
                                    {-30.0, -30.0, -30.0, -30.0});
  auto_declare<std::vector<double>>("state_estimator_settings.contact_estimator_settings.beta1", 
                                    {2.0, 2.0, 2.0, 2.0});
  auto_declare<double>("state_estimator_settings.contact_estimator_settings.contact_force_covariance_alpha", 0.01);
  auto_declare<double>("state_estimator_settings.contact_estimator_settings.contact_probability_threshold", 0.5);
  auto_declare<double>("state_estimator_settings.noise_params.gyroscope_noise", 0.01);
  auto_declare<double>("state_estimator_settings.noise_params.accelerometer_noise", 0.1);
  auto_declare<double>("state_estimator_settings.noise_params.gyroscope_bias_noise", 0.00001);
  auto_declare<double>("state_estimator_settings.noise_params.accelerometer_bias_noise", 0.0001);
  auto_declare<double>("state_estimator_settings.noise_params.contact_noise", 0.1);
  auto_declare<bool>("state_estimator_settings.dynamic_contact_estimation", false);
  auto_declare<double>("state_estimator_settings.contact_position_noise", 0.01);
  auto_declare<double>("state_estimator_settings.contact_rotation_noise", 0.01);
  auto_declare<int>("state_estimator_settings.lpf_gyro_accel_cutoff_frequency", 250);
  auto_declare<int>("state_estimator_settings.lpf_lin_accel_cutoff_frequency", 250);
  auto_declare<int>("state_estimator_settings.lpf_dqJ_cutoff_frequency", 10);
  auto_declare<int>("state_estimator_settings.lpf_ddqJ_cutoff_frequency", 5);
  auto_declare<int>("state_estimator_settings.lpf_tauJ_cutoff_frequency", 10);
}

controller_interface::CallbackReturn UnitreeController::read_parameters() 
{
  // interfaces
  joint_names_ = get_node()->get_parameter("joints").as_string_array();
  sensor_names_ = get_node()->get_parameter("sensors").as_string_array();
  // node parameters
  control_rate_  = static_cast<double>(get_node()->get_parameter("control_rate").get_value<int>());

  if (joint_names_.size() != 12)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "'joints' parameter has wrong size");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (sensor_names_.size() != 5)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "'sensors' parameter has wrong size");
    return controller_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(get_node()->get_logger(), "Controller will be updated at %.2f Hz.", control_rate_);
  if (control_rate_ > 0.0)
  {
    control_period_ = 1.0 / control_rate_; // seconds
  }
  else
  {
    RCLCPP_ERROR(get_node()->get_logger(), "'control_rate_' must be positive, got %lf.", control_rate_);
    return controller_interface::CallbackReturn::ERROR;
  }

  legged_state_estimator::LeggedStateEstimatorSettings state_estimator_settings;
  state_estimator_settings.urdf_path = get_node()->get_parameter("state_estimator_settings.urdf_path").as_string();
  state_estimator_settings.imu_frame = get_node()->get_parameter("state_estimator_settings.imu_frame").as_string();
  state_estimator_settings.contact_frames = get_node()->get_parameter("state_estimator_settings.contact_frames").as_string_array();
  state_estimator_settings.contact_estimator_settings.beta0 
      = get_node()->get_parameter("state_estimator_settings.contact_estimator_settings.beta0").as_double_array();
  state_estimator_settings.contact_estimator_settings.beta1 
      = get_node()->get_parameter("state_estimator_settings.contact_estimator_settings.beta1").as_double_array();
  state_estimator_settings.contact_estimator_settings.contact_force_covariance_alpha 
      = get_node()->get_parameter("state_estimator_settings.contact_estimator_settings.contact_force_covariance_alpha").as_double();
  state_estimator_settings.contact_estimator_settings.contact_probability_threshold 
      = get_node()->get_parameter("state_estimator_settings.contact_estimator_settings.contact_probability_threshold").as_double();
  state_estimator_settings.noise_params.setGyroscopeNoise(
    get_node()->get_parameter("state_estimator_settings.noise_params.gyroscope_noise").as_double());
  state_estimator_settings.noise_params.setAccelerometerNoise(
    get_node()->get_parameter("state_estimator_settings.noise_params.accelerometer_noise").as_double());
  state_estimator_settings.noise_params.setGyroscopeBiasNoise(
    get_node()->get_parameter("state_estimator_settings.noise_params.gyroscope_bias_noise").as_double());
  state_estimator_settings.noise_params.setAccelerometerBiasNoise(
    get_node()->get_parameter("state_estimator_settings.noise_params.accelerometer_bias_noise").as_double());
  state_estimator_settings.noise_params.setContactNoise(
    get_node()->get_parameter("state_estimator_settings.noise_params.contact_noise").as_double());
  state_estimator_settings.dynamic_contact_estimation 
      = get_node()->get_parameter("state_estimator_settings.dynamic_contact_estimation").as_bool();
  state_estimator_settings.contact_position_noise
      = get_node()->get_parameter("state_estimator_settings.contact_position_noise").as_double();
  state_estimator_settings.contact_rotation_noise
      = get_node()->get_parameter("state_estimator_settings.contact_rotation_noise").as_double();
  state_estimator_settings.lpf_gyro_accel_cutoff_frequency
      = static_cast<double>(get_node()->get_parameter("state_estimator_settings.lpf_gyro_accel_cutoff_frequency").as_int());
  state_estimator_settings.lpf_lin_accel_cutoff_frequency
      = static_cast<double>(get_node()->get_parameter("state_estimator_settings.lpf_lin_accel_cutoff_frequency").as_int());
  state_estimator_settings.lpf_dqJ_cutoff_frequency
      = static_cast<double>(get_node()->get_parameter("state_estimator_settings.lpf_dqJ_cutoff_frequency").as_int());
  state_estimator_settings.lpf_ddqJ_cutoff_frequency
      = static_cast<double>(get_node()->get_parameter("state_estimator_settings.lpf_ddqJ_cutoff_frequency").as_int());
  state_estimator_settings.lpf_tauJ_cutoff_frequency
      = static_cast<double>(get_node()->get_parameter("state_estimator_settings.lpf_tauJ_cutoff_frequency").as_int());

  try {
    state_estimator_ = legged_state_estimator::LeggedStateEstimator(state_estimator_settings);
  }
  catch (const std::exception & e) {
    fprintf(stderr, "Exception thrown during constructing LeggedStateEstimator with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

std::vector<std::string> UnitreeController::get_joint_names() const {
  return joint_names_;  
}

std::vector<std::string> UnitreeController::get_sensor_names() const {
  return sensor_names_;  
}

controller_interface::return_type UnitreeController::update(
    const rclcpp::Time & time, const rclcpp::Duration & period,
    const UnitreeStates & states, UnitreeCommands & commands) 
{
  RCLCPP_INFO(get_node()->get_logger(), "derived time: %.2f [s]", time.seconds());
  return controller_interface::return_type::OK;
}

} // namespace unitree_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  unitree_controller::UnitreeController, 
  controller_interface::ControllerInterface)