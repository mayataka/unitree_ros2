float64 t                          # Time 
float64[12] qJ                     # Joint position measurements
float64[12] vJ                     # Joint velocity measurements
float64[12] tauJ                   # Joint torque measurements
float64[4] imu_orientation         # Imu orientation estimation expressed as a quaternion (x, y, z, w)
float64[3] imu_angular_velocity    # Imu angular velocity
float64[3] imu_linear_acceleration # Imu linear acceleration 
float64[4] foot_force_sensor       # Force sensor measurements 