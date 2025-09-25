# Comprehensive data bus for control system debugging and monitoring
# Used for system identification and data collection

Header header

# --- 1. Control Commands (from Scenario Node) ---
float64 accel_cmd
float64 brake_cmd
float64 steer_cmd

# --- 2. Competition_topic Ground Truth Data ---
# True values from MORAI simulator
float64 true_velocity_x   # From Competition_topic velocity.x
float64 true_accel        # From Competition_topic accel (previous command)
float64 true_brake        # From Competition_topic brake (previous command)
float64 true_wheel_angle  # From Competition_topic wheel_angle

# --- 3. IMU Data ---
# Raw IMU data from /imu topic
sensor_msgs/Imu imu_data

# --- 4. GPS Data ---
# Raw GPS data from /gps topic
morai_msgs/GPSMessage gps_data

# --- 5. TF-based State Estimation ---
# Position from TF (map -> base_link)
float64 x_position
float64 y_position
float64 z_position

# Velocity estimation from TF
float64 estimated_velocity_x
float64 estimated_velocity_y
float64 estimated_velocity_z

# --- 5.1. IMU-based Velocity Estimation ---
# Velocity from IMU integration (body frame)
float64 imu_velocity_x
float64 imu_velocity_y
float64 imu_velocity_z

# --- 5.2. GPS-based Velocity Estimation ---
# Velocity from GPS differentiation (world frame)
float64 gps_velocity_x
float64 gps_velocity_y
float64 gps_velocity_z

# --- 6. Contextual Information ---
string scenario_type  # "step_accel", "step_brake", "prbs", "chirp"
int32 scenario_step   # Current step in the scenario
float64 scenario_time # Time since scenario started

# --- 7. Additional Debug Info ---
float64 dt            # Time delta for velocity estimation
bool is_valid         # Whether the data is valid (TF available, etc.)