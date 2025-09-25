# Lateral control system data for system identification
# Used for collecting lateral dynamics data and MPC development

Header header

# --- 1. Control Inputs ---
float64 steering_angle      # [rad] Steering wheel angle command
float64 steering_angle_deg  # [deg] Same as above in degrees for convenience
float64 vehicle_speed       # [m/s] Current longitudinal velocity

# --- 2. Raw Sensor Data ---
sensor_msgs/Imu imu_data    # Full IMU data including angular rates
morai_msgs/GPSMessage gps_data  # GPS position
morai_msgs/EgoVehicleStatus ego_status  # MORAI vehicle status

# --- 3. Measured States ---
float64 yaw_rate           # [rad/s] Measured yaw rate from IMU
float64 yaw_rate_deg       # [deg/s] Same in degrees
float64 lateral_acceleration  # [m/s²] Lateral acceleration from IMU
float64 lateral_velocity   # [m/s] Estimated lateral velocity

# --- 4. Vehicle Pose ---
float64 x_position         # [m] X position in world frame
float64 y_position         # [m] Y position in world frame
float64 heading            # [rad] Vehicle heading angle
float64 heading_deg        # [deg] Vehicle heading in degrees

# --- 5. Velocity Estimates ---
float64 vx_body           # [m/s] Longitudinal velocity in body frame
float64 vy_body           # [m/s] Lateral velocity in body frame
float64 vx_world          # [m/s] X velocity in world frame
float64 vy_world          # [m/s] Y velocity in world frame

# --- 6. Path Tracking Errors (if path is available) ---
float64 lateral_error      # [m] Cross-track error from reference path
float64 heading_error      # [rad] Heading error from path tangent
float64 heading_error_deg  # [deg] Heading error in degrees
float64 path_curvature     # [1/m] Curvature of reference path at closest point

# --- 7. Derived Quantities ---
float64 sideslip_angle     # [rad] Vehicle sideslip angle (β)
float64 sideslip_angle_deg # [deg] Sideslip angle in degrees
float64 steering_gradient  # Understeer gradient (if calculable)
float64 turning_radius     # [m] Instantaneous turning radius

# --- 8. Scenario Information ---
string scenario_type       # Type of test scenario
int32 scenario_step        # Current step in scenario
float64 scenario_time      # [s] Time since scenario start
float64 scenario_progress  # [%] Scenario completion percentage

# --- 9. Frequency Response Data (for sine sweep) ---
float64 input_frequency    # [Hz] Current input frequency (for sine sweep)
float64 input_amplitude    # [rad] Input amplitude
float64 output_amplitude   # [rad/s] Output (yaw rate) amplitude
float64 phase_lag         # [rad] Phase lag between input and output

# --- 10. Data Quality ---
float64 dt                # [s] Time delta since last sample
bool is_valid             # Data validity flag
string validity_reason    # Reason if data is invalid
float64 gps_quality       # GPS signal quality (0-1)
float64 imu_quality       # IMU data quality (0-1)