# Debug information for Lateral MPC controller
Header header

# Current errors
float64 lateral_error        # [m] Cross-track error
float64 lateral_error_rate   # [m/s] Rate of change of lateral error
float64 heading_error        # [rad] Heading error
float64 heading_error_rate   # [rad/s] Rate of change of heading error

# Control outputs
float64 steering_command     # [rad] Commanded steering angle
float64 steering_rate       # [rad/s] Steering angle rate
float64 feedforward_steering # [rad] Feedforward component
float64 feedback_steering    # [rad] Feedback component

# MPC solver information
float64 cost_value          # Current cost function value
float64 solver_time         # [ms] Time taken to solve
string solver_status        # Solver status (optimal, suboptimal, failed)
int32 solver_iterations     # Number of iterations

# Prediction horizon
float64[] predicted_lateral_errors  # Predicted errors over horizon
float64[] predicted_heading_errors
float64[] predicted_steering      # Predicted control sequence

# Path information
float64 path_curvature      # [1/m] Current path curvature
float64 lookahead_curvature # [1/m] Curvature at lookahead distance
float64 preview_distance    # [m] Current preview distance

# Model parameters (for adaptive MPC)
float64 current_speed       # [m/s] Current vehicle speed
string control_mode         # Always "dynamic"
float64 understeer_gradient # Current understeer gradient estimate