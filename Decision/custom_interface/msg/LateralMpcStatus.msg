# Status message for Lateral MPC controller
Header header

# Controller state
bool is_active              # Controller active flag
bool is_initialized         # Initialization complete
string control_mode         # Always "dynamic"

# Performance metrics
float64 lateral_rmse        # [m] Root mean square lateral error
float64 max_lateral_error   # [m] Maximum lateral error
float64 heading_rmse        # [rad] Root mean square heading error
float64 avg_solver_time     # [ms] Average solver time

# Path tracking status
bool path_available         # Valid path available
int32 closest_waypoint_idx  # Index of closest waypoint
float64 distance_to_goal    # [m] Distance to path goal
float64 path_completion     # [%] Path completion percentage

# System health
bool solver_healthy         # MPC solver working properly
bool state_estimation_ok    # State estimation valid
int32 consecutive_failures  # Number of consecutive solver failures
string error_message        # Current error/warning message