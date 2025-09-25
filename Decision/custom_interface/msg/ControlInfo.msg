# Control information from Lateral MPC
# This message contains lateral control output and current path index

float64 steering           # Steering angle in radians
int32   current_idx        # Index of the current nearest point in the path

# Extended fields for lateral MPC dynamic horizon introspection
int32   horizon_steps      # Current MPC prediction horizon Np [steps]
float64 roi_curvature      # Curvature metric over ROI (per mode)
string  roi_curvature_mode # Aggregation mode: "max_abs" or "rms"
