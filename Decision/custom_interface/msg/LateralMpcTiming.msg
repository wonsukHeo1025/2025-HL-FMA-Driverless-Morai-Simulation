# Detailed timing breakdown for lateral MPC pipeline
Header header

# Overall
float64 loop_period_ms         # wall time per control loop (ms)
float64 loop_frequency_hz      # measured average frequency

# Stage timings (ms)
float64 t_state_estimation_ms
float64 t_error_computation_ms
float64 t_reference_build_ms   # finding closest idx, curvatures ahead
float64 t_mpc_setup_ms         # model matrices + param assignment
float64 t_mpc_solve_ms         # solver-only wall time
float64 t_publish_ms           # control/debug publish and any post-processing

# Solver stats
string  solver_status
float64 solver_cost
int32   solver_iterations

# Queue/latency hints (optional)
float64 last_control_info_pub_age_ms
float64 last_debug_pub_age_ms
