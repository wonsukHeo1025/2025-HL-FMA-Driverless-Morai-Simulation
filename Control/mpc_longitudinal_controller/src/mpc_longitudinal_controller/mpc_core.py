#!/usr/bin/env python3

import cvxpy as cp
import numpy as np
from typing import Dict, Optional


class MpcCore:
    """
    Model Predictive Control (MPC) core implementation for longitudinal vehicle control.
    This class is ROS-independent and focuses purely on the MPC optimization problem.
    """
    
    def __init__(self, A: float, B: float, d: float, params: Dict):
        """
        Initialize MPC controller with system model and parameters.
        
        Args:
            A: System matrix parameter (discrete-time)
            B: Input matrix parameter (discrete-time)
            d: Disturbance/offset parameter
            params: Dictionary containing MPC parameters
                - Np: Prediction horizon
                - Nc: Control horizon
                - Q: State weight (tracking error penalty)
                - R: Input weight (control effort penalty)
                - R_delta: Input change weight (jerk penalty) [optional]
                - v_min: Minimum velocity constraint (m/s)
                - v_max: Maximum velocity constraint (m/s)
                - u_min: Minimum control input constraint
                - u_max: Maximum control input constraint
                - delta_u_max: Maximum input change constraint [optional]
        """
        self.A = A
        self.B = B
        self.d = d
        
        # Default parameters
        default_params = {
            'Np': 20,          # Prediction horizon
            'Nc': 5,           # Control horizon
            'Q': 1.0,          # State weight
            'R': 0.5,          # Input weight
            'R_delta': 0.1,    # Input change weight (jerk penalty)
            'v_min': -5.56,    # Minimum velocity (m/s) - -20 km/h (allows reverse)
            'v_max': 11.1,     # Maximum velocity (m/s) - 40 km/h
            'u_min': -1.0,     # Maximum brake
            'u_max': 1.0,      # Maximum throttle
            'delta_u_max': 0.2 # Maximum input change per step
        }
        
        # Merge with provided parameters
        self.params = {**default_params, **params}
        
        # CVXPY variables (will be set in _build_problem)
        self.problem = None
        self.u_var = None
        self.x_var = None
        self.delta_u_var = None
        self.x0_param = None
        self.x_ref_param = None
        self.u_prev_param = None
        self.R_delta_param = None  # (deprecated) runtime-adjustable jerk weight
        self.delta_u_max_param = None  # runtime-adjustable jerk limit
        
        # Last-solve diagnostics
        self.last_solve_time_ms = 0.0
        self.last_num_iters = 0
        self.last_status = "not_solved"
        
        # Build the optimization problem
        self._build_problem()
    
    def _build_problem(self):
        """
        Build the CVXPY optimization problem.
        This sets up the problem structure once, allowing for efficient repeated solving.
        """
        Np = self.params['Np']
        Nc = self.params['Nc']
        Q = self.params['Q']
        R = self.params['R']
        R_delta = self.params['R_delta']
        
        # Define CVXPY parameters (values updated at solve time)
        self.x0_param = cp.Parameter((1, 1), name="initial_state")
        self.x_ref_param = cp.Parameter((Np, 1), name="reference_trajectory")
        self.u_prev_param = cp.Parameter((1, 1), name="previous_input")
        
        # Define optimization variables
        self.u_var = cp.Variable((Nc, 1), name="control_input")
        self.x_var = cp.Variable((Np + 1, 1), name="state")
        self.delta_u_var = cp.Variable((Nc, 1), name="input_change")
        
        # Optional state constraint slack variables to preserve feasibility
        use_state_slack = self.params.get('use_state_slack', True)
        slack_weight = float(self.params.get('slack_weight', 1000.0))
        slack_max = float(self.params.get('slack_max', 0.5))  # [m/s]
        s_lo = cp.Variable((Np, 1), nonneg=True, name="slack_lower") if use_state_slack else None
        s_hi = cp.Variable((Np, 1), nonneg=True, name="slack_upper") if use_state_slack else None
        
        # Keep jerk weight constant in quadratic cost for solver stability
        self.R_delta_param = None
        # Runtime-adjustable jerk limit as parameter (linear constraints)
        self.delta_u_max_param = cp.Parameter(nonneg=True, name="delta_u_max")
        self.delta_u_max_param.value = float(self.params.get('delta_u_max', 0.2))
        
        # Initialize cost function
        cost = 0
        
        # Initialize constraints list
        constraints = []
        
        # Initial state constraint
        constraints.append(self.x_var[0] == self.x0_param)
        
        # Build cost function and dynamics constraints
        for k in range(Np):
            # State tracking cost
            cost += Q * cp.sum_squares(self.x_var[k+1] - self.x_ref_param[k])
            
            # Control input index (use last control after control horizon)
            u_idx = min(k, Nc - 1)
            
            # System dynamics constraint
            constraints.append(
                self.x_var[k+1] == self.A * self.x_var[k] + 
                self.B * self.u_var[u_idx] + self.d
            )
            
            # State constraints (with optional slack)
            if use_state_slack:
                constraints.append(self.x_var[k+1] + s_lo[k] >= self.params['v_min'])
                constraints.append(self.x_var[k+1] - s_hi[k] <= self.params['v_max'])
                # Slack caps to avoid masking modeling errors
                constraints.append(s_lo[k] <= slack_max)
                constraints.append(s_hi[k] <= slack_max)
                # Penalize slack usage heavily
                cost += slack_weight * (cp.sum_squares(s_lo[k]) + cp.sum_squares(s_hi[k]))
            else:
                constraints.append(self.x_var[k+1] >= self.params['v_min'])
                constraints.append(self.x_var[k+1] <= self.params['v_max'])
        
        # Control cost and constraints
        for k in range(Nc):
            # Control effort cost
            cost += R * cp.sum_squares(self.u_var[k])
            
            # Control input constraints
            constraints.append(self.u_var[k] >= self.params['u_min'])
            constraints.append(self.u_var[k] <= self.params['u_max'])
            
            # Input change (jerk) constraints and cost
            if k == 0:
                # First input change relative to previous input
                constraints.append(self.delta_u_var[k] == self.u_var[k] - self.u_prev_param)
            else:
                # Subsequent input changes
                constraints.append(self.delta_u_var[k] == self.u_var[k] - self.u_var[k-1])
            
            # Jerk penalty in cost function (constant weight)
            cost += R_delta * cp.sum_squares(self.delta_u_var[k])
            
            # Optional: Hard constraint on input change
            if self.delta_u_max_param is not None:
                # Linear bounds on input change using parameter
                constraints.append(self.delta_u_var[k] <= self.delta_u_max_param)
                constraints.append(self.delta_u_var[k] >= -self.delta_u_max_param)
        
        # Create the optimization problem
        self.problem = cp.Problem(cp.Minimize(cost), constraints)

    def set_delta_u_max(self, delta_u_max: float):
        """Update jerk limit constraint without rebuilding the problem."""
        try:
            if self.delta_u_max_param is not None:
                self.delta_u_max_param.value = float(max(0.0, delta_u_max))
        except Exception:
            pass
    
    def solve(self, current_velocity: float, target_velocity_trajectory: np.ndarray, 
              previous_input: float = 0.0) -> tuple:
        """
        Solve the MPC optimization problem.
        
        Args:
            current_velocity: Current vehicle velocity (m/s)
            target_velocity_trajectory: Target velocity for prediction horizon (m/s)
                                       Shape: (Np,) or (Np, 1)
            previous_input: Previous control input (for jerk calculation)
        
        Returns:
            tuple: (optimal_input, solve_status)
                - optimal_input: First element of optimal control sequence
                - solve_status: Optimization solver status string
        """
        # Update parameter values
        self.x0_param.value = np.array([[current_velocity]])
        
        # Ensure target trajectory has correct shape
        if target_velocity_trajectory.ndim == 1:
            target_velocity_trajectory = target_velocity_trajectory.reshape(-1, 1)
        
        # If trajectory is shorter than prediction horizon, pad with last value
        if len(target_velocity_trajectory) < self.params['Np']:
            last_value = target_velocity_trajectory[-1]
            padding_length = self.params['Np'] - len(target_velocity_trajectory)
            padding = np.full((padding_length, 1), last_value)
            target_velocity_trajectory = np.vstack([target_velocity_trajectory, padding])
        
        self.x_ref_param.value = target_velocity_trajectory[:self.params['Np']].reshape(-1, 1)
        self.u_prev_param.value = np.array([[previous_input]])
        
        # Solve the optimization problem
        try:
            # Optional OSQP time limit (seconds) if provided in params (>0)
            solver_kwargs = dict(
                solver=cp.OSQP,
                warm_start=True,
                verbose=False,
                max_iter=int(self.params.get('max_iter', 1200)),
                eps_abs=float(self.params.get('eps_abs', 1e-3)),
                eps_rel=float(self.params.get('eps_rel', 1e-3)),
            )
            tl = float(self.params.get('time_limit', -1.0))
            if tl and tl > 0.0:
                solver_kwargs['time_limit'] = tl
            import time as _time
            _t0 = _time.time()
            self.problem.solve(**solver_kwargs)
            self.last_solve_time_ms = (_time.time() - _t0) * 1000.0
            # Iterations (best-effort; may be missing for some versions)
            try:
                self.last_num_iters = int(getattr(self.problem, 'solver_stats', None).num_iters)
            except Exception:
                self.last_num_iters = 0
        except Exception as e:
            print(f"MPC solver error: {e}")
            # Fallback to SCS
            try:
                import time as _time
                _t0 = _time.time()
                self.problem.solve(solver=cp.SCS, warm_start=True, verbose=False, max_iters=5000)
                self.last_solve_time_ms = (_time.time() - _t0) * 1000.0
                try:
                    self.last_num_iters = int(getattr(self.problem, 'solver_stats', None).num_iters)
                except Exception:
                    self.last_num_iters = 0
            except Exception as e2:
                print(f"MPC fallback solver error (SCS): {e2}")
                return 0.0, "error"
        
        # Check solution status
        if self.problem.status not in ["optimal", "optimal_inaccurate"]:
            # Try SCS if OSQP reported infeasible or failed to converge
            try:
                import time as _time
                _t0 = _time.time()
                self.problem.solve(solver=cp.SCS, warm_start=True, verbose=False, max_iters=5000)
                self.last_solve_time_ms = (_time.time() - _t0) * 1000.0
                try:
                    self.last_num_iters = int(getattr(self.problem, 'solver_stats', None).num_iters)
                except Exception:
                    self.last_num_iters = 0
            except Exception:
                pass
            if self.problem.status not in ["optimal", "optimal_inaccurate"]:
                print(f"Warning: MPC problem status: {self.problem.status}")
                self.last_status = self.problem.status
                return 0.0, self.problem.status
        
        # Return first control input
        optimal_input = float(self.u_var.value[0, 0])
        self.last_status = self.problem.status
        return optimal_input, self.problem.status

    def get_last_solve_stats(self) -> Dict:
        """Return last solver diagnostics."""
        return {
            'solve_time_ms': float(self.last_solve_time_ms),
            'num_iters': int(self.last_num_iters),
            'status': str(self.last_status),
        }
    
    def get_predicted_trajectory(self) -> Optional[np.ndarray]:
        """
        Get the predicted state trajectory from the last solution.
        
        Returns:
            Predicted state trajectory or None if no solution available
        """
        if self.x_var.value is not None:
            return self.x_var.value.flatten()
        return None
    
    def get_control_sequence(self) -> Optional[np.ndarray]:
        """
        Get the full control sequence from the last solution.
        
        Returns:
            Control sequence or None if no solution available
        """
        if self.u_var.value is not None:
            return self.u_var.value.flatten()
        return None
    
    def update_model(self, A: float = None, B: float = None, d: float = None):
        """
        Update system model parameters and rebuild the problem.
        
        Args:
            A: New A parameter (if None, keep current)
            B: New B parameter (if None, keep current)
            d: New d parameter (if None, keep current)
        """
        if A is not None:
            self.A = A
        if B is not None:
            self.B = B
        if d is not None:
            self.d = d
        
        # Rebuild problem with new parameters
        self._build_problem()
    
    def update_params(self, params: Dict):
        """
        Update MPC parameters and rebuild the problem.
        
        Args:
            params: Dictionary with parameters to update
        """
        self.params.update(params)
        # If only jerk weight changed, adjust parameter without rebuild
        if list(params.keys()) == ['R_delta'] or (len(params) == 1 and 'R_delta' in params):
            self.set_jerk_weight(params['R_delta'])
            return
        self._build_problem()

