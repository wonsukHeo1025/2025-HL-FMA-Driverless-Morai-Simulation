#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Kinematic Bicycle Model MPC Core Implementation

Simplified MPC controller using kinematic bicycle model for lateral control.
Much simpler than dynamic model - only requires wheelbase L.
"""

import numpy as np
import cvxpy as cp
from typing import Dict, Optional, Tuple, List
import time

# Import OSQP backend if available
try:
    from .osqp_backend_kinematic import KinematicOsqpSolver
    OSQP_AVAILABLE = True
except ImportError:
    OSQP_AVAILABLE = False
    print("Warning: OSQP backend not available, falling back to CVXPY")


class KinematicMpcCore:
    """Kinematic bicycle model MPC controller for lateral path tracking."""
    
    def __init__(self, model_params: Dict, control_params: Dict):
        """
        Initialize Kinematic MPC controller.
        
        Args:
            model_params: Model parameters (L=wheelbase)
            control_params: Control parameters (Np, Nc, Q, R, etc.)
        """
        self.model_params = model_params
        self.control_params = control_params

        # Cache base Q/P for use across backends (OSQP/CVXPY)
        try:
            q_list = self.control_params.get('Q_kinematic', [100, 50])
            self.Q_base_np = np.diag(q_list if isinstance(q_list, list) else list(q_list))
        except Exception:
            self.Q_base_np = np.diag([100.0, 50.0])
        try:
            p_list = self.control_params.get('P_kinematic', None)
            if p_list is None:
                self.P_base_np = 10.0 * self.Q_base_np
            else:
                self.P_base_np = np.diag(p_list if isinstance(p_list, list) else list(p_list))
        except Exception:
            self.P_base_np = 10.0 * self.Q_base_np
        
        # State dimension for kinematic model
        self.nx = 2  # [e_y, e_psi] - lateral error and heading error
        
        # Control dimension
        self.nu = 1  # steering angle
        
        # Choose solver backend
        self.solver_backend = control_params.get('solver_backend', 'cvxpy')
        
        if self.solver_backend == 'osqp' and OSQP_AVAILABLE:
            # Use direct OSQP backend
            self._setup_osqp_backend()
            print("Using OSQP backend for Kinematic MPC")
        else:
            # Use CVXPY (default)
            if self.solver_backend == 'osqp' and not OSQP_AVAILABLE:
                print("Warning: OSQP backend requested but not available, using CVXPY")
            self._setup_optimization_problem()
            self.solver_backend = 'cvxpy'  # Ensure we know which backend is active
        
        # Solver statistics
        self.last_solve_time = 0.0
        self.last_cost = 0.0
        self.last_status = "not_solved"
        self.last_u = 0.0
        
    def _setup_optimization_problem(self):
        """Setup CVXPY optimization problem (called once)."""
        Np = self.control_params['Np']  # Prediction horizon
        Nc = self.control_params['Nc']  # Control horizon
        
        # Decision variables
        self.x = cp.Variable((self.nx, Np + 1))  # States
        self.u = cp.Variable((self.nu, Nc))       # Controls
        
        # Parameters (updated each iteration)
        self.x0 = cp.Parameter(self.nx)           # Initial state
        self.x_ref = cp.Parameter((self.nx, Np + 1))  # Reference (zeros for error states)
        self.u_prev = cp.Parameter()              # Previous control
        
        # Time-varying A and B matrices
        self.A_params = [cp.Parameter((self.nx, self.nx)) for _ in range(Np)]
        self.B_params = [cp.Parameter((self.nx, self.nu)) for _ in range(Np)]
        
        # Curvature feedforward as additive disturbance
        self.curvature = cp.Parameter(Np)     # Path curvature sequence
        self.E_params = [cp.Parameter(self.nx) for _ in range(Np)]
        
        # Cost function (base weights)
        Q_base = np.diag(self.control_params['Q_kinematic'])  # [e_y, e_psi] weights
        R = self.control_params['R']
        R_delta = self.control_params['R_delta']
        P_base = self.control_params.get('P_kinematic', Q_base * 10)  # Terminal cost
        if not isinstance(P_base, np.ndarray):
            P_base = np.diag(P_base) if isinstance(P_base, list) else P_base * np.eye(self.nx)

        # Keep base weights for run-time scaling
        self.Q_base = Q_base
        self.P_base = P_base

        # Time-varying scaling parameters for Q and terminal P
        self.q_scales = cp.Parameter(Np, nonneg=True)   # stage scales >= 0
        self.p_scale_end = cp.Parameter(nonneg=True)    # terminal scale >= 0

        # Band soft-constraint parameters and slack variables (optional)
        self.band_enable = bool(self.control_params.get('band_enable', True))
        if self.band_enable:
            self.band_half_width_seq = cp.Parameter(Np + 1, nonneg=True)  # w_k >= 0 for k=0..Np
            self.band_lambda = cp.Parameter(nonneg=True)
            self.s_pos = cp.Variable(Np + 1, nonneg=True)
            self.s_neg = cp.Variable(Np + 1, nonneg=True)

        # IRLS robust weighting on lateral error (additional term)
        self.irls_weights = cp.Parameter(Np + 1, nonneg=True)
        
        cost = 0
        constraints = []
        
        # Initial condition
        constraints.append(self.x[:, 0] == self.x0)
        
        # Stage costs and dynamics
        for k in range(Np):
            # State cost
            cost += self.q_scales[k] * cp.quad_form(self.x[:, k] - self.x_ref[:, k], Q_base)
            
            # Control cost (only for first Nc steps)
            if k < Nc:
                cost += R * cp.square(self.u[0, k])
                
                # Control rate cost
                if k == 0:
                    cost += R_delta * cp.square(self.u[0, k] - self.u_prev)
                else:
                    cost += R_delta * cp.square(self.u[0, k] - self.u[0, k-1])
                    
                # Dynamics constraint with curvature disturbance
                constraints.append(
                    self.x[:, k+1] == self.A_params[k] @ self.x[:, k] +
                                      self.B_params[k] @ self.u[:, k] +
                                      self.E_params[k] * self.curvature[k]
                )
            else:
                # After control horizon, keep last control
                constraints.append(
                    self.x[:, k+1] == self.A_params[k] @ self.x[:, k] +
                                      self.B_params[k] @ self.u[:, Nc-1] +
                                      self.E_params[k] * self.curvature[k]
                )
        
        # Terminal cost (scaled)
        cost += self.p_scale_end * cp.quad_form(self.x[:, Np] - self.x_ref[:, Np], P_base)

        # Band soft-constraint penalties (quadratic on slack)
        if self.band_enable:
            cost += self.band_lambda * (cp.sum_squares(self.s_pos) + cp.sum_squares(self.s_neg))

        # IRLS additional cost on lateral error (k=0..Np)
        lat_err_seq = self.x[0, :] - self.x_ref[0, :]
        cost += cp.sum(cp.multiply(self.irls_weights, cp.square(lat_err_seq)))
        
        # Control constraints
        delta_limits = self.control_params['delta_limits']
        constraints.append(self.u >= delta_limits[0])
        constraints.append(self.u <= delta_limits[1])
        
        # Control rate constraints
        delta_rate_max = self.control_params['delta_rate_max']
        for k in range(Nc):
            if k == 0:
                constraints.append(cp.abs(self.u[0, k] - self.u_prev) <= delta_rate_max)
            else:
                constraints.append(cp.abs(self.u[0, k] - self.u[0, k-1]) <= delta_rate_max)

        # Band soft-constraint inequalities for k = 0..Np (optional)
        if self.band_enable:
            # e_y(k) within band: |e_y(k)| <= w_k  (soft via slacks)
            for k in range(Np + 1):
                e_lat_k = self.x[0, k] - self.x_ref[0, k]
                constraints.append(e_lat_k - self.band_half_width_seq[k] <= self.s_pos[k])
                constraints.append(-e_lat_k - self.band_half_width_seq[k] <= self.s_neg[k])
        
        # Create problem
        self.problem = cp.Problem(cp.Minimize(cost), constraints)
    
    def _setup_osqp_backend(self):
        """Setup direct OSQP backend solver."""
        Np = self.control_params['Np']
        Nc = self.control_params['Nc']
        Ts = self.control_params['Ts']
        
        # Extract weights
        Q = np.diag(self.control_params['Q_kinematic'])
        R = self.control_params['R']
        R_delta = self.control_params['R_delta']
        P_term = self.control_params.get('P_kinematic', Q * 10)
        if not isinstance(P_term, np.ndarray):
            P_term = np.diag(P_term) if isinstance(P_term, list) else P_term * np.eye(self.nx)
        
        # Extract constraints
        delta_limits = self.control_params['delta_limits']
        delta_rate_max = self.control_params['delta_rate_max']
        
        # Create OSQP solver with full configuration
        self.osqp_solver = KinematicOsqpSolver(
            nx=self.nx,
            nu=self.nu,
            Np=Np,
            Nc=Nc,
            Q=Q,
            R=R,
            R_delta=R_delta,
            P_term=P_term,
            delta_limits=delta_limits,
            delta_rate_max=delta_rate_max,
            Ts=Ts,
            config=self.control_params  # Pass full config for advanced features
        )
        
    def _get_discrete_matrices(self, v_x: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Get discrete-time model matrices for kinematic bicycle model.
        
        Continuous-time kinematic bicycle error dynamics:
        d/dt [e_y]   = [v_x * sin(e_psi)]
        d/dt [e_psi] = [v_x/L * tan(delta) - v_x * kappa]
        
        Linearized around e_psi=0, delta=0:
        d/dt [e_y]   = [0  v_x] [e_y]   + [0]      * delta + [0]    * kappa
        d/dt [e_psi] = [0   0 ] [e_psi]   [v_x/L]           [-v_x]
        
        Args:
            v_x: Current vehicle speed [m/s]
            
        Returns:
            Tuple of (A_d, B_d, E_d) discrete matrices.
        """
        Ts = self.control_params['Ts']
        L = self.model_params['L']  # Wheelbase
        
        # Avoid division by zero at low speeds
        if abs(v_x) < 0.1:
            v_x = 0.1 if v_x >= 0 else -0.1
        
        # Continuous-time linearized matrices
        A_c = np.array([
            [0, v_x],
            [0, 0]
        ])
        
        B_c = np.array([
            [0],
            [v_x/L]
        ])
        
        # Curvature disturbance
        E_c = np.array([0, -v_x])
        
        # Simple Euler discretization (good enough for kinematic model)
        A_d = np.eye(2) + A_c * Ts
        B_d = B_c * Ts
        E_d = E_c * Ts
        
        return A_d, B_d, E_d
    
    def solve(self, current_state: np.ndarray, current_speed: float,
              reference_path: Optional[Dict] = None) -> Tuple[float, Dict]:
        """
        Solve MPC optimization problem.
        
        Args:
            current_state: Current error state [e_y, e_psi]
            current_speed: Current vehicle speed [m/s]
            reference_path: Optional reference path information with curvatures
            
        Returns:
            Tuple of (optimal_steering, debug_info)
        """
        start_time = time.time()
        
        # Ensure state has correct dimension
        if len(current_state) != 2:
            print(f"Warning: Expected 2D state, got {len(current_state)}D. Using first 2 elements.")
            current_state = current_state[:2]
        
        # Use appropriate backend
        if self.solver_backend == 'osqp':
            # Provide time-varying weights schedule to OSQP backend (fast)
            Np = self.control_params['Np']
            if bool(self.control_params.get('time_varying_weights_enable', False)):
                q_front = float(self.control_params.get('q_scale_front', 0.7))
                q_back = float(self.control_params.get('q_scale_back', 1.3))
                q_scales = np.linspace(q_front, q_back, Np)
                p_scale_end = float(self.control_params.get('p_scale_end', 1.3))
                self.osqp_weight_schedule = (q_scales, p_scale_end)
            else:
                self.osqp_weight_schedule = (None, None)
            return self._solve_osqp(current_state, current_speed, reference_path, start_time)
        else:
            return self._solve_cvxpy(current_state, current_speed, reference_path, start_time)
    
    def _solve_cvxpy(self, current_state: np.ndarray, current_speed: float,
                     reference_path: Optional[Dict], start_time: float) -> Tuple[float, Dict]:
        """Solve using CVXPY backend."""
        
        # Update model for current speed
        Np = self.control_params['Np']
        
        # Get time-varying model matrices
        for k in range(Np):
            # Could predict speed changes, but using constant for now
            A_d, B_d, E_d = self._get_discrete_matrices(current_speed)
            self.A_params[k].value = A_d
            self.B_params[k].value = B_d
            self.E_params[k].value = E_d
        
        # Set parameters
        self.x0.value = current_state
        self.x_ref.value = np.zeros((self.nx, Np + 1))  # Zero error is goal
        self.u_prev.value = self.last_u
        
        # Curvature disturbance
        if reference_path:
            curvatures = reference_path.get('curvatures', np.zeros(Np))
            if isinstance(curvatures, list):
                curvatures = np.array(curvatures)
            if curvatures.shape[0] < Np:
                curvatures = np.pad(curvatures, (0, Np - curvatures.shape[0]), 'edge')
            self.curvature.value = curvatures[:Np]
            kappa_seq = self.curvature.value
        else:
            self.curvature.value = np.zeros(Np)
            kappa_seq = self.curvature.value

        # Configure time-varying Q/P scales
        if bool(self.control_params.get('time_varying_weights_enable', True)):
            q_front = float(self.control_params.get('q_scale_front', 0.7))
            q_back = float(self.control_params.get('q_scale_back', 1.3))
            # Linear ramp from front to back
            if Np > 1:
                q_scales = np.linspace(q_front, q_back, Np)
            else:
                q_scales = np.array([q_back])
            p_scale_end = float(self.control_params.get('p_scale_end', 1.3))
        else:
            q_scales = np.ones(Np)
            p_scale_end = 1.0
        self.q_scales.value = q_scales
        self.p_scale_end.value = p_scale_end

        # Configure band soft-constraint parameters (only if enabled)
        if getattr(self, 'band_enable', False):
            w0 = float(self.control_params.get('band_base_width', 0.2))
            use_sched = bool(self.control_params.get('band_use_speed_curvature', True))
            w_min = float(self.control_params.get('band_width_min', 0.1))
            w_max = float(self.control_params.get('band_width_max', 0.35))
            gain_v = float(self.control_params.get('band_width_speed_gain', 0.0))
            gain_k = float(self.control_params.get('band_width_curvature_gain', 0.0))

            if use_sched and kappa_seq is not None:
                # Build sequence for k = 0..Np (use last curvature for terminal)
                if isinstance(kappa_seq, np.ndarray):
                    kappa_full = np.concatenate([kappa_seq, [kappa_seq[-1] if kappa_seq.size > 0 else 0.0]])
                else:
                    kappa_full = np.zeros(Np + 1)
                w_seq = w0 + gain_v * float(current_speed) - gain_k * np.abs(kappa_full)
                w_seq = np.clip(w_seq, w_min, w_max)
            else:
                w_seq = np.clip(np.ones(Np + 1) * w0, w_min, w_max)

            self.band_half_width_seq.value = w_seq
            # If not provided, default lambda to 10x e_y weight
            q_lat = float(self.Q_base[0, 0]) if isinstance(self.Q_base, np.ndarray) else float(self.control_params['Q_kinematic'][0])
            self.band_lambda.value = float(self.control_params.get('band_lambda', 10.0 * q_lat))

        # Initialize IRLS weights (0 disables extra term on first solve)
        irls_enable = bool(self.control_params.get('irls_enable', False))
        self.irls_weights.value = np.zeros(Np + 1)
        
        # Solve optimization (optionally with IRLS refinement)
        try:
            # Fast OSQP settings via CVXPY; allow overrides via control_params
            solver_kwargs = {
                'solver': cp.OSQP,
                'warm_start': True,
                'verbose': False,
                # Safe defaults (not hard cuts)
                'max_iter': int(self.control_params.get('osqp_max_iter', 400)),
                'eps_abs': float(self.control_params.get('osqp_eps_abs', 1e-3)),
                'eps_rel': float(self.control_params.get('osqp_eps_rel', 1e-3)),
                'polish': bool(self.control_params.get('osqp_polish', False)),
            }
            # Only set a time limit if explicitly positive
            if float(self.control_params.get('osqp_time_limit', -1)) > 0.0:
                solver_kwargs['time_limit'] = float(self.control_params['osqp_time_limit'])

            # Initial solve
            self.problem.solve(**solver_kwargs)

            # Optional IRLS refinements
            if irls_enable and self.problem.status in ["optimal", "optimal_inaccurate"]:
                max_iters = int(self.control_params.get('irls_max_iters', 1))
                kappa_huber = float(self.control_params.get('irls_kappa', 0.2))
                eps = float(self.control_params.get('irls_epsilon', 1e-3))
                for _ in range(max(0, max_iters)):
                    # Predicted lateral error sequence (k=0..Np)
                    if self.x.value is None:
                        break
                    e_seq = np.array(self.x.value[0, :]).flatten()
                    abs_e = np.abs(e_seq)
                    w_vec = np.where(abs_e <= kappa_huber, 1.0, kappa_huber / (abs_e + eps))
                    self.irls_weights.value = w_vec
                    # Re-solve with updated weights
                    self.problem.solve(**solver_kwargs)

            if self.problem.status in ["optimal", "optimal_inaccurate"]:
                optimal_steering = float(self.u.value[0, 0])
                self.last_u = optimal_steering
                self.last_status = "optimal"
                self.last_cost = float(self.problem.value)
            else:
                # Fallback to previous control
                optimal_steering = self.last_u
                self.last_status = f"failed_{self.problem.status}"

        except Exception as e:
            print(f"MPC solver error: {e}")
            optimal_steering = self.last_u
            self.last_status = "exception"
        
        # Solver time
        self.last_solve_time = (time.time() - start_time) * 1000  # ms
        
        # Debug information
        debug_info = {
            'predicted_states': self.x.value if self.x.value is not None else None,
            'control_sequence': self.u.value if self.u.value is not None else None,
            'cost': self.last_cost,
            'solver_time': self.last_solve_time,
            'solver_status': self.last_status,
            'solver_iterations': getattr(self.problem, 'solver_stats', None).num_iters if getattr(self.problem, 'solver_stats', None) is not None else None,
            'current_speed': current_speed,
            'wheelbase': self.model_params['L']
        }
        
        return optimal_steering, debug_info
    
    def _solve_osqp(self, current_state: np.ndarray, current_speed: float,
                    reference_path: Optional[Dict], start_time: float) -> Tuple[float, Dict]:
        """Solve using direct OSQP backend."""
        
        # Get prediction horizon
        Np = self.control_params['Np']
        
        # Build time-varying model matrices
        A_seq = []
        B_seq = []
        E_seq = []
        
        for k in range(Np):
            # Could predict speed changes, but using constant for now
            A_d, B_d, E_d = self._get_discrete_matrices(current_speed)
            A_seq.append(A_d)
            B_seq.append(B_d)
            E_seq.append(E_d)
        
        # Extract curvatures
        if reference_path:
            curvatures = reference_path.get('curvatures', np.zeros(Np))
            if isinstance(curvatures, list):
                curvatures = np.array(curvatures)
            if curvatures.shape[0] < Np:
                curvatures = np.pad(curvatures, (0, Np - curvatures.shape[0]), 'edge')
            kappa_seq = curvatures[:Np]
        else:
            kappa_seq = np.zeros(Np)
        
        # Get solver settings from control_params
        max_iter = self.control_params.get('osqp_max_iter', 400)
        eps_abs = self.control_params.get('osqp_eps_abs', 1e-3)
        eps_rel = self.control_params.get('osqp_eps_rel', 1e-3)
        
        # Solve with OSQP backend
        try:
            # Band-aware Q-weighting (밴드 의존 Q 스케줄링)
            band_aware_q_weighting = bool(self.control_params.get('band_aware_q_weighting', False))
            active_Q = None  # Will be passed to update_and_solve if band-aware is enabled
            
            if band_aware_q_weighting:
                # Get lateral error from current state
                lateral_error = float(current_state[0])  # e_y
                band_w = float(self.control_params.get('band_base_width', 1.0))
                
                # Check if inside or outside band
                if abs(lateral_error) <= band_w:
                    # Inside band: lower Q_y, higher Q_psi to encourage heading alignment
                    q_inside = self.control_params.get('Q_kinematic_inside_band', [5.0, 2000.0])
                    active_Q = np.array(q_inside) if isinstance(q_inside, list) else q_inside
                else:
                    # Outside band: use original high weights for fast return
                    q_outside = self.control_params.get('Q_kinematic_outside_band', [200.0, 250.0])
                    active_Q = np.array(q_outside) if isinstance(q_outside, list) else q_outside
                
                # Ensure Q is a 2x2 diagonal matrix (consistent structure)
                if active_Q.ndim == 1:
                    active_Q = np.diag(active_Q)
                
                # Also update Q_base_np for band_lambda calculation
                self.Q_base_np = active_Q
                
                # Track band state changes for debugging
                current_band_state = abs(lateral_error) <= band_w
                if not hasattr(self, '_last_band_state'):
                    self._last_band_state = current_band_state
                elif current_band_state != self._last_band_state:
                    # State changed - you can log this if needed
                    self._last_band_state = current_band_state
            
            # Pass weight schedule if available
            q_scales = None
            p_scale_end = None
            if hasattr(self, 'osqp_weight_schedule'):
                q_scales, p_scale_end = self.osqp_weight_schedule
            # Band soft-constraint schedule and weights for OSQP backend
            band_enable = bool(self.control_params.get('band_enable', False))
            band_w_seq = None
            band_lambda = 0.0
            if band_enable:
                # Build w_seq length Np+1 using same scheduler
                w0 = float(self.control_params.get('band_base_width', 0.2))
                w_min = float(self.control_params.get('band_width_min', 0.1))
                w_max = float(self.control_params.get('band_width_max', 0.35))
                gain_v = float(self.control_params.get('band_width_speed_gain', 0.0))
                gain_k = float(self.control_params.get('band_width_curvature_gain', 0.0))
                if reference_path and 'curvatures' in reference_path and reference_path['curvatures'] is not None:
                    curv = reference_path['curvatures']
                    if isinstance(curv, list):
                        curv = np.array(curv)
                else:
                    curv = np.zeros(Np)
                if curv.shape[0] < Np:
                    curv = np.pad(curv, (0, Np - curv.shape[0]), 'edge')
                kappa_full = np.concatenate([curv, [curv[-1] if curv.size > 0 else 0.0]])
                w_seq = w0 + gain_v * float(current_speed) - gain_k * np.abs(kappa_full)
                band_w_seq = np.clip(w_seq, w_min, w_max)
                # Use cached numeric base Q to avoid attribute issues
                q_lat = float(self.Q_base_np[0, 0])
                band_lambda = float(self.control_params.get('band_lambda', 10.0 * q_lat))

            # IRLS settings
            irls_enable = bool(self.control_params.get('irls_enable', False))
            irls_max_iters = int(self.control_params.get('irls_max_iters', 0))
            irls_kappa = float(self.control_params.get('irls_kappa', 0.2))
            irls_epsilon = float(self.control_params.get('irls_epsilon', 1e-3))

            optimal_steering, debug_info = self.osqp_solver.update_and_solve(
                x0=current_state,
                A_seq=A_seq,
                B_seq=B_seq,
                E_seq=E_seq,
                kappa_seq=kappa_seq,
                u_prev=self.last_u,
                q_scales=q_scales,
                p_scale_end=p_scale_end,
                band_enable=band_enable,
                band_w_seq=band_w_seq,
                band_lambda=band_lambda,
                irls_enable=irls_enable,
                irls_max_iters=irls_max_iters,
                irls_kappa=irls_kappa,
                irls_epsilon=irls_epsilon,
                max_iter=max_iter,
                eps_abs=eps_abs,
                eps_rel=eps_rel,
                active_Q=active_Q,  # Pass band-aware Q if enabled
                v_x=current_speed,  # For feedforward calculation
                L=self.model_params['L']  # Wheelbase for feedforward
            )
            
            # Update statistics
            self.last_u = optimal_steering
            self.last_status = debug_info['solver_status']
            self.last_cost = debug_info['cost']
            
        except Exception as e:
            print(f"OSQP solver error: {e}")
            optimal_steering = self.last_u
            debug_info = {
                'predicted_states': None,
                'control_sequence': None,
                'cost': np.inf,
                'solver_time': (time.time() - start_time) * 1000,
                'solver_status': 'exception',
                'solver_iterations': None,
                'current_speed': current_speed,
                'wheelbase': self.model_params['L']
            }
            self.last_status = 'exception'
        
        # Add additional debug info
        debug_info['current_speed'] = current_speed
        debug_info['wheelbase'] = self.model_params['L']
        debug_info['backend'] = 'osqp'
        
        # Update solve time
        self.last_solve_time = debug_info['solver_time']
        
        return optimal_steering, debug_info
    
    def calculate_feedforward_steering(self, curvature: float, v_x: float) -> float:
        """
        Calculate feedforward steering based on path curvature.
        For kinematic model: delta = L * kappa (pure geometric relationship)
        
        Args:
            curvature: Path curvature [1/m]
            v_x: Vehicle speed [m/s] (not used in kinematic model)
            
        Returns:
            Feedforward steering angle [rad]
        """
        L = self.model_params['L']
        
        # Simple kinematic relationship
        delta_ff = L * curvature
        
        # Limit to reasonable values
        max_steer = self.control_params['delta_limits'][1]
        delta_ff = np.clip(delta_ff, -max_steer, max_steer)
        
        return delta_ff
    
    def reset(self):
        """Reset controller state."""
        self.last_u = 0.0
        self.last_solve_time = 0.0
        self.last_cost = 0.0
        self.last_status = "not_solved"