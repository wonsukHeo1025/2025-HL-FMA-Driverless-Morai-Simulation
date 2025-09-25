#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Lateral MPC Core Implementation

This module implements the core MPC optimization for lateral control
using dynamic bicycle model with error dynamics formulation.
"""

import numpy as np
import cvxpy as cp
from typing import Dict, Optional, Tuple, List
import time

# Import OSQP backend if available
try:
    from .osqp_backend_dynamic import LateralOsqpSolver
    OSQP_AVAILABLE = True
except ImportError:
    OSQP_AVAILABLE = False
    print("Warning: OSQP backend not available for dynamic MPC, falling back to CVXPY")


# Dynamic bicycle model only - no kinematic mode


class LateralMpcCore:
    """Core MPC controller for lateral path tracking."""
    
    def __init__(self, model_params: Dict, control_params: Dict):
        """
        Initialize Lateral MPC controller.
        
        Args:
            model_params: Model parameters (L, m, Iz, Caf, Car, etc.)
            control_params: Control parameters (Np, Nc, Q, R, etc.)
        """
        self.model_params = model_params
        self.control_params = control_params
        
        # Dynamic bicycle model only
        self.nx = 4  # [e_y, e_y_dot, e_psi, e_psi_dot]
            
        # Control dimension
        self.nu = 1  # steering angle
        
        # Choose solver backend
        self.solver_backend = control_params.get('solver_backend', 'cvxpy')
        
        if self.solver_backend == 'osqp' and OSQP_AVAILABLE:
            # Use direct OSQP backend
            self._setup_osqp_backend()
            print("Using OSQP backend for Dynamic MPC")
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
        
        # Time-varying A and B matrices (list of 2D parameters)
        self.A_params = [cp.Parameter((self.nx, self.nx)) for _ in range(Np)]
        self.B_params = [cp.Parameter((self.nx, self.nu)) for _ in range(Np)]
        
        # Curvature feedforward as additive disturbance
        self.curvature = cp.Parameter(Np)     # Path curvature sequence over horizon
        # Time-varying disturbance gain matrices (list of vectors)
        self.E_params = [cp.Parameter(self.nx) for _ in range(Np)]
        
        # Cost function (base weights)
        Q_base = self.control_params['Q']
        R = self.control_params['R']
        R_delta = self.control_params['R_delta']
        P_base = self.control_params.get('P', Q_base * 10)  # Terminal cost
        
        # Keep base weights and define scaling/robust parameters
        self.Q_base = Q_base
        self.P_base = P_base
        self.q_scales = [cp.Parameter(nonneg=True) for _ in range(Np)]
        self.p_scale_end = cp.Parameter(nonneg=True)

        # Band soft-constraint: parameters and slacks (optional)
        self.band_enable = bool(self.control_params.get('band_enable', True))
        if self.band_enable:
            self.band_half_width_seq = cp.Parameter(Np + 1, nonneg=True)
            self.band_lambda = cp.Parameter(nonneg=True)
            self.s_pos = cp.Variable(Np + 1, nonneg=True)
            self.s_neg = cp.Variable(Np + 1, nonneg=True)

        # IRLS robust weighting on lateral error (k=0..Np)
        self.irls_weights = cp.Parameter(Np + 1, nonneg=True)

        cost = 0
        constraints = []
        
        # Initial condition
        constraints.append(self.x[:, 0] == self.x0)
        
        # Stage costs and dynamics
        for k in range(Np):
            # State cost (scaled)
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

        # Band soft-constraint penalties
        if self.band_enable:
            cost += self.band_lambda * (cp.sum_squares(self.s_pos) + cp.sum_squares(self.s_neg))

        # IRLS additional cost on lateral error sequence
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

        # Band soft-constraint inequalities: |e_y(k)| <= w_k via slacks (optional)
        if self.band_enable:
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
        Q = self.control_params['Q']
        if not isinstance(Q, np.ndarray):
            Q = np.diag(Q) if isinstance(Q, list) else Q * np.eye(self.nx)
        R = self.control_params['R']
        R_delta = self.control_params['R_delta']
        P_term = self.control_params.get('P', Q * 10)
        if not isinstance(P_term, np.ndarray):
            P_term = np.diag(P_term) if isinstance(P_term, list) else P_term * np.eye(self.nx)
        
        # Extract constraints
        delta_limits = self.control_params['delta_limits']
        delta_rate_max = self.control_params['delta_rate_max']
        
        # Create OSQP solver
        self.osqp_solver = LateralOsqpSolver(
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
            Ts=Ts
        )
        
    def _get_discrete_matrices(self, v_x: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Get discrete-time model matrices for dynamic bicycle model.
        
        Args:
            v_x: Current vehicle speed [m/s]
            
        Returns:
            Tuple of (A_d, B_d, E_d) discrete matrices.
            E_d maps path curvature kappa to state update (error dynamics disturbance).
        """
        Ts = self.control_params['Ts']
        
        # Dynamic bicycle model parameters
        m = self.model_params['m']
        Iz = self.model_params['Iz']
        lf = self.model_params['lf']
        lr = self.model_params['lr']
        Caf = self.model_params['Caf']
        Car = self.model_params['Car']
        
        # Avoid division by zero at low speeds
        if abs(v_x) < 0.1:
            v_x = 0.1 if v_x >= 0 else -0.1
        
        # Continuous-time error dynamics matrices
        A_c = np.array([
            [0, 1, 0, 0],
            [0, -(Caf + Car)/(m*v_x), (Caf + Car)/m, 
             (-lf*Caf + lr*Car)/(m*v_x)],
            [0, 0, 0, 1],
            [0, (-lf*Caf + lr*Car)/(Iz*v_x), (lf*Caf - lr*Car)/Iz,
             -(lf**2*Caf + lr**2*Car)/(Iz*v_x)]
        ])
        
        B_c = np.array([[0], [Caf/m], [0], [lf*Caf/Iz]])

        # Curvature disturbance (from PRD error dynamics): [0, -v_x^2, 0, 0]^T * kappa
        E_c = np.array([0.0, -v_x**2, 0.0, 0.0])
        
        # Zero-order hold discretization (more accurate than Euler)
        # For simplicity, using matrix exponential approximation
        A_d = np.eye(4) + A_c * Ts + 0.5 * A_c @ A_c * Ts**2
        B_d = (np.eye(4) * Ts + 0.5 * A_c * Ts**2) @ B_c
        E_d = (np.eye(4) * Ts + 0.5 * A_c * Ts**2) @ E_c
        
        return A_d, B_d, E_d
    
    def solve(self, current_state: np.ndarray, current_speed: float,
              reference_path: Optional[Dict] = None) -> Tuple[float, Dict]:
        """
        Solve MPC optimization problem.
        
        Args:
            current_state: Current error state [e_y, (e_y_dot), e_psi, (e_psi_dot)]
            current_speed: Current vehicle speed [m/s]
            reference_path: Optional reference path information
            
        Returns:
            Tuple of (optimal_steering, debug_info)
        """
        start_time = time.time()
        
        # Use appropriate backend
        if self.solver_backend == 'osqp':
            # Provide time-varying weights to the backend for speed
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
        
        # Get time-varying model matrices (speed might change over horizon)
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
            self.curvature.value = curvatures
            kappa_seq = self.curvature.value
        else:
            self.curvature.value = np.zeros(Np)
            kappa_seq = self.curvature.value

        # Configure time-varying Q/P scales (ramp)
        if bool(self.control_params.get('time_varying_weights_enable', True)):
            q_front = float(self.control_params.get('q_scale_front', 0.7))
            q_back = float(self.control_params.get('q_scale_back', 1.3))
            if Np > 1:
                q_scales = np.linspace(q_front, q_back, Np)
            else:
                q_scales = np.array([q_back])
            p_scale_end = float(self.control_params.get('p_scale_end', 1.3))
        else:
            q_scales = np.ones(Np)
            p_scale_end = 1.0
        for k in range(Np):
            self.q_scales[k].value = float(q_scales[k])
        self.p_scale_end.value = p_scale_end

        # Configure band soft-constraint parameters (optional)
        if self.band_enable:
            w0 = float(self.control_params.get('band_base_width', 0.2))
            use_sched = bool(self.control_params.get('band_use_speed_curvature', True))
            w_min = float(self.control_params.get('band_width_min', 0.1))
            w_max = float(self.control_params.get('band_width_max', 0.35))
            gain_v = float(self.control_params.get('band_width_speed_gain', 0.0))
            gain_k = float(self.control_params.get('band_width_curvature_gain', 0.0))

            if use_sched and kappa_seq is not None:
                if isinstance(kappa_seq, np.ndarray):
                    kappa_full = np.concatenate([kappa_seq, [kappa_seq[-1] if kappa_seq.size > 0 else 0.0]])
                else:
                    kappa_full = np.zeros(Np + 1)
                w_seq = w0 + gain_v * float(current_speed) - gain_k * np.abs(kappa_full)
                w_seq = np.clip(w_seq, w_min, w_max)
            else:
                w_seq = np.clip(np.ones(Np + 1) * w0, w_min, w_max)
            self.band_half_width_seq.value = w_seq
            q_lat = float(self.Q_base[0, 0]) if isinstance(self.Q_base, np.ndarray) else float(np.diag(self.Q_base)[0])
            self.band_lambda.value = float(self.control_params.get('band_lambda', 10.0 * q_lat))

        # Initialize IRLS weights (disabled initially)
        irls_enable = bool(self.control_params.get('irls_enable', False))
        self.irls_weights.value = np.zeros(Np + 1)
        
        # Solve optimization (optionally with IRLS)
        try:
            # Fast OSQP settings via CVXPY; allow overrides via control_params
            solver_kwargs = {
                'solver': cp.OSQP,
                'warm_start': True,
                'verbose': False,
                'max_iter': int(self.control_params.get('osqp_max_iter', 400)),
                'eps_abs': float(self.control_params.get('osqp_eps_abs', 1e-3)),
                'eps_rel': float(self.control_params.get('osqp_eps_rel', 1e-3)),
                'polish': bool(self.control_params.get('osqp_polish', False)),
            }
            if float(self.control_params.get('osqp_time_limit', -1)) > 0.0:
                solver_kwargs['time_limit'] = float(self.control_params['osqp_time_limit'])

            # If previous solve was slow, neutralize heavy features this tick
            soft_time_ms = float(self.control_params.get('solve_time_soft_limit_ms', 20.0))
            slow_tick = self.last_solve_time > soft_time_ms
            if slow_tick:
                # Neutralize IRLS and band penalties
                self.irls_weights.value = np.zeros(Np + 1)
                if self.band_enable:
                    self.band_lambda.value = 0.0
                    self.band_half_width_seq.value = np.ones(Np + 1) * 100.0

            self.problem.solve(**solver_kwargs)
            
            # IRLS refinements
            if irls_enable and self.problem.status in ["optimal", "optimal_inaccurate"]:
                max_iters = int(self.control_params.get('irls_max_iters', 1))
                kappa_huber = float(self.control_params.get('irls_kappa', 0.2))
                eps = float(self.control_params.get('irls_epsilon', 1e-3))
                for _ in range(max(0, max_iters)):
                    if self.x.value is None:
                        break
                    e_seq = np.array(self.x.value[0, :]).flatten()
                    abs_e = np.abs(e_seq)
                    w_vec = np.where(abs_e <= kappa_huber, 1.0, kappa_huber / (abs_e + eps))
                    self.irls_weights.value = w_vec
                    self.problem.solve(solver=cp.OSQP, warm_start=True, verbose=False)

            if self.problem.status in ["optimal", "optimal_inaccurate"]:
                optimal_steering = self.u.value[0, 0]
                self.last_u = optimal_steering
                self.last_status = "optimal"
                self.last_cost = self.problem.value
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
            'solver_iterations': getattr(self.problem, 'solver_stats', None).num_iters if getattr(self.problem, 'solver_stats', None) is not None else None
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
            # Pass weight schedule if available
            q_scales = None
            p_scale_end = None
            if hasattr(self, 'osqp_weight_schedule'):
                q_scales, p_scale_end = self.osqp_weight_schedule
            optimal_steering, debug_info = self.osqp_solver.update_and_solve(
                x0=current_state,
                A_seq=A_seq,
                B_seq=B_seq,
                E_seq=E_seq,
                kappa_seq=kappa_seq,
                u_prev=self.last_u,
                q_scales=q_scales,
                p_scale_end=p_scale_end,
                max_iter=max_iter,
                eps_abs=eps_abs,
                eps_rel=eps_rel
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
                'current_speed': current_speed
            }
            self.last_status = 'exception'
        
        # Add additional debug info
        debug_info['current_speed'] = current_speed
        debug_info['backend'] = 'osqp'
        
        # Update solve time
        self.last_solve_time = debug_info['solver_time']
        
        return optimal_steering, debug_info
    
    def calculate_feedforward_steering(self, curvature: float, v_x: float) -> float:
        """
        Calculate feedforward steering based on path curvature.
        Dynamic model with understeer compensation.
        
        Args:
            curvature: Path curvature [1/m]
            v_x: Vehicle speed [m/s]
            
        Returns:
            Feedforward steering angle [rad]
        """
        L = self.model_params['L']
        
        # Include understeer compensation
        if 'Kv' in self.model_params:
            Kv = self.model_params['Kv']
            delta_ff = L * curvature + Kv * v_x**2 * curvature
        else:
            # Estimate from cornering stiffness
            m = self.model_params['m']
            lf = self.model_params['lf']
            lr = self.model_params['lr']
            Caf = self.model_params['Caf']
            Car = self.model_params['Car']
            
            Kv = (m/L) * (lr/Caf - lf/Car)
            delta_ff = L * curvature + Kv * v_x**2 * curvature
        
        return delta_ff
    
    def reset(self):
        """Reset controller state."""
        self.last_u = 0.0
        self.last_solve_time = 0.0
        self.last_cost = 0.0
        self.last_status = "not_solved"