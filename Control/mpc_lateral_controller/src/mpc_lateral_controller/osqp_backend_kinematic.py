#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
OSQP Backend for Kinematic MPC Controller

Direct OSQP implementation for kinematic bicycle model MPC.
Eliminates CVXPY overhead for real-time performance.
"""

import numpy as np
import osqp
import scipy.sparse as sparse
from typing import Dict, Tuple, Optional, List
import time

# Try to import rospy, but don't fail if not available
try:
    import rospy
except ImportError:
    # Create a dummy rospy module for testing without ROS
    class DummyRospy:
        @staticmethod
        def logdebug(msg):
            pass
    rospy = DummyRospy()

# Import helper to suppress OSQP warnings
try:
    from .osqp_suppress import suppress_stderr_only
except ImportError:
    # Fallback if import fails
    import contextlib
    @contextlib.contextmanager
    def suppress_stderr_only():
        yield


class KinematicOsqpSolver:
    """Direct OSQP solver for kinematic MPC without CVXPY overhead."""
    
    def __init__(self, nx: int, nu: int, Np: int, Nc: int,
                 Q: np.ndarray, R: float, R_delta: float, P_term: np.ndarray,
                 delta_limits: List[float], delta_rate_max: float, Ts: float,
                 config: Optional[Dict] = None):
        """
        Initialize OSQP solver for kinematic MPC.
        
        Args:
            nx: State dimension (2 for kinematic: e_y, e_psi)
            nu: Control dimension (1: steering angle)
            Np: Prediction horizon
            Nc: Control horizon
            Q: State cost matrix (2x2)
            R: Control cost scalar
            R_delta: Control rate cost scalar
            P_term: Terminal cost matrix (2x2)
            delta_limits: [min, max] steering limits
            delta_rate_max: Maximum steering rate
            Ts: Sampling time
            config: Optional configuration dictionary for advanced features
        """
        self.nx = nx
        self.nu = nu
        self.Np = Np
        self.Nc = Nc
        self.Ts = Ts
        
        # Cost weights (store base for time-varying updates)
        self.Q = Q if isinstance(Q, np.ndarray) else np.diag(Q)
        self.R = R
        self.R_delta = R_delta
        self.P_term = P_term if isinstance(P_term, np.ndarray) else np.diag(P_term)
        
        # Constraints
        self.delta_min = delta_limits[0]
        self.delta_max = delta_limits[1]
        self.delta_rate_max = delta_rate_max
        
        # Store original values for state-dependent smoothing
        self.delta_rate_max_base = delta_rate_max
        self.R_delta_base = R_delta
        
        # Load configuration for advanced features
        self.config = config or {}
        
        # Band-aware Q weighting with continuous blending
        self.band_aware_q = self.config.get('band_aware_q_weighting', False)
        if self.band_aware_q:
            q_inside = self.config.get('Q_kinematic_inside_band', [5.0, 800.0])
            q_outside = self.config.get('Q_kinematic_outside_band', [250.0, 400.0])
            self.Q_inside = np.diag(q_inside)
            self.Q_outside = np.diag(q_outside)
            self.band_blend_hyst = self.config.get('band_blend_hysteresis', True)
            self.band_hyst_inner = self.config.get('band_hyst_ratio_inner', 0.9)
            self.band_hyst_outer = self.config.get('band_hyst_ratio_outer', 1.1)
        
        # Input cost centering (feedforward reference)
        self.input_cost_centering = self.config.get('input_cost_centering', False)
        self.understeer_comp = self.config.get('understeer_compensation', False)
        self.kappa_smoothing_alpha = float(self.config.get('kappa_smoothing_alpha', 0.7))
        self._last_kappa = 0.0
        
        # State-dependent smoothing
        self.state_dep_smoothing = self.config.get('state_dep_smoothing_enable', False)
        if self.state_dep_smoothing:
            self.inside_delta_rate_max = self.config.get('inside_delta_rate_max', 0.025)
            self.inside_R_delta = self.config.get('inside_R_delta', 15.0)
            self.outside_delta_rate_max = self.config.get('outside_delta_rate_max', 0.015)
            self.outside_R_delta = self.config.get('outside_R_delta', 30.0)
        
        # Variable dimensions (include slack variables for band soft-constraints)
        self.n_states = nx * (Np + 1)  # Total state variables
        self.n_controls = nu * Nc      # Total control variables
        self.n_slack = 2 * (Np + 1)    # s_pos[0..Np], s_neg[0..Np]
        self.n_vars = self.n_states + self.n_controls + self.n_slack
        
        # Constraint dimensions
        self.n_eq_init = nx  # Initial state equality
        self.n_eq_dyn = nx * Np  # Dynamics equality
        self.n_ineq_u = 2 * Nc  # Control box constraints
        self.n_ineq_du = 2 * Nc  # Control rate constraints
        # Band soft-constraint inequalities and slack nonnegativity
        self.n_ineq_band = 2 * (Np + 1)     # e_lat - s_pos <= w,  -e_lat - s_neg <= w
        self.n_ineq_s_nonneg = 2 * (Np + 1) # s_pos >= 0, s_neg >= 0
        self.n_constraints = (self.n_eq_init + self.n_eq_dyn + self.n_ineq_u + self.n_ineq_du +
                              self.n_ineq_band + self.n_ineq_s_nonneg)
        
        # Build constant matrices (initial with unit scales)
        self._build_constant_matrices()
        
        # Setup OSQP solver
        self._setup_solver()
        
        # Warm start storage
        self.last_solution = None
        self.consecutive_failures = 0
        self.last_lateral_error = 0.0
        
    def _x_index(self, k: int) -> slice:
        """Get state variable indices for time step k."""
        start = k * self.nx
        return slice(start, start + self.nx)
    
    def _u_index(self, k: int) -> int:
        """Get control variable index for time step k."""
        return self.n_states + k * self.nu
    
    def _s_pos_index(self, k: int) -> int:
        """Get s_pos index for time step k (0..Np)."""
        return self.n_states + self.n_controls + k

    def _s_neg_index(self, k: int) -> int:
        """Get s_neg index for time step k (0..Np)."""
        return self.n_states + self.n_controls + (self.Np + 1) + k
    
    def _build_constant_matrices(self):
        """Build constant cost matrix P and constraint pattern."""
        
        # ========== Cost Matrix P (with time-varying weight support) ==========
        # P is block diagonal: [P_X, P_U]
        # IMPORTANT: OSQP minimizes (1/2) z'Pz + q'z, so we need to multiply by 2
        # Build with default scales (ones) and store structure indices for fast updates

        # Default scales (per stage and terminal)
        self.q_scales_default = np.ones(self.Np)
        self.p_scale_default = 1.0

        # Build state cost block with defaults
        P_X_blocks = []
        for k in range(self.Np):
            P_X_blocks.append(2 * (self.q_scales_default[k] * self.Q))
        P_X_blocks.append(2 * (self.p_scale_default * self.P_term))
        P_X = sparse.block_diag(P_X_blocks, format='csc')
        
        # Control cost block P_U
        # Need to build T matrix for rate penalties
        T = np.zeros((self.Nc, self.Nc))
        for i in range(self.Nc):
            T[i, i] = 1
            if i > 0:
                T[i, i-1] = -1
        
        # P_U = 2 * (R * I + R_delta * T^T @ T)
        # Factor of 2 because OSQP has 1/2 in objective
        P_U_dense = 2 * (self.R * np.eye(self.Nc) + self.R_delta * T.T @ T)
        P_U = sparse.csc_matrix(P_U_dense)
        
        # Slack penalty block: pre-allocate diagonal structure so sparsity stays constant
        # Build with identity, then zero out values before setup
        P_S = sparse.eye(2 * (self.Np + 1), format='csc')

        # Combine into full P matrix
        self.P = sparse.block_diag([P_X, P_U, P_S], format='csc')

        # Cache index ranges for state/control/slack parts in P for updates
        self.P_state_nnz = P_X.nnz
        self.P_control_nnz = P_U.nnz
        self.P_slack_nnz = P_S.nnz
        # Keep masks to update only specific blocks when weights change
        self._P_state_indices = np.arange(self.P_state_nnz)
        self._P_control_indices = np.arange(self.P_state_nnz, self.P_state_nnz + self.P_control_nnz)
        self._P_slack_indices = np.arange(self.P_state_nnz + self.P_control_nnz,
                                          self.P_state_nnz + self.P_control_nnz + self.P_slack_nnz)

        # Initialize slack diagonal values to small positive
        # IMPORTANT: Using 0.0 causes OSQP to eliminate zeros, changing nnz count
        # This leads to "new number of elements out of bounds" error during updates
        # We use a small value (1e-6) to maintain sparsity structure
        # This value is small enough to not affect optimization but large enough
        # to prevent OSQP from eliminating these entries
        self.P.data[self._P_slack_indices] = 1e-6
        
        # ========== Constraint Matrix A Pattern ==========
        # We'll build the sparsity pattern, then update values each iteration
        
        rows = []
        cols = []
        self.A_data_indices = {}  # Maps constraint type to data indices
        data_counter = 0
        row_counter = 0
        
        # 1. Initial state constraint: x_0 = x_init
        for i in range(self.nx):
            rows.append(row_counter + i)
            cols.append(i)
            if 'init_state' not in self.A_data_indices:
                self.A_data_indices['init_state'] = []
            self.A_data_indices['init_state'].append(data_counter)
            data_counter += 1
        row_counter += self.nx
        
        # 2. Dynamics constraints: x_{k+1} = A_k x_k + B_k u_k + E_k kappa_k
        # Rearranged as: x_{k+1} - A_k x_k - B_k u_k = E_k kappa_k
        for k in range(self.Np):
            # x_{k+1} coefficient (I)
            for i in range(self.nx):
                rows.append(row_counter + i)
                cols.append((k + 1) * self.nx + i)
                if f'dyn_{k}_xnext' not in self.A_data_indices:
                    self.A_data_indices[f'dyn_{k}_xnext'] = []
                self.A_data_indices[f'dyn_{k}_xnext'].append(data_counter)
                data_counter += 1
            
            # -A_k @ x_k coefficient
            for i in range(self.nx):
                for j in range(self.nx):
                    rows.append(row_counter + i)
                    cols.append(k * self.nx + j)
                    if f'dyn_{k}_A' not in self.A_data_indices:
                        self.A_data_indices[f'dyn_{k}_A'] = []
                    self.A_data_indices[f'dyn_{k}_A'].append(data_counter)
                    data_counter += 1
            
            # -B_k @ u_k coefficient
            # Select appropriate control (u_k for k < Nc, u_{Nc-1} for k >= Nc)
            u_idx = min(k, self.Nc - 1)
            for i in range(self.nx):
                rows.append(row_counter + i)
                cols.append(self.n_states + u_idx)
                if f'dyn_{k}_B' not in self.A_data_indices:
                    self.A_data_indices[f'dyn_{k}_B'] = []
                self.A_data_indices[f'dyn_{k}_B'].append(data_counter)
                data_counter += 1
            
            row_counter += self.nx
        
        # 3. Control box constraints: delta_min <= u_k <= delta_max
        for k in range(self.Nc):
            # Lower bound
            rows.append(row_counter)
            cols.append(self.n_states + k)
            if f'u_box_{k}' not in self.A_data_indices:
                self.A_data_indices[f'u_box_{k}'] = []
            self.A_data_indices[f'u_box_{k}'].append(data_counter)
            data_counter += 1
            row_counter += 1
            
            # Upper bound
            rows.append(row_counter)
            cols.append(self.n_states + k)
            self.A_data_indices[f'u_box_{k}'].append(data_counter)
            data_counter += 1
            row_counter += 1
        
        # 4. Control rate constraints: -delta_rate <= u_k - u_{k-1} <= delta_rate
        for k in range(self.Nc):
            if k == 0:
                # u_0 - u_prev: only u_0 coefficient
                # Lower bound
                rows.append(row_counter)
                cols.append(self.n_states)
                if f'du_{k}' not in self.A_data_indices:
                    self.A_data_indices[f'du_{k}'] = []
                self.A_data_indices[f'du_{k}'].append(data_counter)
                data_counter += 1
                row_counter += 1
                
                # Upper bound
                rows.append(row_counter)
                cols.append(self.n_states)
                self.A_data_indices[f'du_{k}'].append(data_counter)
                data_counter += 1
                row_counter += 1
            else:
                # u_k - u_{k-1}: both coefficients
                if f'du_{k}' not in self.A_data_indices:
                    self.A_data_indices[f'du_{k}'] = []
                    
                # Lower bound
                rows.append(row_counter)
                cols.append(self.n_states + k)
                self.A_data_indices[f'du_{k}'].append(data_counter)
                data_counter += 1
                
                rows.append(row_counter)
                cols.append(self.n_states + k - 1)
                self.A_data_indices[f'du_{k}'].append(data_counter)
                data_counter += 1
                row_counter += 1
                
                # Upper bound  
                rows.append(row_counter)
                cols.append(self.n_states + k)
                self.A_data_indices[f'du_{k}'].append(data_counter)
                data_counter += 1
                
                rows.append(row_counter)
                cols.append(self.n_states + k - 1)
                self.A_data_indices[f'du_{k}'].append(data_counter)
                data_counter += 1
                row_counter += 1
        
        # 5. Band soft-constraint inequalities: |e_y(k)| <= w_k + slack
        for k in range(self.Np + 1):
            # e_lat_k - s_pos_k <= w_k
            rows.append(row_counter)
            cols.append(k * self.nx + 0)  # +1 on e_lat
            if 'band_pos_elat' not in self.A_data_indices:
                self.A_data_indices['band_pos_elat'] = []
            self.A_data_indices['band_pos_elat'].append(data_counter); data_counter += 1
            rows.append(row_counter); cols.append(self._s_pos_index(k))  # -1 on s_pos
            if 'band_pos_s' not in self.A_data_indices:
                self.A_data_indices['band_pos_s'] = []
            self.A_data_indices['band_pos_s'].append(data_counter); data_counter += 1
            row_counter += 1

            # -e_lat_k - s_neg_k <= w_k
            rows.append(row_counter); cols.append(k * self.nx + 0)  # -1 on e_lat
            if 'band_neg_elat' not in self.A_data_indices:
                self.A_data_indices['band_neg_elat'] = []
            self.A_data_indices['band_neg_elat'].append(data_counter); data_counter += 1
            rows.append(row_counter); cols.append(self._s_neg_index(k))  # -1 on s_neg
            if 'band_neg_s' not in self.A_data_indices:
                self.A_data_indices['band_neg_s'] = []
            self.A_data_indices['band_neg_s'].append(data_counter); data_counter += 1
            row_counter += 1

        # 6. Slack nonnegativity: s_pos >= 0, s_neg >= 0
        for k in range(self.Np + 1):
            rows.append(row_counter); cols.append(self._s_pos_index(k))
            if 'spos_nonneg' not in self.A_data_indices:
                self.A_data_indices['spos_nonneg'] = []
            self.A_data_indices['spos_nonneg'].append(data_counter); data_counter += 1
            row_counter += 1

            rows.append(row_counter); cols.append(self._s_neg_index(k))
            if 'sneg_nonneg' not in self.A_data_indices:
                self.A_data_indices['sneg_nonneg'] = []
            self.A_data_indices['sneg_nonneg'].append(data_counter); data_counter += 1
            row_counter += 1

        # Create sparse matrix pattern
        data = np.ones(len(rows))  # Placeholder values
        self.A_pattern = sparse.csc_matrix((data, (rows, cols)), 
                                          shape=(self.n_constraints, self.n_vars))
        
        # Store pattern info
        self.A_rows = rows
        self.A_cols = cols
        self.n_A_elements = len(rows)
        
    def _setup_solver(self, max_iter=800, eps_abs=2e-3, eps_rel=2e-3):
        """Setup OSQP solver with initial problem.
        
        Args:
            max_iter: Maximum iterations (default: 800, increased from 400)
            eps_abs: Absolute tolerance (default: 2e-3, relaxed from 1e-3)
            eps_rel: Relative tolerance (default: 2e-3, relaxed from 1e-3)
        """
        
        # Initial q vector (will be updated each iteration)
        self.q = np.zeros(self.n_vars)
        
        # Initial constraint bounds (will be updated each iteration)
        self.l = np.zeros(self.n_constraints)
        self.u = np.zeros(self.n_constraints)
        
        # Create OSQP solver
        self.solver = osqp.OSQP()
        
        # Setup with initial values
        self.solver.setup(
            P=self.P,
            q=self.q,
            A=self.A_pattern,
            l=self.l,
            u=self.u,
            warm_start=True,
            verbose=False,
            adaptive_rho=True,  # Keep adaptive rho enabled for better convergence
            max_iter=max_iter,
            eps_abs=eps_abs,
            eps_rel=eps_rel,
            polish=False,
            alpha=1.6  # Standard relaxation parameter
        )
        
    def update_and_solve(self, x0: np.ndarray, A_seq: List[np.ndarray], 
                        B_seq: List[np.ndarray], E_seq: List[np.ndarray],
                        kappa_seq: np.ndarray, u_prev: float,
                        max_iter: Optional[int] = None,
                        eps_abs: Optional[float] = None,
                        eps_rel: Optional[float] = None,
                        q_scales: Optional[np.ndarray] = None,
                        p_scale_end: Optional[float] = None,
                        band_enable: bool = False,
                        band_w_seq: Optional[np.ndarray] = None,
                        band_lambda: float = 0.0,
                        irls_enable: bool = False,
                        irls_max_iters: int = 0,
                        irls_kappa: float = 0.2,
                        irls_epsilon: float = 1e-3,
                        active_Q: Optional[np.ndarray] = None,
                        v_x: Optional[float] = None,
                        L: Optional[float] = None) -> Tuple[float, Dict]:
        """
        Update problem data and solve.
        
        Args:
            x0: Initial state [e_y, e_psi]
            A_seq: List of A matrices for each time step
            B_seq: List of B matrices for each time step
            E_seq: List of E vectors for each time step
            kappa_seq: Curvature sequence
            u_prev: Previous control input
            max_iter: Optional override for max iterations
            eps_abs: Optional override for absolute tolerance
            eps_rel: Optional override for relative tolerance
            v_x: Vehicle speed for feedforward calculation
            L: Wheelbase for feedforward calculation
            
        Returns:
            Tuple of (optimal_control, debug_info)
        """
        start_time = time.time()
        
        # ========== Band-aware Q weighting with continuous blending ==========
        if self.band_aware_q and band_enable and band_w_seq is not None:
            # Get current lateral error and band width
            e_y = abs(x0[0])
            w = band_w_seq[0] if len(band_w_seq) > 0 else 0.5
            
            # Calculate blending boundaries with hysteresis
            if self.band_blend_hyst:
                w_in = self.band_hyst_inner * w
                w_out = self.band_hyst_outer * w
            else:
                w_in = w
                w_out = w
            
            # Calculate continuous blending factor
            if e_y <= w_in:
                s = 0.0  # Fully inside
            elif e_y >= w_out:
                s = 1.0  # Fully outside
            else:
                # Linear interpolation between w_in and w_out
                s = (e_y - w_in) / (w_out - w_in)
            
            # Blend Q matrices
            Q_blended = (1 - s) * self.Q_inside + s * self.Q_outside
            active_Q = Q_blended
            
            # Debug logging (rospy-independent)
            if hasattr(self, '_debug_print') and self._debug_print:
                print(f"Band-aware Q: e_y={e_y:.3f}, w={w:.3f}, s={s:.3f}")
        
        # ========== State-dependent smoothing ==========
        if self.state_dep_smoothing and band_enable and band_w_seq is not None:
            e_y = abs(x0[0])
            w = band_w_seq[0] if len(band_w_seq) > 0 else 0.5
            
            # Check if inside or outside band (with some margin to avoid chattering)
            if e_y < 0.95 * w:  # Inside band
                self.delta_rate_max = self.inside_delta_rate_max
                self.R_delta = self.inside_R_delta
            else:  # Outside band
                self.delta_rate_max = self.outside_delta_rate_max
                self.R_delta = self.outside_R_delta
        
        # ========== Update P matrix for time-varying weights (if provided) ==========
        # NOTE: P matrix updates may cause OSQP warnings when Q changes,
        # but the solver still works correctly. We suppress stderr temporarily.
        if q_scales is not None or p_scale_end is not None or active_Q is not None:
            if q_scales is None:
                q_scales = self.q_scales_default
            if p_scale_end is None:
                p_scale_end = self.p_scale_default
            
            # Use active_Q if provided (for band-aware Q switching), otherwise use self.Q
            Q_to_use = active_Q if active_Q is not None else self.Q
            # Ensure Q is 2x2 diagonal matrix
            if not isinstance(Q_to_use, np.ndarray):
                Q_to_use = np.diag(Q_to_use)
            elif Q_to_use.ndim == 1:
                Q_to_use = np.diag(Q_to_use)
            
            P_term_to_use = 10.0 * Q_to_use if active_Q is not None else self.P_term

            # Build new P_X data values only (preserve sparsity structure)
            # We only update the diagonal values, not the structure
            P_X_new_values = []
            for k in range(self.Np):
                scale = float(q_scales[k])
                # Add diagonal elements for this stage (2x2 diagonal)
                P_X_new_values.append(2 * scale * Q_to_use[0, 0])
                P_X_new_values.append(2 * scale * Q_to_use[1, 1])
            # Terminal cost
            P_X_new_values.append(2 * float(p_scale_end) * P_term_to_use[0, 0])
            P_X_new_values.append(2 * float(p_scale_end) * P_term_to_use[1, 1])
            
            P_X_new_values = np.array(P_X_new_values)

            # Update the P matrix data directly with new values
            # This preserves the sparsity structure
            P_new_data = self.P.data.copy()
            
            # Check that we have the right number of values
            if len(P_X_new_values) == len(self._P_state_indices):
                P_new_data[self._P_state_indices] = P_X_new_values
                
                # Update P matrix
                # OSQP will print a warning about nnz if Q changes, but it still works
                # We suppress stderr to hide the warning
                try:
                    with suppress_stderr_only():
                        self.solver.update(Px=P_new_data)
                except:
                    # If update fails (e.g., "out of bounds"), continue with original P
                    # The solver will still work, just without the new weights
                    pass

        # ========== Calculate feedforward steering ==========
        delta_ff_seq = np.zeros(self.Nc)
        if self.input_cost_centering and L is not None and v_x is not None:
            # Feedforward steering: delta_ff = L * kappa
            # With optional understeer compensation: delta_ff = L * kappa * (1 + Kv * v^2)
            kappa_f = np.zeros_like(kappa_seq)
            # Exponential smoothing of curvature to reduce command jitter
            alpha = self.kappa_smoothing_alpha
            last = self._last_kappa
            for k in range(len(kappa_seq)):
                last = alpha * last + (1.0 - alpha) * float(kappa_seq[k])
                kappa_f[k] = last
            self._last_kappa = last

            for k in range(min(self.Nc, len(kappa_f))):
                delta_ff = L * kappa_f[k]
                if self.understeer_comp:
                    # Simple understeer model (Kv could be tuned)
                    Kv = 0.001  # Understeer gradient (typical value)
                    delta_ff *= (1 + Kv * v_x * v_x)
                delta_ff_seq[k] = delta_ff
        
        # ========== Update q vector ==========
        # Only control part has non-zero q from rate penalty
        self.q[:self.n_states] = 0  # State part
        
        if self.input_cost_centering and L is not None:
            # Input cost centered on feedforward: R*(u - delta_ff)^2
            # Expanding: R*u^2 - 2*R*u*delta_ff + R*delta_ff^2
            # The quadratic term R*u^2 is in P matrix
            # The linear term is: q = -2*R*delta_ff
            q_U = np.zeros(self.Nc)
            for k in range(self.Nc):
                q_U[k] = -2 * self.R * delta_ff_seq[k]
            
            # Add rate penalty term for first control
            q_U[0] += -2 * self.R_delta * u_prev
        else:
            # Original formulation without feedforward centering
            q_U = np.zeros(self.Nc)
            q_U[0] = -2 * self.R_delta * u_prev  # Negative for correct gradient
        
        self.q[self.n_states:self.n_states + self.n_controls] = q_U
        # Slack linear penalties (L1) for band soft-constraints
        if band_enable and band_lambda > 0.0:
            # s_pos and s_neg both penalized with band_lambda
            start_s = self.n_states + self.n_controls
            end_s = start_s + self.n_slack
            self.q[start_s:end_s] = band_lambda
        else:
            start_s = self.n_states + self.n_controls
            end_s = start_s + self.n_slack
            self.q[start_s:end_s] = 0.0
        
        # ========== Update A matrix values ==========
        A_data = np.zeros(self.n_A_elements)
        
        # 1. Initial state constraint
        for i, idx in enumerate(self.A_data_indices['init_state']):
            A_data[idx] = 1.0
        
        # 2. Dynamics constraints
        for k in range(self.Np):
            # x_{k+1} coefficient (I)
            for i, idx in enumerate(self.A_data_indices[f'dyn_{k}_xnext']):
                A_data[idx] = 1.0
            
            # -A_k coefficient
            A_k_flat = -A_seq[k].flatten()
            for i, idx in enumerate(self.A_data_indices[f'dyn_{k}_A']):
                A_data[idx] = A_k_flat[i]
            
            # -B_k coefficient
            B_k_flat = -B_seq[k].flatten()
            for i, idx in enumerate(self.A_data_indices[f'dyn_{k}_B']):
                A_data[idx] = B_k_flat[i]
        
        # 3. Control box constraints
        for k in range(self.Nc):
            for idx in self.A_data_indices[f'u_box_{k}']:
                A_data[idx] = 1.0
        
        # 4. Control rate constraints
        for k in range(self.Nc):
            if k == 0:
                # u_0 coefficient
                for idx in self.A_data_indices[f'du_{k}']:
                    A_data[idx] = 1.0
            else:
                # u_k and -u_{k-1} coefficients
                indices = self.A_data_indices[f'du_{k}']
                A_data[indices[0]] = 1.0   # u_k for lower bound
                A_data[indices[1]] = -1.0  # -u_{k-1} for lower bound
                A_data[indices[2]] = 1.0   # u_k for upper bound
                A_data[indices[3]] = -1.0  # -u_{k-1} for upper bound
        # 5. Band-constraint coefficients and slack nonnegativity
        for idx in self.A_data_indices.get('band_pos_elat', []):
            A_data[idx] = 1.0
        for idx in self.A_data_indices.get('band_pos_s', []):
            A_data[idx] = -1.0
        for idx in self.A_data_indices.get('band_neg_elat', []):
            A_data[idx] = -1.0
        for idx in self.A_data_indices.get('band_neg_s', []):
            A_data[idx] = -1.0
        for idx in self.A_data_indices.get('spos_nonneg', []):
            A_data[idx] = 1.0
        for idx in self.A_data_indices.get('sneg_nonneg', []):
            A_data[idx] = 1.0
        
        # ========== Update constraint bounds ==========
        row = 0
        
        # 1. Initial state
        self.l[row:row+self.nx] = x0
        self.u[row:row+self.nx] = x0
        row += self.nx
        
        # 2. Dynamics (E_k * kappa_k on RHS)
        for k in range(self.Np):
            # Make sure kappa_seq has enough elements
            if k < len(kappa_seq):
                rhs = E_seq[k] * kappa_seq[k]
            else:
                rhs = E_seq[k] * 0.0  # Use zero if not enough curvatures provided
            self.l[row:row+self.nx] = rhs
            self.u[row:row+self.nx] = rhs
            row += self.nx
        
        # 3. Control box constraints
        for k in range(self.Nc):
            self.l[row] = self.delta_min
            self.u[row] = np.inf
            row += 1
            
            self.l[row] = -np.inf
            self.u[row] = self.delta_max
            row += 1
        
        # 4. Control rate constraints
        for k in range(self.Nc):
            if k == 0:
                # u_0 - u_prev constraints: -delta_rate_max <= u_0 - u_prev <= delta_rate_max
                # Since A has coefficient 1 for u_0, bounds are: u_prev - delta_rate_max <= u_0 <= u_prev + delta_rate_max
                self.l[row] = -self.delta_rate_max + u_prev
                self.u[row] = np.inf
                row += 1
                
                self.l[row] = -np.inf
                self.u[row] = self.delta_rate_max + u_prev
                row += 1
            else:
                # u_k - u_{k-1}
                self.l[row] = -self.delta_rate_max
                self.u[row] = np.inf
                row += 1
                
                self.l[row] = -np.inf
                self.u[row] = self.delta_rate_max
                row += 1
        # 5. Band soft-constraint bounds: e_lat - s_pos <= w_k and -e_lat - s_neg <= w_k
        if band_enable and band_w_seq is not None and len(band_w_seq) >= (self.Np + 1):
            for k in range(self.Np + 1):
                self.l[row] = -np.inf
                self.u[row] = float(band_w_seq[k])
                row += 1
                self.l[row] = -np.inf
                self.u[row] = float(band_w_seq[k])
                row += 1
        else:
            big = 1e6
            for k in range(self.Np + 1):
                self.l[row] = -np.inf
                self.u[row] = big
                row += 1
                self.l[row] = -np.inf
                self.u[row] = big
                row += 1
        # 6. Slack nonnegativity bounds: s_pos >= 0, s_neg >= 0
        for k in range(self.Np + 1):
            self.l[row] = 0.0
            self.u[row] = np.inf
            row += 1
            self.l[row] = 0.0
            self.u[row] = np.inf
            row += 1
        
        # ========== Update OSQP and solve ==========
        # Update solver settings if provided
        if max_iter is not None or eps_abs is not None or eps_rel is not None:
            settings = {}
            if max_iter is not None:
                settings['max_iter'] = max_iter
            if eps_abs is not None:
                settings['eps_abs'] = eps_abs
            if eps_rel is not None:
                settings['eps_rel'] = eps_rel
            self.solver.update_settings(**settings)
        
        # Create updated A matrix with new values
        A_updated = sparse.csc_matrix((A_data, (self.A_rows, self.A_cols)), 
                                      shape=(self.n_constraints, self.n_vars))
        
        # Update problem data
        # Note: OSQP update expects the data array directly for Ax
        self.solver.update(
            q=self.q,
            Ax=A_updated.data,  # Pass the data array of the CSC matrix
            l=self.l,
            u=self.u
        )
        
        # Warm start logic with reset on large errors or consecutive failures
        # Reset warm start if:
        # 1. Large lateral error (> 1.0m)
        # 2. Large heading error (> 0.3 rad)
        # 3. Multiple consecutive failures
        should_reset_warm_start = False
        if abs(x0[0]) > 1.0:  # Large lateral error
            should_reset_warm_start = True
            pass  # Resetting due to large lateral error
        elif abs(x0[1]) > 0.3:  # Large heading error
            should_reset_warm_start = True
            pass  # Resetting due to large heading error
        elif self.consecutive_failures >= 2:
            should_reset_warm_start = True
            pass  # Resetting due to consecutive failures
        
        if self.last_solution is not None and not should_reset_warm_start:
            self.solver.warm_start(x=self.last_solution)
        else:
            # Clear warm start
            self.last_solution = None
        
        # Solve with potential retry on failure
        results = self.solver.solve()
        
        # Get numeric status value (for version compatibility)
        if hasattr(results.info, 'status_val'):
            status_val = results.info.status_val
        else:
            # Fallback for older versions
            status_val = results.info.status if isinstance(results.info.status, int) else -1
        
        # Retry logic for max_iter_reached (status_val = 7 or -2)
        if status_val in [7, -2] and results.info.iter >= self.solver.settings.max_iter - 1:
            # First solve failed, retrying with relaxed settings
            
            # Update settings for retry with more relaxed tolerances
            self.solver.update_settings(
                max_iter=1200,  # Increase iterations
                eps_abs=5e-3,   # Relax absolute tolerance
                eps_rel=5e-3    # Relax relative tolerance
            )
            
            # Clear warm start for retry
            self.solver.warm_start(x=None, y=None)
            
            # Retry solve
            results = self.solver.solve()
            
            # Restore original settings for next iteration
            self.solver.update_settings(
                max_iter=max_iter or 800,
                eps_abs=eps_abs or 2e-3,
                eps_rel=eps_rel or 2e-3
            )
            
            # Update status after retry
            if hasattr(results.info, 'status_val'):
                status_val = results.info.status_val
            else:
                status_val = results.info.status if isinstance(results.info.status, int) else -1
        
        # Extract solution
        # OSQP status codes: 1=solved, 2=solved_inaccurate
        if status_val == 1 or status_val == 2:  # solved or solved_inaccurate
            self.last_solution = results.x
            u0 = results.x[self._u_index(0)]
            
            # Extract predicted trajectory for debug
            X_pred = np.zeros((self.nx, self.Np + 1))
            U_pred = np.zeros((self.nu, self.Nc))
            
            for k in range(self.Np + 1):
                X_pred[:, k] = results.x[self._x_index(k)]
            
            for k in range(self.Nc):
                U_pred[0, k] = results.x[self._u_index(k)]
            
            status = 'optimal'
            cost = results.info.obj_val
            
            # Reset consecutive failures on success
            self.consecutive_failures = 0
        else:
            # Debug: print why it failed
            status_map = {
                3: 'primal_infeasible',
                5: 'dual_infeasible',
                7: 'max_iter_reached',
                -2: 'unsolved',
                -3: 'sigint',
                -4: 'time_limit_reached'
            }
            status_str = status_map.get(results.info.status, f'unknown_{results.info.status}')
            
            # Improved fallback strategy
            # Simple P-controller fallback based on lateral error
            k_p = 0.5  # Proportional gain
            k_heading = 0.3  # Heading error gain
            fallback_steering = -k_p * x0[0] - k_heading * x0[1]  # Negative for correct direction
            
            # Apply limits
            fallback_steering = np.clip(fallback_steering, self.delta_min, self.delta_max)
            
            # Apply rate limit from previous control
            max_change = self.delta_rate_max
            fallback_steering = np.clip(fallback_steering, u_prev - max_change, u_prev + max_change)
            
            u0 = fallback_steering
            X_pred = None
            U_pred = None
            status = f'failed_{status_str}_with_fallback'
            cost = np.inf
            
            # Track consecutive failures
            self.consecutive_failures += 1
            rospy.logwarn(f"OSQP failed ({status_str}), using P-controller fallback. Consecutive failures: {self.consecutive_failures}")
        
        solve_time = (time.time() - start_time) * 1000  # ms
        
        # Debug info
        status_str_map = {
            1: 'solved',
            2: 'solved_inaccurate',
            3: 'primal_infeasible',
            5: 'dual_infeasible',
            7: 'max_iter_reached',
            -2: 'unsolved',
            -3: 'sigint',
            -4: 'time_limit_reached'
        }
        
        # Get status string
        if isinstance(results.info.status, str):
            osqp_status_str = results.info.status
        else:
            osqp_status_str = status_str_map.get(status_val, f'unknown_{status_val}')
        
        debug_info = {
            'predicted_states': X_pred,
            'control_sequence': U_pred,
            'cost': cost,
            'solver_time': solve_time,
            'solver_status': status,
            'solver_iterations': results.info.iter,
            'osqp_status': osqp_status_str,
            'osqp_status_val': status_val,
            'osqp_pri_res': getattr(results.info, 'pri_res', None),
            'osqp_dua_res': getattr(results.info, 'dua_res', None)
        }
        
        return u0, debug_info