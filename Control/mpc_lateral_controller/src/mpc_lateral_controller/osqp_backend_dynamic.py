#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
OSQP Backend for Dynamic MPC Controller

Direct OSQP implementation for dynamic bicycle model MPC.
Eliminates CVXPY overhead for real-time performance.
"""

import numpy as np
import osqp
import scipy.sparse as sparse
from typing import Dict, Tuple, Optional, List
import time


class LateralOsqpSolver:
    """Direct OSQP solver for dynamic MPC without CVXPY overhead."""
    
    def __init__(self, nx: int, nu: int, Np: int, Nc: int,
                 Q: np.ndarray, R: float, R_delta: float, P_term: np.ndarray,
                 delta_limits: List[float], delta_rate_max: float, Ts: float):
        """
        Initialize OSQP solver for dynamic MPC.
        
        Args:
            nx: State dimension (4 for dynamic: e_y, e_y_dot, e_psi, e_psi_dot)
            nu: Control dimension (1: steering angle)
            Np: Prediction horizon
            Nc: Control horizon
            Q: State cost matrix (4x4)
            R: Control cost scalar
            R_delta: Control rate cost scalar
            P_term: Terminal cost matrix (4x4)
            delta_limits: [min, max] steering limits
            delta_rate_max: Maximum steering rate
            Ts: Sampling time
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
        
        # Variable dimensions
        self.n_vars = nx * (Np + 1) + nu * Nc  # Total decision variables
        self.n_states = nx * (Np + 1)  # Total state variables
        self.n_controls = nu * Nc  # Total control variables
        
        # Constraint dimensions
        self.n_eq_init = nx  # Initial state equality
        self.n_eq_dyn = nx * Np  # Dynamics equality
        self.n_ineq_u = 2 * Nc  # Control box constraints
        self.n_ineq_du = 2 * Nc  # Control rate constraints
        self.n_constraints = self.n_eq_init + self.n_eq_dyn + self.n_ineq_u + self.n_ineq_du
        
        # Build constant matrices (initial with unit scales)
        self._build_constant_matrices()
        
        # Setup OSQP solver
        self._setup_solver()
        
        # Warm start storage
        self.last_solution = None
        
    def _x_index(self, k: int) -> slice:
        """Get state variable indices for time step k."""
        start = k * self.nx
        return slice(start, start + self.nx)
    
    def _u_index(self, k: int) -> int:
        """Get control variable index for time step k."""
        return self.n_states + k * self.nu
    
    def _build_constant_matrices(self):
        """Build constant cost matrix P and constraint pattern."""
        
        # ========== Cost Matrix P (with time-varying weight support) ==========
        # P is block diagonal: [P_X, P_U]
        # IMPORTANT: OSQP minimizes (1/2) z'Pz + q'z, so we need to multiply by 2

        # Default scales (per stage and terminal)
        self.q_scales_default = np.ones(self.Np)
        self.p_scale_default = 1.0

        # State cost block P_X with defaults
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
        
        # Combine into full P matrix
        self.P = sparse.block_diag([P_X, P_U], format='csc')

        # Cache indices for fast updates of state block
        self.P_state_nnz = P_X.nnz
        self.P_control_nnz = P_U.nnz
        self._P_state_indices = np.arange(self.P_state_nnz)
        self._P_control_indices = np.arange(self.P_state_nnz, self.P_state_nnz + self.P_control_nnz)
        
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
        
        # Create sparse matrix pattern
        data = np.ones(len(rows))  # Placeholder values
        self.A_pattern = sparse.csc_matrix((data, (rows, cols)), 
                                          shape=(self.n_constraints, self.n_vars))
        
        # Store pattern info
        self.A_rows = rows
        self.A_cols = cols
        self.n_A_elements = len(rows)
        
    def _setup_solver(self):
        """Setup OSQP solver with initial problem."""
        
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
            max_iter=400,
            eps_abs=1e-3,
            eps_rel=1e-3,
            polish=False,
            alpha=1.6  # Relaxation parameter
        )
        
    def update_and_solve(self, x0: np.ndarray, A_seq: List[np.ndarray], 
                        B_seq: List[np.ndarray], E_seq: List[np.ndarray],
                        kappa_seq: np.ndarray, u_prev: float,
                        max_iter: Optional[int] = None,
                        eps_abs: Optional[float] = None,
                        eps_rel: Optional[float] = None,
                        q_scales: Optional[np.ndarray] = None,
                        p_scale_end: Optional[float] = None) -> Tuple[float, Dict]:
        """
        Update problem data and solve.
        
        Args:
            x0: Initial state [e_y, e_y_dot, e_psi, e_psi_dot]
            A_seq: List of A matrices for each time step (4x4)
            B_seq: List of B matrices for each time step (4x1)
            E_seq: List of E vectors for each time step (4,)
            kappa_seq: Curvature sequence
            u_prev: Previous control input
            max_iter: Optional override for max iterations
            eps_abs: Optional override for absolute tolerance
            eps_rel: Optional override for relative tolerance
            
        Returns:
            Tuple of (optimal_control, debug_info)
        """
        start_time = time.time()
        
        # ========== Update P for time-varying weights (if provided) ==========
        if q_scales is not None or p_scale_end is not None:
            if q_scales is None:
                q_scales = self.q_scales_default
            if p_scale_end is None:
                p_scale_end = self.p_scale_default

            P_X_blocks = []
            for k in range(self.Np):
                P_X_blocks.append(2 * (float(q_scales[k]) * self.Q))
            P_X_blocks.append(2 * (float(p_scale_end) * self.P_term))
            P_X = sparse.block_diag(P_X_blocks, format='csc')

            P_new_data = self.P.data.copy()
            P_new_data[self._P_state_indices] = P_X.data
            self.solver.update(Px=P_new_data)

        # ========== Update q vector ==========
        # Only control part has non-zero q from rate penalty
        self.q[:self.n_states] = 0  # State part
        
        # Control part: q_U = 2 * R_delta * T^T @ t_0 * u_prev
        # Where t_0 = [-1, 0, 0, ..., 0]
        # Note: T^T @ t_0 = [-1, 0, 0, ...] so we need negative sign
        q_U = np.zeros(self.Nc)
        q_U[0] = -2 * self.R_delta * u_prev  # Negative for correct gradient
        self.q[self.n_states:] = q_U
        
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
        
        # Warm start with previous solution if available
        if self.last_solution is not None:
            self.solver.warm_start(x=self.last_solution)
        
        # Solve
        results = self.solver.solve()
        
        # Get numeric status value (for version compatibility)
        if hasattr(results.info, 'status_val'):
            status_val = results.info.status_val
        else:
            # Fallback for older versions
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
            status_str = status_map.get(status_val, f'unknown_{status_val}')
            
            u0 = u_prev  # Fallback to previous control
            X_pred = None
            U_pred = None
            status = f'failed_{status_str}'
            cost = np.inf
        
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