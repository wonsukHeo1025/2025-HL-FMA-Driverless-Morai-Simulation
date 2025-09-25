#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Detailed debug script for OSQP backend to check constraint setup.
"""

import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'src'))

import numpy as np
import scipy.sparse as sparse
from mpc_lateral_controller.osqp_backend_kinematic import KinematicOsqpSolver


def check_constraint_feasibility():
    """Check if the constraints are feasible."""
    
    # Simple parameters
    nx = 2
    nu = 1
    Np = 3  # Small for debugging
    Nc = 2  # Small for debugging
    Ts = 0.1
    
    # Weights
    Q = np.diag([10.0, 1.0])
    R = 0.1
    R_delta = 1.0
    P_term = np.diag([20.0, 2.0])
    
    # Constraints
    delta_limits = [-0.5, 0.5]
    delta_rate_max = 0.3
    
    # Create solver
    print("Creating OSQP solver...")
    solver = KinematicOsqpSolver(
        nx=nx,
        nu=nu,
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
    
    # Test case
    x0 = np.array([0.1, 0.05])
    v_x = 10.0
    L = 2.7
    
    # Simple kinematic model matrices
    A_d = np.array([
        [1.0, v_x * Ts],
        [0.0, 1.0]
    ])
    
    B_d = np.array([
        [0.0],
        [v_x / L * Ts]
    ])
    
    E_d = np.array([0.0, -v_x * Ts])
    
    # Constant matrices for all time steps
    A_seq = [A_d] * Np
    B_seq = [B_d] * Np
    E_seq = [E_d] * Np
    
    # Zero curvature
    kappa_seq = np.zeros(Np)
    
    # Previous control
    u_prev = 0.0
    
    # Manually build the constraint vectors to understand the issue
    n_vars = nx * (Np + 1) + nu * Nc
    n_constraints = nx + nx * Np + 2 * Nc + 2 * Nc
    
    print(f"\nProblem dimensions:")
    print(f"  Variables: {n_vars} (states: {nx*(Np+1)}, controls: {nu*Nc})")
    print(f"  Constraints: {n_constraints}")
    print(f"    - Initial state: {nx}")
    print(f"    - Dynamics: {nx * Np}")
    print(f"    - Control box: {2 * Nc}")
    print(f"    - Control rate: {2 * Nc}")
    
    # Build A matrix values manually
    A_data = np.zeros(solver.n_A_elements)
    l = np.zeros(n_constraints)
    u = np.zeros(n_constraints)
    
    row = 0
    
    # 1. Initial state constraint
    print(f"\n1. Initial state constraints (rows {row}-{row+nx-1}):")
    for i in range(nx):
        l[row] = x0[i]
        u[row] = x0[i]
        print(f"   Row {row}: x0[{i}] = {x0[i]:.4f}")
        row += 1
    
    # 2. Dynamics constraints
    print(f"\n2. Dynamics constraints (rows {row}-{row+nx*Np-1}):")
    for k in range(Np):
        rhs = E_seq[k] * kappa_seq[k]
        for i in range(nx):
            l[row] = rhs[i]
            u[row] = rhs[i]
            print(f"   Row {row}: x_{k+1}[{i}] - A*x_{k}[{i}] - B*u_{min(k,Nc-1)} = {rhs[i]:.4f}")
            row += 1
    
    # 3. Control box constraints
    print(f"\n3. Control box constraints (rows {row}-{row+2*Nc-1}):")
    for k in range(Nc):
        # Lower bound
        l[row] = delta_limits[0]
        u[row] = np.inf
        print(f"   Row {row}: u[{k}] >= {delta_limits[0]:.2f}")
        row += 1
        
        # Upper bound
        l[row] = -np.inf
        u[row] = delta_limits[1]
        print(f"   Row {row}: u[{k}] <= {delta_limits[1]:.2f}")
        row += 1
    
    # 4. Control rate constraints
    print(f"\n4. Control rate constraints (rows {row}-{row+2*Nc-1}):")
    for k in range(Nc):
        if k == 0:
            # u_0 - u_prev
            l[row] = -delta_rate_max - u_prev
            u[row] = np.inf
            print(f"   Row {row}: u[0] - u_prev >= {-delta_rate_max - u_prev:.2f}")
            row += 1
            
            l[row] = -np.inf
            u[row] = delta_rate_max - u_prev
            print(f"   Row {row}: u[0] - u_prev <= {delta_rate_max - u_prev:.2f}")
            row += 1
        else:
            # u_k - u_{k-1}
            l[row] = -delta_rate_max
            u[row] = np.inf
            print(f"   Row {row}: u[{k}] - u[{k-1}] >= {-delta_rate_max:.2f}")
            row += 1
            
            l[row] = -np.inf
            u[row] = delta_rate_max
            print(f"   Row {row}: u[{k}] - u[{k-1}] <= {delta_rate_max:.2f}")
            row += 1
    
    # Try to find a feasible solution manually
    print("\n" + "="*60)
    print("Manual feasibility check:")
    print("="*60)
    
    # Simple feasible solution: all states follow dynamics, all controls are zero
    z_test = np.zeros(n_vars)
    
    # Set initial state
    z_test[0:nx] = x0
    
    # Propagate dynamics with zero control
    for k in range(Np):
        x_k = z_test[k*nx:(k+1)*nx]
        u_k = 0.0  # Zero control
        x_next = A_d @ x_k + B_d.flatten() * u_k + E_d * kappa_seq[k]
        z_test[(k+1)*nx:(k+2)*nx] = x_next
        print(f"Step {k}: x={x_k}, u={u_k:.3f}, x_next={x_next}")
    
    print(f"\nTest solution: {z_test}")
    
    # Check if this solution is feasible
    # Build actual A matrix for checking
    print("\n" + "="*60)
    print("Building and checking actual constraint matrix...")
    print("="*60)
    
    # Call the solver to build its matrices
    u0, debug = solver.update_and_solve(
        x0=x0,
        A_seq=A_seq,
        B_seq=B_seq,
        E_seq=E_seq,
        kappa_seq=kappa_seq,
        u_prev=u_prev
    )
    
    print(f"\nSolver result: u0={u0:.6f}, status={debug['solver_status']}")
    
    # Check the actual constraint values
    A_dense = solver.A_pattern.toarray()
    print(f"\nA matrix shape: {A_dense.shape}")
    print(f"A matrix rank: {np.linalg.matrix_rank(A_dense)}")
    
    # Check if constraints are consistent
    for i in range(n_constraints):
        if l[i] > u[i]:
            print(f"ERROR: Constraint {i} is infeasible: {l[i]:.4f} > {u[i]:.4f}")
    
    return solver


if __name__ == '__main__':
    check_constraint_feasibility()