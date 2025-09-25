#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Debug script for OSQP backend.
"""

import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'src'))

import numpy as np
from mpc_lateral_controller.osqp_backend_kinematic import KinematicOsqpSolver


def debug_osqp():
    """Debug OSQP solver directly."""
    
    # Simple parameters
    nx = 2
    nu = 1
    Np = 10
    Nc = 5
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
    
    print(f"Solver created. Problem size: {solver.n_vars} variables, {solver.n_constraints} constraints")
    print(f"P shape: {solver.P.shape}, nnz: {solver.P.nnz}")
    print(f"A shape: {solver.A_pattern.shape}, nnz: {solver.A_pattern.nnz}")
    
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
    
    print("\nSolving with OSQP...")
    u0, debug = solver.update_and_solve(
        x0=x0,
        A_seq=A_seq,
        B_seq=B_seq,
        E_seq=E_seq,
        kappa_seq=kappa_seq,
        u_prev=u_prev
    )
    
    print(f"\nResults:")
    print(f"  Optimal control: {u0:.6f}")
    print(f"  Status: {debug['solver_status']}")
    print(f"  OSQP status: {debug['osqp_status']}")
    print(f"  Cost: {debug['cost']:.6f}")
    print(f"  Solver time: {debug['solver_time']:.2f} ms")
    print(f"  Iterations: {debug['solver_iterations']}")
    
    if debug['predicted_states'] is not None:
        print(f"\nPredicted trajectory:")
        for k in range(min(5, Np+1)):
            print(f"  Step {k}: e_y={debug['predicted_states'][0,k]:.4f}, e_psi={debug['predicted_states'][1,k]:.4f}")
    
    if debug['control_sequence'] is not None:
        print(f"\nControl sequence:")
        for k in range(min(5, Nc)):
            print(f"  u[{k}] = {debug['control_sequence'][0,k]:.4f}")
    
    # Verify constraints
    print(f"\nConstraint checks:")
    print(f"  Control in bounds: {delta_limits[0]} <= {u0:.4f} <= {delta_limits[1]} : {delta_limits[0] <= u0 <= delta_limits[1]}")
    du = u0 - u_prev
    print(f"  Rate in bounds: |{du:.4f}| <= {delta_rate_max} : {abs(du) <= delta_rate_max}")
    
    return u0, debug


if __name__ == '__main__':
    debug_osqp()