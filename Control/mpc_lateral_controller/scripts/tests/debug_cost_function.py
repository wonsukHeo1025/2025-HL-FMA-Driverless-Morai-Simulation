#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Debug cost function formulation between CVXPY and OSQP.
"""

import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'src'))

import numpy as np
from mpc_lateral_controller.kinematic_mpc_core import KinematicMpcCore
from mpc_lateral_controller.osqp_backend_kinematic import KinematicOsqpSolver


def debug_cost_formulation():
    """Debug the cost function formulation."""
    
    # Simple test parameters
    nx = 2
    nu = 1
    Np = 3
    Nc = 2
    Ts = 0.1
    
    # Weights
    Q = np.diag([10.0, 1.0])
    R = 0.1
    R_delta = 1.0
    P_term = np.diag([20.0, 2.0])
    
    print("="*60)
    print("COST FUNCTION DEBUGGING")
    print("="*60)
    print(f"\nWeights:")
    print(f"Q = diag({np.diag(Q)})")
    print(f"R = {R}")
    print(f"R_delta = {R_delta}")
    print(f"P_term = diag({np.diag(P_term)})")
    
    # Create OSQP solver
    solver = KinematicOsqpSolver(
        nx=nx,
        nu=nu,
        Np=Np,
        Nc=Nc,
        Q=Q,
        R=R,
        R_delta=R_delta,
        P_term=P_term,
        delta_limits=[-0.5, 0.5],
        delta_rate_max=0.3,
        Ts=Ts
    )
    
    # Check P matrix
    P = solver.P.toarray()
    print(f"\nOSQP P matrix shape: {P.shape}")
    print(f"P matrix (cost):")
    
    # Print state cost part
    print("\nState cost block (first 8x8):")
    state_block = P[:nx*(Np+1), :nx*(Np+1)]
    print(state_block)
    
    # Print control cost part
    print(f"\nControl cost block (last {Nc}x{Nc}):")
    control_block = P[nx*(Np+1):, nx*(Np+1):]
    print(control_block)
    
    # Check if the factor of 2 is correctly applied
    print("\n" + "="*60)
    print("COST MATRIX ANALYSIS:")
    print("="*60)
    
    # Expected state cost diagonal
    expected_state_diag = []
    for i in range(Np):
        expected_state_diag.extend(np.diag(Q))
    expected_state_diag.extend(np.diag(P_term))
    
    actual_state_diag = np.diag(state_block)
    
    print(f"\nExpected state diagonal: {expected_state_diag}")
    print(f"Actual state diagonal: {actual_state_diag}")
    print(f"State diagonal match: {np.allclose(expected_state_diag, actual_state_diag)}")
    
    # Check control cost
    print(f"\nControl cost matrix:\n{control_block}")
    
    # Build expected control cost
    T = np.zeros((Nc, Nc))
    for i in range(Nc):
        T[i, i] = 1
        if i > 0:
            T[i, i-1] = -1
    
    expected_control = 2 * (R * np.eye(Nc) + R_delta * T.T @ T)
    print(f"\nExpected control cost (with factor 2):\n{expected_control}")
    print(f"Control cost match: {np.allclose(control_block, expected_control)}")
    
    # Test with a simple problem
    print("\n" + "="*60)
    print("SIMPLE PROBLEM TEST:")
    print("="*60)
    
    x0 = np.array([0.1, 0.05])
    v_x = 10.0
    L = 2.7
    
    A_d = np.array([[1.0, v_x * Ts], [0.0, 1.0]])
    B_d = np.array([[0.0], [v_x / L * Ts]])
    E_d = np.array([0.0, -v_x * Ts])
    
    A_seq = [A_d] * Np
    B_seq = [B_d] * Np
    E_seq = [E_d] * Np
    kappa_seq = np.zeros(Np)
    u_prev = 0.0
    
    u0, debug = solver.update_and_solve(
        x0=x0,
        A_seq=A_seq,
        B_seq=B_seq,
        E_seq=E_seq,
        kappa_seq=kappa_seq,
        u_prev=u_prev
    )
    
    print(f"\nSolution: u0 = {u0:.6f}")
    print(f"Status: {debug['solver_status']}")
    print(f"Cost: {debug['cost']:.3f}")
    
    if debug['control_sequence'] is not None:
        print(f"Control sequence: {debug['control_sequence'].flatten()}")
    
    # Manually calculate expected cost for zero reference
    if debug['predicted_states'] is not None:
        X = debug['predicted_states']
        U = debug['control_sequence']
        
        manual_cost = 0
        # State costs
        for k in range(Np):
            x_k = X[:, k]
            manual_cost += x_k.T @ Q @ x_k
        # Terminal cost
        x_N = X[:, Np]
        manual_cost += x_N.T @ P_term @ x_N
        
        # Control costs
        for k in range(Nc):
            u_k = U[0, k]
            manual_cost += R * u_k**2
            
            if k == 0:
                du = u_k - u_prev
            else:
                du = u_k - U[0, k-1]
            manual_cost += R_delta * du**2
        
        print(f"\nManually calculated cost: {manual_cost:.3f}")
        print(f"OSQP reported cost: {debug['cost']:.3f}")
        print(f"Cost ratio: {debug['cost'] / manual_cost:.3f}")


if __name__ == '__main__':
    debug_cost_formulation()