#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Debug curvature handling in OSQP backend.
"""

import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'src'))

import numpy as np
from mpc_lateral_controller.osqp_backend_kinematic import KinematicOsqpSolver
from mpc_lateral_controller.kinematic_mpc_core import KinematicMpcCore


def debug_curvature_in_osqp():
    """Debug how curvature is handled in OSQP backend."""
    
    # Simple parameters
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
    
    # Model parameters
    v_x = 15.0
    L = 2.7
    
    # Discrete matrices
    A_d = np.array([
        [1.0, v_x * Ts],
        [0.0, 1.0]
    ])
    
    B_d = np.array([
        [0.0],
        [v_x / L * Ts]
    ])
    
    E_d = np.array([0.0, -v_x * Ts])
    
    print("="*60)
    print("CURVATURE DEBUGGING")
    print("="*60)
    print(f"\nModel matrices:")
    print(f"A_d:\n{A_d}")
    print(f"B_d:\n{B_d}")
    print(f"E_d: {E_d}")
    
    # Test cases
    test_cases = [
        {
            'name': 'Zero curvature',
            'kappa': np.zeros(Np)
        },
        {
            'name': 'Constant curvature 0.01',
            'kappa': np.ones(Np) * 0.01
        }
    ]
    
    # Create solver
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
    
    for test in test_cases:
        print(f"\n{'='*40}")
        print(f"Test: {test['name']}")
        print(f"{'='*40}")
        print(f"Curvatures: {test['kappa']}")
        
        # Calculate E * kappa for each step
        print(f"\nDisturbance terms (E * kappa):")
        for k in range(Np):
            dist = E_d * test['kappa'][k]
            print(f"  Step {k}: {dist}")
        
        # Initial state
        x0 = np.array([0.5, 0.1])
        
        # Solve
        A_seq = [A_d] * Np
        B_seq = [B_d] * Np
        E_seq = [E_d] * Np
        
        u0, debug = solver.update_and_solve(
            x0=x0,
            A_seq=A_seq,
            B_seq=B_seq,
            E_seq=E_seq,
            kappa_seq=test['kappa'],
            u_prev=0.0
        )
        
        print(f"\nSolution:")
        print(f"  u0 = {u0:.6f}")
        print(f"  Status: {debug['solver_status']}")
        print(f"  Cost: {debug['cost']:.3f}")
        
        if debug['predicted_states'] is not None:
            print(f"\nPredicted states:")
            X = debug['predicted_states']
            U = debug['control_sequence']
            
            for k in range(Np + 1):
                if k < Np:
                    # Check dynamics manually
                    x_k = X[:, k]
                    u_k = U[0, min(k, Nc-1)]
                    x_next_pred = A_d @ x_k + B_d.flatten() * u_k + E_d * test['kappa'][k]
                    x_next_actual = X[:, k+1]
                    
                    print(f"  k={k}: x={X[:,k]}, u={u_k:.4f}")
                    print(f"        Next predicted: {x_next_pred}")
                    print(f"        Next actual:    {x_next_actual}")
                    print(f"        Dynamics error: {np.linalg.norm(x_next_pred - x_next_actual):.6f}")
                else:
                    print(f"  k={k}: x={X[:,k]} (terminal)")
    
    # Now compare with KinematicMpcCore
    print("\n" + "="*60)
    print("COMPARISON WITH KinematicMpcCore")
    print("="*60)
    
    model_params = {'L': L}
    control_params = {
        'Np': Np,
        'Nc': Nc,
        'Ts': Ts,
        'Q_kinematic': [10.0, 1.0],
        'R': R,
        'R_delta': R_delta,
        'P_kinematic': [20.0, 2.0],
        'delta_limits': [-0.5, 0.5],
        'delta_rate_max': 0.3,
        'solver_backend': 'osqp'
    }
    
    mpc = KinematicMpcCore(model_params, control_params)
    
    for test in test_cases:
        print(f"\n{test['name']}:")
        u, debug = mpc.solve(
            current_state=np.array([0.5, 0.1]),
            current_speed=v_x,
            reference_path={'curvatures': test['kappa']}
        )
        print(f"  Control: {u:.6f}")
        print(f"  Cost: {debug['cost']:.3f}")


if __name__ == '__main__':
    debug_curvature_in_osqp()