#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Compare CVXPY and OSQP solutions in detail.
"""

import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'src'))

import numpy as np
from mpc_lateral_controller.kinematic_mpc_core import KinematicMpcCore


def compare_solutions():
    """Compare CVXPY and OSQP solutions for the same problem."""
    
    # Model parameters
    model_params = {
        'L': 2.7  # Wheelbase
    }
    
    # Control parameters
    base_control_params = {
        'Np': 5,  # Small for detailed analysis
        'Nc': 3,   # Small for detailed analysis
        'Ts': 0.1,  # Sampling time
        'Q_kinematic': [10.0, 1.0],  # [e_y, e_psi] weights
        'R': 0.1,  # Control cost
        'R_delta': 1.0,  # Control rate cost
        'P_kinematic': [20.0, 2.0],  # Terminal cost
        'delta_limits': [-0.5, 0.5],  # Steering limits [rad]
        'delta_rate_max': 0.3,  # Maximum steering rate [rad/step]
        'osqp_max_iter': 1000,
        'osqp_eps_abs': 1e-6,
        'osqp_eps_rel': 1e-6
    }
    
    # Test case
    state = np.array([0.5, 0.1])  # Lateral error, heading error
    speed = 10.0
    curvatures = np.zeros(base_control_params['Np'])
    
    print("="*60)
    print("DETAILED COMPARISON: CVXPY vs OSQP")
    print("="*60)
    print(f"\nInitial state: e_y={state[0]:.3f}, e_psi={state[1]:.3f}")
    print(f"Speed: {speed:.1f} m/s")
    print(f"Curvatures: {curvatures}")
    
    # Create CVXPY controller
    control_params_cvxpy = base_control_params.copy()
    control_params_cvxpy['solver_backend'] = 'cvxpy'
    mpc_cvxpy = KinematicMpcCore(model_params, control_params_cvxpy)
    
    # Create OSQP controller
    control_params_osqp = base_control_params.copy()
    control_params_osqp['solver_backend'] = 'osqp'
    mpc_osqp = KinematicMpcCore(model_params, control_params_osqp)
    
    # Solve with CVXPY
    print("\n" + "-"*40)
    print("CVXPY Solution:")
    print("-"*40)
    u_cvxpy, debug_cvxpy = mpc_cvxpy.solve(
        current_state=state,
        current_speed=speed,
        reference_path={'curvatures': curvatures}
    )
    
    print(f"Control: {u_cvxpy:.6f}")
    print(f"Status: {debug_cvxpy['solver_status']}")
    print(f"Cost: {debug_cvxpy['cost']:.3f}")
    print(f"Solve time: {debug_cvxpy['solver_time']:.2f} ms")
    
    if debug_cvxpy['predicted_states'] is not None:
        print("\nPredicted states (CVXPY):")
        for k in range(base_control_params['Np'] + 1):
            print(f"  k={k}: e_y={debug_cvxpy['predicted_states'][0,k]:>7.4f}, "
                  f"e_psi={debug_cvxpy['predicted_states'][1,k]:>7.4f}")
    
    if debug_cvxpy['control_sequence'] is not None:
        print("\nControl sequence (CVXPY):")
        for k in range(base_control_params['Nc']):
            print(f"  u[{k}] = {debug_cvxpy['control_sequence'][0,k]:>7.4f}")
    
    # Solve with OSQP
    print("\n" + "-"*40)
    print("OSQP Solution:")
    print("-"*40)
    u_osqp, debug_osqp = mpc_osqp.solve(
        current_state=state,
        current_speed=speed,
        reference_path={'curvatures': curvatures}
    )
    
    print(f"Control: {u_osqp:.6f}")
    print(f"Status: {debug_osqp['solver_status']}")
    print(f"OSQP status: {debug_osqp.get('osqp_status', 'N/A')}")
    print(f"Cost: {debug_osqp['cost']:.3f}")
    print(f"Solve time: {debug_osqp['solver_time']:.2f} ms")
    print(f"Iterations: {debug_osqp.get('solver_iterations', 'N/A')}")
    
    if debug_osqp['predicted_states'] is not None:
        print("\nPredicted states (OSQP):")
        for k in range(base_control_params['Np'] + 1):
            print(f"  k={k}: e_y={debug_osqp['predicted_states'][0,k]:>7.4f}, "
                  f"e_psi={debug_osqp['predicted_states'][1,k]:>7.4f}")
    
    if debug_osqp['control_sequence'] is not None:
        print("\nControl sequence (OSQP):")
        for k in range(base_control_params['Nc']):
            print(f"  u[{k}] = {debug_osqp['control_sequence'][0,k]:>7.4f}")
    
    # Compare
    print("\n" + "="*60)
    print("COMPARISON RESULTS:")
    print("="*60)
    diff = abs(u_cvxpy - u_osqp)
    print(f"Control difference: {diff:.6f}")
    print(f"Match (< 1e-4): {'✓' if diff < 1e-4 else '✗'}")
    
    if debug_cvxpy['cost'] > 0 and debug_osqp['cost'] > 0:
        cost_diff = abs(debug_cvxpy['cost'] - debug_osqp['cost'])
        print(f"Cost difference: {cost_diff:.3f}")
        print(f"Relative cost diff: {cost_diff/debug_cvxpy['cost']*100:.1f}%")
    
    print(f"Speedup: {debug_cvxpy['solver_time']/debug_osqp['solver_time']:.1f}x")
    
    # Additional debugging
    print("\n" + "="*60)
    print("DEBUGGING INFO:")
    print("="*60)
    
    # Check model matrices
    A_d, B_d, E_d = mpc_cvxpy._get_discrete_matrices(speed)
    print(f"\nDiscrete model matrices:")
    print(f"A_d:\n{A_d}")
    print(f"B_d:\n{B_d}")
    print(f"E_d: {E_d}")
    
    # Check if OSQP problem is feasible
    if debug_osqp['solver_status'].startswith('failed'):
        print(f"\n⚠️ OSQP failed with: {debug_osqp['solver_status']}")
        if 'osqp_pri_res' in debug_osqp:
            print(f"  Primal residual: {debug_osqp['osqp_pri_res']}")
        if 'osqp_dua_res' in debug_osqp:
            print(f"  Dual residual: {debug_osqp['osqp_dua_res']}")


if __name__ == '__main__':
    compare_solutions()