#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Test curvature handling in OSQP backend.
"""

import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'src'))

import numpy as np
from mpc_lateral_controller.kinematic_mpc_core import KinematicMpcCore


def test_curvature():
    """Test with and without curvature."""
    
    # Model parameters
    model_params = {'L': 2.7}
    
    # Control parameters
    control_params = {
        'Np': 10,
        'Nc': 5,
        'Ts': 0.1,
        'Q_kinematic': [10.0, 1.0],
        'R': 0.1,
        'R_delta': 1.0,
        'P_kinematic': [20.0, 2.0],
        'delta_limits': [-0.5, 0.5],
        'delta_rate_max': 0.3,
        'osqp_max_iter': 1000,
        'osqp_eps_abs': 1e-6,
        'osqp_eps_rel': 1e-6
    }
    
    # Test cases
    test_cases = [
        {
            'name': 'Zero curvature',
            'state': np.array([0.5, 0.1]),
            'curvatures': np.zeros(10)
        },
        {
            'name': 'Constant curvature',
            'state': np.array([0.5, 0.1]),
            'curvatures': np.ones(10) * 0.01
        },
        {
            'name': 'Varying curvature',
            'state': np.array([0.5, 0.1]),
            'curvatures': np.linspace(0, 0.02, 10)
        }
    ]
    
    for backend in ['cvxpy', 'osqp']:
        print(f"\n{'='*60}")
        print(f"Testing with {backend.upper()} backend")
        print('='*60)
        
        params = control_params.copy()
        params['solver_backend'] = backend
        mpc = KinematicMpcCore(model_params, params)
        
        for test in test_cases:
            u, debug = mpc.solve(
                current_state=test['state'],
                current_speed=15.0,
                reference_path={'curvatures': test['curvatures']}
            )
            
            print(f"\n{test['name']}:")
            print(f"  Control: {u:.6f}")
            print(f"  Cost: {debug['cost']:.3f}")
            print(f"  Status: {debug['solver_status']}")
            if debug['control_sequence'] is not None:
                print(f"  Control seq: {debug['control_sequence'].flatten()[:3]}")
    
    # Direct comparison
    print(f"\n{'='*60}")
    print("DIRECT COMPARISON")
    print('='*60)
    
    cvxpy_params = control_params.copy()
    cvxpy_params['solver_backend'] = 'cvxpy'
    mpc_cvxpy = KinematicMpcCore(model_params, cvxpy_params)
    
    osqp_params = control_params.copy()
    osqp_params['solver_backend'] = 'osqp'
    mpc_osqp = KinematicMpcCore(model_params, osqp_params)
    
    for test in test_cases:
        # Reset controllers to ensure clean state
        mpc_cvxpy.reset()
        mpc_osqp.reset()
        
        u_cvxpy, debug_cvxpy = mpc_cvxpy.solve(
            current_state=test['state'],
            current_speed=15.0,
            reference_path={'curvatures': test['curvatures']}
        )
        
        u_osqp, debug_osqp = mpc_osqp.solve(
            current_state=test['state'],
            current_speed=15.0,
            reference_path={'curvatures': test['curvatures']}
        )
        
        diff = abs(u_cvxpy - u_osqp)
        match = diff < 1e-4
        
        print(f"\n{test['name']}:")
        print(f"  CVXPY: {u_cvxpy:.6f}, Cost: {debug_cvxpy['cost']:.3f}")
        print(f"  OSQP:  {u_osqp:.6f}, Cost: {debug_osqp['cost']:.3f}")
        print(f"  Diff:  {diff:.6f} {'✓' if match else '✗'}")
        print(f"  Curvatures: {test['curvatures'][:3]}...")


if __name__ == '__main__':
    test_curvature()