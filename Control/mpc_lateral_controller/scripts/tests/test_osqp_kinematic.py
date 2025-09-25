#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Test script for OSQP backend numerical equivalence and performance.

Compares CVXPY and direct OSQP implementations for kinematic MPC.
"""

import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'src'))

import numpy as np
import time
from mpc_lateral_controller.kinematic_mpc_core import KinematicMpcCore
import matplotlib.pyplot as plt


def create_test_scenario():
    """Create test scenario with various conditions."""
    scenarios = []
    
    # Scenario 1: Straight line with small errors
    scenarios.append({
        'name': 'Straight Line',
        'state': np.array([0.1, 0.05]),  # Small lateral and heading error
        'speed': 10.0,  # m/s
        'curvatures': np.zeros(100)
    })
    
    # Scenario 2: Curved path
    scenarios.append({
        'name': 'Curved Path',
        'state': np.array([0.5, 0.1]),
        'speed': 15.0,
        'curvatures': np.ones(100) * 0.01  # Constant curvature
    })
    
    # Scenario 3: Large errors
    scenarios.append({
        'name': 'Large Errors',
        'state': np.array([1.0, 0.2]),
        'speed': 5.0,
        'curvatures': np.linspace(0, 0.02, 100)
    })
    
    # Scenario 4: S-curve
    scenarios.append({
        'name': 'S-Curve',
        'state': np.array([0.3, 0.15]),
        'speed': 20.0,
        'curvatures': np.sin(np.linspace(0, 2*np.pi, 100)) * 0.015
    })
    
    return scenarios


def test_numerical_equivalence():
    """Test numerical equivalence between CVXPY and OSQP backends."""
    print("\n" + "="*60)
    print("NUMERICAL EQUIVALENCE TEST")
    print("="*60)
    
    # Model parameters
    model_params = {
        'L': 2.7  # Wheelbase
    }
    
    # Control parameters for both backends
    base_control_params = {
        'Np': 10,  # Prediction horizon
        'Nc': 5,   # Control horizon
        'Ts': 0.1,  # Sampling time
        'Q_kinematic': [10.0, 1.0],  # [e_y, e_psi] weights
        'R': 0.1,  # Control cost
        'R_delta': 1.0,  # Control rate cost
        'P_kinematic': [20.0, 2.0],  # Terminal cost
        'delta_limits': [-0.5, 0.5],  # Steering limits [rad]
        'delta_rate_max': 0.3,  # Maximum steering rate [rad/step]
        'osqp_max_iter': 1000,
        'osqp_eps_abs': 1e-5,  # Tighter tolerance for equivalence test
        'osqp_eps_rel': 1e-5   # Tighter tolerance for equivalence test
    }
    
    # Create controllers
    control_params_cvxpy = base_control_params.copy()
    control_params_cvxpy['solver_backend'] = 'cvxpy'
    
    control_params_osqp = base_control_params.copy()
    control_params_osqp['solver_backend'] = 'osqp'
    
    mpc_cvxpy = KinematicMpcCore(model_params, control_params_cvxpy)
    mpc_osqp = KinematicMpcCore(model_params, control_params_osqp)
    
    # Test scenarios
    scenarios = create_test_scenario()
    
    print(f"\n{'Scenario':<20} {'CVXPY u0':<12} {'OSQP u0':<12} {'Diff':<12} {'Match':<8}")
    print("-" * 60)
    
    all_match = True
    for scenario in scenarios:
        # Reset controllers to ensure clean state
        mpc_cvxpy.reset()
        mpc_osqp.reset()
        # Solve with CVXPY
        u_cvxpy, debug_cvxpy = mpc_cvxpy.solve(
            current_state=scenario['state'],
            current_speed=scenario['speed'],
            reference_path={'curvatures': scenario['curvatures']}
        )
        
        # Solve with OSQP
        u_osqp, debug_osqp = mpc_osqp.solve(
            current_state=scenario['state'],
            current_speed=scenario['speed'],
            reference_path={'curvatures': scenario['curvatures']}
        )
        
        # Compare
        diff = abs(u_cvxpy - u_osqp)
        match = diff < 1e-4  # Tolerance
        all_match = all_match and match
        
        print(f"{scenario['name']:<20} {u_cvxpy:>11.6f} {u_osqp:>11.6f} {diff:>11.6f} {'✓' if match else '✗':<8}")
    
    print("\n" + "="*60)
    if all_match:
        print("✓ NUMERICAL EQUIVALENCE TEST PASSED")
    else:
        print("✗ NUMERICAL EQUIVALENCE TEST FAILED")
    print("="*60)
    
    return all_match


def test_performance():
    """Compare performance between CVXPY and OSQP backends."""
    print("\n" + "="*60)
    print("PERFORMANCE COMPARISON TEST")
    print("="*60)
    
    # Model parameters
    model_params = {
        'L': 2.7
    }
    
    # Test different horizon sizes
    horizon_configs = [
        {'Np': 10, 'Nc': 5, 'label': 'Small (Np=10, Nc=5)'},
        {'Np': 20, 'Nc': 10, 'label': 'Medium (Np=20, Nc=10)'},
        {'Np': 50, 'Nc': 20, 'label': 'Large (Np=50, Nc=20)'},
        {'Np': 100, 'Nc': 20, 'label': 'Very Large (Np=100, Nc=20)'}
    ]
    
    results = []
    
    for config in horizon_configs:
        print(f"\nTesting {config['label']}...")
        
        # Control parameters
        control_params_base = {
            'Np': config['Np'],
            'Nc': config['Nc'],
            'Ts': 0.1,
            'Q_kinematic': [10.0, 1.0],
            'R': 0.1,
            'R_delta': 1.0,
            'P_kinematic': [20.0, 2.0],
            'delta_limits': [-0.5, 0.5],
            'delta_rate_max': 0.3,
            'osqp_max_iter': 400,
            'osqp_eps_abs': 1e-3,
            'osqp_eps_rel': 1e-3
        }
        
        # Create controllers
        control_params_cvxpy = control_params_base.copy()
        control_params_cvxpy['solver_backend'] = 'cvxpy'
        
        control_params_osqp = control_params_base.copy()
        control_params_osqp['solver_backend'] = 'osqp'
        
        mpc_cvxpy = KinematicMpcCore(model_params, control_params_cvxpy)
        mpc_osqp = KinematicMpcCore(model_params, control_params_osqp)
        
        # Run multiple iterations for timing
        n_iterations = 50
        state = np.array([0.5, 0.1])
        speed = 10.0
        curvatures = np.sin(np.linspace(0, 2*np.pi, config['Np'])) * 0.01
        
        # Warm up
        for _ in range(5):
            mpc_cvxpy.solve(state, speed, {'curvatures': curvatures})
            mpc_osqp.solve(state, speed, {'curvatures': curvatures})
        
        # Time CVXPY
        cvxpy_times = []
        for _ in range(n_iterations):
            _, debug = mpc_cvxpy.solve(state, speed, {'curvatures': curvatures})
            cvxpy_times.append(debug['solver_time'])
        
        # Time OSQP
        osqp_times = []
        for _ in range(n_iterations):
            _, debug = mpc_osqp.solve(state, speed, {'curvatures': curvatures})
            osqp_times.append(debug['solver_time'])
        
        # Calculate statistics
        cvxpy_mean = np.mean(cvxpy_times)
        cvxpy_std = np.std(cvxpy_times)
        osqp_mean = np.mean(osqp_times)
        osqp_std = np.std(osqp_times)
        speedup = cvxpy_mean / osqp_mean
        
        results.append({
            'config': config['label'],
            'cvxpy_mean': cvxpy_mean,
            'cvxpy_std': cvxpy_std,
            'osqp_mean': osqp_mean,
            'osqp_std': osqp_std,
            'speedup': speedup
        })
        
        print(f"  CVXPY: {cvxpy_mean:.2f} ± {cvxpy_std:.2f} ms")
        print(f"  OSQP:  {osqp_mean:.2f} ± {osqp_std:.2f} ms")
        print(f"  Speedup: {speedup:.2f}x")
    
    # Summary table
    print("\n" + "="*60)
    print("PERFORMANCE SUMMARY")
    print("="*60)
    print(f"\n{'Configuration':<25} {'CVXPY [ms]':<15} {'OSQP [ms]':<15} {'Speedup':<10}")
    print("-" * 65)
    
    for result in results:
        print(f"{result['config']:<25} "
              f"{result['cvxpy_mean']:.2f}±{result['cvxpy_std']:.2f}  "
              f"{result['osqp_mean']:.2f}±{result['osqp_std']:.2f}  "
              f"{result['speedup']:.2f}x")
    
    # Calculate average speedup
    avg_speedup = np.mean([r['speedup'] for r in results])
    print(f"\nAverage Speedup: {avg_speedup:.2f}x")
    
    # Check if OSQP meets real-time requirements
    print("\n" + "="*60)
    print("REAL-TIME FEASIBILITY (50 Hz = 20 ms)")
    print("="*60)
    
    for result in results:
        if result['osqp_mean'] < 20:
            print(f"✓ {result['config']}: {result['osqp_mean']:.2f} ms < 20 ms")
        else:
            print(f"✗ {result['config']}: {result['osqp_mean']:.2f} ms > 20 ms")
    
    return results


def test_constraint_satisfaction():
    """Test that OSQP backend satisfies all constraints."""
    print("\n" + "="*60)
    print("CONSTRAINT SATISFACTION TEST")
    print("="*60)
    
    # Model parameters
    model_params = {
        'L': 2.7
    }
    
    # Control parameters
    control_params = {
        'Np': 20,
        'Nc': 10,
        'Ts': 0.1,
        'Q_kinematic': [10.0, 1.0],
        'R': 0.1,
        'R_delta': 1.0,
        'P_kinematic': [20.0, 2.0],
        'delta_limits': [-0.3, 0.3],
        'delta_rate_max': 0.1,
        'solver_backend': 'osqp',
        'osqp_max_iter': 400,
        'osqp_eps_abs': 1e-4,
        'osqp_eps_rel': 1e-4
    }
    
    mpc = KinematicMpcCore(model_params, control_params)
    
    # Test with extreme initial conditions
    test_cases = [
        {'name': 'Large lateral error', 'state': np.array([2.0, 0.0])},
        {'name': 'Large heading error', 'state': np.array([0.0, 0.5])},
        {'name': 'Both large errors', 'state': np.array([1.5, 0.3])}
    ]
    
    all_satisfied = True
    
    for test in test_cases:
        print(f"\nTesting: {test['name']}")
        
        # Simulate for multiple steps
        state = test['state'].copy()
        speed = 10.0
        u_prev = 0.0
        
        violations = []
        
        for step in range(10):
            u, debug = mpc.solve(state, speed, {'curvatures': np.zeros(control_params['Np'])})
            
            # Check control limits
            if u < control_params['delta_limits'][0] or u > control_params['delta_limits'][1]:
                violations.append(f"Step {step}: Control limit violated: {u:.4f}")
                all_satisfied = False
            
            # Check rate limits
            du = u - u_prev
            if abs(du) > control_params['delta_rate_max'] * 1.01:  # Small tolerance
                violations.append(f"Step {step}: Rate limit violated: {du:.4f}")
                all_satisfied = False
            
            # Simple state update (for testing)
            state[0] += control_params['Ts'] * speed * state[1]  # Lateral error update
            state[1] += control_params['Ts'] * speed / model_params['L'] * u  # Heading error update
            state *= 0.9  # Decay for stability
            
            u_prev = u
        
        if violations:
            print("  ✗ Constraint violations found:")
            for v in violations:
                print(f"    - {v}")
        else:
            print("  ✓ All constraints satisfied")
    
    print("\n" + "="*60)
    if all_satisfied:
        print("✓ CONSTRAINT SATISFACTION TEST PASSED")
    else:
        print("✗ CONSTRAINT SATISFACTION TEST FAILED")
    print("="*60)
    
    return all_satisfied


def main():
    """Run all tests."""
    print("\n" + "="*60)
    print("OSQP BACKEND TEST SUITE FOR KINEMATIC MPC")
    print("="*60)
    
    # Run tests
    equiv_passed = test_numerical_equivalence()
    perf_results = test_performance()
    constraints_passed = test_constraint_satisfaction()
    
    # Final summary
    print("\n" + "="*60)
    print("FINAL TEST SUMMARY")
    print("="*60)
    
    if equiv_passed:
        print("✓ Numerical Equivalence: PASSED")
    else:
        print("✗ Numerical Equivalence: FAILED")
    
    print("✓ Performance: TESTED (see results above)")
    
    if constraints_passed:
        print("✓ Constraint Satisfaction: PASSED")
    else:
        print("✗ Constraint Satisfaction: FAILED")
    
    if equiv_passed and constraints_passed:
        print("\n🎉 ALL TESTS PASSED - OSQP backend is ready for use!")
    else:
        print("\n⚠️ Some tests failed - review results above")


if __name__ == '__main__':
    main()