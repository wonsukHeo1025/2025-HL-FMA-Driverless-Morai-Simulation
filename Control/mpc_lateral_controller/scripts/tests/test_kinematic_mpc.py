#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Test script for Kinematic MPC Core

Simple test to verify the kinematic bicycle model MPC implementation.
"""

import numpy as np
import sys
import os

# Add parent directory to path for imports
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.mpc_lateral_controller.kinematic_mpc_core import KinematicMpcCore


def test_straight_line():
    """Test MPC on straight line (zero curvature)."""
    print("\n=== Testing Straight Line Tracking ===")
    
    # Model parameters
    model_params = {
        'L': 3.0  # Wheelbase [m]
    }
    
    # Control parameters
    control_params = {
        'Np': 20,  # Prediction horizon
        'Nc': 5,   # Control horizon
        'Ts': 0.02,  # Sample time (50Hz)
        'Q_kinematic': [100, 50],  # [e_y, e_psi] weights
        'P_kinematic': [1000, 500],  # Terminal cost
        'R': 1.0,
        'R_delta': 10.0,
        'delta_limits': [-0.7, 0.7],
        'delta_rate_max': 0.05
    }
    
    # Initialize controller
    mpc = KinematicMpcCore(model_params, control_params)
    
    # Test case: Small lateral error, no heading error
    current_state = np.array([0.5, 0.0])  # [e_y=0.5m, e_psi=0]
    current_speed = 10.0  # m/s
    
    # Straight path (zero curvature)
    reference_path = {
        'curvatures': np.zeros(20)
    }
    
    # Solve MPC
    steering, debug_info = mpc.solve(current_state, current_speed, reference_path)
    
    print(f"Initial state: e_y={current_state[0]:.2f}m, e_psi={current_state[1]:.2f}rad")
    print(f"Vehicle speed: {current_speed:.1f} m/s")
    print(f"Optimal steering: {np.degrees(steering):.2f} degrees")
    print(f"Solver status: {debug_info['solver_status']}")
    print(f"Solver time: {debug_info['solver_time']:.2f} ms")
    print(f"Cost: {debug_info['cost']:.2f}")
    
    return steering, debug_info


def test_curved_path():
    """Test MPC on curved path."""
    print("\n=== Testing Curved Path Tracking ===")
    
    # Model parameters
    model_params = {
        'L': 3.0  # Wheelbase [m]
    }
    
    # Control parameters
    control_params = {
        'Np': 20,
        'Nc': 5,
        'Ts': 0.02,
        'Q_kinematic': [100, 50],
        'P_kinematic': [1000, 500],
        'R': 1.0,
        'R_delta': 10.0,
        'delta_limits': [-0.7, 0.7],
        'delta_rate_max': 0.05
    }
    
    # Initialize controller
    mpc = KinematicMpcCore(model_params, control_params)
    
    # Test case: On path but approaching curve
    current_state = np.array([0.0, 0.0])  # [e_y=0, e_psi=0]
    current_speed = 10.0  # m/s
    
    # Curved path (constant radius turn)
    radius = 50.0  # m
    curvature = 1.0 / radius
    reference_path = {
        'curvatures': np.ones(20) * curvature
    }
    
    # Calculate feedforward
    feedforward = mpc.calculate_feedforward_steering(curvature, current_speed)
    
    # Solve MPC
    steering, debug_info = mpc.solve(current_state, current_speed, reference_path)
    
    print(f"Initial state: e_y={current_state[0]:.2f}m, e_psi={current_state[1]:.2f}rad")
    print(f"Vehicle speed: {current_speed:.1f} m/s")
    print(f"Path radius: {radius:.1f} m")
    print(f"Path curvature: {curvature:.4f} 1/m")
    print(f"Feedforward steering: {np.degrees(feedforward):.2f} degrees")
    print(f"MPC optimal steering: {np.degrees(steering):.2f} degrees")
    print(f"Solver status: {debug_info['solver_status']}")
    print(f"Solver time: {debug_info['solver_time']:.2f} ms")
    
    return steering, debug_info


def test_tracking_with_errors():
    """Test MPC with both lateral and heading errors."""
    print("\n=== Testing Error Correction ===")
    
    # Model parameters
    model_params = {
        'L': 3.0
    }
    
    # Control parameters
    control_params = {
        'Np': 20,
        'Nc': 5,
        'Ts': 0.02,
        'Q_kinematic': [100, 50],
        'P_kinematic': [1000, 500],
        'R': 1.0,
        'R_delta': 10.0,
        'delta_limits': [-0.7, 0.7],
        'delta_rate_max': 0.05
    }
    
    # Initialize controller
    mpc = KinematicMpcCore(model_params, control_params)
    
    # Test case: Both lateral and heading errors
    current_state = np.array([1.0, 0.1])  # [e_y=1m, e_psi=0.1rad (~5.7 deg)]
    current_speed = 10.0  # m/s
    
    # Slight curve
    reference_path = {
        'curvatures': np.ones(20) * 0.01  # Gentle curve
    }
    
    # Solve MPC
    steering, debug_info = mpc.solve(current_state, current_speed, reference_path)
    
    print(f"Initial state: e_y={current_state[0]:.2f}m, e_psi={np.degrees(current_state[1]):.2f}deg")
    print(f"Vehicle speed: {current_speed:.1f} m/s")
    print(f"Optimal steering: {np.degrees(steering):.2f} degrees")
    print(f"Solver status: {debug_info['solver_status']}")
    
    # Show predicted trajectory
    if debug_info['predicted_states'] is not None:
        states = debug_info['predicted_states']
        print("\nPredicted states (first 5 steps):")
        for i in range(min(5, states.shape[1])):
            print(f"  Step {i}: e_y={states[0,i]:.3f}m, e_psi={np.degrees(states[1,i]):.2f}deg")
    
    return steering, debug_info


def main():
    """Run all tests."""
    print("="*60)
    print("Kinematic Bicycle Model MPC Test")
    print("="*60)
    
    try:
        # Run tests
        test_straight_line()
        test_curved_path()
        test_tracking_with_errors()
        
        print("\n" + "="*60)
        print("All tests completed successfully!")
        print("="*60)
        
    except Exception as e:
        print(f"\nError during testing: {e}")
        import traceback
        traceback.print_exc()
        return 1
    
    return 0


if __name__ == '__main__':
    exit(main())