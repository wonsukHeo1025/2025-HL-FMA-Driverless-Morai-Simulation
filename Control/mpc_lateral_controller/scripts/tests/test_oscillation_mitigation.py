#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Test script for oscillation mitigation features in lateral MPC.
Tests the new features:
1. Band-aware Q weighting with continuous blending
2. Time-varying prediction horizon weights
3. Input cost feedforward centering
4. State-dependent smoothing
"""

import numpy as np
import matplotlib.pyplot as plt
import sys
import os

# Add parent directory to path
sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from src.mpc_lateral_controller.kinematic_mpc_core import KinematicMpcCore


def test_band_aware_q_weighting():
    """Test band-aware Q weighting with continuous blending."""
    print("\n=== Testing Band-aware Q Weighting ===")
    
    # Model and control parameters with new features enabled
    model_params = {
        'L': 3.0,  # Wheelbase
        'mode': 'kinematic'
    }
    
    control_params = {
        'Np': 75,  # Prediction horizon
        'Nc': 10,  # Control horizon
        'Ts': 0.02,  # 50Hz
        'Q_kinematic': [200.0, 250.0],  # Default weights
        'P_kinematic': [2000.0, 2500.0],
        'R': 1.0,
        'R_delta': 40.0,
        'delta_limits': [-0.698, 0.698],
        'delta_rate_max': 0.015,
        
        # New features
        'band_aware_q_weighting': True,
        'Q_kinematic_inside_band': [5.0, 800.0],  # Inside: low y, high psi
        'Q_kinematic_outside_band': [250.0, 400.0],  # Outside: fast return
        'band_blend_hysteresis': True,
        'band_hyst_ratio_inner': 0.9,
        'band_hyst_ratio_outer': 1.1,
        
        'band_enable': True,
        'band_base_width': 0.5,
        'band_lambda': 3000.0,
        
        'solver_backend': 'osqp',
        'osqp_max_iter': 1000,
        'osqp_eps_abs': 2e-3,
        'osqp_eps_rel': 2e-3
    }
    
    # Create MPC controller
    mpc = KinematicMpcCore(model_params, control_params)
    
    # Test different lateral errors (inside/outside band)
    test_states = [
        np.array([0.2, 0.1]),   # Inside band
        np.array([0.45, 0.1]),  # Near boundary
        np.array([0.55, 0.1]),  # Just outside
        np.array([0.8, 0.2]),   # Far outside
    ]
    
    for state in test_states:
        # Simple straight path (zero curvature)
        reference_path = {
            'curvatures': np.zeros(control_params['Np'])
        }
        
        steering, debug = mpc.solve(state, 10.0, reference_path)
        
        print(f"State: e_y={state[0]:.2f}m, e_psi={state[1]:.2f}rad")
        print(f"  Steering: {steering:.4f} rad")
        print(f"  Solver status: {debug['solver_status']}")
        print(f"  Cost: {debug['cost']:.2f}")


def test_feedforward_centering():
    """Test input cost feedforward centering."""
    print("\n=== Testing Feedforward Centering ===")
    
    model_params = {
        'L': 3.0,
        'mode': 'kinematic'
    }
    
    # Test with and without feedforward centering
    configs = [
        {'input_cost_centering': False, 'name': 'Without FF'},
        {'input_cost_centering': True, 'name': 'With FF'},
    ]
    
    for config in configs:
        control_params = {
            'Np': 75,
            'Nc': 10,
            'Ts': 0.02,
            'Q_kinematic': [200.0, 250.0],
            'P_kinematic': [2000.0, 2500.0],
            'R': 1.0,
            'R_delta': 40.0,
            'delta_limits': [-0.698, 0.698],
            'delta_rate_max': 0.015,
            
            'input_cost_centering': config['input_cost_centering'],
            'understeer_compensation': False,
            
            'solver_backend': 'osqp',
            'osqp_max_iter': 1000,
        }
        
        mpc = KinematicMpcCore(model_params, control_params)
        
        # Test on curved path
        curvature = 0.1  # 10m radius
        reference_path = {
            'curvatures': np.ones(control_params['Np']) * curvature
        }
        
        # Small error state
        state = np.array([0.05, 0.02])
        steering, debug = mpc.solve(state, 10.0, reference_path)
        
        # Expected feedforward
        delta_ff = model_params['L'] * curvature
        
        print(f"\n{config['name']}:")
        print(f"  Curvature: {curvature:.3f} 1/m")
        print(f"  Expected FF: {delta_ff:.4f} rad")
        print(f"  Actual steering: {steering:.4f} rad")
        print(f"  Difference: {abs(steering - delta_ff):.4f} rad")


def test_state_dependent_smoothing():
    """Test state-dependent constraint and smoothing scheduling."""
    print("\n=== Testing State-dependent Smoothing ===")
    
    model_params = {
        'L': 3.0,
        'mode': 'kinematic'
    }
    
    control_params = {
        'Np': 75,
        'Nc': 10,
        'Ts': 0.02,
        'Q_kinematic': [200.0, 250.0],
        'P_kinematic': [2000.0, 2500.0],
        'R': 1.0,
        'R_delta': 40.0,
        'delta_limits': [-0.698, 0.698],
        'delta_rate_max': 0.015,
        
        # State-dependent smoothing
        'state_dep_smoothing_enable': True,
        'inside_delta_rate_max': 0.025,  # Faster inside
        'inside_R_delta': 15.0,           # Less smooth inside
        'outside_delta_rate_max': 0.015,  # Slower outside
        'outside_R_delta': 30.0,          # Smoother outside
        
        'band_enable': True,
        'band_base_width': 0.5,
        
        'solver_backend': 'osqp',
        'osqp_max_iter': 1000,
    }
    
    mpc = KinematicMpcCore(model_params, control_params)
    
    # Simulate step response for inside and outside band
    test_cases = [
        {'state': np.array([0.3, 0.0]), 'label': 'Inside band'},
        {'state': np.array([0.7, 0.0]), 'label': 'Outside band'},
    ]
    
    for case in test_cases:
        print(f"\n{case['label']} (e_y={case['state'][0]:.1f}m):")
        
        # Simulate multiple steps
        states = []
        controls = []
        state = case['state'].copy()
        
        for _ in range(20):
            reference_path = {'curvatures': np.zeros(control_params['Np'])}
            steering, _ = mpc.solve(state, 10.0, reference_path)
            
            states.append(state.copy())
            controls.append(steering)
            
            # Simple state update (Euler)
            A, B, _ = mpc._get_discrete_matrices(10.0)
            state = A @ state + B.flatten() * steering
        
        # Analyze control smoothness
        control_changes = np.diff(controls)
        max_change = np.max(np.abs(control_changes)) if len(control_changes) > 0 else 0
        avg_change = np.mean(np.abs(control_changes)) if len(control_changes) > 0 else 0
        
        print(f"  Max control change: {max_change:.4f} rad")
        print(f"  Avg control change: {avg_change:.4f} rad")
        print(f"  Final e_y: {states[-1][0]:.4f} m")


def plot_oscillation_comparison():
    """Compare oscillation behavior with and without mitigation."""
    print("\n=== Oscillation Comparison ===")
    
    model_params = {
        'L': 3.0,
        'mode': 'kinematic'
    }
    
    # Configuration without mitigation
    control_params_base = {
        'Np': 75,
        'Nc': 10,
        'Ts': 0.02,
        'Q_kinematic': [200.0, 250.0],
        'P_kinematic': [2000.0, 2500.0],
        'R': 1.0,
        'R_delta': 40.0,
        'delta_limits': [-0.698, 0.698],
        'delta_rate_max': 0.015,
        'band_enable': True,
        'band_base_width': 0.5,
        'band_lambda': 3000.0,
        'solver_backend': 'osqp',
    }
    
    # Configuration with all mitigations
    control_params_mitigated = control_params_base.copy()
    control_params_mitigated.update({
        'band_aware_q_weighting': True,
        'Q_kinematic_inside_band': [5.0, 800.0],
        'Q_kinematic_outside_band': [250.0, 400.0],
        'band_blend_hysteresis': True,
        'band_hyst_ratio_inner': 0.9,
        'band_hyst_ratio_outer': 1.1,
        'input_cost_centering': True,
        'state_dep_smoothing_enable': True,
        'inside_delta_rate_max': 0.025,
        'inside_R_delta': 15.0,
        'outside_delta_rate_max': 0.015,
        'outside_R_delta': 30.0,
        'time_varying_weights_enable': True,
        'q_scale_front': 0.6,
        'q_scale_back': 1.6,
        'p_scale_end': 2.0,
    })
    
    # Simulate both controllers
    configs = [
        {'params': control_params_base, 'label': 'Without mitigation', 'color': 'r'},
        {'params': control_params_mitigated, 'label': 'With mitigation', 'color': 'b'},
    ]
    
    plt.figure(figsize=(12, 8))
    
    for config in configs:
        mpc = KinematicMpcCore(model_params, config['params'])
        
        # Initial state with small heading error (typical oscillation trigger)
        state = np.array([0.4, 0.15])
        
        # Simulate
        time_steps = 150
        states = []
        controls = []
        
        for _ in range(time_steps):
            reference_path = {'curvatures': np.zeros(config['params']['Np'])}
            steering, _ = mpc.solve(state, 10.0, reference_path)
            
            states.append(state.copy())
            controls.append(steering)
            
            # State update
            A, B, _ = mpc._get_discrete_matrices(10.0)
            state = A @ state + B.flatten() * steering
        
        states = np.array(states)
        controls = np.array(controls)
        time = np.arange(time_steps) * config['params']['Ts']
        
        # Plot results
        plt.subplot(3, 1, 1)
        plt.plot(time, states[:, 0], config['color'], label=config['label'], linewidth=2)
        plt.axhline(y=config['params']['band_base_width'], color='gray', linestyle='--', alpha=0.5)
        plt.axhline(y=-config['params']['band_base_width'], color='gray', linestyle='--', alpha=0.5)
        plt.ylabel('Lateral Error [m]')
        plt.legend()
        plt.grid(True, alpha=0.3)
        plt.title('Oscillation Mitigation Comparison')
        
        plt.subplot(3, 1, 2)
        plt.plot(time, states[:, 1], config['color'], linewidth=2)
        plt.ylabel('Heading Error [rad]')
        plt.grid(True, alpha=0.3)
        
        plt.subplot(3, 1, 3)
        plt.plot(time, controls, config['color'], linewidth=2)
        plt.ylabel('Steering [rad]')
        plt.xlabel('Time [s]')
        plt.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('/home/user1/catkin_ws/src/Control/mpc_lateral_controller/docs/oscillation_comparison.png')
    print("\nPlot saved to docs/oscillation_comparison.png")
    
    # Calculate metrics
    for i, config in enumerate(configs):
        if i == 0:
            states_base = states
            controls_base = controls
        else:
            # Zero crossing count
            zc_base = np.sum(np.diff(np.sign(states_base[:, 0])) != 0)
            zc_mit = np.sum(np.diff(np.sign(states[:, 0])) != 0)
            
            # Control effort
            effort_base = np.sum(np.abs(np.diff(controls_base)))
            effort_mit = np.sum(np.abs(np.diff(controls)))
            
            print(f"\nMetrics:")
            print(f"  Zero crossings - Base: {zc_base}, Mitigated: {zc_mit}")
            print(f"  Control effort - Base: {effort_base:.3f}, Mitigated: {effort_mit:.3f}")
            print(f"  Improvement: {(1 - zc_mit/max(zc_base, 1))*100:.1f}% fewer crossings")


if __name__ == "__main__":
    print("=" * 60)
    print("Testing Lateral MPC Oscillation Mitigation Features")
    print("=" * 60)
    
    # Run tests
    test_band_aware_q_weighting()
    test_feedforward_centering()
    test_state_dependent_smoothing()
    plot_oscillation_comparison()
    
    print("\n" + "=" * 60)
    print("All tests completed successfully!")
    print("=" * 60)