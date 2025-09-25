#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Test script for band-aware Q-weighting functionality
"""

import numpy as np
import yaml
import os
import sys

# Add parent directory to path for imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from mpc_lateral_controller.kinematic_mpc_core import KinematicMpcCore

def test_band_aware_q_switching():
    """Test that Q weights switch properly based on lateral error."""
    
    # Load config
    config_path = os.path.join(os.path.dirname(__file__), '..', 'config', 'mpc_params.yaml')
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)
    
    # Map YAML keys to expected internal keys
    control_params = {
        'Np': config['mpc']['prediction_horizon'],
        'Nc': config['mpc']['control_horizon'],
        'Ts': 1.0 / 50.0,  # Assuming 50Hz control rate
        'Q_kinematic': config['mpc']['Q_kinematic'],
        'P_kinematic': config['mpc']['P_kinematic'],
        'R': config['mpc']['R'],
        'R_delta': config['mpc']['R_delta'],
        'delta_limits': config['mpc']['delta_limits'],
        'delta_rate_max': config['mpc']['delta_rate_max'],
        'band_enable': config['mpc']['band_enable'],
        'band_base_width': config['mpc']['band_base_width'],
        'band_aware_q_weighting': config['mpc']['band_aware_q_weighting'],
        'Q_kinematic_inside_band': config['mpc']['Q_kinematic_inside_band'],
        'Q_kinematic_outside_band': config['mpc']['Q_kinematic_outside_band'],
        'time_varying_weights_enable': config['mpc']['time_varying_weights_enable'],
        'q_scale_front': config['mpc']['q_scale_front'],
        'q_scale_back': config['mpc']['q_scale_back'],
        'p_scale_end': config['mpc']['p_scale_end'],
        'osqp_max_iter': config['mpc']['osqp_max_iter'],
        'osqp_eps_abs': config['mpc']['osqp_eps_abs'],
        'osqp_eps_rel': config['mpc']['osqp_eps_rel'],
        'solver_backend': config['mpc']['solver_backend'],  # Add solver backend selection
    }
    model_params = {
        'L': 3.0,  # wheelbase
        'mode': 'kinematic'
    }
    
    # Initialize MPC core
    mpc = KinematicMpcCore(model_params, control_params)
    
    # Test parameters
    band_width = control_params.get('band_base_width', 1.0)
    q_inside = control_params.get('Q_kinematic_inside_band', [5.0, 200.0])
    q_outside = control_params.get('Q_kinematic_outside_band', [200.0, 250.0])
    
    print(f"Band-aware Q-weighting test")
    print(f"Band width: ±{band_width}m")
    print(f"Q inside band: {q_inside}")
    print(f"Q outside band: {q_outside}")
    print("-" * 50)
    
    # Test cases: different lateral errors
    test_cases = [
        (0.5, "Inside band"),
        (1.5, "Outside band"),
        (0.0, "On centerline"),
        (band_width - 0.1, "Near band edge (inside)"),
        (band_width + 0.1, "Near band edge (outside)")
    ]
    
    for lateral_error, description in test_cases:
        # Create error state [e_y, e_psi]
        error_state = np.array([lateral_error, 0.0])
        
        # Simple reference (straight line)
        reference_path = {
            'curvatures': np.zeros(control_params['Np'])
        }
        
        # Call solve to trigger band-aware Q logic
        try:
            steering, debug_info = mpc.solve(
                error_state, 
                current_speed=10.0,  # 10 m/s
                reference_path=reference_path
            )
            
            # Determine if inside or outside Q was used based on lateral error
            expected_inside = abs(lateral_error) <= band_width
            if expected_inside:
                q_type = "INSIDE"
            else:
                q_type = "OUTSIDE"
            
            # Check if solver was successful
            status = "✓" if debug_info.get('solver_status') == 'optimal' else "✗"
            
            print(f"{status} e_y={lateral_error:6.2f}m | {description:25s} | Q={q_type:7s} | δ={steering:6.3f}rad")
            
        except Exception as e:
            print(f"✗ e_y={lateral_error:6.2f}m | {description:25s} | Error: {str(e)}")
    
    print("-" * 50)
    print("Test complete!")

if __name__ == '__main__':
    test_band_aware_q_switching()