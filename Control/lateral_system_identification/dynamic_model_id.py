#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Dynamic Bicycle Model Parameter Identification

This script identifies parameters for the dynamic bicycle model including:
- Cornering stiffness (Caf, Car)
- Yaw moment of inertia (Iz)
- Understeer gradient (Kv)

The dynamic model accounts for tire slip and is suitable for higher speeds.
"""

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.optimize import minimize, differential_evolution
from scipy.signal import find_peaks
from scipy import signal
import yaml
from typing import Dict, List, Tuple, Optional
import os


class DynamicModelIdentifier:
    """Identifies parameters for dynamic bicycle model."""
    
    def __init__(self, vehicle_params: Optional[Dict] = None):
        """
        Initialize identifier with known vehicle parameters.
        
        Args:
            vehicle_params: Dictionary with known parameters (m, lf, lr, L)
        """
        # Default parameters (MORAI vehicle - actual measurements)
        self.params = {
            'm': 1901,      # Mass [kg] (18650 N / 9.81)
            'lf': 1.7,      # Front axle to CG [m]
            'lr': 1.3,      # Rear axle to CG [m]
            'L': 3.0,       # Wheelbase [m]
            'Iz': 2500,     # Yaw inertia [kg*m^2] (initial guess)
            'Caf': 100000,  # Front cornering stiffness [N/rad] (initial guess)
            'Car': 120000,  # Rear cornering stiffness [N/rad] (initial guess)
        }
        
        if vehicle_params:
            self.params.update(vehicle_params)
            
        self.identified_params = {}
        self.validation_metrics = {}
        
    def load_data(self, csv_file: str) -> pd.DataFrame:
        """Load data from CSV file."""
        if not os.path.exists(csv_file):
            raise FileNotFoundError(f"Data file not found: {csv_file}")
            
        df = pd.read_csv(csv_file)
        return df
    
    def estimate_understeer_gradient(self, steady_state_data: pd.DataFrame) -> float:
        """
        Estimate understeer gradient from steady-state cornering data.
        
        Uses: δ = L/R + Kv * ay where ay = v²/R
        With Ackermann bias removal: δ_dyn = δ - L*ay/v²
        Linear regression: δ_dyn = Kv * ay
        
        Args:
            steady_state_data: DataFrame with steady-state segments
            
        Returns:
            Understeer gradient Kv [rad/(m/s²)]
        """
        # Use is_steady_state flag if available
        if 'is_steady_state' in steady_state_data.columns:
            # Filter by steady-state flag
            segments = []
            steady_data = steady_state_data[steady_state_data['is_steady_state'] == True].copy()
            
            # If all data is steady-state, treat as one or multiple segments
            if len(steady_data) > 0:
                # For simplicity, if we have steady-state data, just use it
                # Group by significant changes in steering angle to find different steady conditions
                if 'steering_angle' in steady_data.columns:
                    # Round steering angle to nearest 0.01 rad to group similar conditions
                    steady_data['steering_group'] = np.round(steady_data['steering_angle'], 2)
                    
                    # Create segments for each unique steering condition
                    for steering_val, group in steady_data.groupby('steering_group'):
                        if len(group) >= 20:  # At least 20 samples
                            segments.append(group)
                else:
                    # If no steering angle, use entire steady data as one segment
                    segments = [steady_data]
        else:
            # Extract steady-state points using existing method
            segments = self._extract_steady_segments(steady_state_data)
        
        all_delta_dyn = []
        all_ay = []
        all_speeds = []
        
        L = self.params['L']
        
        for segment in segments:
            delta = np.mean(segment['steering_angle'].values)
            ay = np.mean(segment['lateral_acceleration'].values)
            v = np.mean(segment['vehicle_speed'].values)
            omega = np.mean(segment['yaw_rate'].values)
            
            # Skip low yaw rate segments
            if abs(omega) < 0.01:
                continue
            
            # Skip low speed segments (avoid division issues)
            if v < 2.0:  # 2 m/s minimum
                continue
                
            # Apply Ackermann bias correction
            # δ_dyn = δ - L*ay/v²
            delta_dyn = delta - (L * ay / (v * v))
            
            all_delta_dyn.append(delta_dyn)
            all_ay.append(ay)
            all_speeds.append(v)
            
        delta_dyn = np.array(all_delta_dyn)
        ay = np.array(all_ay)
        
        # Linear regression: δ_dyn = Kv * ay
        # The slope is the understeer gradient
        if len(ay) > 2:
            # Fit linear model through origin (since δ_dyn should be 0 when ay=0)
            # Using least squares: Kv = sum(δ_dyn * ay) / sum(ay²)
            Kv = np.sum(delta_dyn * ay) / np.sum(ay * ay)
            
            # Calculate R-squared for quality metric
            delta_dyn_pred = Kv * ay
            ss_res = np.sum((delta_dyn - delta_dyn_pred) ** 2)
            ss_tot = np.sum(delta_dyn ** 2)  # Since mean is 0 for through-origin fit
            r_squared = 1 - (ss_res / ss_tot) if ss_tot > 0 else 0
            
            self.identified_params['Kv'] = Kv
            self.identified_params['Kv_r2'] = r_squared
            self.identified_params['Kv_n_points'] = len(ay)
            
            # Log results
            print(f"Understeer gradient Kv = {Kv:.6f} rad/(m/s²)")
            print(f"  R² = {r_squared:.4f}, n_points = {len(ay)}")
            print(f"  Speed range: {np.min(all_speeds)*3.6:.1f} - {np.max(all_speeds)*3.6:.1f} km/h")
            
            # Calculate cornering stiffness relationship from Kv
            # Kv = m/L * (lr/Caf - lf/Car)
            # This gives one constraint on Caf and Car
            
            return Kv
        else:
            print("Insufficient steady-state data for Kv estimation")
            return 0.0
    
    def identify_from_step_response(self, step_data: pd.DataFrame) -> Dict:
        """
        Identify Iz, Caf, Car from step steer response.
        
        Uses optimization to match model response to measured data.
        
        Args:
            step_data: DataFrame with step steer response
            
        Returns:
            Dictionary with identified parameters
        """
        # Extract step response segments
        segments = self._extract_step_segments(step_data)
        
        if not segments:
            raise ValueError("No valid step response segments found")
            
        # Use first good segment for identification
        segment = segments[0]
        
        # Initial parameter guess
        x0 = [self.params['Iz'], self.params['Caf'], self.params['Car']]
        
        # Bounds for optimization
        bounds = [
            (1000, 5000),      # Iz [kg*m^2]
            (50000, 200000),   # Caf [N/rad]
            (50000, 200000),   # Car [N/rad]
        ]
        
        # Optimize
        result = differential_evolution(
            self._step_response_cost,
            bounds,
            args=(segment,),
            seed=42,
            maxiter=100,
            popsize=15
        )
        
        # Store results with quality metrics
        self.identified_params['Iz'] = result.x[0]
        self.identified_params['Caf'] = result.x[1]
        self.identified_params['Car'] = result.x[2]
        
        # Calculate RMSE for quality metric
        final_cost = result.fun
        n_points = len(segment)
        rmse = np.sqrt(final_cost / n_points) if n_points > 0 else 0
        
        self.identified_params['step_rmse'] = rmse
        self.identified_params['step_n_points'] = n_points
        self.identified_params['step_segments'] = len(segments)
        
        print(f"Step response identification:")
        print(f"  Iz = {result.x[0]:.1f} kg·m²")
        print(f"  Caf = {result.x[1]:.0f} N/rad")
        print(f"  Car = {result.x[2]:.0f} N/rad")
        print(f"  RMSE = {rmse:.4f} rad/s")
        print(f"  Segments = {len(segments)}, Points = {n_points}")
        
        return self.identified_params
    
    def _step_response_cost(self, params: np.ndarray, data: pd.DataFrame) -> float:
        """
        Cost function for step response fitting.
        
        Args:
            params: [Iz, Caf, Car]
            data: Measured step response data
            
        Returns:
            Cost value (MSE of yaw rate)
        """
        Iz, Caf, Car = params
        
        # Get measured data
        t = data['scenario_time'].values
        delta_measured = data['steering_angle'].values
        yaw_rate_measured = data['yaw_rate'].values
        v = np.mean(data['vehicle_speed'].values)
        
        # Simulate model response
        yaw_rate_model = self._simulate_step_response(
            t, delta_measured, v, Iz, Caf, Car
        )
        
        # Calculate MSE
        mse = np.mean((yaw_rate_measured - yaw_rate_model) ** 2)
        
        return mse
    
    def _simulate_step_response(self, t: np.ndarray, delta: np.ndarray, 
                               v: float, Iz: float, Caf: float, Car: float) -> np.ndarray:
        """
        Simulate dynamic bicycle model response to steering input.
        
        Args:
            t: Time vector
            delta: Steering angle input
            v: Vehicle speed
            Iz, Caf, Car: Model parameters
            
        Returns:
            Yaw rate response
        """
        m = self.params['m']
        lf = self.params['lf']
        lr = self.params['lr']
        
        # State space matrices
        A = np.array([
            [0, 1, 0, 0],
            [0, -(Caf + Car)/(m*v), (Caf + Car)/m, (-lf*Caf + lr*Car)/(m*v)],
            [0, 0, 0, 1],
            [0, (-lf*Caf + lr*Car)/(Iz*v), (lf*Caf - lr*Car)/Iz, 
             -(lf**2*Caf + lr**2*Car)/(Iz*v)]
        ])
        
        B = np.array([[0], [Caf/m], [0], [lf*Caf/Iz]])
        C = np.array([[0, 0, 0, 1]])  # Output is yaw rate
        D = np.array([[0]])
        
        # Create state space system
        sys = signal.StateSpace(A, B, C, D)
        
        # Simulate response
        _, yaw_rate, _ = signal.lsim(sys, delta, t)
        
        return yaw_rate.flatten()
    
    def identify_from_frequency_response(self, sweep_data: pd.DataFrame) -> Dict:
        """
        Identify parameters from sine sweep frequency response.
        
        Args:
            sweep_data: DataFrame with sine sweep data
            
        Returns:
            Dictionary with identified parameters
        """
        # Extract frequency response
        freq, gain, phase = self._extract_frequency_response(sweep_data)
        
        if len(freq) < 5:
            print("Insufficient frequency data points")
            return self.identified_params
            
        # Initial guess
        x0 = [self.params['Iz'], self.params['Caf'], self.params['Car']]
        
        # Optimize to match frequency response
        result = minimize(
            self._frequency_response_cost,
            x0,
            args=(freq, gain, phase, np.mean(sweep_data['vehicle_speed'])),
            bounds=[(1000, 5000), (50000, 200000), (50000, 200000)],
            method='L-BFGS-B'
        )
        
        # Update parameters with quality metrics
        self.identified_params['Iz_freq'] = result.x[0]
        self.identified_params['Caf_freq'] = result.x[1]
        self.identified_params['Car_freq'] = result.x[2]
        
        # Add quality metrics
        self.identified_params['freq_n_points'] = len(freq)
        self.identified_params['freq_range'] = (freq[0], freq[-1]) if len(freq) > 0 else (0, 0)
        self.identified_params['freq_cost'] = result.fun
        
        print(f"Frequency response identification:")
        print(f"  Iz_freq = {result.x[0]:.1f} kg·m²")
        print(f"  Caf_freq = {result.x[1]:.0f} N/rad")
        print(f"  Car_freq = {result.x[2]:.0f} N/rad")
        print(f"  Frequency points = {len(freq)}")
        print(f"  Frequency range = {freq[0]:.2f} - {freq[-1]:.2f} Hz")
        
        return self.identified_params
    
    def _extract_frequency_response(self, sweep_data: pd.DataFrame) -> Tuple:
        """
        Extract frequency response from sine sweep data.
        
        Returns:
            Tuple of (frequencies, gains, phases)
        """
        # Get steering input and yaw rate output
        t = sweep_data['scenario_time'].values
        delta = sweep_data['steering_angle'].values
        yaw_rate = sweep_data['yaw_rate'].values
        
        # Use FFT to extract frequency response
        # This is simplified - better to use windowed segments
        dt = np.mean(np.diff(t))
        fs = 1.0 / dt
        
        # Compute transfer function estimate
        f, Pxy = signal.csd(delta, yaw_rate, fs=fs, nperseg=256)
        f, Pxx = signal.welch(delta, fs=fs, nperseg=256)
        
        # Transfer function
        H = Pxy / Pxx
        
        # Extract magnitude and phase
        gain = np.abs(H)
        phase = np.angle(H)
        
        # Filter valid frequency range (0.1 to 2.5 Hz for 50 km/h)
        # Adaptive upper limit based on speed
        max_freq = 2.5 if sweep_data['vehicle_speed'].mean() * 3.6 > 40 else 2.0
        valid_idx = (f > 0.1) & (f < max_freq)
        
        return f[valid_idx], gain[valid_idx], phase[valid_idx]
    
    def _frequency_response_cost(self, params: np.ndarray, freq: np.ndarray,
                                gain_meas: np.ndarray, phase_meas: np.ndarray,
                                v: float) -> float:
        """Cost function for frequency response fitting."""
        Iz, Caf, Car = params
        
        # Calculate model frequency response
        gain_model, phase_model = self._calculate_frequency_response(
            freq, v, Iz, Caf, Car
        )
        
        # Weighted cost (gain more important than phase)
        cost_gain = np.mean((gain_meas - gain_model) ** 2)
        cost_phase = np.mean((phase_meas - phase_model) ** 2)
        
        return cost_gain + 0.1 * cost_phase
    
    def _calculate_frequency_response(self, freq: np.ndarray, v: float,
                                     Iz: float, Caf: float, Car: float) -> Tuple:
        """Calculate theoretical frequency response."""
        m = self.params['m']
        lf = self.params['lf']
        lr = self.params['lr']
        
        # State space matrices (same as before)
        A = np.array([
            [0, 1, 0, 0],
            [0, -(Caf + Car)/(m*v), (Caf + Car)/m, (-lf*Caf + lr*Car)/(m*v)],
            [0, 0, 0, 1],
            [0, (-lf*Caf + lr*Car)/(Iz*v), (lf*Caf - lr*Car)/Iz, 
             -(lf**2*Caf + lr**2*Car)/(Iz*v)]
        ])
        
        B = np.array([[0], [Caf/m], [0], [lf*Caf/Iz]])
        C = np.array([[0, 0, 0, 1]])  # Yaw rate output
        D = np.array([[0]])
        
        # Create system
        sys = signal.StateSpace(A, B, C, D)
        
        # Calculate frequency response
        w = 2 * np.pi * freq
        w, mag, phase = signal.bode(sys, w)
        
        # Convert magnitude from dB to linear
        gain = 10 ** (mag / 20)
        
        return gain, phase
    
    def _extract_steady_segments(self, data: pd.DataFrame) -> List[pd.DataFrame]:
        """Extract steady-state segments from data."""
        segments = []
        
        # Simple segmentation based on steering angle changes
        steering_diff = np.abs(np.diff(data['steering_angle'].values))
        change_indices = np.where(steering_diff > np.radians(2))[0]
        
        change_indices = np.concatenate([[0], change_indices, [len(data)-1]])
        
        for i in range(len(change_indices)-1):
            start = change_indices[i] + 100  # Skip transient
            end = change_indices[i+1]
            
            if end - start < 150:  # Need at least 3 seconds at 50Hz
                continue
                
            segment = data.iloc[start:end]
            
            # Check steadiness
            if (segment['yaw_rate'].std() < 0.05 and 
                segment['steering_angle'].std() < np.radians(1)):
                segments.append(segment)
                
        return segments
    
    def _extract_step_segments(self, data: pd.DataFrame) -> List[pd.DataFrame]:
        """Extract step response segments from data."""
        segments = []
        
        # Detect step changes in steering
        steering = data['steering_angle'].values
        steering_diff = np.diff(steering)
        
        # Use same threshold as segmentation.py: 2 degrees (0.035 rad)
        step_threshold = np.radians(1.8)  # Slightly below 2 degrees to catch 2° steps
        step_indices = np.where(np.abs(steering_diff) >= step_threshold)[0]
        
        for idx in step_indices:
            # Check minimum step size with speed adaptation
            if idx > 0 and idx < len(steering) - 1:
                step_size = abs(steering[idx+1] - steering[idx-1])
                
                # Speed-adaptive minimum step size
                min_step = np.radians(2)  # Default 2 degrees
                if 'vehicle_speed' in data.columns and idx < len(data):
                    speed_kmh = data.iloc[idx]['vehicle_speed'] * 3.6
                    if speed_kmh < 30:
                        min_step = np.radians(4)  # 4 degrees for low speed
                    elif speed_kmh > 40:
                        min_step = np.radians(2)  # 2 degrees for high speed
                    else:
                        # Linear interpolation between 30-40 km/h
                        min_step = np.radians(4 - (speed_kmh - 30) * 0.2)
                
                if step_size < min_step:
                    continue
            
            # Extract segment around step (1s before, 6s after for settling)
            pre_samples = 50   # 1 second at 50Hz
            post_samples = 300  # 6 seconds at 50Hz
            start = max(0, idx - pre_samples)
            end = min(len(data), idx + post_samples)
            
            segment = data.iloc[start:end].copy()
            # Reset scenario_time to ensure monotonic increasing
            segment = segment.reset_index(drop=True)
            # Create monotonic time vector based on index
            dt = 0.02  # Assuming 50 Hz
            segment['scenario_time'] = np.arange(len(segment)) * dt
            segments.append(segment)
            
        print(f"Detected {len(segments)} step segments with 2° threshold")
        return segments
    
    def validate_model(self, test_data: pd.DataFrame) -> Dict:
        """
        Validate identified model on test data.
        
        Args:
            test_data: Test dataset
            
        Returns:
            Dictionary with validation metrics
        """
        # Simulate model with identified parameters
        # Compare with measured data
        # This is simplified - would need proper implementation
        
        metrics = {
            'yaw_rate_rmse': 0.0,
            'lateral_accel_rmse': 0.0,
            'understeer_error': 0.0
        }
        
        return metrics
    
    def plot_results(self, data: pd.DataFrame, save_path: Optional[str] = None):
        """Plot identification results."""
        fig, axes = plt.subplots(2, 3, figsize=(15, 10))
        
        # Plot 1: Understeer gradient
        ax = axes[0, 0]
        segments = self._extract_steady_segments(data)
        for segment in segments:
            ay = np.mean(segment['lateral_acceleration'].values)
            delta = np.degrees(np.mean(segment['steering_angle'].values))
            ax.scatter(ay, delta, alpha=0.6)
            
        if 'Kv' in self.identified_params:
            ay_range = np.linspace(0, 10, 100)
            delta_fit = np.degrees(self.identified_params['Kv'] * ay_range)
            label_text = f"Kv = {self.identified_params['Kv']:.4f} rad/(m/s²)"
            if 'Kv_r2' in self.identified_params:
                label_text += f"\nR² = {self.identified_params['Kv_r2']:.4f}"
            if 'Kv_n_points' in self.identified_params:
                label_text += f"\nn = {self.identified_params['Kv_n_points']} points"
            ax.plot(ay_range, delta_fit, 'r-', label=label_text)
            ax.legend()
            
        ax.set_xlabel('Lateral Acceleration [m/s²]')
        ax.set_ylabel('Steering Angle [deg]')
        ax.set_title('Understeer Gradient')
        ax.grid(True)
        
        # Plot 2: Step response example
        ax = axes[0, 1]
        step_segments = self._extract_step_segments(data)
        if step_segments:
            segment = step_segments[0]
            t = segment['scenario_time'].values
            ax.plot(t, np.degrees(segment['steering_angle'].values), label='Steering')
            ax.plot(t, np.degrees(segment['yaw_rate'].values), label='Yaw Rate')
            
            # Add model response if available
            if all(k in self.identified_params for k in ['Iz', 'Caf', 'Car']):
                v = np.mean(segment['vehicle_speed'].values)
                yaw_model = self._simulate_step_response(
                    t, segment['steering_angle'].values, v,
                    self.identified_params['Iz'],
                    self.identified_params['Caf'],
                    self.identified_params['Car']
                )
                ax.plot(t, np.degrees(yaw_model), 'r--', label='Model')
                
            ax.set_xlabel('Time [s]')
            ax.set_ylabel('Value [deg or deg/s]')
            ax.set_title('Step Response')
            ax.legend()
            ax.grid(True)
        
        # Plot 3: Frequency response (if available)
        ax = axes[0, 2]
        if 'input_frequency' in data.columns:
            freq, gain, phase = self._extract_frequency_response(data)
            if len(freq) > 0:
                ax.semilogx(freq, 20*np.log10(gain))
                ax.set_xlabel('Frequency [Hz]')
                ax.set_ylabel('Gain [dB]')
                ax.set_title('Frequency Response')
                ax.grid(True)
        
        # Plot 4: Parameter summary
        ax = axes[1, 0]
        ax.axis('off')
        param_text = "Identified Parameters:\n\n"
        for key, value in self.identified_params.items():
            param_text += f"{key}: {value:.2f}\n"
        ax.text(0.1, 0.5, param_text, fontsize=10, family='monospace')
        
        # Plot 5: Cornering stiffness relationship
        ax = axes[1, 1]
        if 'Kv' in self.identified_params and 'Caf' in self.identified_params:
            # Show relationship between Kv and cornering stiffness
            ax.text(0.1, 0.5, 
                   f"From Kv: Caf/Car ratio constraint\n"
                   f"Caf = {self.identified_params.get('Caf', 0):.0f} N/rad\n"
                   f"Car = {self.identified_params.get('Car', 0):.0f} N/rad",
                   fontsize=10)
        ax.axis('off')
        
        # Plot 6: Validation metrics
        ax = axes[1, 2]
        ax.axis('off')
        if self.validation_metrics:
            metric_text = "Validation Metrics:\n\n"
            for key, value in self.validation_metrics.items():
                metric_text += f"{key}: {value:.4f}\n"
            ax.text(0.1, 0.5, metric_text, fontsize=10, family='monospace')
        
        plt.suptitle('Dynamic Bicycle Model Identification Results')
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=150)
            print(f"Figure saved to {save_path}")
            
        plt.show()
    
    def save_parameters(self, output_file: str):
        """Save identified parameters to YAML file."""
        params = {
            'model_type': 'dynamic_bicycle',
            'known_parameters': {
                'm': float(self.params['m']),
                'lf': float(self.params['lf']),
                'lr': float(self.params['lr']),
                'L': float(self.params['L'])
            },
            'identified_parameters': {
                k: float(v) for k, v in self.identified_params.items()
            },
            'validation_metrics': self.validation_metrics
        }
        
        with open(output_file, 'w') as f:
            yaml.dump(params, f, default_flow_style=False)
            
        print(f"Parameters saved to {output_file}")


def main():
    """Main function for command-line usage."""
    import argparse
    
    parser = argparse.ArgumentParser(description='Identify dynamic bicycle model parameters')
    parser.add_argument('data_file', help='Path to CSV data file')
    parser.add_argument('--scenario', choices=['steady_state', 'step_steer', 'sine_sweep'],
                       default='step_steer', help='Type of data scenario')
    parser.add_argument('--mass', type=float, default=1800, help='Vehicle mass [kg]')
    parser.add_argument('--lf', type=float, default=1.3, help='Front axle to CG [m]')
    parser.add_argument('--lr', type=float, default=1.575, help='Rear axle to CG [m]')
    parser.add_argument('--output', help='Output file for parameters')
    parser.add_argument('--plot', action='store_true', help='Show plots')
    parser.add_argument('--save-plot', help='Path to save plot')
    
    args = parser.parse_args()
    
    # Set up vehicle parameters
    vehicle_params = {
        'm': args.mass,
        'lf': args.lf,
        'lr': args.lr,
        'L': args.lf + args.lr
    }
    
    # Create identifier
    identifier = DynamicModelIdentifier(vehicle_params)
    
    # Load data
    print(f"Loading data from {args.data_file}")
    data = identifier.load_data(args.data_file)
    
    # Identify based on scenario type
    if args.scenario == 'steady_state':
        print("Estimating understeer gradient...")
        Kv = identifier.estimate_understeer_gradient(data)
        print(f"Understeer gradient Kv = {Kv:.4f} rad/(m/s²)")
        
    elif args.scenario == 'step_steer':
        print("Identifying from step response...")
        params = identifier.identify_from_step_response(data)
        print("Identified parameters:")
        for key, value in params.items():
            print(f"  {key}: {value:.2f}")
            
    elif args.scenario == 'sine_sweep':
        print("Identifying from frequency response...")
        params = identifier.identify_from_frequency_response(data)
        print("Identified parameters:")
        for key, value in params.items():
            print(f"  {key}: {value:.2f}")
    
    # Save parameters
    if args.output:
        identifier.save_parameters(args.output)
    
    # Plot results
    if args.plot or args.save_plot:
        identifier.plot_results(data, save_path=args.save_plot)


if __name__ == '__main__':
    main()