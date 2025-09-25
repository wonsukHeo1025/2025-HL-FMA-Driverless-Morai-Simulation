#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Kinematic Bicycle Model Parameter Identification

This script identifies wheelbase (L) from steady-state cornering data.
The kinematic model is suitable for low speeds (<40 km/h) where tire slip is minimal.
"""

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.optimize import least_squares
from typing import Dict, List, Tuple, Optional
import os


class KinematicModelIdentifier:
    """Identifies parameters for kinematic bicycle model."""
    
    def __init__(self, wheelbase_nominal: float = 3.0):
        """
        Initialize identifier with nominal parameters.
        
        Args:
            wheelbase_nominal: Nominal wheelbase in meters (default for Ioniq5)
        """
        self.L_nominal = wheelbase_nominal
        self.L_identified = None
        self.validation_metrics = {}
        
    def load_data(self, csv_file: str) -> pd.DataFrame:
        """
        Load data from CSV file.
        
        Args:
            csv_file: Path to CSV file from steady-state cornering
            
        Returns:
            DataFrame with loaded data
        """
        if not os.path.exists(csv_file):
            raise FileNotFoundError(f"Data file not found: {csv_file}")
            
        df = pd.read_csv(csv_file)
        
        # Check required columns
        required_cols = ['steering_angle', 'vehicle_speed', 'yaw_rate', 
                        'lateral_acceleration', 'scenario_time']
        missing = [col for col in required_cols if col not in df.columns]
        if missing:
            raise ValueError(f"Missing required columns: {missing}")
            
        return df
    
    def extract_steady_state_segments(self, df: pd.DataFrame, 
                                     min_duration: float = 3.0) -> List[pd.DataFrame]:
        """
        Extract steady-state segments from data.
        
        Args:
            df: Full dataset
            min_duration: Minimum duration for steady-state (seconds)
            
        Returns:
            List of DataFrames, each containing one steady-state segment
        """
        segments = []
        
        # Detect changes in steering angle
        steering_diff = np.abs(np.diff(df['steering_angle'].values))
        change_indices = np.where(steering_diff > np.radians(2))[0]  # 2 degree threshold
        
        # Add start and end indices
        change_indices = np.concatenate([[0], change_indices, [len(df)-1]])
        
        for i in range(len(change_indices)-1):
            start_idx = change_indices[i] + int(2.0 * 50)  # Skip 2 seconds after change
            end_idx = change_indices[i+1]
            
            if end_idx - start_idx < min_duration * 50:  # Assuming 50Hz
                continue
                
            segment = df.iloc[start_idx:end_idx]
            
            # Check if segment is steady (low variance)
            if (segment['yaw_rate'].std() < 0.05 and 
                segment['steering_angle'].std() < np.radians(1)):
                segments.append(segment)
                
        return segments
    
    def identify_wheelbase(self, segments: List[pd.DataFrame]) -> float:
        """
        Identify wheelbase from steady-state segments.
        
        Uses the kinematic relationship: δ = L/R where R = v/ω
        
        Args:
            segments: List of steady-state data segments
            
        Returns:
            Identified wheelbase in meters
        """
        all_steering = []
        all_radius = []
        
        for segment in segments:
            # Average values for this segment
            delta = np.mean(segment['steering_angle'].values)
            v = np.mean(segment['vehicle_speed'].values)
            omega = np.mean(segment['yaw_rate'].values)
            
            # Skip if yaw rate is too small (straight driving)
            if abs(omega) < 0.01:
                continue
                
            # Calculate turning radius
            R = v / omega
            
            all_steering.append(abs(delta))
            all_radius.append(abs(R))
            
        # Convert to arrays
        steering = np.array(all_steering)
        radius = np.array(all_radius)
        
        # Linear fit: δ = L/R -> L = δ * R
        # Use least squares for robustness
        def residuals(L):
            return steering - L / radius
            
        result = least_squares(residuals, x0=[self.L_nominal], bounds=(2.0, 4.0))
        
        self.L_identified = result.x[0]
        
        # Calculate R-squared
        predicted = self.L_identified / radius
        ss_res = np.sum((steering - predicted) ** 2)
        ss_tot = np.sum((steering - np.mean(steering)) ** 2)
        r_squared = 1 - (ss_res / ss_tot) if ss_tot > 0 else 0
        
        self.validation_metrics['r_squared'] = r_squared
        self.validation_metrics['rmse'] = np.sqrt(np.mean((steering - predicted) ** 2))
        
        return self.L_identified
    
    def validate_model(self, test_segments: List[pd.DataFrame]) -> Dict:
        """
        Validate identified model on test data.
        
        Args:
            test_segments: Test data segments
            
        Returns:
            Dictionary with validation metrics
        """
        if self.L_identified is None:
            raise ValueError("Model not yet identified. Run identify_wheelbase first.")
            
        errors = []
        
        for segment in test_segments:
            delta_actual = np.mean(segment['steering_angle'].values)
            v = np.mean(segment['vehicle_speed'].values)
            omega = np.mean(segment['yaw_rate'].values)
            
            if abs(omega) < 0.01:
                continue
                
            R = v / omega
            delta_predicted = self.L_identified / R
            
            error = delta_actual - delta_predicted
            errors.append(error)
            
        errors = np.array(errors)
        
        metrics = {
            'mean_error': np.mean(errors),
            'std_error': np.std(errors),
            'rmse': np.sqrt(np.mean(errors ** 2)),
            'max_error': np.max(np.abs(errors))
        }
        
        return metrics
    
    def plot_results(self, segments: List[pd.DataFrame], save_path: Optional[str] = None):
        """
        Plot identification results.
        
        Args:
            segments: Data segments used for identification
            save_path: Optional path to save figure
        """
        fig, axes = plt.subplots(2, 2, figsize=(12, 10))
        
        # Plot 1: Steering vs 1/R
        ax = axes[0, 0]
        for segment in segments:
            delta = np.mean(segment['steering_angle'].values)
            v = np.mean(segment['vehicle_speed'].values)
            omega = np.mean(segment['yaw_rate'].values)
            
            if abs(omega) > 0.01:
                R = v / omega
                ax.scatter(1/R, np.degrees(delta), alpha=0.6)
        
        # Add fitted line
        R_range = np.linspace(10, 100, 100)
        delta_fit = self.L_identified / R_range
        ax.plot(1/R_range, np.degrees(delta_fit), 'r-', 
                label=f'L = {self.L_identified:.3f}m')
        ax.set_xlabel('1/R [1/m]')
        ax.set_ylabel('Steering Angle [deg]')
        ax.set_title('Kinematic Model Fit')
        ax.legend()
        ax.grid(True)
        
        # Plot 2: Residuals
        ax = axes[0, 1]
        residuals = []
        speeds = []
        for segment in segments:
            delta = np.mean(segment['steering_angle'].values)
            v = np.mean(segment['vehicle_speed'].values)
            omega = np.mean(segment['yaw_rate'].values)
            
            if abs(omega) > 0.01:
                R = v / omega
                delta_pred = self.L_identified / R
                residuals.append(np.degrees(delta - delta_pred))
                speeds.append(v * 3.6)  # Convert to km/h
                
        ax.scatter(speeds, residuals, alpha=0.6)
        ax.axhline(y=0, color='r', linestyle='--')
        ax.set_xlabel('Speed [km/h]')
        ax.set_ylabel('Residual [deg]')
        ax.set_title('Model Residuals vs Speed')
        ax.grid(True)
        
        # Plot 3: Time series example
        ax = axes[1, 0]
        if segments:
            segment = segments[0]  # Use first segment as example
            time = segment['scenario_time'].values
            ax.plot(time, np.degrees(segment['steering_angle'].values), 
                   label='Steering Angle')
            ax.plot(time, np.degrees(segment['yaw_rate'].values), 
                   label='Yaw Rate')
            ax.set_xlabel('Time [s]')
            ax.set_ylabel('Value [deg or deg/s]')
            ax.set_title('Example Steady-State Segment')
            ax.legend()
            ax.grid(True)
        
        # Plot 4: Model info
        ax = axes[1, 1]
        ax.axis('off')
        info_text = f"""
        Kinematic Bicycle Model Identification Results
        
        Nominal Wheelbase: {self.L_nominal:.3f} m
        Identified Wheelbase: {self.L_identified:.3f} m
        Difference: {(self.L_identified - self.L_nominal):.3f} m
        ({(self.L_identified - self.L_nominal)/self.L_nominal*100:.1f}%)
        
        Fit Quality:
        R²: {self.validation_metrics.get('r_squared', 0):.4f}
        RMSE: {np.degrees(self.validation_metrics.get('rmse', 0)):.2f}°
        
        Number of segments: {len(segments)}
        """
        ax.text(0.1, 0.5, info_text, fontsize=10, family='monospace',
               verticalalignment='center')
        
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=150)
            print(f"Figure saved to {save_path}")
        
        plt.show()
        
    def save_parameters(self, output_file: str):
        """
        Save identified parameters to file.
        
        Args:
            output_file: Path to output YAML/JSON file
        """
        import yaml
        
        params = {
            'model_type': 'kinematic_bicycle',
            'parameters': {
                'wheelbase': float(self.L_identified),
                'wheelbase_nominal': float(self.L_nominal)
            },
            'validation': self.validation_metrics
        }
        
        with open(output_file, 'w') as f:
            yaml.dump(params, f, default_flow_style=False)
            
        print(f"Parameters saved to {output_file}")


def main():
    """Main function for command-line usage."""
    import argparse
    
    parser = argparse.ArgumentParser(description='Identify kinematic bicycle model parameters')
    parser.add_argument('data_file', help='Path to CSV data file')
    parser.add_argument('--wheelbase', type=float, default=2.875,
                       help='Nominal wheelbase in meters')
    parser.add_argument('--output', help='Output file for parameters')
    parser.add_argument('--plot', action='store_true', help='Show plots')
    parser.add_argument('--save-plot', help='Path to save plot')
    
    args = parser.parse_args()
    
    # Create identifier
    identifier = KinematicModelIdentifier(wheelbase_nominal=args.wheelbase)
    
    # Load data
    print(f"Loading data from {args.data_file}")
    df = identifier.load_data(args.data_file)
    
    # Extract steady-state segments
    print("Extracting steady-state segments...")
    segments = identifier.extract_steady_state_segments(df)
    print(f"Found {len(segments)} steady-state segments")
    
    if len(segments) < 3:
        print("Warning: Few segments found. Results may be unreliable.")
    
    # Identify parameters
    print("Identifying wheelbase...")
    L = identifier.identify_wheelbase(segments)
    print(f"Identified wheelbase: {L:.3f} m")
    print(f"Nominal wheelbase: {args.wheelbase:.3f} m")
    print(f"Difference: {(L - args.wheelbase):.3f} m ({(L - args.wheelbase)/args.wheelbase*100:.1f}%)")
    
    # Validate
    print("\nValidation metrics:")
    for key, value in identifier.validation_metrics.items():
        if 'error' in key or 'rmse' in key:
            print(f"  {key}: {np.degrees(value):.3f}°")
        else:
            print(f"  {key}: {value:.4f}")
    
    # Save parameters
    if args.output:
        identifier.save_parameters(args.output)
    
    # Plot results
    if args.plot or args.save_plot:
        identifier.plot_results(segments, save_path=args.save_plot)


if __name__ == '__main__':
    main()