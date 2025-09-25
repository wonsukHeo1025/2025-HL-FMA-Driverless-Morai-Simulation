#!/usr/bin/env python3
"""
Integrated Lateral System Identification Tool

Main entry point for processing multiple CSV files and identifying
lateral dynamics parameters.
"""

import os
import sys
import argparse
import logging
import yaml
import numpy as np
import pandas as pd
from typing import Dict, List, Optional, Tuple
from datetime import datetime

# Add src to path
sys.path.append(os.path.join(os.path.dirname(__file__), 'src'))

# Import modules
from data_io import CSVLoader, DataAggregator
from preprocessing import SignalMapper, SignalFilter, Segmentation
# Note: identification, validation, and reporting modules are not yet implemented
# They are placeholders for future development
# Currently using dynamic_model_id.py directly for identification

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


class IntegratedSystemIdentifier:
    """Main class for integrated system identification."""
    
    def __init__(self, vehicle_params: Optional[Dict] = None):
        """
        Initialize integrated system identifier.
        
        Args:
            vehicle_params: Known vehicle parameters (m, lf, lr, L)
        """
        # Default vehicle parameters (MORAI vehicle - actual measurements)
        self.vehicle_params = {
            'm': 1901.0,    # Mass [kg] (18650 N / 9.81)
            'lf': 1.7,      # Front axle to CG [m]
            'lr': 1.3,      # Rear axle to CG [m]
            'L': 3.0        # Wheelbase [m]
        }
        
        if vehicle_params:
            self.vehicle_params.update(vehicle_params)
            
        # Initialize components
        self.csv_loader = CSVLoader()
        self.aggregator = DataAggregator()
        self.mapper = SignalMapper()
        self.filter = SignalFilter()
        
        # Results storage
        self.identified_params = {}
        self.quality_metrics = {}
        self.validation_results = {}
        
    def process_directory(self, directory: str) -> Dict:
        """
        Process all CSV files in a directory.
        
        Args:
            directory: Path to directory containing CSV files
            
        Returns:
            Dictionary with identified parameters
        """
        logger.info(f"Processing directory: {directory}")
        
        # Load all CSV files
        dataframes = self.csv_loader.load_directory(directory)
        
        if not dataframes:
            logger.error("No valid CSV files found")
            return {}
            
        # Aggregate data
        combined_data = self.aggregator.aggregate(dataframes)
        logger.info(self.aggregator.summary())
        
        # Map signals to standard format
        combined_data = self.mapper.map_signals(combined_data)
        
        # Filter and preprocess
        combined_data = self.filter.process(combined_data)
        
        # Process each scenario type
        results = {}
        
        # Steady-state identification
        if 'steady_state_cornering' in self.aggregator.scenario_data:
            logger.info("Processing steady-state data...")
            results['steady_state'] = self._process_steady_state(
                self.aggregator.get_scenario_data('steady_state_cornering')
            )
            
        # Step response identification
        if 'step_steer' in self.aggregator.scenario_data:
            logger.info("Processing step response data...")
            results['step_response'] = self._process_step_response(
                self.aggregator.get_scenario_data('step_steer')
            )
            
        # Frequency response identification
        if 'sine_sweep' in self.aggregator.scenario_data:
            logger.info("Processing frequency response data...")
            results['frequency_response'] = self._process_frequency_response(
                self.aggregator.get_scenario_data('sine_sweep')
            )
            
        # Combine results
        self._combine_results(results)
        
        # Validate model
        self._validate_model(combined_data)
        
        return self.identified_params
        
    def _process_steady_state(self, data: pd.DataFrame) -> Dict:
        """Process steady-state cornering data."""
        # Create identifier (this would import from the actual module)
        from dynamic_model_id import DynamicModelIdentifier
        
        identifier = DynamicModelIdentifier(self.vehicle_params)
        
        # Map signals
        data = self.mapper.map_signals(data)
        data = self.filter.process(data)
        
        # Estimate understeer gradient
        Kv = identifier.estimate_understeer_gradient(data)
        
        return {
            'Kv': Kv,
            'data_points': len(data),
            'speed_range': (data['vehicle_speed'].min(), data['vehicle_speed'].max())
        }
        
    def _process_step_response(self, data: pd.DataFrame) -> Dict:
        """Process step steer response data."""
        from dynamic_model_id import DynamicModelIdentifier
        
        identifier = DynamicModelIdentifier(self.vehicle_params)
        
        # Map and filter
        data = self.mapper.map_signals(data)
        data = self.filter.process(data)
        
        # Identify parameters
        params = identifier.identify_from_step_response(data)
        
        return params
        
    def _process_frequency_response(self, data: pd.DataFrame) -> Dict:
        """Process sine sweep data."""
        from dynamic_model_id import DynamicModelIdentifier
        
        identifier = DynamicModelIdentifier(self.vehicle_params)
        
        # Map and filter
        data = self.mapper.map_signals(data)
        data = self.filter.process(data)
        
        # Identify parameters
        params = identifier.identify_from_frequency_response(data)
        
        return params
        
    def _combine_results(self, results: Dict):
        """Combine results from different identification methods."""
        # Extract parameters
        if 'steady_state' in results:
            self.identified_params['Kv'] = results['steady_state'].get('Kv', 0)
            
        if 'step_response' in results:
            self.identified_params['Iz'] = results['step_response'].get('Iz', 0)
            self.identified_params['Caf'] = results['step_response'].get('Caf', 0)
            self.identified_params['Car'] = results['step_response'].get('Car', 0)
            
        if 'frequency_response' in results:
            self.identified_params['Iz_freq'] = results['frequency_response'].get('Iz_freq', 0)
            self.identified_params['Caf_freq'] = results['frequency_response'].get('Caf_freq', 0)
            self.identified_params['Car_freq'] = results['frequency_response'].get('Car_freq', 0)
            
        # Calculate average if multiple estimates available
        if 'Iz' in self.identified_params and 'Iz_freq' in self.identified_params:
            self.identified_params['Iz_combined'] = (
                self.identified_params['Iz'] + self.identified_params['Iz_freq']
            ) / 2
            
        logger.info("Combined identification results:")
        for key, value in self.identified_params.items():
            logger.info(f"  {key}: {value:.2f}")
            
    def _validate_model(self, data: pd.DataFrame):
        """Validate identified model."""
        # Split data for validation
        train_data, test_data = self.aggregator.split_train_test(test_ratio=0.2)
        
        if len(test_data) > 0:
            # Calculate validation metrics
            # This would use the actual model to predict and compare
            self.validation_results = {
                'test_samples': len(test_data),
                'rmse_yaw_rate': 0.0,  # Would be calculated
                'rmse_lateral_accel': 0.0  # Would be calculated
            }
            
    def save_results(self, output_file: str):
        """
        Save identification results to YAML file.
        
        Args:
            output_file: Path to output YAML file
        """
        results = {
            'timestamp': datetime.now().isoformat(),
            'vehicle_parameters': self.vehicle_params,
            'identified_parameters': self.identified_params,
            'quality_metrics': self.quality_metrics,
            'validation_results': self.validation_results,
            'metadata': self.aggregator.get_metadata()
        }
        
        # Convert numpy types to Python types for YAML
        def convert_types(obj):
            if isinstance(obj, np.ndarray):
                return obj.tolist()
            elif isinstance(obj, (np.int64, np.int32)):
                return int(obj)
            elif isinstance(obj, (np.float64, np.float32)):
                return float(obj)
            elif isinstance(obj, dict):
                return {k: convert_types(v) for k, v in obj.items()}
            elif isinstance(obj, list):
                return [convert_types(item) for item in obj]
            return obj
            
        results = convert_types(results)
        
        with open(output_file, 'w') as f:
            yaml.dump(results, f, default_flow_style=False)
            
        logger.info(f"Results saved to {output_file}")
        
    def plot_results(self, save_path: Optional[str] = None):
        """
        Generate plots of identification results.
        
        Args:
            save_path: Optional path to save plots
        """
        # This would generate various plots
        # For now, just log
        logger.info("Plotting results...")
        if save_path:
            logger.info(f"Plots would be saved to {save_path}")


def main():
    """Main entry point for CLI."""
    parser = argparse.ArgumentParser(
        description='Integrated Lateral System Identification Tool'
    )
    
    # Required arguments
    parser.add_argument(
        'data_dir',
        help='Directory containing CSV data files'
    )
    
    # Optional arguments
    parser.add_argument(
        '--output', '-o',
        default='identified_parameters.yaml',
        help='Output YAML file (default: identified_parameters.yaml)'
    )
    
    # Vehicle parameters
    parser.add_argument(
        '--mass', '-m',
        type=float,
        default=1901.0,
        help='Vehicle mass in kg (default: 1901)'
    )
    
    parser.add_argument(
        '--lf',
        type=float,
        default=1.7,
        help='Front axle to CG distance in meters (default: 1.7)'
    )
    
    parser.add_argument(
        '--lr',
        type=float,
        default=1.3,
        help='Rear axle to CG distance in meters (default: 1.3)'
    )
    
    # Processing options
    parser.add_argument(
        '--scenario',
        choices=['all', 'steady_state', 'step_steer', 'sine_sweep'],
        default='all',
        help='Scenario type to process (default: all)'
    )
    
    parser.add_argument(
        '--plot',
        action='store_true',
        help='Generate plots'
    )
    
    parser.add_argument(
        '--save-plots',
        help='Directory to save plots'
    )
    
    parser.add_argument(
        '--verbose', '-v',
        action='store_true',
        help='Verbose output'
    )
    
    args = parser.parse_args()
    
    # Set logging level
    if args.verbose:
        logging.getLogger().setLevel(logging.DEBUG)
        
    # Check data directory exists
    if not os.path.exists(args.data_dir):
        logger.error(f"Data directory not found: {args.data_dir}")
        return 1
        
    # Create vehicle parameters
    vehicle_params = {
        'm': args.mass,
        'lf': args.lf,
        'lr': args.lr,
        'L': args.lf + args.lr
    }
    
    # Create identifier
    identifier = IntegratedSystemIdentifier(vehicle_params)
    
    # Process data
    try:
        results = identifier.process_directory(args.data_dir)
        
        if results:
            # Save results
            identifier.save_results(args.output)
            
            # Generate plots if requested
            if args.plot or args.save_plots:
                identifier.plot_results(args.save_plots)
                
            logger.info("Identification completed successfully")
            return 0
        else:
            logger.error("Identification failed")
            return 1
            
    except Exception as e:
        logger.error(f"Error during identification: {str(e)}")
        if args.verbose:
            import traceback
            traceback.print_exc()
        return 1


if __name__ == '__main__':
    sys.exit(main())