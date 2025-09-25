#!/usr/bin/env python3
"""CSV data loader with validation."""

import os
import pandas as pd
import numpy as np
from typing import Dict, List, Optional, Tuple
import logging

logger = logging.getLogger(__name__)


class CSVLoader:
    """Load and validate CSV data files."""
    
    # Required columns for system identification
    REQUIRED_COLUMNS = {
        'timestamp': float,
        'steer_cmd': float,
        'steering_angle_deg': float,
        'true_velocity_x': float,
        'imu_accel_y': float,
        'yaw_rate': float,
        'scenario_type': str,
        'scenario_step': int,
        'scenario_time': float,
        'is_steady_state': bool
    }
    
    # Optional columns
    OPTIONAL_COLUMNS = {
        'x_position': float,
        'y_position': float,
        'gps_latitude': float,
        'gps_longitude': float,
        'estimated_velocity_y': float,
        'imu_accel_x': float,
        'imu_accel_z': float
    }
    
    def __init__(self):
        """Initialize CSV loader."""
        self.validation_report = {}
        
    def load_file(self, filepath: str) -> Optional[pd.DataFrame]:
        """
        Load a single CSV file with validation.
        
        Args:
            filepath: Path to CSV file
            
        Returns:
            Loaded DataFrame or None if validation fails
        """
        if not os.path.exists(filepath):
            logger.error(f"File not found: {filepath}")
            return None
            
        try:
            # Load CSV
            df = pd.read_csv(filepath)
            logger.info(f"Loaded {len(df)} rows from {os.path.basename(filepath)}")
            
            # Validate columns
            is_valid, report = self._validate_columns(df)
            self.validation_report[filepath] = report
            
            if not is_valid:
                logger.error(f"Column validation failed for {filepath}")
                return None
                
            # Validate data quality
            df = self._validate_data_quality(df)
            
            # Add metadata
            df['source_file'] = os.path.basename(filepath)
            
            return df
            
        except Exception as e:
            logger.error(f"Error loading {filepath}: {str(e)}")
            return None
            
    def _validate_columns(self, df: pd.DataFrame) -> Tuple[bool, Dict]:
        """
        Validate required columns exist.
        
        Args:
            df: DataFrame to validate
            
        Returns:
            (is_valid, validation_report)
        """
        report = {
            'missing_required': [],
            'missing_optional': [],
            'extra_columns': []
        }
        
        # Check required columns
        for col, dtype in self.REQUIRED_COLUMNS.items():
            if col not in df.columns:
                report['missing_required'].append(col)
                
        # Check optional columns
        for col in self.OPTIONAL_COLUMNS:
            if col not in df.columns:
                report['missing_optional'].append(col)
                
        # Find extra columns
        expected = set(self.REQUIRED_COLUMNS.keys()) | set(self.OPTIONAL_COLUMNS.keys())
        actual = set(df.columns)
        report['extra_columns'] = list(actual - expected - {'source_file'})
        
        is_valid = len(report['missing_required']) == 0
        
        if not is_valid:
            logger.warning(f"Missing required columns: {report['missing_required']}")
            
        return is_valid, report
        
    def _validate_data_quality(self, df: pd.DataFrame) -> pd.DataFrame:
        """
        Validate and clean data quality issues.
        
        Args:
            df: DataFrame to validate
            
        Returns:
            Cleaned DataFrame
        """
        initial_len = len(df)
        
        # Remove rows with NaN in required columns
        required_cols = [col for col in self.REQUIRED_COLUMNS.keys() if col in df.columns]
        df = df.dropna(subset=required_cols)
        
        if len(df) < initial_len:
            logger.info(f"Removed {initial_len - len(df)} rows with NaN values")
            
        # Check timestamp monotonicity
        if not df['timestamp'].is_monotonic_increasing:
            logger.warning("Timestamps are not monotonic, sorting...")
            df = df.sort_values('timestamp')
            
        # Check for duplicate timestamps
        duplicates = df['timestamp'].duplicated().sum()
        if duplicates > 0:
            logger.warning(f"Found {duplicates} duplicate timestamps, removing...")
            df = df.drop_duplicates(subset=['timestamp'], keep='first')
            
        # Validate ranges
        df = self._validate_ranges(df)
        
        return df
        
    def _validate_ranges(self, df: pd.DataFrame) -> pd.DataFrame:
        """
        Validate data ranges and flag outliers.
        
        Args:
            df: DataFrame to validate
            
        Returns:
            DataFrame with outliers flagged
        """
        # Physical limits
        limits = {
            'true_velocity_x': (0, 50),      # m/s (0-180 km/h)
            'steering_angle_deg': (-30, 30),  # degrees
            'yaw_rate': (-2, 2),              # rad/s
            'imu_accel_y': (-10, 10),         # m/s²
        }
        
        df['quality_flag'] = True
        
        for col, (min_val, max_val) in limits.items():
            if col in df.columns:
                outliers = (df[col] < min_val) | (df[col] > max_val)
                if outliers.any():
                    logger.warning(f"Found {outliers.sum()} outliers in {col}")
                    df.loc[outliers, 'quality_flag'] = False
                    
        return df
        
    def load_directory(self, directory: str, pattern: Optional[str] = None) -> List[pd.DataFrame]:
        """
        Load all CSV files from a directory.
        
        Args:
            directory: Directory path
            pattern: Optional filename pattern (e.g., '*steady_state*.csv')
            
        Returns:
            List of loaded DataFrames
        """
        if not os.path.exists(directory):
            logger.error(f"Directory not found: {directory}")
            return []
            
        # Find CSV files
        csv_files = []
        for file in os.listdir(directory):
            if file.endswith('.csv'):
                if pattern is None or self._match_pattern(file, pattern):
                    csv_files.append(os.path.join(directory, file))
                    
        logger.info(f"Found {len(csv_files)} CSV files in {directory}")
        
        # Load each file
        dataframes = []
        for filepath in sorted(csv_files):
            df = self.load_file(filepath)
            if df is not None:
                dataframes.append(df)
                
        return dataframes
        
    def _match_pattern(self, filename: str, pattern: str) -> bool:
        """
        Check if filename matches pattern.
        
        Args:
            filename: Filename to check
            pattern: Pattern with wildcards
            
        Returns:
            True if matches
        """
        import fnmatch
        return fnmatch.fnmatch(filename, pattern)
        
    def get_validation_report(self) -> Dict:
        """Get validation report for all loaded files."""
        return self.validation_report