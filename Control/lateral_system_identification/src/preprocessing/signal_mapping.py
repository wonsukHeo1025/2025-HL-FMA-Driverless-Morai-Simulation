#!/usr/bin/env python3
"""Signal mapping and unit conversion."""

import pandas as pd
import numpy as np
from typing import Dict, Optional
import logging

logger = logging.getLogger(__name__)


class SignalMapper:
    """Map and convert signals to standard units."""
    
    # Standard signal mappings
    SIGNAL_MAPPINGS = {
        'steering_angle': {
            'source': 'steering_angle_deg',
            'conversion': lambda x: np.radians(x),  # deg to rad
            'unit': 'rad'
        },
        'vehicle_speed': {
            'source': 'true_velocity_x',
            'conversion': lambda x: x,  # already in m/s
            'unit': 'm/s'
        },
        'lateral_acceleration': {
            'source': 'imu_accel_y',
            'conversion': lambda x: x,  # already in m/s²
            'unit': 'm/s²'
        },
        'yaw_rate': {
            'source': 'yaw_rate',
            'conversion': lambda x: x,  # already in rad/s
            'unit': 'rad/s'
        },
        'longitudinal_acceleration': {
            'source': 'imu_accel_x',
            'conversion': lambda x: x,  # already in m/s²
            'unit': 'm/s²'
        }
    }
    
    def __init__(self, custom_mappings: Optional[Dict] = None):
        """
        Initialize signal mapper.
        
        Args:
            custom_mappings: Optional custom signal mappings
        """
        self.mappings = self.SIGNAL_MAPPINGS.copy()
        if custom_mappings:
            self.mappings.update(custom_mappings)
            
    def map_signals(self, df: pd.DataFrame) -> pd.DataFrame:
        """
        Map and convert signals to standard format.
        
        Args:
            df: Input DataFrame
            
        Returns:
            DataFrame with mapped signals
        """
        df_mapped = df.copy()
        
        for signal_name, mapping in self.mappings.items():
            source_col = mapping['source']
            
            if source_col in df.columns:
                # Apply conversion
                df_mapped[signal_name] = mapping['conversion'](df[source_col])
                logger.debug(f"Mapped {source_col} -> {signal_name} ({mapping['unit']})")
            else:
                logger.warning(f"Source column {source_col} not found for {signal_name}")
                
        # Add derived signals
        df_mapped = self._add_derived_signals(df_mapped)
        
        return df_mapped
        
    def _add_derived_signals(self, df: pd.DataFrame) -> pd.DataFrame:
        """
        Add derived signals.
        
        Args:
            df: Input DataFrame
            
        Returns:
            DataFrame with derived signals
        """
        # Add sideslip angle if lateral velocity available
        if 'estimated_velocity_y' in df.columns and 'vehicle_speed' in df.columns:
            # Avoid division by zero
            mask = df['vehicle_speed'] > 0.5
            df.loc[mask, 'sideslip_angle'] = np.arctan2(
                df.loc[mask, 'estimated_velocity_y'],
                df.loc[mask, 'vehicle_speed']
            )
            logger.debug("Added sideslip angle calculation")
            
        # Add curvature from yaw rate and speed
        if 'yaw_rate' in df.columns and 'vehicle_speed' in df.columns:
            mask = df['vehicle_speed'] > 0.5
            df.loc[mask, 'path_curvature'] = (
                df.loc[mask, 'yaw_rate'] / df.loc[mask, 'vehicle_speed']
            )
            logger.debug("Added path curvature calculation")
            
        # Add turning radius
        if 'path_curvature' in df.columns:
            mask = np.abs(df['path_curvature']) > 1e-6
            df.loc[mask, 'turning_radius'] = 1.0 / df.loc[mask, 'path_curvature']
            logger.debug("Added turning radius calculation")
            
        # Calculate accelerations if not available
        if 'vehicle_speed' in df.columns and 'longitudinal_acceleration' not in df.columns:
            # Calculate from velocity derivative
            dt = df['timestamp'].diff().median()
            if dt > 0:
                df['longitudinal_acceleration'] = df['vehicle_speed'].diff() / dt
                # Smooth with rolling mean
                df['longitudinal_acceleration'] = (
                    df['longitudinal_acceleration'].rolling(5, center=True).mean()
                )
                logger.debug("Calculated longitudinal acceleration from velocity")
                
        return df
        
    def validate_units(self, df: pd.DataFrame) -> Dict[str, bool]:
        """
        Validate signal units are in expected ranges.
        
        Args:
            df: DataFrame to validate
            
        Returns:
            Validation results
        """
        validation = {}
        
        # Expected ranges for validation
        expected_ranges = {
            'steering_angle': (-0.6, 0.6),      # rad (~±35 deg)
            'vehicle_speed': (0, 60),            # m/s (0-216 km/h)
            'yaw_rate': (-3, 3),                 # rad/s
            'lateral_acceleration': (-15, 15),    # m/s²
        }
        
        for signal, (min_val, max_val) in expected_ranges.items():
            if signal in df.columns:
                actual_min = df[signal].min()
                actual_max = df[signal].max()
                
                is_valid = (actual_min >= min_val * 0.8 and 
                           actual_max <= max_val * 1.2)
                
                validation[signal] = is_valid
                
                if not is_valid:
                    logger.warning(
                        f"{signal} range [{actual_min:.2f}, {actual_max:.2f}] "
                        f"outside expected [{min_val}, {max_val}]"
                    )
                    
        return validation
        
    def get_signal_info(self) -> Dict:
        """Get information about available signal mappings."""
        info = {}
        for signal, mapping in self.mappings.items():
            info[signal] = {
                'source': mapping['source'],
                'unit': mapping['unit']
            }
        return info