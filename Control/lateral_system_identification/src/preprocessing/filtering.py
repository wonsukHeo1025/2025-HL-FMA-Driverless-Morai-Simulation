#!/usr/bin/env python3
"""Signal filtering and resampling."""

import pandas as pd
import numpy as np
from scipy import signal
from typing import Dict, Optional, Tuple
import logging

logger = logging.getLogger(__name__)


class SignalFilter:
    """Filter and resample signals."""
    
    DEFAULT_CONFIG = {
        'speed_threshold': 0.5,         # [m/s] minimum speed
        'max_steering': 0.35,           # [rad] max steering angle for linear region
        'max_lateral_accel': 4.0,       # [m/s²] max lateral acceleration
        'lowpass_cutoff': 10.0,         # [Hz] lowpass filter cutoff
        'resample_rate': 50.0,          # [Hz] target sampling rate
        'filter_order': 2                # Butterworth filter order
    }
    
    def __init__(self, config: Optional[Dict] = None):
        """
        Initialize signal filter.
        
        Args:
            config: Optional filter configuration
        """
        self.config = self.DEFAULT_CONFIG.copy()
        if config:
            self.config.update(config)
            
    def process(self, df: pd.DataFrame) -> pd.DataFrame:
        """
        Apply all filtering and preprocessing steps.
        
        Args:
            df: Input DataFrame
            
        Returns:
            Filtered DataFrame
        """
        df_filtered = df.copy()
        
        # 1. Remove low speed data
        df_filtered = self._filter_speed(df_filtered)
        
        # 2. Filter to linear region
        df_filtered = self._filter_linear_region(df_filtered)
        
        # 3. Remove outliers
        df_filtered = self._remove_outliers(df_filtered)
        
        # 4. Apply lowpass filter
        df_filtered = self._apply_lowpass_filter(df_filtered)
        
        # 5. Resample if needed
        df_filtered = self._resample_uniform(df_filtered)
        
        logger.info(f"Filtered data: {len(df)} -> {len(df_filtered)} samples")
        
        return df_filtered
        
    def _filter_speed(self, df: pd.DataFrame) -> pd.DataFrame:
        """
        Remove data below minimum speed threshold.
        
        Args:
            df: Input DataFrame
            
        Returns:
            Filtered DataFrame
        """
        if 'vehicle_speed' not in df.columns:
            return df
            
        mask = df['vehicle_speed'] >= self.config['speed_threshold']
        filtered = df[mask].copy()
        
        removed = len(df) - len(filtered)
        if removed > 0:
            logger.debug(f"Removed {removed} samples below speed threshold")
            
        return filtered
        
    def _filter_linear_region(self, df: pd.DataFrame) -> pd.DataFrame:
        """
        Filter to linear tire region.
        
        Args:
            df: Input DataFrame
            
        Returns:
            Filtered DataFrame
        """
        mask = pd.Series(True, index=df.index)
        
        # Filter by steering angle
        if 'steering_angle' in df.columns:
            mask &= np.abs(df['steering_angle']) <= self.config['max_steering']
            
        # Filter by lateral acceleration
        if 'lateral_acceleration' in df.columns:
            mask &= np.abs(df['lateral_acceleration']) <= self.config['max_lateral_accel']
            
        filtered = df[mask].copy()
        
        removed = len(df) - len(filtered)
        if removed > 0:
            logger.debug(f"Removed {removed} samples outside linear region")
            
        return filtered
        
    def _remove_outliers(self, df: pd.DataFrame, n_sigma: float = 3.0) -> pd.DataFrame:
        """
        Remove statistical outliers using n-sigma rule.
        
        Args:
            df: Input DataFrame
            n_sigma: Number of standard deviations for outlier detection
            
        Returns:
            Filtered DataFrame
        """
        signals_to_check = ['yaw_rate', 'lateral_acceleration', 'steering_angle']
        
        mask = pd.Series(True, index=df.index)
        
        for signal_name in signals_to_check:
            if signal_name in df.columns:
                # Calculate statistics
                mean = df[signal_name].mean()
                std = df[signal_name].std()
                
                # Mark outliers
                outlier_mask = np.abs(df[signal_name] - mean) > n_sigma * std
                mask &= ~outlier_mask
                
                n_outliers = outlier_mask.sum()
                if n_outliers > 0:
                    logger.debug(f"Found {n_outliers} outliers in {signal_name}")
                    
        filtered = df[mask].copy()
        
        removed = len(df) - len(filtered)
        if removed > 0:
            logger.info(f"Removed {removed} outlier samples")
            
        return filtered
        
    def _apply_lowpass_filter(self, df: pd.DataFrame) -> pd.DataFrame:
        """
        Apply lowpass Butterworth filter to signals.
        
        Args:
            df: Input DataFrame
            
        Returns:
            Filtered DataFrame
        """
        # Signals to filter
        signals_to_filter = ['yaw_rate', 'lateral_acceleration']
        
        # Estimate sampling frequency
        dt = df['timestamp'].diff().median()
        if dt <= 0:
            logger.warning("Invalid timestamp data, skipping lowpass filter")
            return df
            
        fs = 1.0 / dt
        
        # Design filter
        nyquist = fs / 2
        if self.config['lowpass_cutoff'] >= nyquist:
            logger.warning(f"Cutoff frequency {self.config['lowpass_cutoff']} >= Nyquist {nyquist}")
            return df
            
        sos = signal.butter(
            self.config['filter_order'],
            self.config['lowpass_cutoff'],
            btype='low',
            fs=fs,
            output='sos'
        )
        
        df_filtered = df.copy()
        
        for signal_name in signals_to_filter:
            if signal_name in df.columns:
                # Apply filter (forward-backward for zero phase)
                filtered_signal = signal.sosfiltfilt(sos, df[signal_name].values)
                df_filtered[signal_name] = filtered_signal
                logger.debug(f"Applied lowpass filter to {signal_name}")
                
        return df_filtered
        
    def _resample_uniform(self, df: pd.DataFrame) -> pd.DataFrame:
        """
        Resample to uniform sampling rate if needed.
        
        Args:
            df: Input DataFrame
            
        Returns:
            Resampled DataFrame
        """
        # Check current sampling rate
        dt = df['timestamp'].diff().median()
        if dt <= 0:
            return df
            
        current_fs = 1.0 / dt
        target_fs = self.config['resample_rate']
        
        # Only resample if significantly different (>10% difference)
        if abs(current_fs - target_fs) / target_fs < 0.1:
            logger.debug(f"Sampling rate {current_fs:.1f} Hz close to target, skipping resample")
            return df
            
        logger.info(f"Resampling from {current_fs:.1f} Hz to {target_fs:.1f} Hz")
        
        # Create uniform time grid
        t_start = df['timestamp'].min()
        t_end = df['timestamp'].max()
        dt_new = 1.0 / target_fs
        t_uniform = np.arange(t_start, t_end, dt_new)
        
        # Interpolate all numeric columns
        df_resampled = pd.DataFrame({'timestamp': t_uniform})
        
        for col in df.columns:
            if col == 'timestamp':
                continue
                
            if pd.api.types.is_numeric_dtype(df[col]):
                # Linear interpolation for numeric data
                interp_values = np.interp(t_uniform, df['timestamp'].values, df[col].values)
                df_resampled[col] = interp_values
            else:
                # Forward fill for non-numeric data
                # Create a temporary series with uniform index
                temp_series = pd.Series(df[col].values, index=df['timestamp'].values)
                reindexed = temp_series.reindex(t_uniform, method='ffill')
                df_resampled[col] = reindexed.values
                
        return df_resampled
        
    def hampel_filter(self, data: np.ndarray, window_size: int = 5, 
                     n_sigma: float = 3.0) -> Tuple[np.ndarray, np.ndarray]:
        """
        Apply Hampel filter for outlier detection.
        
        Args:
            data: Input data array
            window_size: Window size for median calculation
            n_sigma: Number of MAD for outlier detection
            
        Returns:
            (filtered_data, outlier_indices)
        """
        n = len(data)
        filtered = data.copy()
        outliers = np.zeros(n, dtype=bool)
        
        for i in range(n):
            # Define window
            start = max(0, i - window_size // 2)
            end = min(n, i + window_size // 2 + 1)
            window = data[start:end]
            
            # Calculate median and MAD
            median = np.median(window)
            mad = np.median(np.abs(window - median))
            
            # Detect outlier
            if np.abs(data[i] - median) > n_sigma * mad * 1.4826:  # MAD to std conversion
                filtered[i] = median
                outliers[i] = True
                
        return filtered, outliers