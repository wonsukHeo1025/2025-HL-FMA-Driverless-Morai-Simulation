#!/usr/bin/env python3
"""Data segmentation for different test scenarios."""

import pandas as pd
import numpy as np
from typing import List, Dict, Optional, Tuple
import logging

logger = logging.getLogger(__name__)


class Segmentation:
    """Segment data for different identification scenarios."""
    
    DEFAULT_CRITERIA = {
        'steady_state': {
            'min_duration': 3.0,        # [s] minimum duration
            'max_yaw_rate_std': 0.05,   # [rad/s] max std deviation
            'max_steering_std': 0.017,   # [rad] max std deviation (1 deg)
            'speed_tolerance': 0.5       # [m/s] speed variation tolerance
        },
        'step': {
            'threshold': 0.035,          # [rad] step detection threshold (2 deg)
            'pre_step_time': 1.0,        # [s] time before step
            'post_step_time': 6.0,       # [s] time after step
            'min_step_size': 0.035,      # [rad] minimum step size (2 deg) - lowered from 5 deg
            'settling_time': 4.0,        # [s] expected settling time
            'speed_adaptive': True       # Enable speed-based dynamic thresholds
        },
        'sweep': {
            'freq_range': [0.1, 2.5],    # [Hz] valid frequency range - extended to 2.5 Hz for 50 km/h
            'min_duration': 30.0,        # [s] minimum sweep duration
            'nperseg': 256,              # FFT window size
            'overlap': 0.5               # Window overlap ratio
        }
    }
    
    def __init__(self, criteria: Optional[Dict] = None):
        """
        Initialize segmentation.
        
        Args:
            criteria: Optional custom segmentation criteria
        """
        self.criteria = self.DEFAULT_CRITERIA.copy()
        if criteria:
            self.criteria.update(criteria)
            
    def extract_steady_state_segments(self, df: pd.DataFrame) -> List[pd.DataFrame]:
        """
        Extract steady-state segments from data.
        
        Args:
            df: Input DataFrame
            
        Returns:
            List of steady-state segments
        """
        segments = []
        criteria = self.criteria['steady_state']
        
        # Use is_steady_state flag if available
        if 'is_steady_state' in df.columns:
            # Group by consecutive steady-state regions
            df['segment_id'] = (df['is_steady_state'] != df['is_steady_state'].shift()).cumsum()
            
            for segment_id, group in df[df['is_steady_state']].groupby('segment_id'):
                duration = group['timestamp'].max() - group['timestamp'].min()
                
                if duration >= criteria['min_duration']:
                    # Additional validation
                    if self._validate_steady_state(group, criteria):
                        segments.append(group.copy())
        else:
            # Detect steady-state regions manually
            segments = self._detect_steady_state_regions(df, criteria)
            
        logger.info(f"Extracted {len(segments)} steady-state segments")
        
        # Add segment metadata
        for i, segment in enumerate(segments):
            segment['segment_number'] = i
            segment['segment_duration'] = segment['timestamp'].max() - segment['timestamp'].min()
            
        return segments
        
    def _validate_steady_state(self, segment: pd.DataFrame, criteria: Dict) -> bool:
        """
        Validate if segment meets steady-state criteria.
        
        Args:
            segment: Data segment
            criteria: Validation criteria
            
        Returns:
            True if valid steady-state
        """
        # Check yaw rate variation
        if 'yaw_rate' in segment.columns:
            yaw_std = segment['yaw_rate'].std()
            if yaw_std > criteria['max_yaw_rate_std']:
                return False
                
        # Check steering variation
        if 'steering_angle' in segment.columns:
            steer_std = segment['steering_angle'].std()
            if steer_std > criteria['max_steering_std']:
                return False
                
        # Check speed variation
        if 'vehicle_speed' in segment.columns:
            speed_range = segment['vehicle_speed'].max() - segment['vehicle_speed'].min()
            if speed_range > criteria['speed_tolerance']:
                return False
                
        return True
        
    def _detect_steady_state_regions(self, df: pd.DataFrame, criteria: Dict) -> List[pd.DataFrame]:
        """
        Automatically detect steady-state regions.
        
        Args:
            df: Input DataFrame
            criteria: Detection criteria
            
        Returns:
            List of detected segments
        """
        segments = []
        
        # Use sliding window to detect steady regions
        window_size = int(criteria['min_duration'] * 50)  # Assuming 50 Hz
        stride = window_size // 2
        
        for i in range(0, len(df) - window_size, stride):
            window = df.iloc[i:i+window_size]
            
            if self._validate_steady_state(window, criteria):
                # Check if overlaps with existing segments
                if not segments or window.index[0] > segments[-1].index[-1]:
                    segments.append(window.copy())
                    
        return segments
        
    def extract_step_segments(self, df: pd.DataFrame) -> List[Dict]:
        """
        Extract step response segments from data.
        
        Args:
            df: Input DataFrame
            
        Returns:
            List of step response segments with metadata
        """
        segments = []
        criteria = self.criteria['step']
        
        if 'steering_angle' not in df.columns:
            logger.warning("No steering_angle column found for step detection")
            return segments
            
        # Detect step changes
        steering = df['steering_angle'].values
        steering_diff = np.diff(steering)
        
        # Find step locations
        step_indices = np.where(np.abs(steering_diff) > criteria['threshold'])[0]
        
        logger.debug(f"Found {len(step_indices)} potential steps")
        
        for idx in step_indices:
            # Check minimum step size
            if idx > 0 and idx < len(steering) - 1:
                step_size = abs(steering[idx+1] - steering[idx-1])
                
                # Speed-adaptive threshold if enabled
                min_step = criteria['min_step_size']
                if criteria.get('speed_adaptive', False) and 'vehicle_speed' in df.columns:
                    speed_kmh = df.iloc[idx]['vehicle_speed'] * 3.6
                    if speed_kmh < 30:
                        min_step = 0.070  # 4 deg for low speed
                    elif speed_kmh > 40:
                        min_step = 0.035  # 2 deg for high speed
                    else:
                        # Linear interpolation between 30-40 km/h
                        min_step = 0.070 - (speed_kmh - 30) * 0.0035
                
                if step_size < min_step:
                    continue
                    
            # Extract segment around step
            pre_samples = int(criteria['pre_step_time'] * 50)  # Assuming 50 Hz
            post_samples = int(criteria['post_step_time'] * 50)
            
            start_idx = max(0, idx - pre_samples)
            end_idx = min(len(df), idx + post_samples)
            
            segment = df.iloc[start_idx:end_idx].copy()
            
            # Add metadata
            segment_info = {
                'data': segment,
                'step_index': idx - start_idx,
                'step_time': df.iloc[idx]['timestamp'],
                'step_magnitude': steering_diff[idx] if idx < len(steering_diff) else 0,
                'initial_steering': steering[idx-1] if idx > 0 else steering[0],
                'final_steering': steering[idx+1] if idx < len(steering)-1 else steering[-1],
                'vehicle_speed': segment['vehicle_speed'].mean() if 'vehicle_speed' in segment.columns else 0
            }
            
            segments.append(segment_info)
            
        logger.info(f"Extracted {len(segments)} step response segments")
        
        return segments
        
    def extract_sweep_segments(self, df: pd.DataFrame) -> List[pd.DataFrame]:
        """
        Extract frequency sweep segments from data.
        
        Args:
            df: Input DataFrame
            
        Returns:
            List of sweep segments
        """
        segments = []
        criteria = self.criteria['sweep']
        
        # Look for continuous sweep periods
        if 'scenario_type' in df.columns:
            # Group by scenario type
            sweep_data = df[df['scenario_type'] == 'sine_sweep']
            
            if len(sweep_data) > 0:
                # Split into continuous segments
                time_gaps = np.diff(sweep_data['timestamp'])
                gap_indices = np.where(time_gaps > 1.0)[0]  # 1 second gap
                
                start_idx = 0
                for gap_idx in gap_indices:
                    segment = sweep_data.iloc[start_idx:gap_idx+1]
                    if self._validate_sweep_segment(segment, criteria):
                        segments.append(segment.copy())
                    start_idx = gap_idx + 1
                    
                # Add last segment
                if start_idx < len(sweep_data):
                    segment = sweep_data.iloc[start_idx:]
                    if self._validate_sweep_segment(segment, criteria):
                        segments.append(segment.copy())
        else:
            # Try to detect sweep patterns
            segments = self._detect_sweep_patterns(df, criteria)
            
        logger.info(f"Extracted {len(segments)} sweep segments")
        
        return segments
        
    def _validate_sweep_segment(self, segment: pd.DataFrame, criteria: Dict) -> bool:
        """
        Validate if segment is a valid frequency sweep.
        
        Args:
            segment: Data segment
            criteria: Validation criteria
            
        Returns:
            True if valid sweep
        """
        # Check duration
        duration = segment['timestamp'].max() - segment['timestamp'].min()
        if duration < criteria['min_duration']:
            return False
            
        # Could add frequency content validation here
        
        return True
        
    def _detect_sweep_patterns(self, df: pd.DataFrame, criteria: Dict) -> List[pd.DataFrame]:
        """
        Detect sinusoidal sweep patterns in data.
        
        Args:
            df: Input DataFrame
            criteria: Detection criteria
            
        Returns:
            List of detected sweep segments
        """
        segments = []
        
        if 'steering_angle' not in df.columns:
            return segments
            
        # Use FFT to detect periodic content
        # This is a simplified implementation
        window_size = int(criteria['min_duration'] * 50)  # Assuming 50 Hz
        
        for i in range(0, len(df) - window_size, window_size // 2):
            window = df.iloc[i:i+window_size]
            
            # Check if contains sinusoidal content
            steering = window['steering_angle'].values
            
            # Simple check: variance should be significant
            if np.std(steering) > 0.05:  # More than ~3 degrees variation
                # Could add more sophisticated frequency analysis here
                segments.append(window.copy())
                
        return segments
        
    def get_segment_statistics(self, segments: List[pd.DataFrame]) -> Dict:
        """
        Calculate statistics for extracted segments.
        
        Args:
            segments: List of data segments
            
        Returns:
            Dictionary with segment statistics
        """
        if not segments:
            return {}
            
        stats = {
            'n_segments': len(segments),
            'total_duration': sum(
                seg['timestamp'].max() - seg['timestamp'].min() 
                for seg in segments
            ),
            'avg_duration': np.mean([
                seg['timestamp'].max() - seg['timestamp'].min() 
                for seg in segments
            ]),
            'total_samples': sum(len(seg) for seg in segments)
        }
        
        # Add per-segment statistics
        segment_stats = []
        for i, seg in enumerate(segments):
            seg_stat = {
                'segment_id': i,
                'duration': seg['timestamp'].max() - seg['timestamp'].min(),
                'samples': len(seg)
            }
            
            if 'vehicle_speed' in seg.columns:
                seg_stat['avg_speed'] = seg['vehicle_speed'].mean()
                seg_stat['speed_range'] = (
                    seg['vehicle_speed'].min(),
                    seg['vehicle_speed'].max()
                )
                
            if 'steering_angle' in seg.columns:
                seg_stat['steering_range'] = (
                    np.degrees(seg['steering_angle'].min()),
                    np.degrees(seg['steering_angle'].max())
                )
                
            segment_stats.append(seg_stat)
            
        stats['segments'] = segment_stats
        
        return stats