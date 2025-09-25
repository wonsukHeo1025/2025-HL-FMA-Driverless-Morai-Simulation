#!/usr/bin/env python3
"""Data aggregator for combining multiple CSV files."""

import pandas as pd
import numpy as np
from typing import List, Dict, Optional, Tuple
import logging

logger = logging.getLogger(__name__)


class DataAggregator:
    """Aggregate and organize data from multiple CSV files."""
    
    def __init__(self):
        """Initialize data aggregator."""
        self.combined_data = None
        self.scenario_data = {}
        self.metadata = {}
        
    def aggregate(self, dataframes: List[pd.DataFrame]) -> pd.DataFrame:
        """
        Aggregate multiple dataframes into a single sorted dataframe.
        
        Args:
            dataframes: List of DataFrames to aggregate
            
        Returns:
            Combined and sorted DataFrame
        """
        if not dataframes:
            logger.error("No dataframes to aggregate")
            return pd.DataFrame()
            
        # Combine all dataframes
        logger.info(f"Aggregating {len(dataframes)} dataframes...")
        self.combined_data = pd.concat(dataframes, ignore_index=True)
        
        # Sort by timestamp
        self.combined_data = self.combined_data.sort_values('timestamp').reset_index(drop=True)
        
        # Calculate time gaps
        self._analyze_time_gaps()
        
        # Group by scenario type
        self._group_by_scenario()
        
        # Store metadata
        self._collect_metadata()
        
        logger.info(f"Aggregated {len(self.combined_data)} total samples")
        
        return self.combined_data
        
    def _analyze_time_gaps(self):
        """Analyze time gaps between consecutive samples."""
        if self.combined_data is None or len(self.combined_data) < 2:
            return
            
        # Calculate time differences
        time_diffs = np.diff(self.combined_data['timestamp'])
        
        # Find large gaps (> 1 second)
        large_gaps = np.where(time_diffs > 1.0)[0]
        
        if len(large_gaps) > 0:
            logger.info(f"Found {len(large_gaps)} time gaps > 1 second")
            
            # Add session markers
            session_id = 0
            self.combined_data['session_id'] = 0
            
            for gap_idx in large_gaps:
                session_id += 1
                self.combined_data.loc[gap_idx+1:, 'session_id'] = session_id
                
            logger.info(f"Data contains {session_id + 1} distinct sessions")
            
    def _group_by_scenario(self):
        """Group data by scenario type."""
        if self.combined_data is None:
            return
            
        # Group by scenario type
        scenario_types = self.combined_data['scenario_type'].unique()
        
        for scenario in scenario_types:
            scenario_data = self.combined_data[
                self.combined_data['scenario_type'] == scenario
            ].copy()
            
            if len(scenario_data) > 0:
                self.scenario_data[scenario] = scenario_data
                logger.info(f"Scenario '{scenario}': {len(scenario_data)} samples")
                
    def _collect_metadata(self):
        """Collect metadata about the aggregated data."""
        if self.combined_data is None:
            return
            
        self.metadata = {
            'total_samples': len(self.combined_data),
            'time_range': (
                self.combined_data['timestamp'].min(),
                self.combined_data['timestamp'].max()
            ),
            'duration_seconds': (
                self.combined_data['timestamp'].max() - 
                self.combined_data['timestamp'].min()
            ),
            'scenarios': list(self.scenario_data.keys()),
            'scenario_counts': {
                scenario: len(data) 
                for scenario, data in self.scenario_data.items()
            },
            'speed_range': (
                self.combined_data['true_velocity_x'].min(),
                self.combined_data['true_velocity_x'].max()
            ),
            'steering_range': (
                self.combined_data['steering_angle_deg'].min(),
                self.combined_data['steering_angle_deg'].max()
            ),
            'source_files': self.combined_data['source_file'].unique().tolist()
                if 'source_file' in self.combined_data.columns else []
        }
        
    def get_scenario_data(self, scenario_type: str) -> Optional[pd.DataFrame]:
        """
        Get data for a specific scenario type.
        
        Args:
            scenario_type: Type of scenario
            
        Returns:
            DataFrame for the scenario or None
        """
        return self.scenario_data.get(scenario_type)
        
    def split_train_test(self, test_ratio: float = 0.2, 
                        random_state: int = 42) -> Tuple[pd.DataFrame, pd.DataFrame]:
        """
        Split data into training and test sets.
        
        Args:
            test_ratio: Ratio of test data
            random_state: Random seed
            
        Returns:
            (train_data, test_data)
        """
        if self.combined_data is None:
            return pd.DataFrame(), pd.DataFrame()
            
        # For time series, use chronological split
        n_samples = len(self.combined_data)
        split_idx = int(n_samples * (1 - test_ratio))
        
        train_data = self.combined_data.iloc[:split_idx].copy()
        test_data = self.combined_data.iloc[split_idx:].copy()
        
        logger.info(f"Split data: {len(train_data)} train, {len(test_data)} test samples")
        
        return train_data, test_data
        
    def get_continuous_segments(self, min_duration: float = 5.0) -> List[pd.DataFrame]:
        """
        Extract continuous data segments.
        
        Args:
            min_duration: Minimum segment duration in seconds
            
        Returns:
            List of continuous segments
        """
        if self.combined_data is None:
            return []
            
        segments = []
        
        if 'session_id' in self.combined_data.columns:
            # Use session markers if available
            for session_id in self.combined_data['session_id'].unique():
                segment = self.combined_data[
                    self.combined_data['session_id'] == session_id
                ].copy()
                
                duration = segment['timestamp'].max() - segment['timestamp'].min()
                if duration >= min_duration:
                    segments.append(segment)
        else:
            # Use time gaps to find segments
            time_diffs = np.diff(self.combined_data['timestamp'])
            gap_indices = np.where(time_diffs > 0.5)[0]  # 0.5 second gap
            
            start_idx = 0
            for gap_idx in gap_indices:
                segment = self.combined_data.iloc[start_idx:gap_idx+1].copy()
                duration = segment['timestamp'].max() - segment['timestamp'].min()
                
                if duration >= min_duration:
                    segments.append(segment)
                    
                start_idx = gap_idx + 1
                
            # Add last segment
            if start_idx < len(self.combined_data):
                segment = self.combined_data.iloc[start_idx:].copy()
                duration = segment['timestamp'].max() - segment['timestamp'].min()
                
                if duration >= min_duration:
                    segments.append(segment)
                    
        logger.info(f"Found {len(segments)} continuous segments >= {min_duration}s")
        
        return segments
        
    def get_metadata(self) -> Dict:
        """Get metadata about aggregated data."""
        return self.metadata
        
    def summary(self) -> str:
        """Get summary string of aggregated data."""
        if self.combined_data is None:
            return "No data aggregated"
            
        summary = [
            "\n=== Data Aggregation Summary ===",
            f"Total samples: {self.metadata['total_samples']}",
            f"Duration: {self.metadata['duration_seconds']:.1f} seconds",
            f"Speed range: {self.metadata['speed_range'][0]:.1f} - {self.metadata['speed_range'][1]:.1f} m/s",
            f"Steering range: {self.metadata['steering_range'][0]:.1f} - {self.metadata['steering_range'][1]:.1f} deg",
            "\nScenarios:"
        ]
        
        for scenario, count in self.metadata['scenario_counts'].items():
            summary.append(f"  - {scenario}: {count} samples")
            
        if self.metadata['source_files']:
            summary.append(f"\nSource files: {len(self.metadata['source_files'])}")
            
        return "\n".join(summary)