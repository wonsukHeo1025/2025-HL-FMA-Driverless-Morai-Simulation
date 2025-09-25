#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""PRBS (Pseudo-Random Binary Sequence) scenario implementation."""

import numpy as np
from typing import Tuple, Optional, Dict, Any
from .base import BaseScenario


class PRBSScenario(BaseScenario):
    """PRBS test scenario for system identification.
    
    Generates pseudo-random binary switching between two acceleration levels.
    """
    
    def setup(self) -> None:
        """Set up PRBS parameters."""
        # Get parameters with defaults
        self.duration = self.params.get('duration', 60.0)
        self.low_level = self.params.get('low_level', 0.3)
        self.high_level = self.params.get('high_level', 0.7)
        self.min_switch_time = self.params.get('min_switch_time', 0.5)
        self.max_switch_time = self.params.get('max_switch_time', 2.0)
        
        # Generate PRBS sequence
        self._generate_prbs_sequence()
        
        # Total duration
        self.total_duration = self.duration
        
        # Log initialization
        import rospy
        rospy.loginfo("PRBS scenario initialized with %d switches",
                     len(self.prbs_switches))
    
    def _generate_prbs_sequence(self) -> None:
        """Generate the PRBS switching times."""
        self.prbs_switches = []
        current_time = 0.0
        
        while current_time < self.duration:
            # Random switch time
            switch_time = np.random.uniform(self.min_switch_time, 
                                           self.max_switch_time)
            self.prbs_switches.append(current_time)
            current_time += switch_time
        
        self.prbs_switches = np.array(self.prbs_switches)
    
    def get_control_command(self, elapsed_time: float) -> Tuple[float, float, float]:
        """Get control commands for PRBS.
        
        Args:
            elapsed_time: Time since scenario start
            
        Returns:
            Tuple of (accel, brake, steer) commands
        """
        accel, brake, steer = 0.0, 0.0, 0.0
        
        # Find current segment
        switches_before = self.prbs_switches[self.prbs_switches <= elapsed_time]
        level_idx = len(switches_before)
        
        # Alternate between low and high levels
        if level_idx % 2 == 0:
            accel = self.low_level
        else:
            accel = self.high_level
        
        self.current_step = level_idx
        
        return accel, brake, steer
    
    def get_total_duration(self) -> float:
        """Get total duration of scenario.
        
        Returns:
            Total duration in seconds
        """
        return self.total_duration