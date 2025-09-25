#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Step acceleration scenario implementation."""

import numpy as np
from typing import Tuple, Optional, Dict, Any
from .base import BaseScenario


class StepAccelScenario(BaseScenario):
    """Step acceleration test scenario.
    
    Applies incremental acceleration steps with rest periods between.
    """
    
    def setup(self) -> None:
        """Set up step acceleration parameters."""
        # Get parameters with defaults
        self.step_duration = self.params.get('step_duration', 10.0)
        self.rest_duration = self.params.get('rest_duration', 5.0)
        self.accel_levels = self.params.get('accel_levels', 
                                           np.arange(0.2, 1.2, 0.2))
        
        # Calculate total duration
        self.total_duration = len(self.accel_levels) * (
            self.step_duration + self.rest_duration
        )
        
        # Log initialization
        import rospy
        rospy.loginfo("Step acceleration scenario initialized with %d steps",
                     len(self.accel_levels))
    
    def get_control_command(self, elapsed_time: float) -> Tuple[float, float, float]:
        """Get control commands for step acceleration.
        
        Args:
            elapsed_time: Time since scenario start
            
        Returns:
            Tuple of (accel, brake, steer) commands
        """
        accel, brake, steer = 0.0, 0.0, 0.0
        
        # Calculate current step
        step_cycle = self.step_duration + self.rest_duration
        step_idx = int(elapsed_time / step_cycle)
        time_in_step = elapsed_time % step_cycle
        
        if step_idx < len(self.accel_levels):
            self.current_step = step_idx
            
            if time_in_step < self.step_duration:
                # Acceleration phase
                accel = self.accel_levels[step_idx]
            # else: Rest phase (accel = 0)
        
        return accel, brake, steer
    
    def get_total_duration(self) -> float:
        """Get total duration of scenario.
        
        Returns:
            Total duration in seconds
        """
        return self.total_duration