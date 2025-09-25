#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Step steering scenario for lateral system identification."""

import numpy as np
from typing import Tuple, Optional, Dict, Any
from .base import BaseScenario

# Conversion constants
DEG_TO_RAD = np.pi / 180.0  # Degree to radian conversion
MAX_STEER_DEG = 40.0  # Maximum steering angle in degrees
MAX_STEER_RAD = MAX_STEER_DEG * DEG_TO_RAD  # Maximum steering angle in radians


class StepSteerScenario(BaseScenario):
    """Step steering input scenario for transient response analysis.
    
    Applies step changes in steering angle to measure dynamic response
    characteristics like yaw rate rise time and settling behavior.
    """
    
    def setup(self) -> None:
        """Set up step steering parameters."""
        # Get parameters with defaults
        self.initial_straight_duration = self.params.get('initial_straight_duration', 5.0)
        self.step_duration = self.params.get('step_duration', 5.0)  # Duration to hold each step
        self.recovery_duration = self.params.get('recovery_duration', 5.0)  # Return to straight
        
        # Step steering angles (in radians)
        self.step_angles = self.params.get('step_angles', 
                                          np.radians([10, -10, 15, -15]))  # Positive and negative
        
        # Step transition time (how fast to apply the step)
        self.step_rise_time = self.params.get('step_rise_time', 0.2)  # 200ms step
        
        # Calculate total duration
        self.total_duration = (self.initial_straight_duration + 
                              len(self.step_angles) * (self.step_duration + self.recovery_duration))
        
        # Log initialization
        import rospy
        rospy.loginfo("Step steer scenario initialized with %d steps", len(self.step_angles))
        rospy.loginfo("Step angles (deg): %s", [np.degrees(a) for a in self.step_angles])
    
    def get_control_command(self, elapsed_time: float) -> Tuple[float, float, float]:
        """Get control commands for step steering.
        
        Args:
            elapsed_time: Time since scenario start
            
        Returns:
            Tuple of (accel, brake, steer) commands
        """
        accel, brake, steer = 0.0, 0.0, 0.0
        
        # Initial straight driving
        if elapsed_time < self.initial_straight_duration:
            steer = 0.0
        else:
            # Time after initial straight
            test_time = elapsed_time - self.initial_straight_duration
            
            # Calculate which step we're in
            cycle_duration = self.step_duration + self.recovery_duration
            step_idx = int(test_time / cycle_duration)
            time_in_cycle = test_time % cycle_duration
            
            if step_idx < len(self.step_angles):
                self.current_step = step_idx
                target_angle = self.step_angles[step_idx]
                
                if time_in_cycle < self.step_rise_time:
                    # Rising edge of step
                    progress = min(1.0, time_in_cycle / self.step_rise_time)
                    steer = target_angle * progress
                elif time_in_cycle < self.step_duration:
                    # Hold step value
                    steer = target_angle
                elif time_in_cycle < self.step_duration + self.step_rise_time:
                    # Falling edge (return to zero)
                    progress = (time_in_cycle - self.step_duration) / self.step_rise_time
                    steer = target_angle * (1.0 - progress)
                else:
                    # Recovery period (straight)
                    steer = 0.0
        
        # Return steering angle in radians (MORAI uses radians directly)
        # MORAI uses standard convention: + for left (CCW), - for right (CW)
        steer_rad = np.clip(steer, -MAX_STEER_RAD, MAX_STEER_RAD)
        
        return accel, brake, steer_rad
    
    def get_total_duration(self) -> float:
        """Get total duration of scenario.
        
        Returns:
            Total duration in seconds
        """
        return self.total_duration
    
    def get_status_string(self) -> str:
        """Get human-readable status string.
        
        Returns:
            Status description
        """
        if not self.is_started:
            return "Not started"
        elif self.is_finished:
            return "Completed"
        else:
            elapsed = self.get_elapsed_time()
            if elapsed < self.initial_straight_duration:
                return f"Initial straight - {elapsed:.1f}s"
            elif self.current_step < len(self.step_angles):
                current_angle = np.degrees(self.step_angles[self.current_step])
                return f"Step {self.current_step+1}/{len(self.step_angles)} ({current_angle:.1f}°)"
            return f"Running - {elapsed:.1f}s"