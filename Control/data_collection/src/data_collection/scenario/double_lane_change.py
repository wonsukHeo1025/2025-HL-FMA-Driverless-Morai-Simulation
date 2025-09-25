#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Double lane change scenario for model validation."""

import numpy as np
from typing import Tuple, Optional, Dict, Any
from .base import BaseScenario

# Conversion constants
DEG_TO_RAD = np.pi / 180.0  # Degree to radian conversion
MAX_STEER_DEG = 40.0  # Maximum steering angle in degrees
MAX_STEER_RAD = MAX_STEER_DEG * DEG_TO_RAD  # Maximum steering angle in radians


class DoubleLaneChangeScenario(BaseScenario):
    """Double lane change maneuver for model validation.
    
    Implements ISO 3888-2 style double lane change maneuver
    to validate lateral dynamics model in realistic driving scenarios.
    """
    
    def setup(self) -> None:
        """Set up double lane change parameters."""
        # Get parameters with defaults
        self.initial_straight_duration = self.params.get('initial_straight_duration', 3.0)
        
        # Lane change parameters
        self.lane_width = self.params.get('lane_width', 3.5)  # meters
        self.maneuver_distance = self.params.get('maneuver_distance', 50.0)  # meters
        self.target_speed = self.params.get('target_speed', 13.89)  # m/s (50 km/h default)
        
        # Calculate maneuver duration based on speed
        self.maneuver_duration = self.maneuver_distance / self.target_speed
        
        # Define waypoints for the maneuver (relative positions)
        # Format: (longitudinal_distance, lateral_offset)
        self.waypoints = [
            (0, 0),                                    # Start in center lane
            (self.maneuver_distance * 0.25, self.lane_width),   # First lane change
            (self.maneuver_distance * 0.5, self.lane_width),    # Hold left lane
            (self.maneuver_distance * 0.75, 0),                 # Return to center
            (self.maneuver_distance, 0)                         # End in center lane
        ]
        
        # Steering controller gains (simple proportional control for tracking)
        self.k_p = self.params.get('k_p', 0.5)  # Proportional gain
        self.lookahead_distance = self.params.get('lookahead_distance', 5.0)  # meters
        
        # Calculate total duration
        self.total_duration = self.initial_straight_duration + self.maneuver_duration + 3.0
        
        # Log initialization
        import rospy
        rospy.loginfo("Double lane change scenario initialized:")
        rospy.loginfo("  Lane width: %.1f m", self.lane_width)
        rospy.loginfo("  Maneuver distance: %.1f m", self.maneuver_distance)
        rospy.loginfo("  Target speed: %.1f km/h", self.target_speed * 3.6)
    
    def get_control_command(self, elapsed_time: float) -> Tuple[float, float, float]:
        """Get control commands for double lane change.
        
        Args:
            elapsed_time: Time since scenario start
            
        Returns:
            Tuple of (accel, brake, steer) commands
        """
        accel, brake, steer = 0.0, 0.0, 0.0
        
        if elapsed_time < self.initial_straight_duration:
            # Initial straight section
            steer = 0.0
            self.current_step = 0
        elif elapsed_time < self.initial_straight_duration + self.maneuver_duration:
            # Double lane change maneuver
            maneuver_time = elapsed_time - self.initial_straight_duration
            self.current_step = 1
            
            # Current longitudinal position (assuming constant speed)
            current_x = self.target_speed * maneuver_time
            
            # Find target lateral position by interpolating waypoints
            target_y = self._interpolate_lateral_position(current_x)
            
            # Simple steering control based on lateral error
            # This is a simplified controller - real implementation would use
            # proper path following algorithms
            lateral_error = target_y  # Assuming vehicle starts at y=0
            
            # Look ahead for smoother control
            lookahead_x = min(current_x + self.lookahead_distance, self.maneuver_distance)
            lookahead_y = self._interpolate_lateral_position(lookahead_x)
            
            # Calculate required steering angle (simplified)
            # Using bicycle model approximation: δ = atan(L * κ)
            # where κ is the path curvature
            dx = self.lookahead_distance
            dy = lookahead_y - target_y
            
            # Approximate steering angle
            if dx > 0:
                heading_error = np.arctan2(dy, dx)
                steer = self.k_p * heading_error + 0.2 * lateral_error / self.lane_width
            else:
                steer = 0.0
            
            # Limit steering angle (15 degrees for lane change maneuver)
            steer = np.clip(steer, -15 * DEG_TO_RAD, 15 * DEG_TO_RAD)
            
        else:
            # Final straight section
            steer = 0.0
            self.current_step = 2
        
        # Return steering angle in radians (MORAI uses radians directly)
        # MORAI uses standard convention: + for left (CCW), - for right (CW)
        steer_rad = np.clip(steer, -MAX_STEER_RAD, MAX_STEER_RAD)
        
        return accel, brake, steer_rad
    
    def _interpolate_lateral_position(self, x: float) -> float:
        """Interpolate lateral position from waypoints.
        
        Args:
            x: Longitudinal position
            
        Returns:
            Lateral position
        """
        # Find surrounding waypoints
        for i in range(len(self.waypoints) - 1):
            x1, y1 = self.waypoints[i]
            x2, y2 = self.waypoints[i + 1]
            
            if x1 <= x <= x2:
                # Linear interpolation
                if x2 - x1 > 0:
                    t = (x - x1) / (x2 - x1)
                    # Use smooth interpolation (cubic)
                    t_smooth = t * t * (3 - 2 * t)  # Smoothstep function
                    return y1 + (y2 - y1) * t_smooth
                else:
                    return y1
        
        # Beyond waypoints
        if x < self.waypoints[0][0]:
            return self.waypoints[0][1]
        else:
            return self.waypoints[-1][1]
    
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
            elif self.current_step == 1:
                maneuver_progress = ((elapsed - self.initial_straight_duration) / 
                                   self.maneuver_duration * 100)
                return f"Lane change maneuver - {maneuver_progress:.1f}%"
            else:
                return f"Final straight - {elapsed:.1f}s"