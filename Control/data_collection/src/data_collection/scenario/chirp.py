#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Chirp signal scenario implementation."""

import numpy as np
from typing import Tuple, Optional, Dict, Any
from .base import BaseScenario


class ChirpScenario(BaseScenario):
    """Chirp signal test scenario for frequency response analysis.
    
    Generates a sinusoidal signal with linearly increasing frequency.
    """
    
    def setup(self) -> None:
        """Set up chirp signal parameters."""
        # Get parameters with defaults
        self.duration = self.params.get('duration', 60.0)
        self.start_freq = self.params.get('start_freq', 0.1)  # Hz
        self.end_freq = self.params.get('end_freq', 2.0)      # Hz
        self.amplitude = self.params.get('amplitude', 0.3)
        self.offset = self.params.get('offset', 0.4)
        
        # Total duration
        self.total_duration = self.duration
        
        # Log initialization
        import rospy
        rospy.loginfo("Chirp scenario initialized: %.1fHz to %.1fHz over %.1fs",
                     self.start_freq, self.end_freq, self.duration)
    
    def get_control_command(self, elapsed_time: float) -> Tuple[float, float, float]:
        """Get control commands for chirp signal.
        
        Args:
            elapsed_time: Time since scenario start
            
        Returns:
            Tuple of (accel, brake, steer) commands
        """
        accel, brake, steer = 0.0, 0.0, 0.0
        
        if elapsed_time < self.duration:
            # Linearly increasing frequency
            freq = self.start_freq + (self.end_freq - self.start_freq) * (
                elapsed_time / self.duration
            )
            
            # Generate chirp signal
            accel = self.offset + self.amplitude * np.sin(
                2 * np.pi * freq * elapsed_time
            )
            
            # Clip to valid range
            accel = np.clip(accel, 0.0, 1.0)
            
            # Update step (use frequency as indicator)
            self.current_step = int(freq * 10)  # Arbitrary scaling for display
        
        return accel, brake, steer
    
    def get_total_duration(self) -> float:
        """Get total duration of scenario.
        
        Returns:
            Total duration in seconds
        """
        return self.total_duration