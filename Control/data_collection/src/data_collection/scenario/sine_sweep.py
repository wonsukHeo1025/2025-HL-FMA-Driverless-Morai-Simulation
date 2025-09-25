#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Sine sweep scenario for lateral frequency response analysis."""

import numpy as np
from typing import Tuple, Optional, Dict, Any
from .base import BaseScenario

# Conversion constants
DEG_TO_RAD = np.pi / 180.0  # Degree to radian conversion
MAX_STEER_DEG = 40.0  # Maximum steering angle in degrees
MAX_STEER_RAD = MAX_STEER_DEG * DEG_TO_RAD  # Maximum steering angle in radians


class SineSweepScenario(BaseScenario):
    """Sine sweep steering input for frequency response characterization.
    
    Applies sinusoidal steering with linearly increasing frequency
    to measure system frequency response (gain and phase).
    """
    
    def setup(self) -> None:
        """Set up sine sweep parameters."""
        # Get parameters with defaults
        self.initial_straight_duration = self.params.get('initial_straight_duration', 5.0)
        self.sweep_duration = self.params.get('sweep_duration', 60.0)  # Total sweep time
        self.final_straight_duration = self.params.get('final_straight_duration', 5.0)
        
        # Frequency range (Hz)
        self.freq_start = self.params.get('freq_start', 0.1)  # Starting frequency
        self.freq_end = self.params.get('freq_end', 2.0)  # Ending frequency
        
        # Amplitude (radians)
        self.amplitude = self.params.get('amplitude', np.radians(5))  # 5 degree default
        
        # Sweep type
        self.sweep_type = self.params.get('sweep_type', 'linear')  # 'linear' or 'logarithmic'
        
        # Calculate total duration
        self.total_duration = (self.initial_straight_duration + 
                              self.sweep_duration + 
                              self.final_straight_duration)
        
        # Log initialization
        import rospy
        rospy.loginfo("Sine sweep scenario initialized:")
        rospy.loginfo("  Frequency range: %.2f - %.2f Hz", self.freq_start, self.freq_end)
        rospy.loginfo("  Amplitude: %.1f degrees", np.degrees(self.amplitude))
        rospy.loginfo("  Sweep type: %s", self.sweep_type)
    
    def get_control_command(self, elapsed_time: float) -> Tuple[float, float, float]:
        """Get control commands for sine sweep.
        
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
        elif elapsed_time < self.initial_straight_duration + self.sweep_duration:
            # Sine sweep section
            sweep_time = elapsed_time - self.initial_straight_duration
            self.current_step = 1
            
            # Calculate instantaneous frequency
            if self.sweep_type == 'linear':
                # Linear frequency sweep
                freq = self.freq_start + (self.freq_end - self.freq_start) * (sweep_time / self.sweep_duration)
            else:  # logarithmic
                # Logarithmic frequency sweep
                log_start = np.log10(self.freq_start)
                log_end = np.log10(self.freq_end)
                log_freq = log_start + (log_end - log_start) * (sweep_time / self.sweep_duration)
                freq = 10 ** log_freq
            
            # Calculate phase (integral of frequency)
            if self.sweep_type == 'linear':
                # For linear sweep: phase = 2π * (f0*t + (f1-f0)*t²/(2*T))
                phase = 2 * np.pi * (
                    self.freq_start * sweep_time + 
                    (self.freq_end - self.freq_start) * sweep_time**2 / (2 * self.sweep_duration)
                )
            else:
                # For log sweep, use numerical integration or approximation
                # Simplified: use instantaneous frequency
                phase = 2 * np.pi * freq * sweep_time
            
            # Generate sine wave
            steer = self.amplitude * np.sin(phase)
            
        else:
            # Final straight section
            steer = 0.0
            self.current_step = 2
        
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
    
    def get_current_frequency(self, elapsed_time: float) -> float:
        """Get current sweep frequency for logging.
        
        Args:
            elapsed_time: Time since scenario start
            
        Returns:
            Current frequency in Hz
        """
        if elapsed_time < self.initial_straight_duration:
            return 0.0
        elif elapsed_time < self.initial_straight_duration + self.sweep_duration:
            sweep_time = elapsed_time - self.initial_straight_duration
            
            if self.sweep_type == 'linear':
                return self.freq_start + (self.freq_end - self.freq_start) * (sweep_time / self.sweep_duration)
            else:  # logarithmic
                log_start = np.log10(self.freq_start)
                log_end = np.log10(self.freq_end)
                log_freq = log_start + (log_end - log_start) * (sweep_time / self.sweep_duration)
                return 10 ** log_freq
        else:
            return 0.0
    
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
            elif elapsed < self.initial_straight_duration + self.sweep_duration:
                freq = self.get_current_frequency(elapsed)
                return f"Sweeping - {freq:.2f} Hz"
            else:
                return f"Final straight - {elapsed:.1f}s"