#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Base scenario abstract class for data collection."""

from abc import ABC, abstractmethod
from typing import Tuple, Optional, Dict, Any


class BaseScenario(ABC):
    """Abstract base class for all data collection scenarios."""
    
    def __init__(self, params: Optional[Dict[str, Any]] = None):
        """Initialize scenario with parameters.
        
        Args:
            params: Dictionary of scenario-specific parameters
        """
        self.params = params or {}
        self.start_time = None
        self.current_step = 0
        self.is_started = False
        self.is_finished = False
        
        # Call scenario-specific setup
        self.setup()
    
    @abstractmethod
    def setup(self) -> None:
        """Set up scenario-specific parameters and initial state."""
        pass
    
    @abstractmethod
    def get_control_command(self, elapsed_time: float) -> Tuple[float, float, float]:
        """Get control commands for current time.
        
        Args:
            elapsed_time: Time elapsed since scenario start (seconds)
            
        Returns:
            Tuple of (accel, brake, steer) commands, each in range [0, 1]
        """
        pass
    
    @abstractmethod
    def get_total_duration(self) -> float:
        """Get total expected duration of scenario.
        
        Returns:
            Total duration in seconds
        """
        pass
    
    def get_control_command_with_state(self, elapsed_time: float, vehicle_state: Any) -> Tuple[float, float, float]:
        """Get control commands with vehicle state (for state-based scenarios).
        
        Args:
            elapsed_time: Time elapsed since scenario start (seconds)
            vehicle_state: Current vehicle state
            
        Returns:
            Tuple of (accel, brake, steer) commands
        """
        # Default implementation falls back to time-based
        return self.get_control_command(elapsed_time)
    
    def is_state_based(self) -> bool:
        """Check if scenario is state-based instead of time-based.
        
        Returns:
            True if scenario needs vehicle state for decisions
        """
        return False
    
    def start(self, start_time: float) -> None:
        """Start the scenario.
        
        Args:
            start_time: Scenario start time (seconds)
        """
        self.start_time = start_time
        self.is_started = True
        self.is_finished = False
        self.current_step = 0
    
    def update(self, current_time: float, vehicle_state: Optional[Any] = None) -> Tuple[float, float, float]:
        """Update scenario state and get control commands.
        
        Args:
            current_time: Current time (seconds)
            vehicle_state: Optional vehicle state for scenarios that need it
            
        Returns:
            Tuple of (accel, brake, steer) commands
        """
        if not self.is_started or self.is_finished:
            return 0.0, 0.0, 0.0
        
        # Initialize start time if needed
        if self.start_time is None:
            self.start_time = current_time
        
        elapsed = current_time - self.start_time
        
        # Check if scenario is complete (may be overridden for state-based scenarios)
        if not self.is_state_based() and elapsed >= self.get_total_duration():
            self.is_finished = True
            return 0.0, 0.0, 0.0
        
        # For state-based scenarios, pass vehicle state
        if self.is_state_based() and vehicle_state is not None:
            return self.get_control_command_with_state(elapsed, vehicle_state)
        
        return self.get_control_command(elapsed)
    
    def is_complete(self) -> bool:
        """Check if scenario is complete.
        
        Returns:
            True if scenario is finished
        """
        return self.is_finished
    
    def get_current_step(self) -> int:
        """Get current step/phase of scenario.
        
        Returns:
            Current step number
        """
        return self.current_step
    
    def get_elapsed_time(self) -> float:
        """Get elapsed time since scenario start.
        
        Returns:
            Elapsed time in seconds, or 0 if not started
        """
        if self.start_time is None:
            return 0.0
        import rospy
        return rospy.Time.now().to_sec() - self.start_time
    
    def reset(self) -> None:
        """Reset scenario to initial state."""
        self.start_time = None
        self.current_step = 0
        self.is_started = False
        self.is_finished = False
        self.setup()
    
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
            total = self.get_total_duration()
            progress = (elapsed / total) * 100 if total > 0 else 0
            return f"Running - Step {self.current_step} ({progress:.1f}% complete)"
    
    def __str__(self) -> str:
        """String representation of scenario."""
        return f"{self.__class__.__name__}: {self.get_status_string()}"