#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Step acceleration with reset scenario - each step starts from zero."""

import numpy as np
from typing import Tuple, Optional, Dict, Any
from enum import Enum
from .base import BaseScenario


class AccelResetState(Enum):
    """States for acceleration reset test."""
    STABILIZING_ZERO = "stabilizing_zero"
    ACCELERATING = "accelerating"
    BRAKING_TO_ZERO = "braking_to_zero"
    COMPLETE = "complete"


class StepAccelResetScenario(BaseScenario):
    """Step acceleration test with reset to zero between steps.
    
    Each acceleration step starts from zero speed:
    1. Ensure vehicle is at 0 km/h
    2. Apply acceleration for fixed duration
    3. Brake to 0 km/h
    4. Stabilize at 0 km/h
    5. Move to next acceleration level
    
    This provides consistent initial conditions for each step,
    ideal for system identification.
    """
    
    def setup(self) -> None:
        """Set up step acceleration with reset parameters."""
        # Get parameters with defaults
        self.accel_levels = self.params.get('accel_levels',
                                           np.arange(0.2, 1.2, 0.2))
        self.accel_duration = self.params.get('accel_duration', 10.0)  # seconds
        self.zero_tolerance = self.params.get('zero_tolerance', 0.5)  # km/h
        self.stabilization_time = self.params.get('stabilization_time', 2.0)  # seconds
        self.max_brake_time = self.params.get('max_brake_time', 15.0)  # max seconds for braking
        
        # State management
        self.state = AccelResetState.STABILIZING_ZERO
        self.step_index = 0
        self.state_start_time = None
        self.speed_stable_time = None
        self.accel_start_time = None
        
        # Log initialization
        import rospy
        rospy.loginfo("Step accel (with reset) scenario initialized with %d steps",
                     len(self.accel_levels))
        rospy.loginfo("Each step: accel for %.1f s, then brake to 0",
                     self.accel_duration)
    
    def is_state_based(self) -> bool:
        """This scenario is state-based for accurate zero detection."""
        return True
    
    def get_control_command_with_state(self, elapsed_time: float, vehicle_state: Any) -> Tuple[float, float, float]:
        """Get control commands based on vehicle state.
        
        Args:
            elapsed_time: Time since scenario start
            vehicle_state: Current vehicle state (includes velocity_x in m/s)
            
        Returns:
            Tuple of (accel, brake, steer) commands
        """
        import rospy
        
        # Get current speed in km/h
        current_speed_kmh = abs(vehicle_state.velocity_x) * 3.6
        current_time = rospy.Time.now().to_sec()
        
        # Check if vehicle is stopped
        is_stopped = current_speed_kmh <= self.zero_tolerance
        
        accel, brake, steer = 0.0, 0.0, 0.0
        
        # State machine
        if self.state == AccelResetState.STABILIZING_ZERO:
            # Ensure vehicle is at zero speed
            if is_stopped:
                if self.speed_stable_time is None:
                    self.speed_stable_time = current_time
                    rospy.loginfo("Vehicle stopped, stabilizing...")
                elif (current_time - self.speed_stable_time) >= self.stabilization_time:
                    # Vehicle has been stopped long enough
                    if self.step_index < len(self.accel_levels):
                        # Start acceleration
                        self.state = AccelResetState.ACCELERATING
                        self.accel_start_time = current_time
                        self.state_start_time = current_time
                        rospy.loginfo("Starting acceleration step %d: %.1f",
                                    self.step_index + 1, self.accel_levels[self.step_index])
                    else:
                        # All steps complete
                        self.state = AccelResetState.COMPLETE
                        self.is_finished = True
                        rospy.loginfo("All acceleration steps complete!")
            else:
                # Not stopped, apply brake
                self.speed_stable_time = None
                brake = 1.0
                rospy.loginfo_throttle(1.0, "Braking to zero: %.1f km/h", current_speed_kmh)
        
        elif self.state == AccelResetState.ACCELERATING:
            # Apply acceleration for fixed duration
            accel_elapsed = current_time - self.accel_start_time
            
            if accel_elapsed < self.accel_duration:
                # Continue accelerating
                accel = self.accel_levels[self.step_index]
                rospy.loginfo_throttle(1.0, "Accel step %d: %.1f (%.1f s, speed: %.1f km/h)",
                                     self.step_index + 1, accel, accel_elapsed, current_speed_kmh)
            else:
                # Acceleration complete, start braking
                self.state = AccelResetState.BRAKING_TO_ZERO
                self.state_start_time = current_time
                rospy.loginfo("Acceleration complete (reached %.1f km/h), braking to zero...",
                            current_speed_kmh)
        
        elif self.state == AccelResetState.BRAKING_TO_ZERO:
            # Brake to zero
            brake_elapsed = current_time - self.state_start_time
            
            if is_stopped:
                # Vehicle stopped, move to stabilization
                self.step_index += 1
                self.state = AccelResetState.STABILIZING_ZERO
                self.speed_stable_time = None
                
                if self.step_index < len(self.accel_levels):
                    rospy.loginfo("Vehicle stopped. Preparing for next step %d...",
                                self.step_index + 1)
                else:
                    rospy.loginfo("Vehicle stopped. All steps complete!")
            else:
                # Continue braking
                brake = 1.0
                rospy.loginfo_throttle(1.0, "Braking to zero: %.1f km/h (%.1f s)",
                                     current_speed_kmh, brake_elapsed)
                
                # Safety check: if braking takes too long
                if brake_elapsed > self.max_brake_time:
                    rospy.logwarn("Brake timeout! Force moving to next step.")
                    self.step_index += 1
                    self.state = AccelResetState.STABILIZING_ZERO
                    self.speed_stable_time = None
        
        # Store current step for logging
        self.current_step = self.step_index
        
        return accel, brake, steer
    
    def get_control_command(self, elapsed_time: float) -> Tuple[float, float, float]:
        """Time-based fallback (basic implementation)."""
        # Simplified time-based control if vehicle state not available
        import rospy
        rospy.logwarn_once("Step accel reset works better with vehicle state!")
        
        accel, brake, steer = 0.0, 0.0, 0.0
        
        # Simple time-based pattern
        cycle_time = self.accel_duration + 10.0  # accel + brake/stabilize
        total_cycles = len(self.accel_levels)
        
        if elapsed_time >= total_cycles * cycle_time:
            self.is_finished = True
            return 0.0, 0.0, 0.0
        
        # Determine current cycle and phase
        cycle_idx = int(elapsed_time / cycle_time)
        time_in_cycle = elapsed_time % cycle_time
        
        if cycle_idx < len(self.accel_levels):
            self.current_step = cycle_idx
            
            if time_in_cycle < self.accel_duration:
                # Acceleration phase
                accel = self.accel_levels[cycle_idx]
            else:
                # Braking phase
                brake = 1.0
        
        return accel, brake, steer
    
    def get_total_duration(self) -> float:
        """Get estimated total duration.
        
        Returns:
            Estimated total duration in seconds
        """
        # Estimate: (accel + brake + stabilize) * steps
        cycle_time = self.accel_duration + self.max_brake_time + self.stabilization_time
        return len(self.accel_levels) * cycle_time
    
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
            return f"State: {self.state.value}, Step {self.step_index + 1}/{len(self.accel_levels)}"