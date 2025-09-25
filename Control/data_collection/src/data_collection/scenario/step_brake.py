#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Step brake scenario implementation - speed-based control."""

import numpy as np
from typing import Tuple, Optional, Dict, Any
from enum import Enum
from .base import BaseScenario


class BrakeTestState(Enum):
    """States for brake test scenario."""
    WAITING_STABILIZATION = "waiting_stabilization"
    BRAKING = "braking"
    RECOVERING = "recovering"
    COMPLETE = "complete"


class StepBrakeScenario(BaseScenario):
    """Step brake test scenario with speed-based control.
    
    Maintains 50 km/h, applies brake steps, recovers to 50 km/h,
    and repeats. All transitions are based on speed stabilization,
    not fixed time periods.
    """
    
    def setup(self) -> None:
        """Set up step brake parameters."""
        # Get parameters with defaults
        self.brake_levels = self.params.get('brake_levels',
                                           np.arange(0.2, 1.2, 0.2))
        self.target_speed = self.params.get('target_speed', 50.0)  # km/h
        self.speed_tolerance = self.params.get('speed_tolerance', 0.5)  # km/h
        self.stabilization_time = self.params.get('stabilization_time', 2.0)  # seconds
        self.brake_duration = self.params.get('brake_duration', 8.0)  # seconds per brake
        self.max_recovery_time = self.params.get('max_recovery_time', 15.0)  # max seconds for recovery
        
        # State management
        self.state = BrakeTestState.WAITING_STABILIZATION
        self.step_index = 0
        self.state_start_time = None
        self.speed_stable_time = None
        self.brake_start_time = None
        
        # Special flags for brake test
        self.needs_speed_buildup = True  # Requires special pre-start handling
        
        # Log initialization
        import rospy
        rospy.loginfo("Step brake scenario initialized with %d steps, target speed: %.1f km/h",
                     len(self.brake_levels), self.target_speed)
        rospy.loginfo("Speed tolerance: ±%.1f km/h, stabilization time: %.1f s",
                     self.speed_tolerance, self.stabilization_time)
    
    def is_state_based(self) -> bool:
        """This scenario is state-based, not time-based."""
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
        
        # Check if speed is within tolerance of target
        speed_error = abs(current_speed_kmh - self.target_speed)
        is_stable = speed_error <= self.speed_tolerance
        
        accel, brake, steer = 0.0, 0.0, 0.0
        
        # State machine
        if self.state == BrakeTestState.WAITING_STABILIZATION:
            # Wait for speed to stabilize at target
            if is_stable:
                if self.speed_stable_time is None:
                    self.speed_stable_time = current_time
                    rospy.loginfo("Speed reached %.1f km/h, stabilizing...", self.target_speed)
                elif (current_time - self.speed_stable_time) >= self.stabilization_time:
                    # Speed has been stable long enough, start braking
                    self.state = BrakeTestState.BRAKING
                    self.brake_start_time = current_time
                    self.state_start_time = current_time
                    rospy.loginfo("Speed stabilized! Starting brake step %d: %.1f",
                                self.step_index + 1, self.brake_levels[self.step_index])
            else:
                # Speed not stable, reset timer
                self.speed_stable_time = None
                rospy.loginfo_throttle(1.0, "Waiting for speed stabilization: %.1f / %.1f km/h",
                                     current_speed_kmh, self.target_speed)
            
            # Signal to use velocity control to maintain target speed
            accel = 0.0  # This signals velocity control mode
        
        elif self.state == BrakeTestState.BRAKING:
            # Apply brake for fixed duration
            brake_elapsed = current_time - self.brake_start_time
            
            if brake_elapsed < self.brake_duration:
                # Continue braking
                brake = self.brake_levels[self.step_index]
                rospy.loginfo_throttle(1.0, "Braking step %d: %.1f (%.1f s, speed: %.1f km/h)",
                                     self.step_index + 1, brake, brake_elapsed, current_speed_kmh)
            else:
                # Brake duration complete, start recovery
                self.state = BrakeTestState.RECOVERING
                self.state_start_time = current_time
                self.speed_stable_time = None
                rospy.loginfo("Brake complete, recovering to %.1f km/h...", self.target_speed)
        
        elif self.state == BrakeTestState.RECOVERING:
            # Recover to target speed
            recovery_elapsed = current_time - self.state_start_time
            
            # Check if we've recovered to target speed
            if is_stable:
                if self.speed_stable_time is None:
                    self.speed_stable_time = current_time
                    rospy.loginfo("Speed recovered to %.1f km/h, stabilizing...", self.target_speed)
                elif (current_time - self.speed_stable_time) >= self.stabilization_time:
                    # Speed recovered and stabilized
                    self.step_index += 1
                    
                    if self.step_index < len(self.brake_levels):
                        # Move to next brake step
                        self.state = BrakeTestState.WAITING_STABILIZATION
                        self.speed_stable_time = current_time - self.stabilization_time  # Already stable
                        rospy.loginfo("Ready for next brake step %d", self.step_index + 1)
                    else:
                        # All steps complete
                        self.state = BrakeTestState.COMPLETE
                        self.is_finished = True
                        rospy.loginfo("All brake steps complete!")
            else:
                # Not yet recovered
                self.speed_stable_time = None
                rospy.loginfo_throttle(1.0, "Recovering: %.1f / %.1f km/h (%.1f s)",
                                     current_speed_kmh, self.target_speed, recovery_elapsed)
                
                # Safety check: if recovery takes too long, abort
                if recovery_elapsed > self.max_recovery_time:
                    rospy.logwarn("Recovery timeout! Moving to next step.")
                    self.step_index += 1
                    if self.step_index < len(self.brake_levels):
                        self.state = BrakeTestState.WAITING_STABILIZATION
                        self.speed_stable_time = None
                    else:
                        self.state = BrakeTestState.COMPLETE
                        self.is_finished = True
            
            # Signal to use velocity control for recovery
            accel = 0.0  # This signals velocity control mode
        
        # Store current step for logging
        self.current_step = self.step_index
        
        return accel, brake, steer
    
    def get_control_command(self, elapsed_time: float) -> Tuple[float, float, float]:
        """Fallback for time-based control (shouldn't be used)."""
        # This scenario requires vehicle state
        import rospy
        rospy.logwarn_once("Step brake scenario requires vehicle state!")
        return 0.0, 0.0, 0.0
    
    def get_total_duration(self) -> float:
        """Get estimated total duration.
        
        Note: Actual duration depends on speed stabilization times.
        
        Returns:
            Estimated total duration in seconds
        """
        # Rough estimate: stabilization + (brake + recovery) * steps
        estimated_cycle = self.brake_duration + self.max_recovery_time + 2 * self.stabilization_time
        return len(self.brake_levels) * estimated_cycle
    
    def requires_speed_buildup(self) -> bool:
        """Check if scenario requires speed buildup before start.
        
        Returns:
            True (always needs to reach 50 km/h first)
        """
        return self.needs_speed_buildup
    
    def get_target_speed(self) -> float:
        """Get target speed for brake test.
        
        Returns:
            Target speed in km/h
        """
        return self.target_speed
    
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
            return f"State: {self.state.value}, Step {self.step_index + 1}/{len(self.brake_levels)}"