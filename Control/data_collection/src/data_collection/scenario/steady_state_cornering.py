#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Steady-state cornering scenario for lateral system identification."""

import numpy as np
from typing import Tuple, Optional, Dict, Any
from .base import BaseScenario

# Conversion constants
DEG_TO_RAD = np.pi / 180.0  # Degree to radian conversion
MAX_STEER_DEG = 40.0  # Maximum steering angle in degrees
MAX_STEER_RAD = MAX_STEER_DEG * DEG_TO_RAD  # Maximum steering angle in radians


class SteadyStateCornering(BaseScenario):
    """Steady-state cornering scenario for understeer gradient estimation.
    
    Applies constant steering angles at various speeds to measure
    steady-state lateral response characteristics.
    """
    
    def setup(self) -> None:
        """Set up steady-state cornering parameters."""
        # Get parameters with defaults
        self.hold_duration = self.params.get('hold_duration', 10.0)  # Time to hold each steering angle
        self.stabilization_duration = self.params.get('stabilization_duration', 3.0)  # Stabilization after reaching angle
        self.steering_tolerance = self.params.get('steering_tolerance', 1.0)  # Tolerance in degrees
        
        # Steering angles to test (in radians)
        self.steering_angles = self.params.get('steering_angles', 
                                              np.radians([5, 10, 15, 20]))  # Convert degrees to radians
        
        # Target speed (m/s) - should be maintained externally
        self.target_speed = self.params.get('target_speed', 11.11)  # 40 km/h default
        
        # State tracking for each angle
        self.current_angle_idx = 0
        self.angle_reached_time = None
        self.angle_stable_time = None
        self.phase = 'transition'  # 'transition', 'stabilizing', 'steady_state'
        self.is_steady_state = False  # Flag for data logging
        
        # Calculate estimated total duration (actual may vary based on transition times)
        estimated_transition = 5.0  # Estimate 5 seconds per transition
        self.total_duration = len(self.steering_angles) * (
            estimated_transition + self.stabilization_duration + self.hold_duration
        )
        
        # Log initialization
        import rospy
        rospy.loginfo("Steady-state cornering scenario initialized with %d steering angles",
                     len(self.steering_angles))
        rospy.loginfo("Steering angles (deg): %s", 
                     [np.degrees(a) for a in self.steering_angles])
    
    def is_state_based(self) -> bool:
        """Enable state-based control for steering monitoring."""
        return True
    
    def get_control_command_with_state(self, elapsed_time: float, vehicle_state) -> Tuple[float, float, float]:
        """Get control commands for steady-state cornering with actual steering monitoring.
        
        Args:
            elapsed_time: Time since scenario start
            vehicle_state: Current vehicle state with actual steering angle
            
        Returns:
            Tuple of (accel, brake, steer) commands
        """
        import rospy
        
        accel, brake = 0.0, 0.0
        
        # Check if we've completed all angles
        if self.current_angle_idx >= len(self.steering_angles):
            self.is_steady_state = False
            self.is_finished = True  # Mark scenario as complete
            rospy.loginfo("모든 조향각 테스트 완료! 시나리오 종료.")
            return accel, brake, 0.0
        
        # Get target angle for current index
        target_angle_rad = self.steering_angles[self.current_angle_idx]
        target_angle_deg = np.degrees(target_angle_rad)
        
        # Get actual steering angle from vehicle state (in degrees from Competition_topic)
        # IMPORTANT: Competition_topic uses OPPOSITE convention (+ for right/CW, - for left/CCW)
        # We negate to match our convention and MORAI GUI (+ for left/CCW, - for right/CW)
        actual_steering_deg = -(vehicle_state.wheel_angle if vehicle_state else 0.0)
        actual_steering_rad = np.radians(actual_steering_deg)
        
        # Calculate steering error
        steering_error_deg = abs(actual_steering_deg - target_angle_deg)
        
        # State machine for each angle
        if self.phase == 'transition':
            # Apply target steering and wait for it to be reached
            steer = target_angle_rad
            
            # Check if target angle is reached (within tolerance)
            if steering_error_deg <= self.steering_tolerance:
                if self.angle_reached_time is None:
                    self.angle_reached_time = rospy.Time.now()
                    rospy.loginfo("조향각 %.1f도 도달! 안정화 시작...", target_angle_deg)
                    self.phase = 'stabilizing'
            else:
                # Log progress occasionally
                rospy.loginfo_throttle(0.5, "조향각 전환중: %.1f / %.1f도", 
                                      actual_steering_deg, target_angle_deg)
                
        elif self.phase == 'stabilizing':
            # Hold target angle and wait for stabilization
            steer = target_angle_rad
            
            if self.angle_reached_time:
                stabilizing_time = (rospy.Time.now() - self.angle_reached_time).to_sec()
                
                if stabilizing_time >= self.stabilization_duration:
                    self.angle_stable_time = rospy.Time.now()
                    self.phase = 'steady_state'
                    rospy.loginfo("조향각 %.1f도 안정화 완료! 정상상태 데이터 수집 시작", target_angle_deg)
                else:
                    remaining = self.stabilization_duration - stabilizing_time
                    rospy.loginfo_throttle(1.0, "안정화중... %.1f초 남음", remaining)
                    
        elif self.phase == 'steady_state':
            # Hold angle and collect steady-state data
            steer = target_angle_rad
            self.is_steady_state = True  # Mark data as valid steady-state
            
            if self.angle_stable_time:
                steady_time = (rospy.Time.now() - self.angle_stable_time).to_sec()
                
                if steady_time >= self.hold_duration:
                    # Move to next angle
                    rospy.loginfo("조향각 %.1f도 완료! 다음 각도로 전환", target_angle_deg)
                    self.current_angle_idx += 1
                    self.angle_reached_time = None
                    self.angle_stable_time = None
                    self.phase = 'transition'
                    self.is_steady_state = False
                else:
                    remaining = self.hold_duration - steady_time
                    rospy.loginfo_throttle(2.0, "정상상태 데이터 수집중... %.1f초 남음", remaining)
        
        else:
            steer = target_angle_rad
            self.is_steady_state = False
        
        # Return steering angle in radians (MORAI uses radians directly)
        # MORAI uses standard convention: + for left (CCW), - for right (CW)
        steer_rad = np.clip(steer, -MAX_STEER_RAD, MAX_STEER_RAD)
        
        # Update current step for status display
        self.current_step = self.current_angle_idx
        
        return accel, brake, steer_rad
    
    def get_control_command(self, elapsed_time: float) -> Tuple[float, float, float]:
        """Fallback method - should not be called since we're state-based."""
        # This shouldn't be called, but provide safe defaults
        return 0.0, 0.0, 0.0
    
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
            if self.current_angle_idx < len(self.steering_angles):
                current_angle = np.degrees(self.steering_angles[self.current_angle_idx])
                
                # Use actual phase from state machine
                phase_korean = {
                    'transition': '전환중',
                    'stabilizing': '안정화중',
                    'steady_state': '정상상태'
                }.get(self.phase, self.phase)
                
                return f"각도 {self.current_angle_idx+1}/{len(self.steering_angles)} ({current_angle:.1f}°) - {phase_korean}"
            return f"완료"