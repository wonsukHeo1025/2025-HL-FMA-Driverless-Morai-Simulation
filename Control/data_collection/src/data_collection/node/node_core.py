#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Core node logic for data collection."""

import rospy
import numpy as np
from typing import Optional, Dict, Any
from enum import Enum

from ..scenario import ScenarioFactory, BaseScenario
from ..logging import CSVLogger
from ..utils import LowPassFilter, AdaptiveBiasEstimator, mps_to_kmph, gps_to_meters
from .interface import ROSInterface, VehicleState


class NodeState(Enum):
    """Node operational states."""
    INIT = "initializing"
    STABILIZING = "stabilizing"
    READY = "ready"
    COUNTDOWN = "countdown"
    SPEED_BUILDING = "speed_building"
    SPEED_STABILIZING = "speed_stabilizing"
    RUNNING = "running"
    COMPLETE = "complete"


class DataCollectionNodeCore:
    """Core logic for data collection node."""
    
    def __init__(self, config: Optional[Dict[str, Any]] = None):
        """Initialize node core.
        
        Args:
            config: Configuration dictionary
        """
        # Configuration
        self.config = config or {}
        self.control_rate = self.config.get('control_rate', 50.0)
        self.control_mode = self.config.get('control_mode', 'throttle')
        self.interactive_mode = self.config.get('interactive', True)
        
        # State management
        self.state = NodeState.INIT
        self.scenario = None
        self.scenario_type = None
        
        # ROS interface
        self.ros_interface = ROSInterface()
        self.ros_interface.set_state_callback(self._on_vehicle_state_update)
        
        # Data logging
        self.logger = None
        
        # Filters and estimators
        self.velocity_filter = LowPassFilter(alpha=0.7)
        self.imu_bias_estimator = AdaptiveBiasEstimator(alpha=0.99)
        
        # State tracking
        self.vehicle_state = VehicleState()
        self.last_position = None
        self.last_position_time = None
        self.estimated_velocity = np.zeros(3)
        
        # Timing
        self.state_start_time = None
        self.countdown_start = None
        self.target_speed_time = None
        
        # Control timer
        self.control_timer = rospy.Timer(
            rospy.Duration(1.0 / self.control_rate),
            self._control_callback
        )
        
        rospy.loginfo("Data collection node core initialized")
    
    def _on_vehicle_state_update(self, state: VehicleState):
        """Handle vehicle state updates from ROS interface.
        
        Args:
            state: Updated vehicle state
        """
        self.vehicle_state = state
        
        # Update velocity estimation from position
        if self.ros_interface.update_tf_data():
            current_time = rospy.Time.now().to_sec()
            
            if self.last_position is not None and self.last_position_time is not None:
                dt = current_time - self.last_position_time
                if dt > 0.001:
                    velocity = (state.position - self.last_position) / dt
                    self.estimated_velocity = self.velocity_filter.update(velocity)
            
            self.last_position = state.position.copy()
            self.last_position_time = current_time
    
    def _control_callback(self, event):
        """Main control loop callback."""
        current_time = rospy.Time.now()
        
        # Handle state machine
        if self.state == NodeState.INIT:
            self._handle_init_state()
        
        elif self.state == NodeState.STABILIZING:
            self._handle_stabilizing_state(current_time)
        
        elif self.state == NodeState.READY:
            self._handle_ready_state()
        
        elif self.state == NodeState.COUNTDOWN:
            self._handle_countdown_state(current_time)
        
        elif self.state == NodeState.SPEED_BUILDING:
            self._handle_speed_building_state(current_time)
        
        elif self.state == NodeState.SPEED_STABILIZING:
            self._handle_speed_stabilizing_state(current_time)
        
        elif self.state == NodeState.RUNNING:
            self._handle_running_state(current_time)
        
        elif self.state == NodeState.COMPLETE:
            self._handle_complete_state()
    
    def _handle_init_state(self):
        """Handle initialization state."""
        # Apply brake to stop vehicle
        self.ros_interface.publish_control_command(
            0.0, 1.0, 0.0, control_mode='throttle'
        )
        
        # Check if vehicle is stopped
        if abs(self.vehicle_state.velocity_x) < 0.1:  # < 0.1 m/s
            self.state = NodeState.STABILIZING
            self.state_start_time = rospy.Time.now()
            rospy.loginfo("Vehicle stopped. Stabilizing...")
    
    def _handle_stabilizing_state(self, current_time):
        """Handle stabilization state."""
        # Release brake but keep stopped
        self.ros_interface.publish_control_command(
            0.0, 0.0, 0.0, control_mode='throttle'
        )
        
        # Wait 1 second for stabilization
        if (current_time - self.state_start_time).to_sec() > 1.0:
            self.state = NodeState.READY
            rospy.loginfo("Vehicle stabilized. Ready for operation.")
    
    def _handle_ready_state(self):
        """Handle ready state - waiting for scenario selection."""
        # Keep vehicle stopped
        self.ros_interface.publish_control_command(
            0.0, 0.0, 0.0, control_mode='throttle'
        )
    
    def _handle_countdown_state(self, current_time):
        """Handle countdown state."""
        # For lateral scenarios with target speed, go to speed building
        if (self.scenario_type in ['steady_state_cornering', 'step_steer', 
                                   'sine_sweep', 'double_lane_change'] and
            hasattr(self.scenario, 'params') and 
            self.scenario.params.get('target_speed', 0) > 0):
            
            self.state = NodeState.SPEED_BUILDING
            self.state_start_time = current_time
            target_speed_kmh = self.scenario.params.get('target_speed', 0) * 3.6
            rospy.loginfo("Building speed to %.1f km/h...", target_speed_kmh)
            return
        
        # Normal countdown for other scenarios
        elapsed = (current_time - self.countdown_start).to_sec()
        remaining = 3.0 - elapsed
        
        if remaining > 0.1:
            # Keep brake during countdown
            self.ros_interface.publish_control_command(
                0.0, 1.0, 0.0, control_mode='throttle'
            )
            
            # Log countdown
            count = int(remaining) + 1
            if not hasattr(self, '_last_count'):
                self._last_count = 0
            if self._last_count != count:
                rospy.loginfo("Starting in %d...", count)
                self._last_count = count
        
        elif remaining > 0:
            # Release brake 0.1s before start
            self.ros_interface.publish_control_command(
                0.0, 0.0, 0.0, control_mode='throttle'
            )
        
        else:
            # Start scenario
            self.state = NodeState.RUNNING
            if self.scenario:
                self.scenario.start(current_time.to_sec())
            rospy.loginfo("GO! Starting data collection...")
    
    def _handle_speed_building_state(self, current_time):
        """Handle speed building state."""
        # Determine target speed based on scenario
        if self.scenario_type == 'step_brake':
            target_speed = 50.0  # km/h for brake test
        elif hasattr(self.scenario, 'params'):
            target_speed = self.scenario.params.get('target_speed', 0) * 3.6  # m/s to km/h
        else:
            target_speed = 40.0  # Default
        
        # Use velocity control to reach target speed
        self.ros_interface.publish_control_command(
            0.0, 0.0, 0.0, control_mode='velocity', target_velocity=target_speed
        )
        
        # Check current speed
        current_speed_kmh = abs(self.vehicle_state.velocity_x) * 3.6
        rospy.loginfo_throttle(0.5, "Building speed: %.1f / %.1f km/h", 
                              current_speed_kmh, target_speed)
        
        # Check if target reached (within ±0.5 km/h)
        if abs(current_speed_kmh - target_speed) <= 0.5:
            if self.target_speed_time is None:
                self.target_speed_time = current_time
                rospy.loginfo("Target speed reached! Stabilizing for 3 seconds...")
            self.state = NodeState.SPEED_STABILIZING
    
    def _handle_speed_stabilizing_state(self, current_time):
        """Handle speed stabilization state."""
        # Determine target speed
        if self.scenario_type == 'step_brake':
            target_speed = 50.0
            stabilization_time = 2.0  # 2 seconds for brake test
        elif hasattr(self.scenario, 'params'):
            target_speed = self.scenario.params.get('target_speed', 0) * 3.6
            stabilization_time = 3.0  # 3 seconds for lateral tests
        else:
            target_speed = 40.0
            stabilization_time = 3.0
        
        # Maintain target speed
        self.ros_interface.publish_control_command(
            0.0, 0.0, 0.0, control_mode='velocity', target_velocity=target_speed
        )
        
        # Check if stabilization complete
        if self.target_speed_time and (current_time - self.target_speed_time).to_sec() >= stabilization_time:
            self.state = NodeState.RUNNING
            if self.scenario:
                self.scenario.start(current_time.to_sec())
            
            if self.scenario_type == 'step_brake':
                rospy.loginfo("Speed stabilized! Starting brake test NOW!")
            else:
                rospy.loginfo("Speed stabilized! Starting %s NOW!", self.scenario_type)
    
    def _handle_running_state(self, current_time):
        """Handle running state - execute scenario."""
        if not self.scenario:
            return
        
        # Get control commands from scenario (pass vehicle state for state-based scenarios)
        accel, brake, steer = self.scenario.update(current_time.to_sec(), self.vehicle_state)
        
        # Apply special handling for brake test
        if self.scenario_type == 'step_brake':
            if brake > 0:
                # Brake phase - use throttle control
                self.ros_interface.publish_control_command(
                    0.0, brake, steer, control_mode='throttle'
                )
            else:
                # Recovery phase - use velocity control
                self.ros_interface.publish_control_command(
                    0.0, 0.0, steer, control_mode='velocity', target_velocity=50.0
                )
        else:
            # Normal control
            # For lateral scenarios in velocity mode, maintain target speed
            if self.control_mode == 'velocity' and hasattr(self.scenario, 'params'):
                target_speed_mps = self.scenario.params.get('target_speed', 0)
                target_speed_kmh = target_speed_mps * 3.6
                self.ros_interface.publish_control_command(
                    accel, brake, steer, 
                    control_mode=self.control_mode,
                    target_velocity=target_speed_kmh
                )
            else:
                self.ros_interface.publish_control_command(
                    accel, brake, steer, control_mode=self.control_mode
                )
        
        # Log data
        if self.logger:
            self._log_current_data(accel, brake, steer)
        
        # Check if scenario complete
        if self.scenario.is_complete():
            self.state = NodeState.COMPLETE
            rospy.loginfo("Scenario complete!")
    
    def _handle_complete_state(self):
        """Handle completion state."""
        # Apply brake
        self.ros_interface.publish_control_command(
            0.0, 1.0, 0.0, control_mode='throttle'
        )
        
        # Save data
        if self.logger:
            self.logger.save()
            self.logger = None
        
        # Reset state for next scenario
        self.state = NodeState.READY
        self.scenario = None
    
    def start_scenario(self, scenario_type: str, params: Optional[Dict] = None):
        """Start a data collection scenario.
        
        Args:
            scenario_type: Type of scenario to run
            params: Optional scenario parameters
        """
        try:
            # Create scenario
            self.scenario = ScenarioFactory.create(scenario_type, params)
            self.scenario_type = scenario_type
            
            # Create logger
            self.logger = CSVLogger(scenario_name=scenario_type)
            
            # Determine start state based on scenario
            if scenario_type == 'step_brake':
                # Brake test - skip initialization, go straight to speed building
                self.state = NodeState.SPEED_BUILDING
                self.target_speed_time = None
                current_speed = abs(self.vehicle_state.velocity_x) * 3.6
                rospy.loginfo("Starting brake test. Current speed: %.1f km/h", current_speed)
                rospy.loginfo("Accelerating to 50 km/h...")
            else:
                # Normal scenarios - countdown
                if self.state == NodeState.READY:
                    self.state = NodeState.COUNTDOWN
                    self.countdown_start = rospy.Time.now()
                    self._last_count = 0
                    rospy.loginfo("Starting %s scenario with countdown...", scenario_type)
                else:
                    rospy.logwarn("Vehicle not ready. Please wait for initialization.")
            
        except ValueError as e:
            rospy.logerr("Failed to start scenario: %s", str(e))
    
    def stop_scenario(self):
        """Stop current scenario."""
        if self.state in [NodeState.COUNTDOWN, NodeState.SPEED_BUILDING, 
                         NodeState.SPEED_STABILIZING, NodeState.RUNNING]:
            # Apply brake
            self.ros_interface.publish_control_command(
                0.0, 1.0, 0.0, control_mode='throttle'
            )
            
            # Save any logged data
            if self.logger:
                self.logger.save()
                self.logger = None
            
            # Reset state
            self.state = NodeState.READY
            self.scenario = None
            rospy.loginfo("Scenario stopped")
    
    def _log_current_data(self, accel_cmd: float, brake_cmd: float, steer_cmd: float):
        """Log current state data.
        
        Args:
            accel_cmd: Acceleration command
            brake_cmd: Brake command  
            steer_cmd: Steering command
        """
        data = {
            # Commands
            'accel_cmd': accel_cmd,
            'brake_cmd': brake_cmd,
            'steer_cmd': steer_cmd,
            
            # Vehicle state
            'true_velocity_x': self.vehicle_state.velocity_x,
            'true_accel': self.vehicle_state.accel,
            'true_brake': self.vehicle_state.brake,
            'true_wheel_angle': self.vehicle_state.wheel_angle,
            
            # Position
            'x_position': self.vehicle_state.position[0],
            'y_position': self.vehicle_state.position[1],
            'z_position': self.vehicle_state.position[2],
            
            # Estimated velocity
            'estimated_velocity_x': self.estimated_velocity[0],
            'estimated_velocity_y': self.estimated_velocity[1],
            'estimated_velocity_z': self.estimated_velocity[2],
            
            # IMU
            'imu_accel_x': self.vehicle_state.imu_linear_accel[0],
            'imu_accel_y': self.vehicle_state.imu_linear_accel[1],
            'imu_accel_z': self.vehicle_state.imu_linear_accel[2],
            
            # GPS
            'gps_latitude': self.vehicle_state.gps_latitude,
            'gps_longitude': self.vehicle_state.gps_longitude,
            'gps_altitude': self.vehicle_state.gps_altitude,
            
            # Scenario info
            'scenario_type': self.scenario_type,
            'scenario_step': self.scenario.get_current_step() if self.scenario else 0,
            'scenario_time': self.scenario.get_elapsed_time() if self.scenario else 0,
            
            # Add steady state flag for steady_state_cornering scenario
            'is_steady_state': getattr(self.scenario, 'is_steady_state', False) if self.scenario else False,
            
            # Add yaw rate and steering angle for lateral scenarios
            'yaw_rate': self.vehicle_state.imu_angular_vel[2] if self.vehicle_state.imu_angular_vel is not None else 0.0,
            # Note: Competition_topic uses opposite convention, negate to match standard (+ for left)
            'steering_angle_deg': -self.vehicle_state.wheel_angle if self.vehicle_state.wheel_angle else 0.0,
        }
        
        self.logger.log(data)
    
    def shutdown(self):
        """Clean shutdown."""
        # Stop scenario
        self.stop_scenario()
        
        # Cancel timer
        if self.control_timer:
            self.control_timer.shutdown()
        
        # Clean up ROS interface
        self.ros_interface.shutdown()
        
        rospy.loginfo("Node core shutdown complete")