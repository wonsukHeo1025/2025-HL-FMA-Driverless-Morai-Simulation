#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""ROS interface layer for data collection node.

This module handles all ROS-specific operations, allowing the core logic
to remain ROS-agnostic and testable.
"""

import rospy
import tf2_ros
import numpy as np
from typing import Optional, Callable, Dict, Any
from dataclasses import dataclass, field

from morai_msgs.msg import CtrlCmd, EgoVehicleStatus, GPSMessage
from custom_interface.msg import SystemIdData
from sensor_msgs.msg import Imu
from std_msgs.msg import Header


@dataclass
class VehicleState:
    """Container for vehicle state data."""
    # Competition topic data
    velocity_x: float = 0.0
    accel: float = 0.0
    brake: float = 0.0
    wheel_angle: float = 0.0
    
    # IMU data
    imu_linear_accel: np.ndarray = field(default_factory=lambda: np.zeros(3))
    imu_angular_vel: np.ndarray = field(default_factory=lambda: np.zeros(3))
    
    # GPS data
    gps_latitude: float = 0.0
    gps_longitude: float = 0.0
    gps_altitude: float = 0.0
    
    # TF data
    position: np.ndarray = field(default_factory=lambda: np.zeros(3))
    orientation: np.ndarray = field(default_factory=lambda: np.zeros(4))  # quaternion
    
    # Timestamps
    last_update: float = 0.0


class ROSInterface:
    """ROS interface handler for data collection node."""
    
    def __init__(self):
        """Initialize ROS interface."""
        # Publishers
        self.ctrl_pub = rospy.Publisher('/ctrl_cmd', CtrlCmd, queue_size=1)
        self.monitor_pub = rospy.Publisher('/control_system_data', 
                                          SystemIdData, queue_size=1)
        
        # TF listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # Vehicle state
        self.vehicle_state = VehicleState()
        
        # Callbacks storage
        self._state_update_callback = None
        
        # Subscribers (initialized when callbacks are set)
        self._subscribers = []
    
    def set_state_callback(self, callback: Callable[[VehicleState], None]):
        """Set callback for state updates.
        
        Args:
            callback: Function to call with updated vehicle state
        """
        self._state_update_callback = callback
        
        # Initialize subscribers
        self._setup_subscribers()
    
    def _setup_subscribers(self):
        """Set up ROS subscribers."""
        # Competition topic subscriber
        self._subscribers.append(
            rospy.Subscriber('/Competition_topic', EgoVehicleStatus,
                           self._competition_callback, queue_size=1)
        )
        
        # IMU subscriber
        self._subscribers.append(
            rospy.Subscriber('/imu', Imu, self._imu_callback, queue_size=1)
        )
        
        # GPS subscriber
        self._subscribers.append(
            rospy.Subscriber('/gps', GPSMessage, self._gps_callback, queue_size=1)
        )
    
    def _competition_callback(self, msg: EgoVehicleStatus):
        """Handle Competition_topic messages."""
        self.vehicle_state.velocity_x = msg.velocity.x  # m/s
        self.vehicle_state.accel = msg.accel
        self.vehicle_state.brake = msg.brake
        self.vehicle_state.wheel_angle = msg.wheel_angle
        
        self._notify_state_update()
    
    def _imu_callback(self, msg: Imu):
        """Handle IMU messages."""
        self.vehicle_state.imu_linear_accel = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ])
        
        self.vehicle_state.imu_angular_vel = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ])
        
        self._notify_state_update()
    
    def _gps_callback(self, msg: GPSMessage):
        """Handle GPS messages."""
        self.vehicle_state.gps_latitude = msg.latitude
        self.vehicle_state.gps_longitude = msg.longitude
        self.vehicle_state.gps_altitude = msg.altitude
        
        self._notify_state_update()
    
    def update_tf_data(self) -> bool:
        """Update TF-based position and orientation.
        
        Returns:
            True if TF data was successfully updated
        """
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_link', rospy.Time(0), rospy.Duration(0.1)
            )
            
            # Update position
            self.vehicle_state.position = np.array([
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z
            ])
            
            # Update orientation (quaternion)
            self.vehicle_state.orientation = np.array([
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w
            ])
            
            return True
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            rospy.logwarn_throttle(1.0, "TF lookup failed: %s", str(e))
            return False
    
    def _notify_state_update(self):
        """Notify callback of state update."""
        self.vehicle_state.last_update = rospy.Time.now().to_sec()
        
        if self._state_update_callback:
            self._state_update_callback(self.vehicle_state)
    
    def publish_control_command(self, accel: float, brake: float, 
                               steer: float, control_mode: str = 'throttle',
                               target_velocity: Optional[float] = None):
        """Publish control command.
        
        Args:
            accel: Acceleration command (0-1)
            brake: Brake command (0-1)
            steer: Steering command (radians)
            control_mode: Control mode ('throttle', 'velocity', 'acceleration')
            target_velocity: Target velocity for velocity control mode (km/h)
        """
        msg = CtrlCmd()
        
        if control_mode == 'throttle':
            msg.longlCmdType = 1
            msg.accel = accel
            msg.brake = brake
            msg.steering = steer
        elif control_mode == 'velocity':
            msg.longlCmdType = 2
            msg.velocity = target_velocity if target_velocity else 0.0
            msg.steering = steer
        elif control_mode == 'acceleration':
            msg.longlCmdType = 3
            msg.acceleration = (accel - brake) * 10.0  # -10 to 10 m/s^2
            msg.steering = steer
        
        self.ctrl_pub.publish(msg)
    
    def publish_monitoring_data(self, data: Dict[str, Any]):
        """Publish monitoring data.
        
        Args:
            data: Dictionary containing monitoring data
        """
        msg = SystemIdData()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "base_link"
        
        # Fill message fields from data dict
        for key, value in data.items():
            if hasattr(msg, key):
                setattr(msg, key, value)
        
        self.monitor_pub.publish(msg)
    
    def shutdown(self):
        """Clean up ROS interface."""
        # Unsubscribe all
        for sub in self._subscribers:
            sub.unregister()
        
        # Clear publishers
        self.ctrl_pub.unregister()
        self.monitor_pub.unregister()