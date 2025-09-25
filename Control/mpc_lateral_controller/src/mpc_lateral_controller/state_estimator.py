#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
State Estimation Module

Handles vehicle state estimation from various sources:
- TF transforms
- GPS/IMU fusion
- Odometry
"""

import numpy as np
import rospy
import tf2_ros
from typing import Dict, Optional
from sensor_msgs.msg import Imu
from morai_msgs.msg import GPSMessage, EgoVehicleStatus
from nav_msgs.msg import Odometry
from collections import deque


class StateEstimator:
    """Estimate vehicle state from various sensor sources."""
    
    def __init__(self, source: str = 'tf', history_size: int = 10):
        """
        Initialize state estimator.
        
        Args:
            source: State estimation source ('tf', 'gps_imu', 'odometry', 'ego_status')
            history_size: Size of history buffer for filtering
        """
        self.source = source
        self.history_size = history_size
        
        # Current state estimate
        self.current_state = {
            'x': 0.0,
            'y': 0.0,
            'z': 0.0,
            'heading': 0.0,
            'vx': 0.0,
            'vy': 0.0,
            'vz': 0.0,
            'yaw_rate': 0.0,
            'ax': 0.0,
            'ay': 0.0
        }
        
        # Previous state for finite differences
        self.last_state = None
        self.last_time = None
        
        # History buffers for filtering
        self.position_history = deque(maxlen=history_size)
        self.velocity_history = deque(maxlen=history_size)
        
        # Sensor data buffers
        self.last_imu = None
        self.last_gps = None
        self.last_ego = None
        self.last_odom = None
        
        # Validity flags
        self.is_initialized = False
        self.is_valid = False
        
    def update_from_tf(self, tf_buffer: tf2_ros.Buffer, ref_frame='reference', base_frame='base') -> bool:
        """
        Update state from TF tree.
        
        Args:
            tf_buffer: TF2 buffer
            ref_frame: Reference frame name
            base_frame: Base frame name
            
        Returns:
            True if successful
        """
        try:
            # Avoid blocking: check availability first, then do a non-blocking lookup
            if not tf_buffer.can_transform(ref_frame, base_frame, rospy.Time(0), rospy.Duration(0.0)):
                self.is_valid = False
                return False
            # Get reference -> base transform (non-blocking)
            transform = tf_buffer.lookup_transform(
                ref_frame, base_frame,
                rospy.Time(0),
                rospy.Duration(0.0)
            )
            
            # Position
            self.current_state['x'] = transform.transform.translation.x
            self.current_state['y'] = transform.transform.translation.y
            self.current_state['z'] = transform.transform.translation.z
            
            # Heading from quaternion
            q = transform.transform.rotation
            self.current_state['heading'] = self._quaternion_to_yaw(q)
            
            # Calculate velocities using finite differences
            current_time = rospy.Time.now()
            if self.last_state is not None and self.last_time is not None:
                dt = (current_time - self.last_time).to_sec()
                
                if dt > 0 and dt < 1.0:  # Sanity check
                    # Linear velocities (world frame)
                    self.current_state['vx'] = (self.current_state['x'] - self.last_state['x']) / dt
                    self.current_state['vy'] = (self.current_state['y'] - self.last_state['y']) / dt
                    self.current_state['vz'] = (self.current_state['z'] - self.last_state['z']) / dt
                    
                    # Yaw rate
                    d_yaw = self._normalize_angle(self.current_state['heading'] - self.last_state['heading'])
                    self.current_state['yaw_rate'] = d_yaw / dt
                    
                    # Accelerations
                    if 'vx' in self.last_state:
                        self.current_state['ax'] = (self.current_state['vx'] - self.last_state['vx']) / dt
                        self.current_state['ay'] = (self.current_state['vy'] - self.last_state['vy']) / dt
            
            # Store for next iteration
            self.last_state = self.current_state.copy()
            self.last_time = current_time
            
            self.is_valid = True
            self.is_initialized = True
            return True
            
        except (tf2_ros.LookupException, 
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            rospy.logwarn_throttle(1.0, f"TF lookup failed: {e}")
            self.is_valid = False
            return False
    
    def update_from_imu(self, imu_msg: Imu):
        """
        Update state from IMU data.
        
        Args:
            imu_msg: IMU message
        """
        self.last_imu = imu_msg
        
        # Extract yaw rate
        self.current_state['yaw_rate'] = imu_msg.angular_velocity.z
        
        # Extract accelerations (body frame)
        # Would need to transform to world frame using current heading
        ax_body = imu_msg.linear_acceleration.x
        ay_body = imu_msg.linear_acceleration.y
        
        # Simple transformation to world frame
        heading = self.current_state['heading']
        cos_h = np.cos(heading)
        sin_h = np.sin(heading)
        
        self.current_state['ax'] = cos_h * ax_body - sin_h * ay_body
        self.current_state['ay'] = sin_h * ax_body + cos_h * ay_body
    
    def update_from_gps(self, gps_msg: GPSMessage):
        """
        Update state from GPS data.
        
        Args:
            gps_msg: GPS message
        """
        self.last_gps = gps_msg
        
        # Convert GPS to local coordinates
        # This is simplified - would need proper GPS to local conversion
        # Assuming eastOffset and northOffset are in local frame
        self.current_state['x'] = gps_msg.eastOffset
        self.current_state['y'] = gps_msg.northOffset
        self.current_state['z'] = gps_msg.altitude
        
        # Add to history for velocity estimation
        self.position_history.append({
            'x': self.current_state['x'],
            'y': self.current_state['y'],
            'time': rospy.Time.now()
        })
        
        # Estimate velocity from position history
        if len(self.position_history) >= 2:
            self._estimate_velocity_from_history()
    
    def update_from_ego_status(self, ego_msg: EgoVehicleStatus):
        """
        Update state from MORAI EgoVehicleStatus.
        
        Args:
            ego_msg: EgoVehicleStatus message
        """
        self.last_ego = ego_msg
        
        # Velocity (MORAI provides this in m/s)
        self.current_state['vx'] = ego_msg.velocity.x
        self.current_state['vy'] = ego_msg.velocity.y
        self.current_state['vz'] = ego_msg.velocity.z
        
        # Position (if available)
        if ego_msg.position.x != 0 or ego_msg.position.y != 0:
            self.current_state['x'] = ego_msg.position.x
            self.current_state['y'] = ego_msg.position.y
            self.current_state['z'] = ego_msg.position.z
        
        # Note: MORAI heading is often empty, need separate estimation
        
        self.is_valid = True
    
    def update_from_odometry(self, odom_msg: Odometry):
        """
        Update state from odometry message.
        
        Args:
            odom_msg: Odometry message
        """
        self.last_odom = odom_msg
        
        # Position
        self.current_state['x'] = odom_msg.pose.pose.position.x
        self.current_state['y'] = odom_msg.pose.pose.position.y
        self.current_state['z'] = odom_msg.pose.pose.position.z
        
        # Heading
        q = odom_msg.pose.pose.orientation
        self.current_state['heading'] = self._quaternion_to_yaw(q)
        
        # Velocities (in body frame)
        self.current_state['vx'] = odom_msg.twist.twist.linear.x
        self.current_state['vy'] = odom_msg.twist.twist.linear.y
        self.current_state['vz'] = odom_msg.twist.twist.linear.z
        
        # Angular velocity
        self.current_state['yaw_rate'] = odom_msg.twist.twist.angular.z
        
        self.is_valid = True
        self.is_initialized = True
    
    def get_error_state(self, lateral_error: float, heading_error: float) -> np.ndarray:
        """
        Get error state vector for MPC (dynamic model only).
        
        Args:
            lateral_error: Lateral position error [m]
            heading_error: Heading error [rad]
            
        Returns:
            Error state vector [e_y, e_y_dot, e_psi, e_psi_dot]
        """
        # Dynamic model: [e_y, e_y_dot, e_psi, e_psi_dot]
        # Estimate error rates
        vx = self.current_state['vx']
        vy = self.current_state['vy']
        yaw_rate = self.current_state['yaw_rate']
        
        # Lateral error rate (approximation)
        e_y_dot = vy + vx * heading_error
        
        # Heading error rate
        e_psi_dot = yaw_rate  # Simplified
        
        return np.array([lateral_error, e_y_dot, heading_error, e_psi_dot])
    
    def get_vehicle_speed(self) -> float:
        """
        Get current vehicle speed.
        
        Returns:
            Speed in m/s
        """
        vx = self.current_state['vx']
        vy = self.current_state['vy']
        return np.sqrt(vx**2 + vy**2)
    
    def get_lateral_velocity(self) -> float:
        """
        Get lateral velocity in body frame.
        
        Returns:
            Lateral velocity [m/s]
        """
        # Transform world velocities to body frame
        vx_world = self.current_state['vx']
        vy_world = self.current_state['vy']
        heading = self.current_state['heading']
        
        # Rotation to body frame
        cos_h = np.cos(heading)
        sin_h = np.sin(heading)
        
        vx_body = cos_h * vx_world + sin_h * vy_world
        vy_body = -sin_h * vx_world + cos_h * vy_world
        
        return vy_body
    
    def _estimate_velocity_from_history(self):
        """Estimate velocity from position history using least squares."""
        if len(self.position_history) < 2:
            return
        
        # Get recent positions
        positions = list(self.position_history)
        
        # Calculate velocities using finite differences
        vx_estimates = []
        vy_estimates = []
        
        for i in range(1, len(positions)):
            dt = (positions[i]['time'] - positions[i-1]['time']).to_sec()
            if dt > 0 and dt < 1.0:
                vx = (positions[i]['x'] - positions[i-1]['x']) / dt
                vy = (positions[i]['y'] - positions[i-1]['y']) / dt
                vx_estimates.append(vx)
                vy_estimates.append(vy)
        
        # Use median for robustness
        if vx_estimates:
            self.current_state['vx'] = np.median(vx_estimates)
            self.current_state['vy'] = np.median(vy_estimates)
    
    def _quaternion_to_yaw(self, quaternion) -> float:
        """Convert quaternion to yaw angle."""
        if hasattr(quaternion, 'w'):
            # ROS message type
            w, x, y, z = quaternion.w, quaternion.x, quaternion.y, quaternion.z
        else:
            # Tuple or list
            x, y, z, w = quaternion
        
        # Yaw from quaternion
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        return np.arctan2(siny_cosp, cosy_cosp)
    
    def _normalize_angle(self, angle: float) -> float:
        """Normalize angle to [-pi, pi]."""
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle
    
    def is_state_valid(self) -> bool:
        """Check if current state estimate is valid."""
        return self.is_valid and self.is_initialized
    
    def reset(self):
        """Reset state estimator."""
        self.current_state = {k: 0.0 for k in self.current_state}
        self.last_state = None
        self.last_time = None
        self.position_history.clear()
        self.velocity_history.clear()
        self.is_initialized = False
        self.is_valid = False