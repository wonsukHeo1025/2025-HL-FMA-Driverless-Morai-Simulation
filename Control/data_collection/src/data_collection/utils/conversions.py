#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Coordinate and unit conversion utilities."""

import numpy as np
from typing import Tuple


def gps_to_meters(latitude: float, longitude: float, 
                  altitude: float = 0.0) -> Tuple[float, float, float]:
    """Convert GPS coordinates to local meter coordinates.
    
    Uses accurate conversion formulas considering Earth's curvature.
    
    Args:
        latitude: Latitude in degrees
        longitude: Longitude in degrees
        altitude: Altitude in meters
        
    Returns:
        Tuple of (x, y, z) in meters
    """
    # Accurate latitude to meters conversion
    lat_to_m = (111132.92 
                - 559.82 * np.cos(2 * np.radians(latitude))
                + 1.175 * np.cos(4 * np.radians(latitude)))
    
    # Accurate longitude to meters conversion
    lon_to_m = (111412.84 * np.cos(np.radians(latitude))
                - 93.5 * np.cos(3 * np.radians(latitude)))
    
    x = longitude * lon_to_m
    y = latitude * lat_to_m
    z = altitude
    
    return x, y, z


def mps_to_kmph(velocity_mps: float) -> float:
    """Convert velocity from m/s to km/h.
    
    Args:
        velocity_mps: Velocity in meters per second
        
    Returns:
        Velocity in kilometers per hour
    """
    return velocity_mps * 3.6


def kmph_to_mps(velocity_kmph: float) -> float:
    """Convert velocity from km/h to m/s.
    
    Args:
        velocity_kmph: Velocity in kilometers per hour
        
    Returns:
        Velocity in meters per second
    """
    return velocity_kmph / 3.6


def normalize_angle(angle: float) -> float:
    """Normalize angle to [-pi, pi] range.
    
    Args:
        angle: Angle in radians
        
    Returns:
        Normalized angle in [-pi, pi]
    """
    while angle > np.pi:
        angle -= 2 * np.pi
    while angle < -np.pi:
        angle += 2 * np.pi
    return angle


def quaternion_to_euler(x: float, y: float, z: float, w: float) -> Tuple[float, float, float]:
    """Convert quaternion to Euler angles.
    
    Args:
        x, y, z, w: Quaternion components
        
    Returns:
        Tuple of (roll, pitch, yaw) in radians
    """
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)
    
    # Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = np.copysign(np.pi / 2, sinp)  # Use 90 degrees if out of range
    else:
        pitch = np.arcsin(sinp)
    
    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)
    
    return roll, pitch, yaw