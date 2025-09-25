#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Path Processing Module

Handles path-related calculations including:
- Path interpolation
- Curvature calculation
- Error computation
- Reference point extraction
"""

import numpy as np
from typing import Dict, List, Tuple, Optional
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import tf2_geometry_msgs


class PathProcessor:
    """Process path information for MPC controller."""
    
    def __init__(self):
        """Initialize path processor."""
        self.path_points = []
        self.curvatures = []
        self.path_length = 0.0
        self.is_valid = False
        # # Index selection hysteresis (simple clamp) state and params - COMMENTED OUT
        # self.last_idx: int = -1
        # self.B_back: int = 3          # allowed backward steps
        # self.F_fwd: int = 12          # allowed forward steps
        # self.d_reinit: float = 5.0    # [m] re-sync threshold when far from window
        # self.ds_mean: float = 0.5     # [m] mean segment length (updated per path)
        
    def update_path(self, path_msg: Path):
        """
        Update path from ROS message.
        
        Args:
            path_msg: ROS Path message
        """
        self.path_points = []
        
        for pose_stamped in path_msg.poses:
            point = {
                'x': pose_stamped.pose.position.x,
                'y': pose_stamped.pose.position.y,
                'z': pose_stamped.pose.position.z,
                'heading': self._quaternion_to_yaw(pose_stamped.pose.orientation)
            }
            self.path_points.append(point)
        
        # Calculate curvatures and path length
        if len(self.path_points) > 1:
            self._calculate_curvatures()
            self._calculate_path_length()
            # # compute mean segment length ignoring degenerate segments - COMMENTED OUT
            # segs = []
            # for i in range(1, len(self.path_points)):
            #     p1 = self.path_points[i-1]
            #     p2 = self.path_points[i]
            #     dist = np.sqrt((p2['x']-p1['x'])**2 + (p2['y']-p1['y'])**2)
            #     if dist > 1e-6:
            #         segs.append(dist)
            # if len(segs) > 0:
            #     self.ds_mean = float(np.mean(segs))
            self.is_valid = True
        else:
            self.is_valid = False
        # # Clamp existing last_idx to new path size (avoid forced reset) - COMMENTED OUT
        # if self.is_valid and self.last_idx >= 0:
        #     self.last_idx = min(self.last_idx, len(self.path_points) - 1)
        # elif not self.is_valid:
        #     self.last_idx = -1
    
    def _quaternion_to_yaw(self, quaternion):
        """Convert quaternion to yaw angle."""
        # Simple conversion for 2D
        siny_cosp = 2 * (quaternion.w * quaternion.z + 
                         quaternion.x * quaternion.y)
        cosy_cosp = 1 - 2 * (quaternion.y * quaternion.y + 
                             quaternion.z * quaternion.z)
        return np.arctan2(siny_cosp, cosy_cosp)
    
    def _calculate_curvatures(self):
        """Calculate curvature at each path point using three-point method."""
        n = len(self.path_points)
        self.curvatures = np.zeros(n)
        
        for i in range(1, n-1):
            p1 = self.path_points[i-1]
            p2 = self.path_points[i]
            p3 = self.path_points[i+1]
            
            self.curvatures[i] = self._three_point_curvature(p1, p2, p3)
        
        # Extrapolate endpoints
        if n > 2:
            self.curvatures[0] = self.curvatures[1]
            self.curvatures[-1] = self.curvatures[-2]
    
    def _three_point_curvature(self, p1: Dict, p2: Dict, p3: Dict) -> float:
        """
        Calculate curvature using three points.
        
        Uses Menger curvature formula: κ = 4*Area / (|p1-p2| * |p2-p3| * |p3-p1|)
        """
        # Extract coordinates
        x1, y1 = p1['x'], p1['y']
        x2, y2 = p2['x'], p2['y']
        x3, y3 = p3['x'], p3['y']
        
        # Calculate distances
        d12 = np.sqrt((x2-x1)**2 + (y2-y1)**2)
        d23 = np.sqrt((x3-x2)**2 + (y3-y2)**2)
        d31 = np.sqrt((x1-x3)**2 + (y1-y3)**2)
        
        # Handle collinear points
        if d12 < 1e-6 or d23 < 1e-6 or d31 < 1e-6:
            return 0.0
        
        # Calculate area using cross product
        area = 0.5 * abs((x2-x1)*(y3-y1) - (x3-x1)*(y2-y1))
        
        # Menger curvature
        if area < 1e-6:
            return 0.0
        
        curvature = 4 * area / (d12 * d23 * d31)
        
        # Add sign based on turn direction
        cross = (x2-x1)*(y3-y2) - (y2-y1)*(x3-x2)
        if cross < 0:
            curvature = -curvature
            
        return curvature
    
    def _calculate_path_length(self):
        """Calculate total path length."""
        self.path_length = 0.0
        
        for i in range(1, len(self.path_points)):
            p1 = self.path_points[i-1]
            p2 = self.path_points[i]
            dist = np.sqrt((p2['x']-p1['x'])**2 + (p2['y']-p1['y'])**2)
            self.path_length += dist
    
    def find_closest_point(self, current_pose: Dict, velocity: float = 0.0, Ts: float = 0.02) -> int:
        """
        Find closest point on path by simple distance.
        
        Args:
            current_pose: Dictionary with 'x', 'y', 'heading' keys
            velocity: Current vehicle velocity [m/s] (unused after simplification)
            Ts: Control period [s] (unused after simplification)
            
        Returns:
            Closest index on path
        """
        if not self.path_points:
            return -1
            
        gx = current_pose.get('x', 0.0)
        gy = current_pose.get('y', 0.0)
        
        # Simple global nearest point search
        min_dist = float('inf')
        closest_idx = 0
        
        for i, point in enumerate(self.path_points):
            dist = np.sqrt((point['x'] - gx)**2 + (point['y'] - gy)**2)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i
        
        return closest_idx
        
        # # ORIGINAL HYSTERESIS LOGIC - COMMENTED OUT
        # psi_vehicle = current_pose.get('heading', 0.0)
        # n = len(self.path_points)
        # 
        # # Initialize or recover when last index is invalid -> use global nearest once
        # if self.last_idx < 0 or self.last_idx >= n:
        #     min_dist_global = float('inf')
        #     idx_global = 0
        #     for i, point in enumerate(self.path_points):
        #         dist = np.sqrt((point['x'] - gx)**2 + (point['y'] - gy)**2)
        #         if dist < min_dist_global:
        #             min_dist_global = dist
        #             idx_global = i
        #     self.last_idx = idx_global
        #     return self.last_idx
        # 
        # # Progress prediction based on velocity
        # idx_hat = self.last_idx + int(round(max(0.0, velocity) * Ts / max(1e-3, self.ds_mean)))
        # 
        # # Dynamic forward window based on velocity (larger window at higher speeds)
        # F_dynamic = max(self.F_fwd, int(np.ceil(max(0.5, velocity) * 5.0 / max(1e-3, self.ds_mean))))
        # 
        # # Window bounds
        # i_min = max(0, self.last_idx - self.B_back)
        # i_max = min(n - 1, self.last_idx + F_dynamic)
        # 
        # # Multi-criteria cost weights
        # w_d = 1.0      # distance weight
        # w_psi = 0.8    # heading weight  
        # w_idx = 0.2    # progress weight
        # 
        # best_i = self.last_idx
        # best_J = float('inf')
        # 
        # for i in range(i_min, i_max + 1):
        #     p = self.path_points[i]
        #     
        #     # Distance cost
        #     d = np.sqrt((p['x'] - gx)**2 + (p['y'] - gy)**2)
        #     
        #     # Path heading from segment
        #     psi_path = self._get_path_heading_at_index(i)
        #     
        #     # Heading difference cost
        #     psi_diff = abs(self._normalize_angle(psi_path - psi_vehicle))
        #     
        #     # Progress difference cost
        #     idx_diff = abs(i - idx_hat)
        #     
        #     # Combined cost
        #     J = w_d * d + w_psi * psi_diff + w_idx * idx_diff
        #     
        #     # Optional: Forward-only gating (penalize backward points)
        #     if i < n - 1:
        #         # Get tangent vector
        #         p_next = self.path_points[i + 1]
        #         tx = p_next['x'] - p['x']
        #         ty = p_next['y'] - p['y']
        #         norm = np.sqrt(tx**2 + ty**2)
        #         if norm > 1e-6:
        #             tx /= norm
        #             ty /= norm
        #             # Check if point is behind vehicle relative to path direction
        #             dot_product = (p['x'] - gx) * tx + (p['y'] - gy) * ty
        #             if dot_product < 0.0:
        #                 J *= 1.5  # Penalize backward points
        #     
        #     if J < best_J:
        #         best_J = J
        #         best_i = i
        # 
        # # Check if need re-initialization (too far from window)
        # best_dist = np.sqrt((self.path_points[best_i]['x'] - gx)**2 + 
        #                    (self.path_points[best_i]['y'] - gy)**2)
        # 
        # if best_dist > self.d_reinit:
        #     # Re-sync with global search
        #     min_dist_global = float('inf')
        #     idx_global = 0
        #     for i, point in enumerate(self.path_points):
        #         dist = np.sqrt((point['x'] - gx)**2 + (point['y'] - gy)**2)
        #         if dist < min_dist_global:
        #             min_dist_global = dist
        #             idx_global = i
        #     self.last_idx = idx_global
        #     return self.last_idx
        # 
        # self.last_idx = best_i
        # return self.last_idx
    
    def _get_path_heading_at_index(self, idx: int) -> float:
        """
        Get path heading at given index from segment tangent.
        
        Args:
            idx: Path index
            
        Returns:
            Path heading in radians
        """
        if idx < 0 or idx >= len(self.path_points):
            return 0.0
            
        if idx < len(self.path_points) - 1:
            # Use forward segment
            p1 = self.path_points[idx]
            p2 = self.path_points[idx + 1]
            dx = p2['x'] - p1['x']
            dy = p2['y'] - p1['y']
            if abs(dx) > 1e-6 or abs(dy) > 1e-6:
                return np.arctan2(dy, dx)
        elif idx > 0:
            # Last point: use backward segment
            p1 = self.path_points[idx - 1]
            p2 = self.path_points[idx]
            dx = p2['x'] - p1['x']
            dy = p2['y'] - p1['y']
            if abs(dx) > 1e-6 or abs(dy) > 1e-6:
                return np.arctan2(dy, dx)
        
        # Fallback to stored heading if available
        return self.path_points[idx].get('heading', 0.0)
    
    def calculate_errors(self, current_pose: Dict, velocity: float = 0.0, Ts: float = 0.02) -> Tuple[float, float]:
        """
        Calculate lateral and heading errors.
        
        Args:
            current_pose: Dictionary with 'x', 'y', 'heading' keys
            velocity: Current vehicle velocity [m/s]
            Ts: Control period [s]
            
        Returns:
            Tuple of (lateral_error, heading_error) in meters and radians
        """
        if not self.is_valid:
            return 0.0, 0.0
        
        # Find closest point with hysteresis
        closest_idx = self.find_closest_point(current_pose, velocity, Ts)
        
        if closest_idx < 0 or closest_idx >= len(self.path_points) - 1:
            return 0.0, 0.0
        
        # Get path segment
        p1 = self.path_points[closest_idx]
        p2 = self.path_points[closest_idx + 1]
        
        # Path tangent vector
        dx = p2['x'] - p1['x']
        dy = p2['y'] - p1['y']
        segment_length = np.sqrt(dx**2 + dy**2)
        
        if segment_length < 1e-6:
            return 0.0, 0.0
        
        # Normalize tangent
        tx = dx / segment_length
        ty = dy / segment_length
        
        # Normal vector (perpendicular to tangent, pointing left)
        # MPC convention: positive lateral error when vehicle is to the LEFT of path
        # This matches the steering convention where positive steering = left turn
        nx = -ty   # Left 90-degree rotation (changed sign)
        ny = tx    # Left 90-degree rotation (changed sign)
        
        # Vector from path point to vehicle
        vx = current_pose['x'] - p1['x']
        vy = current_pose['y'] - p1['y']
        
        # Lateral error (signed distance to path)
        lateral_error = vx * nx + vy * ny
        
        # Path heading
        path_heading = np.arctan2(dy, dx)
        
        # Heading error
        heading_error = self._normalize_angle(current_pose['heading'] - path_heading)
        
        return lateral_error, heading_error
    
    def get_reference_points(self, current_idx: int, preview_distances: List[float]) -> List[Dict]:
        """
        Get reference points at specified preview distances.
        
        Args:
            current_idx: Current closest point index
            preview_distances: List of preview distances in meters
            
        Returns:
            List of reference point dictionaries
        """
        references = []
        
        for dist in preview_distances:
            ref_idx = self._get_index_at_distance(current_idx, dist)
            
            if 0 <= ref_idx < len(self.path_points):
                ref_point = self.path_points[ref_idx].copy()
                ref_point['curvature'] = self.curvatures[ref_idx] if ref_idx < len(self.curvatures) else 0.0
                references.append(ref_point)
            else:
                # Use last point if beyond path
                if self.path_points:
                    ref_point = self.path_points[-1].copy()
                    ref_point['curvature'] = 0.0
                    references.append(ref_point)
        
        return references
    
    def _get_index_at_distance(self, start_idx: int, distance: float) -> int:
        """
        Get path index at specified distance from start index.
        
        Args:
            start_idx: Starting index
            distance: Distance to travel along path
            
        Returns:
            Index at distance
        """
        if not self.path_points or start_idx < 0:
            return -1
        
        # Check upper bound
        if start_idx >= len(self.path_points):
            return len(self.path_points) - 1 if self.path_points else -1
        
        # Handle distance <= 0 case
        if distance <= 0:
            return start_idx
        
        accumulated_dist = 0.0
        current_idx = start_idx
        
        while current_idx < len(self.path_points) - 1:
            p1 = self.path_points[current_idx]
            p2 = self.path_points[current_idx + 1]
            
            segment_dist = np.sqrt((p2['x']-p1['x'])**2 + (p2['y']-p1['y'])**2)
            
            if accumulated_dist + segment_dist >= distance:
                # Target is within this segment
                # Could interpolate, but returning next index for simplicity
                return current_idx + 1
            
            accumulated_dist += segment_dist
            current_idx += 1
        
        return len(self.path_points) - 1
    
    def get_curvatures_ahead(self, current_idx: int, num_points: int, 
                            spacing_m: float = 1.0) -> np.ndarray:
        """
        Get curvatures at points ahead.
        
        Args:
            current_idx: Current index
            num_points: Number of points to get
            spacing_m: Spacing between points in meters
            
        Returns:
            Array of curvatures
        """
        # Validate and clamp current_idx
        if not self.path_points or self.curvatures is None or len(self.curvatures) == 0:
            return np.zeros(num_points)
        
        current_idx = max(0, min(current_idx, len(self.path_points) - 1))
        
        curvatures = []
        
        for i in range(num_points):
            dist = i * spacing_m
            idx = self._get_index_at_distance(current_idx, dist)
            
            if 0 <= idx < len(self.curvatures):
                curvatures.append(self.curvatures[idx])
            else:
                curvatures.append(0.0)
        
        return np.array(curvatures)
    
    def _normalize_angle(self, angle: float) -> float:
        """Normalize angle to [-pi, pi]."""
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle
    
    def get_path_completion(self, current_idx: int) -> float:
        """
        Get path completion percentage.
        
        Args:
            current_idx: Current closest point index
            
        Returns:
            Completion percentage (0-100)
        """
        if not self.path_points or current_idx < 0:
            return 0.0
        
        if current_idx >= len(self.path_points) - 1:
            return 100.0
        
        return (current_idx / (len(self.path_points) - 1)) * 100.0