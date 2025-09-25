#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Path Visualizer Node

Visualizes path and MPC predictions in RViz.
"""

import rospy
import numpy as np
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from custom_interface.msg import LateralMpcDebug


class PathVisualizer:
    """Visualize paths and MPC predictions."""
    
    def __init__(self):
        """Initialize visualizer."""
        rospy.init_node('path_visualizer')
        
        # Publishers
        self.path_marker_pub = rospy.Publisher('/lateral_mpc/path_markers', MarkerArray, queue_size=1)
        self.prediction_pub = rospy.Publisher('/lateral_mpc/predicted_path_viz', Path, queue_size=1)
        self.error_marker_pub = rospy.Publisher('/lateral_mpc/error_markers', MarkerArray, queue_size=1)
        
        # Subscribers
        self.path_sub = rospy.Subscriber('/planning/global/path', Path, self.path_callback)
        self.debug_sub = rospy.Subscriber('/lateral_mpc/debug', LateralMpcDebug, self.debug_callback)
        
        # Data storage
        self.current_path = None
        self.current_debug = None
        
        # Visualization timer
        self.viz_timer = rospy.Timer(rospy.Duration(0.1), self.visualize)
        
        rospy.loginfo("Path visualizer initialized")
    
    def path_callback(self, msg):
        """Store path message."""
        self.current_path = msg
    
    def debug_callback(self, msg):
        """Store debug message."""
        self.current_debug = msg
    
    def visualize(self, event):
        """Main visualization callback."""
        if self.current_path:
            self.visualize_path()
        
        if self.current_debug:
            self.visualize_predictions()
            self.visualize_errors()
    
    def visualize_path(self):
        """Visualize reference path with curvature coloring."""
        if not self.current_path:
            return
        
        markers = MarkerArray()
        
        # Path line strip
        path_marker = Marker()
        path_marker.header = self.current_path.header
        path_marker.ns = "reference_path"
        path_marker.id = 0
        path_marker.type = Marker.LINE_STRIP
        path_marker.action = Marker.ADD
        path_marker.scale.x = 0.2
        path_marker.color.r = 0.0
        path_marker.color.g = 1.0
        path_marker.color.b = 0.0
        path_marker.color.a = 0.8
        
        for pose in self.current_path.poses:
            path_marker.points.append(pose.pose.position)
        
        markers.markers.append(path_marker)
        
        # Waypoint spheres
        for i, pose in enumerate(self.current_path.poses[::10]):  # Every 10th point
            sphere = Marker()
            sphere.header = self.current_path.header
            sphere.ns = "waypoints"
            sphere.id = i + 1
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose = pose.pose
            sphere.scale.x = 0.3
            sphere.scale.y = 0.3
            sphere.scale.z = 0.3
            sphere.color.r = 0.0
            sphere.color.g = 0.8
            sphere.color.b = 0.2
            sphere.color.a = 0.6
            
            markers.markers.append(sphere)
        
        self.path_marker_pub.publish(markers)
    
    def visualize_predictions(self):
        """Visualize MPC predicted trajectory."""
        if not self.current_debug or not self.current_debug.predicted_lateral_errors:
            return
        
        # Create predicted path
        pred_path = Path()
        pred_path.header.stamp = rospy.Time.now()
        pred_path.header.frame_id = "reference"
        
        # This is simplified - would need actual predicted positions
        # For now, just show relative to current position
        for i, lateral_error in enumerate(self.current_debug.predicted_lateral_errors):
            pose = PoseStamped()
            pose.header = pred_path.header
            
            # Simple visualization: offset from straight line
            pose.pose.position.x = i * 1.0  # 1m spacing
            pose.pose.position.y = -lateral_error  # Negative because error is signed
            pose.pose.position.z = 0.0
            
            pred_path.poses.append(pose)
        
        self.prediction_pub.publish(pred_path)
    
    def visualize_errors(self):
        """Visualize tracking errors."""
        if not self.current_debug:
            return
        
        markers = MarkerArray()
        
        # Lateral error text
        lat_text = Marker()
        lat_text.header.stamp = rospy.Time.now()
        lat_text.header.frame_id = "base"
        lat_text.ns = "errors"
        lat_text.id = 0
        lat_text.type = Marker.TEXT_VIEW_FACING
        lat_text.action = Marker.ADD
        lat_text.pose.position.x = 2.0
        lat_text.pose.position.y = 0.0
        lat_text.pose.position.z = 2.0
        lat_text.scale.z = 0.5
        lat_text.color.r = 1.0
        lat_text.color.g = 1.0
        lat_text.color.b = 1.0
        lat_text.color.a = 1.0
        lat_text.text = f"Lateral Error: {self.current_debug.lateral_error:.2f}m"
        
        markers.markers.append(lat_text)
        
        # Heading error text
        head_text = Marker()
        head_text.header = lat_text.header
        head_text.ns = "errors"
        head_text.id = 1
        head_text.type = Marker.TEXT_VIEW_FACING
        head_text.action = Marker.ADD
        head_text.pose.position.x = 2.0
        head_text.pose.position.y = 0.0
        head_text.pose.position.z = 1.5
        head_text.scale.z = 0.5
        head_text.color = lat_text.color
        head_text.text = f"Heading Error: {np.degrees(self.current_debug.heading_error):.1f}°"
        
        markers.markers.append(head_text)
        
        # Steering command arrow
        arrow = Marker()
        arrow.header = lat_text.header
        arrow.ns = "steering"
        arrow.id = 2
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD
        arrow.pose.position.x = 2.0
        arrow.pose.position.y = 0.0
        arrow.pose.position.z = 0.0
        
        # Rotate arrow based on steering command
        from tf.transformations import quaternion_from_euler
        q = quaternion_from_euler(0, 0, self.current_debug.steering_command)
        arrow.pose.orientation.x = q[0]
        arrow.pose.orientation.y = q[1]
        arrow.pose.orientation.z = q[2]
        arrow.pose.orientation.w = q[3]
        
        arrow.scale.x = 2.0  # Length
        arrow.scale.y = 0.2  # Width
        arrow.scale.z = 0.2  # Height
        
        # Color based on steering magnitude
        steering_normalized = abs(self.current_debug.steering_command) / 0.7
        arrow.color.r = steering_normalized
        arrow.color.g = 1.0 - steering_normalized
        arrow.color.b = 0.0
        arrow.color.a = 0.8
        
        markers.markers.append(arrow)
        
        self.error_marker_pub.publish(markers)


def main():
    """Main function."""
    try:
        visualizer = PathVisualizer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
