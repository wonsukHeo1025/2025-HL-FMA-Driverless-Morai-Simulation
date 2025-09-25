#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Front Obstacle Monitor

Subscribes to clustered obstacle centers (Marker POINTS) or processed
PointCloud2, applies a simple front-ROI filter, and logs the nearest
obstacle distance to terminal. Optionally publishes the distance.

Parameters (private ~):
  source: 'clusters' | 'points' (default: 'clusters')
  cluster_topic: topic for visualization_msgs/Marker (default: /vis/perception/lidar/cluster_markers)
  points_topic:  topic for sensor_msgs/PointCloud2 (default: /perception/lidar/processed_points)
  x_min, x_max:  ROI forward range in meters (defaults: 0.0, 40.0)
  y_half:        lateral half width in meters (default: 2.0)
  angle_min_deg, angle_max_deg: bearing range in degrees (defaults: -45, 45)
  distance_type: 'euclidean' | 'along_x' (default: 'euclidean')
  enable_logging: whether to log nearest info (default: true)
  log_period:    seconds between logs (default: 0.5)
  publish_distance_topic: bool to publish Float32 (default: false)
  distance_topic: name for distance topic (default: /perception/obstacle/nearest_distance)
"""

import math
import rospy
from typing import Iterable, Tuple, Optional

from visualization_msgs.msg import Marker
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2 as pc2
from std_msgs.msg import Float32


class FrontObstacleMonitor:
    def __init__(self):
        self.nh = rospy.get_param  # shorthand

        # Parameters
        self.source = rospy.get_param('~source', 'clusters').strip().lower()
        self.cluster_topic = rospy.get_param('~cluster_topic', '/vis/perception/lidar/cluster_markers')
        self.points_topic = rospy.get_param('~points_topic', '/perception/lidar/processed_points')

        self.x_min = float(rospy.get_param('~x_min', 0.0))
        self.x_max = float(rospy.get_param('~x_max', 40.0))
        self.y_half = float(rospy.get_param('~y_half', 2.0))
        self.angle_min_deg = float(rospy.get_param('~angle_min_deg', -45.0))
        self.angle_max_deg = float(rospy.get_param('~angle_max_deg', 45.0))

        self.distance_type = rospy.get_param('~distance_type', 'euclidean').strip().lower()
        if self.distance_type not in ('euclidean', 'along_x'):
            rospy.logwarn("[front_obstacle_monitor] Unknown distance_type '%s', fallback to 'euclidean'",
                          self.distance_type)
            self.distance_type = 'euclidean'

        self.enable_logging = bool(rospy.get_param('~enable_logging', True))
        self.log_period = float(rospy.get_param('~log_period', 0.5))
        self.publish_distance_topic = bool(rospy.get_param('~publish_distance_topic', False))
        self.distance_topic = rospy.get_param('~distance_topic', '/perception/obstacle/nearest_distance')

        self.prev_log_time = rospy.Time(0)

        # Publisher (optional)
        self.dist_pub = None
        if self.publish_distance_topic:
            self.dist_pub = rospy.Publisher(self.distance_topic, Float32, queue_size=1)

        # Subscriber
        if self.source == 'points':
            rospy.Subscriber(self.points_topic, PointCloud2, self._on_points, queue_size=1)
            rospy.loginfo("[front_obstacle_monitor] Source=points, topic=%s", self.points_topic)
        else:
            rospy.Subscriber(self.cluster_topic, Marker, self._on_clusters, queue_size=1)
            rospy.loginfo("[front_obstacle_monitor] Source=clusters, topic=%s", self.cluster_topic)

        rospy.loginfo("[front_obstacle_monitor] ROI: x=[%.1f, %.1f], |y|<=%.1f, angle=[%.0f, %.0f] deg",
                      self.x_min, self.x_max, self.y_half, self.angle_min_deg, self.angle_max_deg)
        rospy.loginfo("[front_obstacle_monitor] distance_type=%s, log_period=%.2fs, publish_distance_topic=%s, enable_logging=%s",
                      self.distance_type, self.log_period, str(self.publish_distance_topic), str(self.enable_logging))

    # ----------------- Callbacks -----------------
    def _on_clusters(self, msg: Marker):
        if msg.type != Marker.POINTS:
            # Accept anyway; we only read points array
            pass
        pts = ((p.x, p.y) for p in msg.points)
        nearest = self._nearest_in_roi(pts)
        self._log_and_publish(nearest, msg.header.frame_id)

    def _on_points(self, msg: PointCloud2):
        # Iterate lazily over x,y fields
        pts_iter = ((p[0], p[1]) for p in pc2.read_points(msg, field_names=("x", "y"), skip_nans=True))
        nearest = self._nearest_in_roi(pts_iter)
        self._log_and_publish(nearest, msg.header.frame_id)

    # ----------------- Core logic -----------------
    def _in_roi(self, x: float, y: float) -> bool:
        if not (self.x_min <= x <= self.x_max):
            return False
        if abs(y) > self.y_half:
            return False
        ang = math.degrees(math.atan2(y, x))
        if ang < self.angle_min_deg or ang > self.angle_max_deg:
            return False
        return True

    def _dist(self, x: float, y: float) -> float:
        if self.distance_type == 'along_x':
            return float(x)
        return math.hypot(x, y)

    def _nearest_in_roi(self, pts_xy: Iterable[Tuple[float, float]]) -> Optional[Tuple[float, float, float]]:
        best = None  # (d, x, y)
        for x, y in pts_xy:
            if not self._in_roi(x, y):
                continue
            d = self._dist(x, y)
            if best is None or d < best[0]:
                best = (d, x, y)
        return best  # or None

    def _log_and_publish(self, nearest: Optional[Tuple[float, float, float]], frame: str):
        if self.enable_logging:
            now = rospy.Time.now()
            if (now - self.prev_log_time).to_sec() >= self.log_period:
                self.prev_log_time = now
                if nearest is None:
                    rospy.loginfo("[front_obstacle] No obstacle in front ROI")
                else:
                    d, x, y = nearest
                    rospy.loginfo("[front_obstacle] nearest=%.2f m (x=%.2f, y=%.2f) frame=%s", d, x, y, frame)

        if self.dist_pub is not None:
            msg = Float32()
            msg.data = nearest[0] if nearest is not None else float('inf')
            self.dist_pub.publish(msg)


def main():
    rospy.init_node('front_obstacle_monitor', anonymous=False)
    _ = FrontObstacleMonitor()
    rospy.spin()


if __name__ == '__main__':
    main()
