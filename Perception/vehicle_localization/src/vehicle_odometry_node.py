#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import tf2_ros
import geometry_msgs.msg as geom
from nav_msgs.msg import Odometry
from morai_msgs.msg import GPSMessage
from morai_msgs.msg import EgoVehicleStatus
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
import math

try:
    import utm as _utm
    _UTM_AVAILABLE = True
except Exception:
    _UTM_AVAILABLE = False


class VehicleOdometryNode:
    def __init__(self):
        rospy.init_node('vehicle_odometry', anonymous=False)

        # Frames and topics
        self.reference_frame = rospy.get_param('~reference_frame', 'reference')
        self.base_frame = rospy.get_param('~base_frame', 'base')
        self.odom_topic = rospy.get_param('~odometry_topic', '/odometry')
        self.gps_topic = rospy.get_param('~gps_topic', '/gps')
        self.imu_topic = rospy.get_param('~imu_topic', '/imu/processed')  # optional
        self.ego_topic = rospy.get_param('~ego_topic', '/Competition_topic')

        # Reference (for fallback equirectangular)
        self.ref_lat = float(rospy.get_param('~ref_lat', 37.338817432893464))
        self.ref_lon = float(rospy.get_param('~ref_lon', 127.89867211359652))
        self._ref_lat_r = math.radians(self.ref_lat)
        self._ref_lon_r = math.radians(self.ref_lon)
        self._R_EARTH = 6378137.0

        # Broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        # Publisher
        self.odom_pub = rospy.Publisher(self.odom_topic, Odometry, queue_size=10)

        # State
        self.last_map_e = None
        self.last_map_n = None
        self.last_speed = 0.0  # longitudinal speed (m/s)
        self.yaw = 0.0
        self.last_quat = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        self.last_angular_z = 0.0

        # Subscribers
        rospy.Subscriber(self.gps_topic, GPSMessage, self._gps_cb, queue_size=50, tcp_nodelay=True)
        rospy.Subscriber(self.imu_topic, Imu, self._imu_cb, queue_size=100, tcp_nodelay=True)
        rospy.Subscriber(self.ego_topic, EgoVehicleStatus, self._ego_cb, queue_size=50, tcp_nodelay=True)

        rospy.loginfo('vehicle_odometry started')

    def _gps_cb(self, msg: GPSMessage):
        if not _UTM_AVAILABLE:
            rospy.logwarn_once("utm package not available. Falling back to equirectangular conversion (set ~ref_lat/~ref_lon).")
            try:
                lat_r = math.radians(msg.latitude)
                lon_r = math.radians(msg.longitude)
                e = (lon_r - self._ref_lon_r) * math.cos(self._ref_lat_r) * self._R_EARTH
                n = (lat_r - self._ref_lat_r) * self._R_EARTH
                map_e = e - getattr(msg, 'eastOffset', 0.0)
                map_n = n - getattr(msg, 'northOffset', 0.0)
            except Exception as e:
                rospy.logwarn_throttle(5.0, 'Fallback conversion failed: %s', e)
                return
        else:
            try:
                utm_e, utm_n, _, _ = _utm.from_latlon(msg.latitude, msg.longitude)
                map_e = utm_e - msg.eastOffset
                map_n = utm_n - msg.northOffset
            except Exception as e:
                rospy.logwarn_throttle(5.0, 'GPS conversion failed: %s', e)
                return

        self.last_map_e = map_e
        self.last_map_n = map_n

        # Publish Odometry (2D)
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = self.reference_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = map_e
        odom.pose.pose.position.y = map_n
        odom.pose.pose.position.z = 0.0
        # Orientation from IMU if available
        odom.pose.pose.orientation = self.last_quat
        # Perfect (simulation) covariance → all zeros
        odom.pose.covariance = [0.0]*36

        # Twist in child frame (base): longitudinal speed and yaw rate
        odom.twist.twist.linear.x = self.last_speed
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = self.last_angular_z
        odom.twist.covariance = [0.0]*36
        self.odom_pub.publish(odom)

        # Broadcast TF reference → base (use current orientation)
        t = geom.TransformStamped()
        t.header.stamp = odom.header.stamp
        t.header.frame_id = self.reference_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = map_e
        t.transform.translation.y = map_n
        t.transform.translation.z = 0.0
        t.transform.rotation = self.last_quat
        self.tf_broadcaster.sendTransform(t)

    def _imu_cb(self, msg: Imu):
        # Orientation (global heading) assumed to be in world frame
        self.last_quat = msg.orientation
        self.last_angular_z = msg.angular_velocity.z

    def _ego_cb(self, msg: EgoVehicleStatus):
        # Longitudinal speed (m/s)
        try:
            self.last_speed = float(msg.velocity.x)
        except Exception:
            pass


def main():
    node = VehicleOdometryNode()
    rospy.spin()


if __name__ == '__main__':
    main()

