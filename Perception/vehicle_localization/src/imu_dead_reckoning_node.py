#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import tf2_ros
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import geometry_msgs.msg as geom
import numpy as np


class ImuDeadReckoningNode:
    def __init__(self):
        rospy.init_node('imu_dead_reckoning', anonymous=False)

        self.reference_frame = rospy.get_param('~reference_frame', 'reference')
        self.base_frame = rospy.get_param('~base_frame', 'base')
        self.base_dr_frame = rospy.get_param('~base_dr_frame', 'base_dr')
        self.broadcast_tf = rospy.get_param('~broadcast_tf', True)
        self.out_topic = rospy.get_param('~out_topic', '/odometry/imu_only')

        self.odom_pub = rospy.Publisher(self.out_topic, Odometry, queue_size=20)

        # TF broadcaster (optional)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        # State (2D DR)
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.vx = 0.0
        self.vy = 0.0

        self.last_t = None

        imu_topic = rospy.get_param('~imu_topic', '/imu/processed')
        rospy.Subscriber(imu_topic, Imu, self._imu_cb, queue_size=200, tcp_nodelay=True)
        rospy.loginfo('imu_dead_reckoning started')

    def _imu_cb(self, msg: Imu):
        t = msg.header.stamp.to_sec() if msg.header.stamp else rospy.Time.now().to_sec()
        if self.last_t is None:
            self.last_t = t
            return

        dt = t - self.last_t
        if dt <= 0.0 or dt > 1.0:
            self.last_t = t
            return

        # Use IMU orientation yaw directly (global heading)
        q = msg.orientation
        # yaw = atan2(2*(w*z + x*y), 1 - 2*(y^2 + z^2))
        sin_yaw = 2.0 * (q.w * q.z + q.x * q.y)
        cos_yaw = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = np.arctan2(sin_yaw, cos_yaw)

        # Linear acceleration in body frame → rotate to world using yaw (2D)
        ax_b = msg.linear_acceleration.x
        ay_b = msg.linear_acceleration.y

        c = np.cos(self.yaw)
        s = np.sin(self.yaw)
        ax_w = ax_b * c - ay_b * s
        ay_w = ax_b * s + ay_b * c

        # Integrate velocity and position
        self.vx += ax_w * dt
        self.vy += ay_w * dt
        self.x += self.vx * dt
        self.y += self.vy * dt

        self.last_t = t

        # Publish odometry (no TF)
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = self.reference_frame
        odom.child_frame_id = self.base_dr_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        # Orientation: yaw only
        qz = np.sin(self.yaw / 2.0)
        qw = np.cos(self.yaw / 2.0)
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.pose.covariance = [0.0]*36

        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.angular.z = msg.angular_velocity.z
        odom.twist.covariance = [0.0]*36

        self.odom_pub.publish(odom)

        if self.broadcast_tf:
            t = geom.TransformStamped()
            t.header.stamp = odom.header.stamp
            t.header.frame_id = self.reference_frame
            t.child_frame_id = self.base_dr_frame
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(t)


def main():
    node = ImuDeadReckoningNode()
    rospy.spin()


if __name__ == '__main__':
    main()


