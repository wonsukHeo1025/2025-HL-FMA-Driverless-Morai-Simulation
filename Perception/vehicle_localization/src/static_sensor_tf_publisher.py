#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import ast
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped, Quaternion


def quaternion_from_euler(roll: float, pitch: float, yaw: float) -> Quaternion:
    """Convert roll, pitch, yaw (radians) to a Quaternion message."""
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return Quaternion(x=qx, y=qy, z=qz, w=qw)


def _coerce_vec3(value, name: str):
    """Accept list/tuple or string like "[x, y, z]" and return list of 3 floats."""
    try:
        if isinstance(value, (list, tuple)):
            if len(value) != 3:
                raise ValueError(f"{name} must have 3 elements")
            return [float(value[0]), float(value[1]), float(value[2])]
        if isinstance(value, str):
            s = value.strip()
            try:
                parsed = ast.literal_eval(s)
            except Exception:
                # Fallback simple split
                s2 = s.strip().lstrip('[').rstrip(']')
                parts = [p for p in s2.replace(',', ' ').split() if p]
                if len(parts) != 3:
                    raise
                return [float(parts[0]), float(parts[1]), float(parts[2])]
            else:
                return _coerce_vec3(parsed, name)
        raise TypeError(f"Unsupported type for {name}: {type(value)}")
    except Exception as e:
        rospy.logerr(f"Failed to parse {name}='{value}': {e}")
        raise


def make_tf(parent: str, child: str, xyz, rpy=None) -> TransformStamped:
    if rpy is None:
        rpy = (0.0, 0.0, 0.0)

    t = TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = parent
    t.child_frame_id = child
    x, y, z = _coerce_vec3(xyz, f"{child}_xyz")
    r, p, yw = _coerce_vec3(rpy, f"{child}_rpy")
    t.transform.translation.x = x
    t.transform.translation.y = y
    t.transform.translation.z = z
    t.transform.rotation = quaternion_from_euler(r, p, yw)
    return t


def main():
    rospy.init_node('static_sensor_tf_publisher', anonymous=False)

    # Parent (vehicle) frame
    base_frame = rospy.get_param('~base_frame', 'base')

    # Child frames
    gps_frame = rospy.get_param('~gps_frame', 'GPS')
    lidar_frame = rospy.get_param('~lidar_frame', 'velodyne')
    imu_frame = rospy.get_param('~imu_frame', 'IMU')
    camera_frame = rospy.get_param('~camera_frame', 'Camera')

    # Offsets (FLU from vehicle center at rear-axle center)
    gps_xyz = rospy.get_param('~gps_xyz', [0.0, 0.0, 1.2])
    lidar_xyz = rospy.get_param('~lidar_xyz', [1.6, 0.0, 1.22])
    imu_xyz = rospy.get_param('~imu_xyz', [0.0, 0.0, 1.18])
    camera_xyz = rospy.get_param('~camera_xyz', [3.5, 0.0, 0.6])

    # Optional orientation offsets (radians). Defaults align sensor frames to vehicle FLU.
    gps_rpy = rospy.get_param('~gps_rpy', [0.0, 0.0, 0.0])
    lidar_rpy = rospy.get_param('~lidar_rpy', [0.0, 0.0, 0.0])
    imu_rpy = rospy.get_param('~imu_rpy', [0.0, 0.0, 0.0])
    camera_rpy = rospy.get_param('~camera_rpy', [0.0, 0.0, 0.0])

    broadcaster = tf2_ros.StaticTransformBroadcaster()

    transforms = [
        make_tf(base_frame, gps_frame, gps_xyz, gps_rpy),
        make_tf(base_frame, lidar_frame, lidar_xyz, lidar_rpy),
        make_tf(base_frame, imu_frame, imu_xyz, imu_rpy),
        make_tf(base_frame, camera_frame, camera_xyz, camera_rpy),
    ]

    # Publish once; keep node alive so latching serves late subscribers.
    broadcaster.sendTransform(transforms)
    rospy.loginfo("Published static TFs: %s", ", ".join([f"{base_frame}->{t.child_frame_id}" for t in transforms]))
    rospy.spin()


if __name__ == '__main__':
    main()
