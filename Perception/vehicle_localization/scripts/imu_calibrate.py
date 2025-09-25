#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Imu
import numpy as np
import os
import yaml


class ImuCalibrator:
    def __init__(self):
        rospy.init_node('imu_calibrate', anonymous=False)

        self.duration = rospy.get_param('~duration', 600.0)  # seconds
        self.topic = rospy.get_param('~imu_topic', '/imu')

        self.acc_samples = []
        self.gyro_samples = []
        self.quat_samples = []

        rospy.Subscriber(self.topic, Imu, self._imu_cb, queue_size=500, tcp_nodelay=True)
        rospy.loginfo(f"Collecting IMU data from {self.topic} for {self.duration:.1f}s...")

    def _imu_cb(self, msg: Imu):
        self.acc_samples.append([msg.linear_acceleration.x,
                                 msg.linear_acceleration.y,
                                 msg.linear_acceleration.z])
        self.gyro_samples.append([msg.angular_velocity.x,
                                  msg.angular_velocity.y,
                                  msg.angular_velocity.z])
        self.quat_samples.append([msg.orientation.x,
                                  msg.orientation.y,
                                  msg.orientation.z,
                                  msg.orientation.w])

    def run(self):
        start = rospy.Time.now().to_sec()
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            if rospy.Time.now().to_sec() - start >= self.duration:
                break
            rate.sleep()

        if len(self.acc_samples) < 10:
            rospy.logerr('Not enough IMU samples. Increase duration or check topic.')
            return 1

        acc = np.array(self.acc_samples)
        gyro = np.array(self.gyro_samples)

        # Bias estimation (assuming device static):
        # - Accelerometer: subtract gravity on Z
        acc_bias = acc.mean(axis=0)
        acc_bias[2] -= 9.81

        gyro_bias = gyro.mean(axis=0)

        # Noise (std dev)
        acc_noise = acc.std(axis=0)
        gyro_noise = gyro.std(axis=0)

        # Prepare YAML payload
        calib = {
            'acc_bias': [float(acc_bias[0]), float(acc_bias[1]), float(acc_bias[2])],
            'gyro_bias': [float(gyro_bias[0]), float(gyro_bias[1]), float(gyro_bias[2])],
            'acc_noise': [float(acc_noise[0]), float(acc_noise[1]), float(acc_noise[2])],
            'gyro_noise': [float(gyro_noise[0]), float(gyro_noise[1]), float(gyro_noise[2])]
        }

        # Save under package data directory by default
        pkg_share = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        # scripts/ → package root
        pkg_root = pkg_share
        data_dir = os.path.join(pkg_root, 'data')
        os.makedirs(data_dir, exist_ok=True)
        out_path = os.path.join(data_dir, 'imu_calibration.yaml')
        with open(out_path, 'w') as f:
            yaml.safe_dump(calib, f, default_flow_style=False)
        print(f"Saved calibration to: {out_path}")

        # Also print to stdout for quick copy/paste
        print(yaml.safe_dump(calib, default_flow_style=False))

        return 0


def main():
    node = ImuCalibrator()
    code = node.run()
    raise SystemExit(code)


if __name__ == '__main__':
    main()


