#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import os
import yaml
import numpy as np

try:
    from scipy import signal as _sig
    _SCIPY_AVAILABLE = True
except Exception:
    _SCIPY_AVAILABLE = False
import rospy
from sensor_msgs.msg import Imu


class ImuProcessorNode:
    def __init__(self):
        rospy.init_node('imu_processor', anonymous=False)

        # Topics and calibration
        self.in_topic = rospy.get_param('~imu_in_topic', '/imu')
        self.out_topic = rospy.get_param('~imu_out_topic', '/imu/processed')
        self.calib_file = rospy.get_param('~calib_file', '')
        if self.calib_file:
            self._load_calibration(self.calib_file)
        else:
            self.acc_bias = self._get_vec_param('~acc_bias', [0.0, 0.0, 0.0])
            self.gyro_bias = self._get_vec_param('~gyro_bias', [0.0, 0.0, 0.0])

        self.pub = rospy.Publisher(self.out_topic, Imu, queue_size=50)

        rospy.Subscriber(self.in_topic, Imu, self._imu_cb, queue_size=200, tcp_nodelay=True)
        rospy.loginfo('imu_processor started')

        # Low-pass filter configuration
        self.use_lpf = rospy.get_param('~use_lpf', True)
        self.lpf_type = rospy.get_param('~lpf_type', 'ema')  # 'ema' or 'butter'
        self.sample_rate_hz = float(rospy.get_param('~sample_rate_hz', 100.0))
        self.cutoff_hz = float(rospy.get_param('~cutoff_hz', 5.0))  # for butter
        self.ema_alpha = float(rospy.get_param('~ema_alpha', 0.1))

        # Filter state
        self._acc_lp = np.zeros(3)
        self._gyro_lp = np.zeros(3)
        self._acc_init = False
        self._gyro_init = False

        # Butterworth setup
        self._butter_ba = None
        self._acc_zi = None
        self._gyro_zi = None
        if self.use_lpf and self.lpf_type == 'butter':
            if not _SCIPY_AVAILABLE:
                rospy.logwarn("SciPy not available; falling back to EMA LPF")
                self.lpf_type = 'ema'
            else:
                # 2nd order low-pass Butterworth
                try:
                    b, a = _sig.butter(N=2, Wn=self.cutoff_hz, btype='low', fs=self.sample_rate_hz)
                    self._butter_ba = (b, a)
                    zi = _sig.lfilter_zi(b, a)
                    # One zi vector per axis
                    self._acc_zi = [zi.copy(), zi.copy(), zi.copy()]
                    self._gyro_zi = [zi.copy(), zi.copy(), zi.copy()]
                except Exception as e:
                    rospy.logwarn(f"Butterworth init failed ({e}); falling back to EMA")
                    self.lpf_type = 'ema'

    def _get_vec_param(self, name, default):
        v = rospy.get_param(name, default)
        if isinstance(v, str):
            try:
                v = json.loads(v)
            except Exception:
                v = default
        if not (isinstance(v, list) and len(v) == 3):
            v = default
        return [float(v[0]), float(v[1]), float(v[2])]

    def _load_calibration(self, path):
        try:
            with open(path, 'r') as f:
                data = yaml.safe_load(f)
            self.acc_bias = [float(x) for x in data.get('acc_bias', [0.0, 0.0, 0.0])]
            self.gyro_bias = [float(x) for x in data.get('gyro_bias', [0.0, 0.0, 0.0])]
            rospy.loginfo(f"Loaded IMU calibration from {path}: acc_bias={self.acc_bias}, gyro_bias={self.gyro_bias}")
        except Exception as e:
            rospy.logwarn(f"Failed to load calibration file {path}: {e}. Falling back to zero biases.")
            self.acc_bias = [0.0, 0.0, 0.0]
            self.gyro_bias = [0.0, 0.0, 0.0]

    def _imu_cb(self, msg: Imu):
        out = Imu()
        out.header = msg.header

        # Copy orientation as-is (we are not estimating orientation here)
        out.orientation = msg.orientation
        out.orientation_covariance = msg.orientation_covariance

        # Bias-correct gyro then LPF
        gyro_raw = np.array([
            msg.angular_velocity.x - self.gyro_bias[0],
            msg.angular_velocity.y - self.gyro_bias[1],
            msg.angular_velocity.z - self.gyro_bias[2]
        ])
        gyro_f = self._apply_lpf(gyro_raw, is_acc=False) if self.use_lpf else gyro_raw
        out.angular_velocity.x = float(gyro_f[0])
        out.angular_velocity.y = float(gyro_f[1])
        out.angular_velocity.z = float(gyro_f[2])
        out.angular_velocity_covariance = msg.angular_velocity_covariance

        # Bias-correct accel (and keep gravity if present), then LPF
        acc_raw = np.array([
            msg.linear_acceleration.x - self.acc_bias[0],
            msg.linear_acceleration.y - self.acc_bias[1],
            msg.linear_acceleration.z - self.acc_bias[2]
        ])
        acc_f = self._apply_lpf(acc_raw, is_acc=True) if self.use_lpf else acc_raw
        out.linear_acceleration.x = float(acc_f[0])
        out.linear_acceleration.y = float(acc_f[1])
        out.linear_acceleration.z = float(acc_f[2])
        out.linear_acceleration_covariance = msg.linear_acceleration_covariance

        self.pub.publish(out)

    def _apply_lpf(self, vec: np.ndarray, is_acc: bool) -> np.ndarray:
        if self.lpf_type == 'ema':
            if is_acc:
                if not self._acc_init:
                    self._acc_lp = vec.copy()
                    self._acc_init = True
                else:
                    self._acc_lp = (1.0 - self.ema_alpha) * self._acc_lp + self.ema_alpha * vec
                return self._acc_lp
            else:
                if not self._gyro_init:
                    self._gyro_lp = vec.copy()
                    self._gyro_init = True
                else:
                    self._gyro_lp = (1.0 - self.ema_alpha) * self._gyro_lp + self.ema_alpha * vec
                return self._gyro_lp

        # Butterworth streaming
        if self._butter_ba is None:
            return vec
        b, a = self._butter_ba
        out = np.zeros_like(vec)
        if is_acc:
            for i in range(3):
                out[i], self._acc_zi[i] = _sig.lfilter(b, a, [vec[i]], zi=self._acc_zi[i])
            return out
        else:
            for i in range(3):
                out[i], self._gyro_zi[i] = _sig.lfilter(b, a, [vec[i]], zi=self._gyro_zi[i])
            return out


def main():
    node = ImuProcessorNode()
    rospy.spin()


if __name__ == '__main__':
    main()


