#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Path Processing Utilities (packaged)

This is the packaged version of path_utils for import as
global_planner.path_utils.
"""

import numpy as np
import math
from geometry_msgs.msg import Quaternion
import tf.transformations as tf_trans


def quaternion_to_yaw(q):
    euler = tf_trans.euler_from_quaternion([q.x, q.y, q.z, q.w])
    return euler[2]


def yaw_to_quaternion(yaw):
    q = tf_trans.quaternion_from_euler(0, 0, yaw)
    return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])


def normalize_angle(angle):
    while angle > np.pi:
        angle -= 2 * np.pi
    while angle < -np.pi:
        angle += 2 * np.pi
    return angle


def calculate_path_curvature(e, n):
    if len(e) < 3:
        return np.zeros_like(e)
    de = np.gradient(e)
    dn = np.gradient(n)
    dde = np.gradient(de)
    ddn = np.gradient(dn)
    numerator = np.abs(de * ddn - dn * dde)
    denominator = (de**2 + dn**2) ** (3 / 2)
    curvature = np.zeros_like(e)
    valid = denominator > 1e-10
    curvature[valid] = numerator[valid] / denominator[valid]
    return curvature


def calculate_path_length(e, n):
    if len(e) < 2:
        return 0.0
    distances = np.sqrt(np.diff(e) ** 2 + np.diff(n) ** 2)
    return np.sum(distances)


def find_closest_point_index(path_e, path_n, current_e, current_n):
    distances = np.sqrt((path_e - current_e) ** 2 + (path_n - current_n) ** 2)
    return int(np.argmin(distances))


def resample_path_linear(e, n, spacing):
    if len(e) < 2:
        return e, n
    distances = np.sqrt(np.diff(e) ** 2 + np.diff(n) ** 2)
    cumulative = np.concatenate([[0], np.cumsum(distances)])
    total_length = cumulative[-1]
    if total_length < spacing:
        return e, n
    n_points = int(total_length / spacing) + 1
    target = np.linspace(0, total_length, n_points)
    resampled_e = np.interp(target, cumulative, e)
    resampled_n = np.interp(target, cumulative, n)
    return resampled_e, resampled_n


def smooth_path_savgol(e, n, window_size=11, poly_order=3):
    from scipy.signal import savgol_filter
    if len(e) < window_size:
        return e, n
    if window_size % 2 == 0:
        window_size += 1
    smoothed_e = savgol_filter(e, window_size, poly_order)
    smoothed_n = savgol_filter(n, window_size, poly_order)
    return smoothed_e, smoothed_n


def calculate_path_yaw(e, n):
    """Vectorized yaw calculation from path points.

    Uses atan2(diff(n), diff(e)) and propagates the last yaw to the end.
    """
    e = np.asarray(e, dtype=float)
    n = np.asarray(n, dtype=float)
    m = len(e)
    if m < 2:
        return np.zeros(m)
    de = np.diff(e)
    dn = np.diff(n)
    yaw = np.empty(m, dtype=float)
    yaw[:-1] = np.arctan2(dn, de)
    yaw[-1] = yaw[-2]
    return yaw


def generate_curvature_based_speed_profile(e, n, max_speed, min_speed, max_curvature=None):
    if len(e) < 3:
        return np.full(len(e), max_speed)
    curvature = calculate_path_curvature(e, n)
    if max_curvature is None:
        max_curvature = np.percentile(np.abs(curvature), 95)
        if max_curvature < 1e-6:
            max_curvature = 1.0
    speed_factor = 1.0 - np.clip(np.abs(curvature) / max_curvature, 0, 1)
    speeds = min_speed + (max_speed - min_speed) * speed_factor
    return speeds


def extract_local_path(global_e, global_n, current_idx, lookahead_distance):
    if current_idx >= len(global_e) - 1:
        return global_e[current_idx:], global_n[current_idx:], current_idx, len(global_e) - 1
    start_idx = current_idx
    cumulative_distance = 0.0
    end_idx = start_idx
    for i in range(start_idx, len(global_e) - 1):
        dist = math.sqrt((global_e[i + 1] - global_e[i]) ** 2 + (global_n[i + 1] - global_n[i]) ** 2)
        cumulative_distance += dist
        if cumulative_distance >= lookahead_distance:
            end_idx = i + 1
            break
    if end_idx <= start_idx:
        end_idx = len(global_e) - 1
    return global_e[start_idx : end_idx + 1], global_n[start_idx : end_idx + 1], start_idx, end_idx


def transform_to_vehicle_frame(path_e, path_n, vehicle_e, vehicle_n, vehicle_yaw):
    cos_yaw = math.cos(vehicle_yaw)
    sin_yaw = math.sin(vehicle_yaw)
    de = path_e - vehicle_e
    dn = path_n - vehicle_n
    local_x = de * cos_yaw + dn * sin_yaw
    local_y = -de * sin_yaw + dn * cos_yaw
    return local_x, local_y


def validate_path_data(e, n, min_points=2, max_distance=None):
    if len(e) != len(n):
        return False, "e and n arrays must have same length"
    if len(e) < min_points:
        return False, f"Path must have at least {min_points} points"
    if np.any(~np.isfinite(e)) or np.any(~np.isfinite(n)):
        return False, "Path contains NaN or infinite values"
    if max_distance is not None:
        distances = np.sqrt(np.diff(e) ** 2 + np.diff(n) ** 2)
        if np.any(distances > max_distance):
            return False, f"Path contains segments longer than {max_distance}"
    return True, "Path data is valid"


def calculate_path_statistics(x, y):
    if len(x) < 2:
        return {}
    stats = {
        'total_points': len(x),
        'path_length': calculate_path_length(x, y),
        'x_range': [float(np.min(x)), float(np.max(x))],
        'y_range': [float(np.min(y)), float(np.max(y))],
        'avg_spacing': calculate_path_length(x, y) / (len(x) - 1),
        'bounding_box_area': (np.max(x) - np.min(x)) * (np.max(y) - np.min(y)),
    }
    if len(x) >= 3:
        curvature = calculate_path_curvature(x, y)
        stats.update({
            'max_curvature': float(np.max(np.abs(curvature))),
            'avg_curvature': float(np.mean(np.abs(curvature))),
            'curvature_std': float(np.std(curvature)),
        })
    return stats


def interpolate_path_points(x, y, target_distances):
    if len(x) < 2 or len(target_distances) == 0:
        return np.array([]), np.array([])
    segment_distances = np.sqrt(np.diff(x) ** 2 + np.diff(y) ** 2)
    cumulative_distances = np.concatenate([[0], np.cumsum(segment_distances)])
    interp_x = np.interp(target_distances, cumulative_distances, x)
    interp_y = np.interp(target_distances, cumulative_distances, y)
    return interp_x, interp_y


def find_path_direction_changes(x, y, angle_threshold=0.1):
    if len(x) < 3:
        return np.array([])
    yaw = calculate_path_yaw(x, y)
    yaw_diff = np.abs(np.diff(yaw))
    yaw_diff = np.minimum(yaw_diff, 2 * np.pi - yaw_diff)
    change_indices = np.where(yaw_diff > angle_threshold)[0] + 1
    return change_indices


def smooth_yaw_angles(yaw, window_size=5):
    if len(yaw) < window_size:
        return yaw
    kernel = np.ones(window_size) / window_size
    smoothed_yaw = np.convolve(yaw, kernel, mode='same')
    smoothed_yaw = np.unwrap(smoothed_yaw)
    return smoothed_yaw
