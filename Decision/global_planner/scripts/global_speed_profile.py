#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Global Speed Profile (script implementation)

Reads global waypoints and publishes a consolidated global speed profile:
- /planning/speed_profile/global (std_msgs/Float32MultiArray, latched)

Additionally, publishes the current closest waypoint index for debugging:
- /planning/global/current_idx (std_msgs/Int32, latched; name via ~current_idx_topic)

Traffic light stopline integration (lightweight):
- Hard-coded ENU stopline points are mapped to current global waypoints.
- When the traffic light state matches configured stop states (default: ['red']),
  the profile is gated to 0 from the next stopline index ahead of the vehicle.
- Physics passes (decel/accel limits) ensure a feasible deceleration to stop.

This keeps behavior reversible: on green, the base profile is restored and
republished using cached path/speeds.
"""

import rospy
import ast
import math
import numpy as np
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Float32MultiArray, String, Int32, UInt8
from visualization_msgs.msg import MarkerArray, Marker

from global_planner.path_utils import calculate_path_curvature

# ---------------------------------------------------------------------------
# Hard-coded ENU stopline coordinates (reference frame)
# These represent stoplines in front of traffic lights. When a red light is
# detected, the speed profile will be zeroed from the next stopline index ahead.
#
# Source files (ENU):
# - Decision/global_planner/data/first_traffic_1.csv
# - Decision/global_planner/data/first_traffic_2.csv
# - Decision/global_planner/data/second_traffic_1.csv
# - Decision/global_planner/data/second_traffic_2.csv
# Columns: index,e,n,u  -> we only use (e,n)
# ---------------------------------------------------------------------------
STOPLINES_ENU = [
    (617.268855, -725.685481),
    (614.796504, -723.697810),
    (771.645992, -825.783016),
    (773.944942, -827.570125),
]


class GlobalSpeedProfileNode:
    def __init__(self):
        rospy.init_node('global_speed_profile', anonymous=False)

        # Parameters
        self.waypoints_topic = rospy.get_param('~waypoints_topic', '/planning/global/path')
        # Consolidated speed profile topic (single source of truth)
        self.speed_topic = rospy.get_param('~speed_profile_topic', '/planning/speed_profile/global')
        # Base limits
        self.max_speed = float(rospy.get_param('~max_speed', 10.0))         # [m/s]
        self.min_speed = float(rospy.get_param('~min_speed', 1.0))          # [m/s]
        # Physical constraints
        self.max_accel = float(rospy.get_param('~max_accel', 1.0))          # [m/s^2]
        self.max_decel = float(rospy.get_param('~max_decel', 3.0))          # [m/s^2]
        self.max_lat_accel = float(rospy.get_param('~max_lat_accel', 1.0))  # [m/s^2]
        # Curvature handling
        self.use_future_curvature = bool(rospy.get_param('~use_future_curvature', True))
        self.lookahead_points = int(rospy.get_param('~lookahead_points', 15))
        # Terminal condition
        self.stop_at_end = bool(rospy.get_param('~stop_at_end', True))
        # Optional speed limit zones: [[start_idx, end_idx, max_speed], ...]
        self.speed_limit_zones = rospy.get_param('~speed_limit_zones', [])
        # Be robust: accept YAML set via <param value="..."> which may arrive as str
        if isinstance(self.speed_limit_zones, str):
            try:
                parsed = ast.literal_eval(self.speed_limit_zones)
                self.speed_limit_zones = parsed
            except Exception:
                self.speed_limit_zones = []
        if not isinstance(self.speed_limit_zones, list):
            self.speed_limit_zones = []
        # Smoothing (applied to initial curvature-based profile only)
        self.speed_smoothing = bool(rospy.get_param('~speed_smoothing', True))
        self.smoothing_window = int(rospy.get_param('~smoothing_window', 5))
        self.republish_rate = float(rospy.get_param('~republish_rate', 0.0))  # 0.0 to disable
        # Expose current closest index for debugging/visualization
        self.current_idx_topic = rospy.get_param('~current_idx_topic', '/planning/global/current_idx')
        # Note: current_idx is published for debugging; no freezing here.

        # Stopline gating / traffic light integration parameters
        # Default ON to allow stop-by-traffic-light behavior out of the box
        self.stopline_enabled = bool(rospy.get_param('~stopline/enabled', True))
        self.stopline_tolerance_m = float(rospy.get_param('~stopline/tolerance_m', 6.0))
        self.traffic_state_topic = rospy.get_param('~traffic_state_topic', '/perception/traffic_light/state')
        # Treat yellow as stop, same as red, by default
        self.stop_states = [s.strip().lower() for s in rospy.get_param('~stopline/stop_states', ['red', 'yellow'])]
        # Emergency brake when very close to stopline (distance-based immediate zeroing)
        self.stopline_emergency_enabled = bool(rospy.get_param('~stopline/emergency_brake/enabled', True))
        self.stopline_emergency_distance_m = float(rospy.get_param('~stopline/emergency_brake/distance_m', 2.0))
        self.odometry_topic = rospy.get_param('~odometry_topic', '/odometry')
        # Collision-based zero gating parameters (range + hold)
        self.collision_flag_topic = rospy.get_param('~collision_flag_topic', '/perception/collision/flag')
        self.collision_gate_enabled = bool(rospy.get_param('~collision_gate/enabled', True))
        self.collision_ranges = rospy.get_param('~collision_gate/ranges', [[2300, 2500], [2600, 2670], [3750, 3960]])
        if isinstance(self.collision_ranges, str):
            try:
                import ast as _ast
                self.collision_ranges = _ast.literal_eval(self.collision_ranges)
            except Exception:
                self.collision_ranges = []
        # Normalize ranges
        _norm = []
        if isinstance(self.collision_ranges, (list, tuple)):
            for r in self.collision_ranges:
                try:
                    a, b = int(r[0]), int(r[1])
                    if a > b:
                        a, b = b, a
                    _norm.append((a, b))
                except Exception:
                    continue
        self.collision_ranges = _norm
        self.collision_hold_sec = float(rospy.get_param('~collision_gate/hold_sec', 1.0))

        # Publisher (latched)
        self.speed_pub = rospy.Publisher(self.speed_topic, Float32MultiArray, queue_size=1, latch=True)
        # Current closest index publisher (latched)
        self.idx_pub = rospy.Publisher(self.current_idx_topic, Int32, queue_size=1, latch=True)

        # Visualization parameters and publisher
        # Default markers off for lighter runtime
        self.publish_markers = bool(rospy.get_param('~publish_markers', False))
        self.marker_topic = rospy.get_param('~marker_topic', '/vis/planning/global/speed_profile')
        self.marker_step = int(rospy.get_param('~marker_step', 7))  # stride; 1=every point
        self.marker_base_scale_xy = float(rospy.get_param('~marker_base_scale_xy', 0.1))
        self.marker_height_scale = float(rospy.get_param('~marker_height_scale', 0.5))  # pillar height per (m/s)
        self.marker_alpha = float(rospy.get_param('~marker_alpha', 0.6))
        self.marker_text_size = float(rospy.get_param('~marker_text_size', 0.3))

        self.marker_pub = None
        if self.publish_markers:
            self.marker_pub = rospy.Publisher(self.marker_topic, MarkerArray, queue_size=1, latch=True)

        # Internal state
        self.last_profile = None
        self.last_markers = None
        self.last_base_speeds = None   # speeds after curvature/limits/smoothing, before gating & physics
        self.last_segdist = None
        self.last_e = None
        self.last_n = None
        self.stop_indices_cache = []
        self.tl_state = 'unknown'
        self.current_e = None
        self.current_n = None
        self.current_idx = None

        # Subscriber
        self.waypoints_sub = rospy.Subscriber(self.waypoints_topic, Path, self.path_callback, queue_size=1)
        if self.stopline_enabled:
            rospy.Subscriber(self.traffic_state_topic, String, self.traffic_state_callback, queue_size=5)
            rospy.Subscriber(self.odometry_topic, Odometry, self.odom_callback, queue_size=20)
        else:
            # Still need odom for current_idx if collision gate is enabled
            rospy.Subscriber(self.odometry_topic, Odometry, self.odom_callback, queue_size=20)
        if self.collision_gate_enabled:
            rospy.Subscriber(self.collision_flag_topic, UInt8, self.collision_flag_callback, queue_size=10)

        # Optional republish timer
        if self.republish_rate > 0.0:
            self.timer = rospy.Timer(rospy.Duration(1.0 / self.republish_rate), self.timer_callback)

        self._log_params_snapshot()
        rospy.loginfo('Global Speed Profile started. Waiting for global waypoints...')

    # ------------------------- TL/Odom Callbacks -------------------------
    def traffic_state_callback(self, msg: String):
        self.tl_state = (msg.data or '').strip().lower()
        # Recompute and publish if we have cached base profile
        if self.stopline_enabled and self.last_base_speeds is not None and self.last_segdist is not None:
            try:
                self._recompute_from_cache_and_publish()
            except Exception as e:
                rospy.logwarn(f'[global_speed_profile] Recompute on TL change failed: {e}')

    def collision_flag_callback(self, msg: UInt8):
        try:
            if not self.collision_gate_enabled:
                return
            flag = int(msg.data)
            if flag != 2:
                return
            # Require current index and ranges
            if self.current_idx is None or not self.collision_ranges:
                return
            idx = int(self.current_idx)
            inside = False
            for (a, b) in self.collision_ranges:
                if a <= idx <= b:
                    inside = True
                    break
            if not inside:
                return
            # Arm hold timer
            self._collision_hold_until = rospy.Time.now() + rospy.Duration(max(0.0, float(self.collision_hold_sec)))
            rospy.logwarn(f'[global_speed_profile] Collision gate armed: idx={idx} ranges={self.collision_ranges} hold={self.collision_hold_sec:.2f}s')
            # Recompute and publish immediately if we have cache
            if self.last_base_speeds is not None and self.last_segdist is not None:
                self._recompute_from_cache_and_publish()
        except Exception as e:
            rospy.logwarn(f'[global_speed_profile] collision_flag_callback error: {e}')

    def odom_callback(self, msg: Odometry):
        self.current_e = float(msg.pose.pose.position.x)
        self.current_n = float(msg.pose.pose.position.y)
        # Update current index if path cached
        if self.last_e is not None and self.last_n is not None and len(self.last_e) > 0:
            dx = self.last_e - self.current_e
            dy = self.last_n - self.current_n
            d2 = dx * dx + dy * dy
            self.current_idx = int(np.argmin(d2))
            # Publish current closest index
            try:
                self.idx_pub.publish(Int32(data=int(self.current_idx)))
            except Exception:
                pass

    def smooth(self, speeds, window):
        if not self.speed_smoothing or speeds is None:
            return speeds
        if len(speeds) < window or window <= 1:
            return speeds
        kernel = np.ones(window) / float(window)
        return np.convolve(speeds, kernel, mode='same')

    def _avg_future(self, arr: np.ndarray, i: int, lookahead: int) -> float:
        n = len(arr)
        if n == 0:
            return 0.0
        j_end = min(n - 1, i + max(0, lookahead))
        return float(np.mean(arr[i : j_end + 1]))

    def path_callback(self, msg: Path):
        try:
            if len(msg.poses) == 0:
                rospy.logwarn_throttle(5.0, 'Received empty global waypoints; skipping speed generation')
                return

            e = np.array([p.pose.position.x for p in msg.poses], dtype=float)
            n = np.array([p.pose.position.y for p in msg.poses], dtype=float)

            N = len(e)
            if N < 2:
                rospy.logwarn_throttle(5.0, 'Waypoints too few for speed profile')
                return

            # Segment distances
            segdist = np.sqrt(np.diff(e) ** 2 + np.diff(n) ** 2)

            # Curvature magnitude (abs)
            kappa_abs = np.abs(calculate_path_curvature(e, n))
            if N >= 2:
                kappa_abs[0] = kappa_abs[1] if N > 1 else kappa_abs[0]
                kappa_abs[-1] = kappa_abs[-2] if N > 1 else kappa_abs[-1]

            # Initial curvature-based speeds with lateral accel limit
            eps = 1e-6
            speeds = np.full(N, self.max_speed, dtype=float)
            if self.use_future_curvature:
                for i in range(N):
                    kbar = self._avg_future(kappa_abs, i, self.lookahead_points)
                    v_curve = math.sqrt(self.max_lat_accel / (kbar + eps))
                    speeds[i] = min(self.max_speed, max(self.min_speed, v_curve))
            else:
                v_curve_arr = np.sqrt(self.max_lat_accel / (kappa_abs + eps))
                speeds = np.minimum(self.max_speed, np.maximum(self.min_speed, v_curve_arr))

            # Optional smoothing of the initial profile only
            speeds = self.smooth(speeds, self.smoothing_window)

            # Apply explicit speed limit zones
            for zone in self.speed_limit_zones:
                try:
                    if not (isinstance(zone, (list, tuple)) and len(zone) == 3):
                        continue
                    start = max(0, int(zone[0]))
                    end = min(N - 1, int(zone[1]))
                    vmax = float(zone[2])
                    if start <= end:
                        speeds[start : end + 1] = np.minimum(speeds[start : end + 1], vmax)
                except Exception:
                    continue

            # Enforce terminal stop if requested
            if self.stop_at_end and N > 0:
                speeds[-1] = 0.0

            # Cache base for reactive re-publish (before gating and physics passes)
            self.last_base_speeds = speeds.copy()
            self.last_segdist = segdist.copy()
            self.last_e = e.copy()
            self.last_n = n.copy()

            # Precompute/memoize stopline indices for this path
            if self.stopline_enabled:
                self.stop_indices_cache = self._map_stop_points_to_indices(e, n, STOPLINES_ENU, self.stopline_tolerance_m)
            else:
                self.stop_indices_cache = []

            # Inject stopline gating (set speeds to zero from the next stop index ahead)
            if self.stopline_enabled and self._is_stop_state_active():
                next_idx = self._select_next_stop_index()
                if next_idx is not None:
                    # Emergency: if very close to the stopline, zero entire profile to brake immediately
                    if (
                        self.stopline_emergency_enabled and
                        self.current_e is not None and self.current_n is not None and
                        0 <= next_idx < len(e)
                    ):
                        try:
                            de = float(e[next_idx]) - float(self.current_e)
                            dn = float(n[next_idx]) - float(self.current_n)
                            dist = math.hypot(de, dn)
                            if dist <= float(self.stopline_emergency_distance_m):
                                speeds[:] = 0.0
                            else:
                                speeds[next_idx:] = 0.0
                        except Exception:
                            speeds[next_idx:] = 0.0
                    else:
                        speeds[next_idx:] = 0.0

            # Collision-based global gating (range + hold)
            if self._is_collision_gate_active():
                speeds[:] = 0.0

            # Backward pass (deceleration limit)
            if N >= 2:
                for i in range(N - 2, -1, -1):
                    ds = max(0.0, float(segdist[i]))
                    v_next = float(speeds[i + 1])
                    v_allow = math.sqrt(max(0.0, v_next * v_next + 2.0 * self.max_decel * ds))
                    if speeds[i] > v_allow:
                        speeds[i] = v_allow

            # Forward pass (acceleration limit)
            if N >= 2:
                for i in range(0, N - 1):
                    ds = max(0.0, float(segdist[i]))
                    v_curr = float(speeds[i])
                    v_allow = math.sqrt(max(0.0, v_curr * v_curr + 2.0 * self.max_accel * ds))
                    if speeds[i + 1] > v_allow:
                        speeds[i + 1] = v_allow

            speed_msg = Float32MultiArray()
            speed_msg.data = speeds.tolist()

            self.speed_pub.publish(speed_msg)
            self.last_profile = speed_msg

            # Visualization markers (pillars + text)
            if self.publish_markers and self.marker_pub is not None:
                frame_id = msg.header.frame_id if msg.header.frame_id else 'reference'
                markers = self._make_speed_markers(frame_id, e, n, speeds)
                self.marker_pub.publish(markers)
                self.last_markers = markers

            rospy.loginfo_once('Published global speed profile to /planning/speed_profile/global')
        except Exception as e:
            rospy.logerr(f'Failed generating speed profile: {e}')

    def timer_callback(self, _):
        if self.last_profile is not None:
            self.speed_pub.publish(self.last_profile)
        if self.publish_markers and self.last_markers is not None and self.marker_pub is not None:
            self.marker_pub.publish(self.last_markers)

    # ------------------------- Debug helpers -------------------------
    def _log_params_snapshot(self):
        try:
            rospy.loginfo(
                '[global_speed_profile] params: max_speed=%.2f, min_speed=%.2f, '
                'max_accel=%.2f, max_decel=%.2f, max_lat_accel=%.2f, '
                'use_future_curvature=%s, lookahead_points=%d, stop_at_end=%s, '
                'publish_markers=%s, marker_topic=%s, marker_step=%d, stopline_enabled=%s, '
                'zones_count=%d',
                self.max_speed, self.min_speed, self.max_accel, self.max_decel, self.max_lat_accel,
                str(self.use_future_curvature), int(self.lookahead_points), str(self.stop_at_end),
                str(self.publish_markers), self.marker_topic, int(self.marker_step), str(self.stopline_enabled),
                int(len(self.speed_limit_zones) if isinstance(self.speed_limit_zones, list) else 0)
            )
        except Exception:
            pass

    # ------------------------- Gating Helpers -------------------------
    def _is_stop_state_active(self) -> bool:
        return (self.tl_state or '').lower() in self.stop_states

    def _map_stop_points_to_indices(self, e: np.ndarray, n: np.ndarray, points, tol_m: float):
        """Map ENU stop points to nearest path indices within tolerance."""
        idxs = []
        if e is None or n is None or len(e) == 0:
            return idxs
        for (sx, sy) in points:
            dx = e - float(sx)
            dy = n - float(sy)
            d2 = dx * dx + dy * dy
            i = int(np.argmin(d2))
            if math.sqrt(float(d2[i])) <= max(0.0, tol_m):
                idxs.append(i)
        idxs = sorted(set(idxs))
        return idxs

    def _select_next_stop_index(self):
        """Select the first stop index ahead of current index; fallback to earliest if current unknown."""
        if not self.stop_indices_cache:
            return None
        if self.current_idx is None:
            return int(self.stop_indices_cache[0])
        for i in self.stop_indices_cache:
            if int(i) > int(self.current_idx):
                return int(i)
        return None

    def _recompute_from_cache_and_publish(self):
        # Guard
        if self.last_base_speeds is None or self.last_segdist is None or self.last_e is None or self.last_n is None:
            return
        speeds = self.last_base_speeds.copy()
        N = len(speeds)
        segdist = self.last_segdist

        # Inject or remove gating according to current TL state
        if self.stopline_enabled and self._is_stop_state_active():
            next_idx = self._select_next_stop_index()
            if next_idx is not None:
                # Emergency: if very close to the stopline, zero entire profile to brake immediately
                if (
                    self.stopline_emergency_enabled and
                    self.current_e is not None and self.current_n is not None and
                    self.last_e is not None and self.last_n is not None and
                    0 <= next_idx < len(self.last_e)
                ):
                    try:
                        de = float(self.last_e[next_idx]) - float(self.current_e)
                        dn = float(self.last_n[next_idx]) - float(self.current_n)
                        dist = math.hypot(de, dn)
                        if dist <= float(self.stopline_emergency_distance_m):
                            speeds[:] = 0.0
                        else:
                            speeds[next_idx:] = 0.0
                    except Exception:
                        speeds[next_idx:] = 0.0
                else:
                    speeds[next_idx:] = 0.0

        # Collision-based global gating (range + hold)
        if self._is_collision_gate_active():
            speeds[:] = 0.0

        # Physics passes
        if N >= 2:
            for i in range(N - 2, -1, -1):
                ds = max(0.0, float(segdist[i]))
                v_next = float(speeds[i + 1])
                v_allow = math.sqrt(max(0.0, v_next * v_next + 2.0 * self.max_decel * ds))
                if speeds[i] > v_allow:
                    speeds[i] = v_allow
        if N >= 2:
            for i in range(0, N - 1):
                ds = max(0.0, float(segdist[i]))
                v_curr = float(speeds[i])
                v_allow = math.sqrt(max(0.0, v_curr * v_curr + 2.0 * self.max_accel * ds))
                if speeds[i + 1] > v_allow:
                    speeds[i + 1] = v_allow

        # Publish
        out = Float32MultiArray()
        out.data = speeds.tolist()
        self.speed_pub.publish(out)
        self.last_profile = out

        # Markers
        if self.publish_markers and self.marker_pub is not None:
            markers = self._make_speed_markers('reference', self.last_e, self.last_n, speeds)
            self.marker_pub.publish(markers)
            self.last_markers = markers

    def _make_speed_markers(self, frame_id, e, n, speeds):
        ma = MarkerArray()
        now = rospy.Time.now()

        # Clear previous markers
        clear = Marker()
        clear.header.frame_id = frame_id or 'reference'
        clear.header.stamp = now
        clear.action = Marker.DELETEALL
        clear.id = 0
        ma.markers.append(clear)

        # Colors (purple translucent)
        r, g, b, a = 0.6, 0.2, 0.8, max(0.0, min(1.0, self.marker_alpha))

        step = max(1, int(self.marker_step))
        N = min(len(e), len(n), len(speeds))
        for idx in range(0, N, step):
            spd = float(speeds[idx])
            height = max(0.02, spd * float(self.marker_height_scale))

            # Pillar marker (cylinder)
            m = Marker()
            m.header.frame_id = frame_id or 'reference'
            m.header.stamp = now
            m.ns = 'speed_pillars'
            m.id = idx
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            m.pose.position.x = float(e[idx])
            m.pose.position.y = float(n[idx])
            m.pose.position.z = height / 2.0
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = max(0.01, float(self.marker_base_scale_xy))
            m.scale.z = height
            m.color.r = r
            m.color.g = g
            m.color.b = b
            m.color.a = a
            ma.markers.append(m)

            # Text marker (km/h)
            t = Marker()
            t.header = m.header
            t.ns = 'speed_text'
            t.id = 100000 + idx
            t.type = Marker.TEXT_VIEW_FACING
            t.action = Marker.ADD
            t.pose.position.x = m.pose.position.x
            t.pose.position.y = m.pose.position.y
            t.pose.position.z = height + (float(self.marker_text_size) * 0.5)
            t.scale.z = max(0.1, float(self.marker_text_size))
            t.color.r = 1.0
            t.color.g = 1.0
            t.color.b = 1.0
            t.color.a = 0.95
            t.text = f"{spd * 3.6:.1f} km/h"
            ma.markers.append(t)

        return ma

    # ------------------------- Collision gate helpers -------------------------
    def _is_collision_gate_active(self) -> bool:
        if not self.collision_gate_enabled:
            return False
        t_until = getattr(self, '_collision_hold_until', None)
        if t_until is None:
            return False
        return rospy.Time.now() < t_until


def main():
    node = GlobalSpeedProfileNode()
    rospy.spin()


if __name__ == '__main__':
    main()
