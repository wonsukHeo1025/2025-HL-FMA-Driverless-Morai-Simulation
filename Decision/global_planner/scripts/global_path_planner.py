#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Global Path Planner (simplified + recommended follow)

Loads a single CSV path (columns: e,n[,u]) and publishes:
- /planning/global/path (nav_msgs/Path, latched)
- /planning/global/waypoints (nav_msgs/Path, latched)

Also publishes preview paths (latched) for visualization/tools:
- /planning/global/path_default
- /planning/global/path_second
- /planning/global/path_third

Notes:
- No topic-based course switching (preview is for visualization only).
 - Optionally follows `/planning/global/recommended_path` (std_msgs/String) to
   select one of: 'default' | 'second' | 'third'.
"""

import os
import ast
import rospy
import pandas as pd
import numpy as np

from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Empty as EmptyMsg  # kept only for backward compatibility if needed (not used)
from std_msgs.msg import String as StringMsg  # kept only for backward compatibility if needed (not used)
from morai_msgs.msg import CollisionData
from std_msgs.msg import UInt8

from global_planner.path_utils import (
    calculate_path_yaw,
    yaw_to_quaternion,
    resample_path_linear,
    smooth_path_savgol,
    calculate_path_length,
)
from global_planner.path_converter import PathConverter


def _resolve_path(path_param: str) -> str:
    """Resolve CSV file path. If relative, anchor at package root."""
    if os.path.isabs(path_param):
        return path_param
    # pkg root (…/Decision/global_planner)
    pkg_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    return os.path.join(pkg_root, path_param)


def _load_csv(full_path: str) -> pd.DataFrame:
    if not os.path.exists(full_path):
        txt_path = full_path.replace('.csv', '.txt')
        if os.path.exists(txt_path):
            converter = PathConverter()
            df = converter.txt_to_dataframe(txt_path)
            converter.dataframe_to_csv(df, full_path)
        else:
            raise FileNotFoundError(f"Neither {full_path} nor {txt_path} found")
    return pd.read_csv(full_path)


def _make_path_msg(frame_id: str, e: np.ndarray, n: np.ndarray, z: np.ndarray, yaw: np.ndarray, stamp: rospy.Time = None) -> Path:
    path = Path()
    path.header.frame_id = frame_id
    path.header.stamp = stamp if stamp is not None else rospy.Time.now()
    for i in range(len(e)):
        ps = PoseStamped()
        ps.header = path.header
        ps.pose.position.x = float(e[i])
        ps.pose.position.y = float(n[i])
        ps.pose.position.z = float(z[i]) if i < len(z) else 0.0
        ori = yaw_to_quaternion(float(yaw[i]) if i < len(yaw) else 0.0)
        ps.pose.orientation = ori
        path.poses.append(ps)
    return path


# Marker visualization removed. RViz should use Path displays.


class UnifiedPathPlannerNode:
    def __init__(self):
        # Parameters
        self.path_file = rospy.get_param('~path_file', 'data/25hl_global_path_ver3.csv')
        # Optional alternate course files for preview publication
        self.path_file_alt = rospy.get_param('~alt_path_file', 'data/second_course.csv')
        self.path_file_third = rospy.get_param('~third_path_file', 'data/third_course.csv')
        self.path_topic = rospy.get_param('~path_topic', '/planning/global/path')
        self.waypoints_topic = rospy.get_param('~waypoints_topic', '/planning/global/path')
        self.map_frame = rospy.get_param('~map_frame', 'reference')
        self.resample_spacing = float(rospy.get_param('~resample_spacing', 0.2))
        self.smoothing_window = int(rospy.get_param('~smoothing_window', 11))
        self.stamp_with_now = bool(rospy.get_param('~stamp_with_now', False))
        # Follow recommended path name published by occupancy evaluator
        self.recommended_topic = rospy.get_param('~recommended_topic', '/planning/global/recommended_path')
        self.follow_recommended = bool(rospy.get_param('~follow_recommended', True))
        # Staged switch policy (third -> second -> default)
        self.staged_switch_enabled = bool(rospy.get_param('~staged_switch/enabled', True))
        self.staged_hold_sec = float(rospy.get_param('~staged_switch/hold_sec', 3.0))
        # Course-freeze windows (by nearest index along current path)
        self.freeze_enabled = bool(rospy.get_param('~freeze_selection/enabled', True))
        # Prefer multi-range parameter; fallback to single range
        raw_ranges = rospy.get_param('~freeze_selection/ranges', '[[1000,1425],[2800,3000], [3500, 3800]]')
        if isinstance(raw_ranges, str):
            try:
                raw_ranges = ast.literal_eval(raw_ranges)
            except Exception:
                raw_ranges = []
        self.freeze_ranges = []
        if isinstance(raw_ranges, (list, tuple)):
            for r in raw_ranges:
                try:
                    a, b = int(r[0]), int(r[1])
                    if a > b:
                        a, b = b, a
                    self.freeze_ranges.append((a, b))
                except Exception:
                    continue
        # Backward-compatible params if ranges empty
        if not self.freeze_ranges:
            f_from = int(rospy.get_param('~freeze_selection/from', 1098))
            f_to   = int(rospy.get_param('~freeze_selection/to',   1425))
            if f_from > f_to:
                f_from, f_to = f_to, f_from
            self.freeze_ranges = [(f_from, f_to)]
        self.odom_topic = rospy.get_param('~odometry_topic', '/odometry')
        # Collision-based demotion
        self.collision_topic = rospy.get_param('~collision_topic', '/CollisionData')
        self.collision_demote_enabled = bool(rospy.get_param('~collision_demote/enabled', True))
        self.collision_hold_sec = float(rospy.get_param('~collision_demote/hold_sec', 5.0))
        # Publish collision flag (UInt8: 0=clear,1=caution,2=danger) for downstream speed gating
        self.collision_flag_topic = rospy.get_param('~collision_flag_topic', '/perception/collision/flag')
        self.pub_collision_flag = rospy.Publisher(self.collision_flag_topic, UInt8, queue_size=10)
        # Restrict collision demotion to specific index ranges
        raw_c_ranges = rospy.get_param('~collision_demote/active_ranges', '[[910,1030]]')
        if isinstance(raw_c_ranges, str):
            try:
                raw_c_ranges = ast.literal_eval(raw_c_ranges)
            except Exception:
                raw_c_ranges = []
        self.collision_active_ranges = []
        if isinstance(raw_c_ranges, (list, tuple)):
            for r in raw_c_ranges:
                try:
                    a, b = int(r[0]), int(r[1])
                    if a > b:
                        a, b = b, a
                    self.collision_active_ranges.append((a, b))
                except Exception:
                    continue
        if not self.collision_active_ranges:
            c_from = int(rospy.get_param('~collision_demote/from', 910))
            c_to   = int(rospy.get_param('~collision_demote/to',   1030))
            if c_from > c_to:
                c_from, c_to = c_to, c_from
            self.collision_active_ranges = [(c_from, c_to)]
        # (Removed) E-stop on collision — rolled back per request

        # Preview publication
        self.publish_previews = bool(rospy.get_param('~publish_previews', True))
        self.path_default_topic = rospy.get_param('~path_default_topic', '/planning/global/path_default')
        self.path_second_topic  = rospy.get_param('~path_second_topic',  '/planning/global/path_second')
        self.path_third_topic   = rospy.get_param('~path_third_topic',   '/planning/global/path_third')

        # Publishers (latched)
        self.path_pub = rospy.Publisher(self.path_topic, Path, queue_size=1, latch=True)
        self.waypoints_pub = rospy.Publisher(self.waypoints_topic, Path, queue_size=1, latch=True)

        # Preview publishers (latched)
        self.preview_pubs = {}
        if self.publish_previews:
            self.preview_pubs['default'] = rospy.Publisher(self.path_default_topic, Path, queue_size=1, latch=True)
            self.preview_pubs['second']  = rospy.Publisher(self.path_second_topic,  Path, queue_size=1, latch=True)
            self.preview_pubs['third']   = rospy.Publisher(self.path_third_topic,   Path, queue_size=1, latch=True)

        # Internal state: active course
        self.current_mode = 'default'
        self.current_path_msg = None
        self.current_e = None
        self.current_n = None
        self.current_idx = None
        self.freeze_active = False
        # Collision override
        self.override_until = rospy.Time(0)
        self.override_mode = None
        # Staged-switch state
        self._staged_timer = None
        self._staged_next_mode = None
        # Path cache: full_path -> {mtime: float, msg: Path}
        self._cache = {}

        # Initial publish
        self._publish_mode(self.current_mode)
        if self.publish_previews:
            self._publish_previews()

        # Subscriber to recommended path changes (if enabled)
        if self.follow_recommended:
            rospy.Subscriber(self.recommended_topic, StringMsg, self._on_recommended)
        # Subscribe to odometry for nearest-index tracking (to apply freeze window)
        try:
            rospy.Subscriber(self.odom_topic, Odometry, self._on_odom)
        except Exception as e:
            rospy.logwarn(f"[global_planner] Failed subscribing odometry at {self.odom_topic}: {e}")
        # Subscribe to collision events
        if self.collision_demote_enabled:
            try:
                rospy.Subscriber(self.collision_topic, CollisionData, self._on_collision)
            except Exception as e:
                rospy.logwarn(f"[global_planner] Failed subscribing collisions at {self.collision_topic}: {e}")

    def _publish_previews(self):
        # Publish the main path also to 'default' preview topic
        try:
            msg = self._get_cached_path_msg(self.path_file)
            self.preview_pubs['default'].publish(msg)
        except Exception as e:
            rospy.logwarn(f"[global_planner] Failed to publish default preview: {e}")
        # Second course preview
        try:
            msg = self._get_cached_path_msg(self.path_file_alt)
            self.preview_pubs['second'].publish(msg)
        except Exception as e:
            rospy.logwarn(f"[global_planner] Failed to publish second preview: {e}")
        # Third course preview
        try:
            msg = self._get_cached_path_msg(self.path_file_third)
            self.preview_pubs['third'].publish(msg)
        except Exception as e:
            rospy.logwarn(f"[global_planner] Failed to publish third preview: {e}")

    def _prepare_path_msg_from_file(self, file_param: str) -> Path:
        full_path = _resolve_path(file_param)
        df = _load_csv(full_path)
        if not all(c in df.columns for c in ['e', 'n']):
            raise ValueError("CSV must contain columns: ['e','n']")

        e = df['e'].to_numpy(dtype=float)
        n = df['n'].to_numpy(dtype=float)
        z = df['u'].to_numpy(dtype=float) if 'u' in df.columns else np.zeros(len(e))

        # Resample and smooth if requested
        if self.resample_spacing and self.resample_spacing > 0:
            e, n = resample_path_linear(e, n, self.resample_spacing)
            z = np.interp(np.linspace(0, len(z) - 1, len(e)), np.arange(len(z)), z)
        if self.smoothing_window and len(e) > self.smoothing_window:
            e, n = smooth_path_savgol(e, n, self.smoothing_window, 3)

        yaw = calculate_path_yaw(e, n)
        stamp = rospy.Time.now() if self.stamp_with_now else rospy.Time(0)
        path_msg = _make_path_msg(self.map_frame, e, n, z, yaw, stamp=stamp)
        return path_msg

    def _publish_mode(self, mode: str):
        file_param = self._file_for_mode(mode)
        full_path = _resolve_path(file_param)
        path_msg = self._get_cached_path_msg(file_param)

        # Publish (latched)
        self.path_pub.publish(path_msg)
        if self.waypoints_pub is not None:
            self.waypoints_pub.publish(path_msg)

        # Cache arrays for nearest-index computation
        try:
            self.current_path_msg = path_msg
            self.current_e = np.array([p.pose.position.x for p in path_msg.poses], dtype=float)
            self.current_n = np.array([p.pose.position.y for p in path_msg.poses], dtype=float)
        except Exception:
            self.current_path_msg = None
            self.current_e = None
            self.current_n = None

        try:
            # Best-effort logging with size only
            rospy.loginfo(
                f"[global_planner] Published '{mode}' path from {full_path}: "
                f"points={len(path_msg.poses)}, frame={self.map_frame}"
            )
        except Exception:
            pass

    def _get_cached_path_msg(self, file_param: str) -> Path:
        full_path = _resolve_path(file_param)
        # If not present, _load_csv inside builder will raise if missing
        mtime = None
        try:
            mtime = os.path.getmtime(full_path)
        except Exception:
            mtime = None
        entry = self._cache.get(full_path)
        if entry is not None and entry.get('mtime') == mtime:
            return entry['msg']
        msg = self._prepare_path_msg_from_file(file_param)
        self._cache[full_path] = {'mtime': mtime, 'msg': msg}
        return msg

    def _file_for_mode(self, mode: str) -> str:
        m = (mode or '').strip().lower()
        if m == 'second':
            return self.path_file_alt
        if m == 'third':
            return self.path_file_third
        return self.path_file

    def _on_recommended(self, msg: StringMsg):
        name = (msg.data or '').strip().lower()
        if name == 'none' or name == 'unknown' or name == '':
            rospy.logwarn_throttle(2.0, f"[global_planner] recommended_path '{msg.data}' ignored")
            return
        if name not in ('default', 'second', 'third'):
            rospy.logwarn_throttle(2.0, f"[global_planner] recommended_path '{msg.data}' not recognized")
            return
        # If a staged switch is in progress, cancel it only when the new recommendation is NOT 'default'.
        # Rationale: during third->second(hold)->default, a new 'default' should keep the staged plan;
        # but a new 'second' or 'third' should cancel staged and follow standard policy.
        if (self._staged_timer is not None or self._staged_next_mode is not None) and name != 'default':
            try:
                rospy.loginfo("[global_planner] New non-default recommendation: cancelling staged switch")
            except Exception:
                pass
            self._cancel_staged()
        # If staged is running and new recommendation is 'default', ignore it and keep the staged plan.
        if (self._staged_timer is not None or self._staged_next_mode is not None) and name == 'default':
            rospy.loginfo_throttle(1.0, "[global_planner] Staged switch in progress: ignoring 'default' recommendation (will switch after hold)")
            return
        if name == self.current_mode:
            return
        # Apply collision override window (hold current)
        if self._override_active():
            rospy.loginfo_throttle(1.0, f"[global_planner] Collision override active (until {self.override_until.to_sec():.2f}) - ignoring recommended_path '{name}'")
            return
        # Apply freeze policy: ignore changes when current nearest index is inside any freeze window
        if self.freeze_enabled and self.freeze_active:
            rospy.loginfo_throttle(1.0, f"[global_planner] Freeze active (idx={self.current_idx}) - ignoring recommended_path '{name}'")
            return
        # Staged switch: if current is 'third' and recommendation is 'default',
        # perform third -> second immediately, hold for N seconds, then switch to default.
        if self.staged_switch_enabled and self.current_mode == 'third' and name == 'default':
            # Cancel any existing staged plan
            self._cancel_staged()
            prev = self.current_mode
            nxt = 'second'
            if prev != nxt:
                self.current_mode = nxt
                rospy.logwarn(f"[global_planner] Staged recommended switch: {prev} -> {self.current_mode} (hold {self.staged_hold_sec:.1f}s) -> default")
                try:
                    self._publish_mode(self.current_mode)
                except Exception as e:
                    rospy.logerr(f"[global_planner] Failed to publish staged path '{self.current_mode}': {e}")
                    # revert on failure
                    self.current_mode = prev
                    return
            # Schedule final switch to default after hold duration
            self._staged_next_mode = 'default'
            try:
                self._staged_timer = rospy.Timer(rospy.Duration(max(0.0, float(self.staged_hold_sec))), self._on_staged_timer, oneshot=True)
            except Exception:
                pass
            return
        prev = self.current_mode
        self.current_mode = name
        rospy.loginfo(f"[global_planner] Following recommended path: {prev} -> {self.current_mode}")
        try:
            self._publish_mode(self.current_mode)
        except Exception as e:
            rospy.logerr(f"[global_planner] Failed to publish recommended path '{self.current_mode}': {e}")
            # revert on failure
            self.current_mode = prev

    def _on_odom(self, msg: Odometry):
        if self.current_e is None or self.current_n is None or len(self.current_e) == 0:
            return
        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
        try:
            dx = self.current_e - x
            dy = self.current_n - y
            d2 = dx * dx + dy * dy
            idx = int(np.argmin(d2))
            self.current_idx = idx
            # Update freeze state: inside any configured window
            active = False
            if self.freeze_enabled and self.freeze_ranges:
                for (a, b) in self.freeze_ranges:
                    if a <= idx <= b:
                        active = True
                        break
            self.freeze_active = active
        except Exception:
            pass

    def _on_collision(self, msg: CollisionData):
        # React to real collisions (unique_id != 0): demote path in configured ranges.
        try:
            # If an override hold window is active, ignore additional collision events
            if self._override_active():
                rospy.loginfo_throttle(1.0, f"[global_planner] Collision ignored during hold window (until {self.override_until.to_sec():.2f})")
                return

            def any_nonzero_uid(objs):
                try:
                    for o in (objs or []):
                        uid = getattr(o, 'unique_id', 0)
                        if int(uid) != 0:
                            return True
                except Exception:
                    return False
                return False

            has_real_collision = False
            ids = []
            try:
                for o in getattr(msg, 'collision_object', []) or []:
                    try:
                        ids.append(int(getattr(o, 'unique_id', 0)))
                    except Exception:
                        ids.append(0)
            except Exception:
                pass
            try:
                for o in getattr(msg, 'collision_objecta', []) or []:
                    try:
                        ids.append(int(getattr(o, 'unique_id', 0)))
                    except Exception:
                        ids.append(0)
            except Exception:
                pass
            for uid in ids:
                if uid != 0:
                    has_real_collision = True
                    break
            if not has_real_collision:
                # Ignore heartbeat/noise frames (unique_id all zeros)
                return
            # Emit a single collision flag pulse (2 then 0) for downstream nodes (e.g., speed fusion)
            try:
                self.pub_collision_flag.publish(UInt8(data=2))
                self.pub_collision_flag.publish(UInt8(data=0))
            except Exception:
                pass
            # Demote path only when inside configured demotion ranges
            allow_demote = False
            if self.current_idx is not None and self.collision_demote_enabled and self.collision_active_ranges:
                for (a, b) in self.collision_active_ranges:
                    if a <= int(self.current_idx) <= b:
                        allow_demote = True
                        break
            if allow_demote:
                cur = (self.current_mode or 'default').strip().lower()
                nxt = 'second' if cur == 'default' else ('third' if cur == 'second' else 'third')
                if nxt == cur:
                    rospy.loginfo_throttle(1.0, f"[global_planner] Collision detected (uids={ids}), already at lowest priority '{cur}'")
                else:
                    prev = self.current_mode
                    self.current_mode = nxt
                    rospy.logwarn(f"[global_planner] Collision detected (uids={ids}) -> switch path: {prev} -> {self.current_mode}")
                    try:
                        self._publish_mode(self.current_mode)
                    except Exception as e:
                        rospy.logerr(f"[global_planner] Failed to publish demoted path '{self.current_mode}': {e}")
                        self.current_mode = prev
                        return
                # Start override hold window for demotion
                self.override_until = rospy.Time.now() + rospy.Duration(max(0.0, float(self.collision_hold_sec)))
                self.override_mode = self.current_mode
        except Exception as e:
            rospy.logwarn(f"[global_planner] Collision handler error: {e}")

    def _override_active(self) -> bool:
        try:
            return rospy.Time.now() < self.override_until
        except Exception:
            return False

    # ------------------------- Staged switch helpers -------------------------
    def _cancel_staged(self):
        try:
            if self._staged_timer is not None:
                self._staged_timer.shutdown()
        except Exception:
            pass
        self._staged_timer = None
        self._staged_next_mode = None

    def _on_staged_timer(self, _event):
        target = self._staged_next_mode
        if not target:
            self._cancel_staged()
            return
        # If collision override or freeze window is active, cancel staged plan (no delay logic in these zones)
        if self._override_active() or (self.freeze_enabled and self.freeze_active):
            try:
                rospy.loginfo("[global_planner] Cancelling staged switch due to freeze/override active")
            except Exception:
                pass
            self._cancel_staged()
            return
        prev = self.current_mode
        if target == prev:
            self._cancel_staged()
            return
        # Publish the target mode
        self.current_mode = target
        rospy.logwarn(f"[global_planner] Staged switch: {prev} -> {self.current_mode}")
        try:
            self._publish_mode(self.current_mode)
        except Exception as e:
            rospy.logerr(f"[global_planner] Failed to publish staged target '{self.current_mode}': {e}")
            # revert on failure
            self.current_mode = prev
        self._cancel_staged()

    # (Removed) E-stop helpers — rolled back


def main():
    rospy.init_node('unified_path_planner', anonymous=False)
    _ = UnifiedPathPlannerNode()
    rospy.spin()


if __name__ == '__main__':
    main()
