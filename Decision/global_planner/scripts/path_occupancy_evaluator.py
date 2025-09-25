#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Path Occupancy Evaluator (corridor viz + occupancy)

Subscribes to three preview paths and publishes corridor boundaries
(left/right LINE_STRIP) as MarkerArray for RViz. Additionally, it
subscribes to cluster markers and odometry to compute per-path
minimum lateral distance (d_lat) within a lookahead window and
publishes simple occupancy summaries.

Topics (params):
  ~path_default_topic (str): /planning/global/path_default
  ~path_second_topic  (str): /planning/global/path_second
  ~path_third_topic   (str): /planning/global/path_third
  ~marker_topic       (str): /vis/planning/global/path_corridors
  ~cluster_topic      (str): /vis/perception/lidar/cluster_markers
  ~odometry_topic     (str): /odometry
  ~occupancy_topic    (str): /planning/global/path_occupancy
  ~scores_topic       (str): /planning/global/path_scores
  ~recommended_topic  (str): /planning/global/recommended_path

Parameters:
  ~corridor_half_width (float): default 1.75 [m]
  ~margin              (float): default 0.25 [m]
  ~subsample_step      (int)  : sample every Nth point for markers (default 5)
  ~line_width          (float): marker scale.x (default 0.08)
  ~color_default       (float[3]): default [0.7, 0.7, 0.7]
  ~color_second        (float[3]): default [0.0, 0.4, 1.0]
  ~color_third         (float[3]): default [0.0, 1.0, 0.0]
  ~lookahead_m         (float): default 30.0
  ~d_in                (float): block when min d_lat <= d_in (default 2.0)
  ~d_out               (float): clear when min d_lat > d_out (default 2.4)
  ~hold_sec            (float): min seconds between state flips (default 1.5)
  ~priority            (list[str]|str): recommendation order (default [default,second,third])
  ~auto_select         (bool): publish recommended path name only (no side-effect) (default false)
"""

import math
from typing import List, Tuple, Dict

import numpy as np
import rospy
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2 as pc2
from geometry_msgs.msg import Point, PointStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String, Float32MultiArray
import tf2_ros
from tf2_geometry_msgs import do_transform_point
from collections import deque


class PathCorridorViz:
    def __init__(self):
        # Params
        self.path_topics = {
            'default': rospy.get_param('~path_default_topic', '/planning/global/path_default'),
            'second':  rospy.get_param('~path_second_topic',  '/planning/global/path_second'),
            'third':   rospy.get_param('~path_third_topic',   '/planning/global/path_third'),
        }
        self.marker_topic = rospy.get_param('~marker_topic', '/vis/planning/global/path_corridors')
        # Obstacle sources
        self.obstacle_source = rospy.get_param('~obstacle_source', 'points')
        if isinstance(self.obstacle_source, str):
            self.obstacle_source = self.obstacle_source.strip().lower()
        else:
            self.obstacle_source = 'points'
        self.cluster_topic = rospy.get_param('~cluster_topic', '/vis/perception/lidar/cluster_markers')
        self.points_topic = rospy.get_param('~points_topic', '/perception/lidar/processed_points')
        self.points_stride = int(rospy.get_param('~points_stride', 3))
        self.points_max = int(rospy.get_param('~points_max', 8000))
        self.min_points_inside = int(rospy.get_param('~min_points_inside', 5))
        self.odom_topic = rospy.get_param('~odometry_topic', '/odometry')
        self.occ_topic = rospy.get_param('~occupancy_topic', '/planning/global/path_occupancy')
        self.scores_topic = rospy.get_param('~scores_topic', '/planning/global/path_scores')
        self.recommended_topic = rospy.get_param('~recommended_topic', '/planning/global/recommended_path')
        self.target_frame = rospy.get_param('~target_frame', 'reference')

        self.corr_half = float(rospy.get_param('~corridor_half_width', 1.75))
        self.margin = float(rospy.get_param('~margin', 0.25))
        self.hysteresis_gap = float(rospy.get_param('~hysteresis_gap', 0.10))
        self.step = max(1, int(rospy.get_param('~subsample_step', 5)))
        self.line_w = float(rospy.get_param('~line_width', 0.08))

        self.colors = {
            'default': self._param_color('~color_default', [0.7, 0.7, 0.7]),
            'second':  self._param_color('~color_second',  [0.0, 0.4, 1.0]),
            'third':   self._param_color('~color_third',   [0.0, 1.0, 0.0]),
        }

        # Storage
        self.paths: Dict[str, Path] = {}
        self.caches: Dict[str, Dict[str, np.ndarray]] = {}
        self.odom_xy = None
        self.clusters_xy = []
        self.points_xy = []

        # Occupancy params/state
        self.lookahead_m = float(rospy.get_param('~lookahead_m', 50.0))
        # Link-mode thresholds (always): derive from corridor settings
        self.d_in = self.corr_half + self.margin
        self.d_out = self.d_in + self.hysteresis_gap
        self.hold_sec = float(rospy.get_param('~hold_sec', 1.5))
        # Debounce + filtering
        self.pre_block_sec = float(rospy.get_param('~pre_block_sec', 0.5))
        self.pre_clear_sec = float(rospy.get_param('~pre_clear_sec', 0.5))
        self.dmin_filter = rospy.get_param('~dmin_filter', 'median')
        if isinstance(self.dmin_filter, str):
            self.dmin_filter = self.dmin_filter.strip().lower()
        self.median_window = int(rospy.get_param('~median_window', 5))
        self.ema_alpha = float(rospy.get_param('~ema_alpha', 0.4))
        self.priority = rospy.get_param('~priority', ['default', 'second', 'third'])
        if isinstance(self.priority, str):
            try:
                txt = self.priority.strip().strip('[]')
                parts = [p.strip().strip("'\"") for p in txt.split(',') if p.strip()]
                self.priority = parts if parts else ['default', 'second', 'third']
            except Exception:
                self.priority = ['default', 'second', 'third']
        self.auto_select = bool(rospy.get_param('~auto_select', False))
        self.state = {
            k: {
                'blocked': False,
                'last_change': rospy.Time(0),
                'dmin_raw': float('inf'),
                'dmin_filt': float('inf'),
                'below_since': None,
                'above_since': None,
                'hist': deque(maxlen=max(3, self.median_window)),
            } for k in ['default','second','third']
        }

        # Publishers (latched where appropriate)
        self.pub = rospy.Publisher(self.marker_topic, MarkerArray, queue_size=1, latch=True)
        self.pub_occ = rospy.Publisher(self.occ_topic, String, queue_size=1, latch=True)
        self.pub_scores = rospy.Publisher(self.scores_topic, Float32MultiArray, queue_size=1, latch=True)
        self.pub_rec = rospy.Publisher(self.recommended_topic, String, queue_size=1, latch=True)

        # Subscribers
        self.subs = []
        for key, topic in self.path_topics.items():
            self.subs.append(rospy.Subscriber(topic, Path, self._make_cb(key)))
        if self.obstacle_source in ('clusters', 'both'):
            self.subs.append(rospy.Subscriber(self.cluster_topic, Marker, self._cb_clusters))
        if self.obstacle_source in ('points', 'both'):
            self.subs.append(rospy.Subscriber(self.points_topic, PointCloud2, self._cb_points))
        self.subs.append(rospy.Subscriber(self.odom_topic, Odometry, self._cb_odom))

        # TF2 listener
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        rospy.loginfo('[path_occupancy_evaluator] Ready. Corridor half=%.2f, margin=%.2f, hysteresis=%.2f, step=%d, lookahead=%.1f, src=%s',
                      self.corr_half, self.margin, self.hysteresis_gap, self.step, self.lookahead_m, self.obstacle_source)

    def _param_color(self, name: str, default: List[float]) -> Tuple[float, float, float]:
        val = rospy.get_param(name, default)
        if isinstance(val, list) and len(val) >= 3:
            return float(val[0]), float(val[1]), float(val[2])
        return tuple(default)

    def _make_cb(self, course: str):
        def cb(msg: Path):
            self.paths[course] = msg
            self.caches[course] = self._build_cache(msg)
            self._publish_corridors()
            self._evaluate_and_publish()
        return cb

    @staticmethod
    def _path_to_xy(msg: Path) -> Tuple[np.ndarray, np.ndarray]:
        n = len(msg.poses)
        xs = np.empty(n, dtype=float)
        ys = np.empty(n, dtype=float)
        for i, ps in enumerate(msg.poses):
            xs[i] = ps.pose.position.x
            ys[i] = ps.pose.position.y
        return xs, ys

    def _compute_bounds(self, xs: np.ndarray, ys: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        if len(xs) < 2:
            return xs, ys, xs, ys
        # Tangents and normals
        dx = np.gradient(xs)
        dy = np.gradient(ys)
        yaw = np.arctan2(dy, dx)
        nx = -np.sin(yaw)
        ny =  np.cos(yaw)
        offs = self.corr_half + self.margin
        lx = xs + offs * nx
        ly = ys + offs * ny
        rx = xs - offs * nx
        ry = ys - offs * ny
        return lx, ly, rx, ry

    def _make_line_strip(self, header, ns: str, mid: int, color: Tuple[float, float, float], points: List[Tuple[float, float]]) -> Marker:
        mk = Marker()
        mk.header = header
        mk.ns = ns
        mk.id = mid
        mk.type = Marker.LINE_STRIP
        mk.action = Marker.ADD
        mk.pose.orientation.w = 1.0
        mk.scale.x = self.line_w
        mk.color.r, mk.color.g, mk.color.b = color
        mk.color.a = 1.0
        mk.points = [Point(x=float(x), y=float(y), z=0.0) for (x, y) in points]
        return mk

    def _publish_corridors(self):
        ma = MarkerArray()
        now = rospy.Time.now()
        # Snapshot to avoid 'dictionary changed size during iteration' from concurrent callbacks
        items = list(self.paths.items())
        for course, path_msg in items:
            if len(path_msg.poses) == 0:
                continue
            xs, ys = self._path_to_xy(path_msg)
            # Subsample for visualization density
            xs = xs[::self.step]
            ys = ys[::self.step]
            lx, ly, rx, ry = self._compute_bounds(xs, ys)
            header = path_msg.header
            header.stamp = now
            col = self.colors.get(course, (1.0, 1.0, 1.0))

            left_pts = list(zip(lx.tolist(), ly.tolist()))
            right_pts = list(zip(rx.tolist(), ry.tolist()))

            ma.markers.append(self._make_line_strip(header, f'corridor_{course}', 0, col, left_pts))
            ma.markers.append(self._make_line_strip(header, f'corridor_{course}', 1, col, right_pts))

        if ma.markers:
            self.pub.publish(ma)

    # ---------------- Occupancy evaluation -----------------
    def _cb_odom(self, msg: Odometry):
        # XY only, transform to target_frame if needed
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        frm = msg.header.frame_id or self.target_frame
        if frm != self.target_frame:
            try:
                ps = PointStamped()
                ps.header.stamp = rospy.Time(0)
                ps.header.frame_id = frm
                ps.point.x = x; ps.point.y = y; ps.point.z = 0.0
                tfm = self.tf_buffer.lookup_transform(self.target_frame, frm, rospy.Time(0), rospy.Duration(0.25))
                tp = do_transform_point(ps, tfm)
                x, y = tp.point.x, tp.point.y
            except Exception as e:
                rospy.logwarn_throttle(1.0, f"[path_occupancy] odom TF fail {frm}->{self.target_frame}: {e}")
        self.odom_xy = (x, y)
        self._evaluate_and_publish()

    def _cb_clusters(self, msg: Marker):
        # POINTS; transform to target_frame if needed; XY only
        frm = msg.header.frame_id or self.target_frame
        pts_xy = []
        if frm == self.target_frame:
            pts_xy = [(p.x, p.y) for p in msg.points]
        else:
            try:
                tfm = self.tf_buffer.lookup_transform(self.target_frame, frm, rospy.Time(0), rospy.Duration(0.25))
                for p in msg.points:
                    ps = PointStamped()
                    ps.header.stamp = rospy.Time(0)
                    ps.header.frame_id = frm
                    ps.point.x = p.x; ps.point.y = p.y; ps.point.z = 0.0
                    tp = do_transform_point(ps, tfm)
                    pts_xy.append((tp.point.x, tp.point.y))
            except Exception as e:
                rospy.logwarn_throttle(1.0, f"[path_occupancy] cluster TF fail {frm}->{self.target_frame}: {e}")
        self.clusters_xy = pts_xy
        self._evaluate_and_publish()

    def _transform_points_xy(self, pts_xyz, src_frame: str) -> list:
        # Transform list of (x,y,z) to target_frame; return [(x,y)]
        if src_frame == self.target_frame:
            return [(float(x), float(y)) for (x, y, _z) in pts_xyz]
        try:
            tfm = self.tf_buffer.lookup_transform(self.target_frame, src_frame, rospy.Time(0), rospy.Duration(0.25))
            tx = tfm.transform.translation.x
            ty = tfm.transform.translation.y
            tz = tfm.transform.translation.z
            qx = tfm.transform.rotation.x
            qy = tfm.transform.rotation.y
            qz = tfm.transform.rotation.z
            qw = tfm.transform.rotation.w
            # quaternion to rotation matrix
            xx, yy, zz = qx*qx, qy*qy, qz*qz
            xy, xz, yz = qx*qy, qx*qz, qy*qz
            wx, wy, wz = qw*qx, qw*qy, qw*qz
            r00 = 1 - 2*(yy + zz); r01 = 2*(xy - wz);     r02 = 2*(xz + wy)
            r10 = 2*(xy + wz);     r11 = 1 - 2*(xx + zz); r12 = 2*(yz - wx)
            r20 = 2*(xz - wy);     r21 = 2*(yz + wx);     r22 = 1 - 2*(xx + yy)
            out = []
            for (x, y, z) in pts_xyz:
                xp = r00*x + r01*y + r02*z + tx
                yp = r10*x + r11*y + r12*z + ty
                out.append((float(xp), float(yp)))
            return out
        except Exception as e:
            rospy.logwarn_throttle(1.0, f"[path_occupancy] points TF fail {src_frame}->{self.target_frame}: {e}")
            return []

    def _cb_points(self, msg: PointCloud2):
        # Read subsampled XY from PointCloud2 and cache
        frm = msg.header.frame_id or self.target_frame
        stride = max(1, int(self.points_stride))
        cap = max(100, int(self.points_max))
        pts_xyz = []
        try:
            for i, p in enumerate(pc2.read_points(msg, field_names=("x","y","z"), skip_nans=True)):
                if (i % stride) != 0:
                    continue
                x, y, z = float(p[0]), float(p[1]), float(p[2])
                pts_xyz.append((x, y, z))
                if len(pts_xyz) >= cap:
                    break
        except Exception as e:
            rospy.logwarn_throttle(1.0, f"[path_occupancy] read_points error: {e}")
            self.points_xy = []
            return
        self.points_xy = self._transform_points_xy(pts_xyz, frm)
        self._evaluate_and_publish()

    @staticmethod
    def _build_cache(msg: Path) -> Dict[str, np.ndarray]:
        xs, ys = PathCorridorViz._path_to_xy(msg)
        n = len(xs)
        if n < 2:
            return {}
        Ax = xs[:-1]; Ay = ys[:-1]
        Bx = xs[1:];  By = ys[1:]
        Dx = Bx - Ax; Dy = By - Ay
        seg_len = np.hypot(Dx, Dy)
        valid = seg_len > 1e-6
        Ax, Ay, Dx, Dy, seg_len = Ax[valid], Ay[valid], Dx[valid], Dy[valid], seg_len[valid]
        seg_len2 = seg_len * seg_len
        s0 = np.concatenate([[0.0], np.cumsum(seg_len[:-1])])
        return {'Ax':Ax,'Ay':Ay,'Dx':Dx,'Dy':Dy,'seg_len':seg_len,'seg_len2':seg_len2,'s0':s0}

    @staticmethod
    def _project_point_to_path(px: float, py: float, cache: Dict[str, np.ndarray]) -> Tuple[float, float]:
        if not cache:
            return float('inf'), float('inf')
        Ax=cache['Ax']; Ay=cache['Ay']; Dx=cache['Dx']; Dy=cache['Dy']
        seg_len=cache['seg_len']; seg_len2=cache['seg_len2']; s0=cache['s0']
        qx = px - Ax; qy = py - Ay
        t = (qx*Dx + qy*Dy) / seg_len2
        t = np.clip(t, 0.0, 1.0)
        proj_x = Ax + t*Dx
        proj_y = Ay + t*Dy
        dx = proj_x - px; dy = proj_y - py
        dists = np.hypot(dx, dy)
        idx = int(np.argmin(dists))
        d_min = float(dists[idx])
        s_at_min = float(s0[idx] + t[idx]*seg_len[idx])
        return d_min, s_at_min

    def _evaluate_and_publish(self):
        # Compute per-path dmin within lookahead; publish summary even if data is partial
        now = rospy.Time.now()
        dmins = {'default': float('inf'), 'second': float('inf'), 'third': float('inf')}
        statuses = {}
        for course in ['default','second','third']:
            msg = self.paths.get(course)
            cache = self.caches.get(course)
            if msg is None or not cache:
                statuses[course] = 'unknown'
                continue
            # Vehicle s along path (fallback to 0 if odom unavailable)
            if self.odom_xy is not None:
                _, s_vehicle = self._project_point_to_path(self.odom_xy[0], self.odom_xy[1], cache)
            else:
                s_vehicle = 0.0
            s_lo = s_vehicle
            s_hi = s_vehicle + self.lookahead_m
            dmin = float('inf')
            inside_count = 0
            eff_half = self.corr_half + self.margin
            # Gather candidates based on source selection
            candidates = []
            if self.obstacle_source in ('clusters', 'both') and self.clusters_xy:
                candidates.extend(self.clusters_xy)
            if self.obstacle_source in ('points', 'both') and self.points_xy:
                candidates.extend(self.points_xy)
            if candidates:
                for (px, py) in candidates:
                    d, s = self._project_point_to_path(px, py, cache)
                    if s_lo <= s <= s_hi:
                        if d < dmin:
                            dmin = d
                        if d <= eff_half:
                            inside_count += 1
            # Save raw and update filter history
            st = self.state[course]
            st['dmin_raw'] = dmin
            if np.isfinite(dmin):
                st['hist'].append(dmin)
            # Filtering
            d_f = dmin
            if self.dmin_filter == 'median' and len(st['hist']) > 0:
                d_f = float(np.median(list(st['hist'])))
            elif self.dmin_filter == 'ema':
                prev = st['dmin_filt'] if np.isfinite(st['dmin_filt']) else dmin
                if np.isfinite(dmin):
                    d_f = self.ema_alpha * dmin + (1.0 - self.ema_alpha) * prev
                else:
                    d_f = prev
            st['dmin_filt'] = d_f
            dmins[course] = d_f
            # Debounce + hysteresis update
            blocked = st['blocked']
            want_block = (d_f <= self.d_in) and (inside_count >= self.min_points_inside)
            want_clear = (d_f > self.d_out)
            # Track durations
            if want_block:
                st['below_since'] = st['below_since'] or now
            else:
                st['below_since'] = None
            if want_clear:
                st['above_since'] = st['above_since'] or now
            else:
                st['above_since'] = None

            can_flip = (now - st['last_change']).to_sec() >= self.hold_sec
            if blocked:
                if want_clear and st['above_since'] is not None:
                    if (now - st['above_since']).to_sec() >= self.pre_clear_sec and can_flip:
                        st['blocked'] = False
                        st['last_change'] = now
            else:
                if want_block and st['below_since'] is not None:
                    if (now - st['below_since']).to_sec() >= self.pre_block_sec and can_flip:
                        st['blocked'] = True
                        st['last_change'] = now
            statuses[course] = 'blocked' if st['blocked'] else 'clear'

        # Publish summary + scores + recommendation
        self.pub_occ.publish(String(data=f"default:{statuses.get('default','?')} second:{statuses.get('second','?')} third:{statuses.get('third','?')}"))
        arr = Float32MultiArray(); arr.data = [dmins['default'], dmins['second'], dmins['third']]
        self.pub_scores.publish(arr)
        rec = 'none'
        for k in self.priority:
            if statuses.get(k) == 'clear':
                rec = k; break
        self.pub_rec.publish(String(data=rec))


def main():
    rospy.init_node('path_occupancy_evaluator', anonymous=False)
    _ = PathCorridorViz()
    rospy.spin()


if __name__ == '__main__':
    main()
