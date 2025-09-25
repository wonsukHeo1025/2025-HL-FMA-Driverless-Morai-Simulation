#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Lateral MPC Controller ROS Node

Main node for lateral MPC control of autonomous vehicle.
"""

import rospy
import numpy as np

# Import from installed package (via setup.py and catkin_python_setup)
from mpc_lateral_controller import LateralMpcCore, PathProcessor, StateEstimator
from mpc_lateral_controller.kinematic_mpc_core import KinematicMpcCore

from nav_msgs.msg import Path, Odometry
from morai_msgs.msg import EgoVehicleStatus
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray, Header, Bool
from visualization_msgs.msg import Marker, MarkerArray
from custom_interface.msg import LateralMpcDebug, LateralMpcStatus, ControlInfo, LateralMpcTiming
import tf2_ros
import tf2_geometry_msgs


class LateralMpcControllerNode:
    """ROS node for lateral MPC controller."""
    
    def __init__(self):
        """Initialize the node."""
        rospy.init_node('lateral_mpc_controller')
        
        # Load parameters
        self._load_parameters()
        
        # Initialize components based on model type
        if self.model_mode == 'kinematic':
            self.mpc_core = KinematicMpcCore(self.model_params, self.control_params)
        else:
            self.mpc_core = LateralMpcCore(self.model_params, self.control_params)
        # Dynamic horizon tracking state
        self._last_horizon_update_time = rospy.Time.now()
        self._last_applied_Np = int(self.control_params.get('Np', 1))
        self._last_roi_kappa_metric = 0.0
        self._last_roi_kappa_agg = 'max_abs'
        self._last_roi_i0 = 0
        self._last_roi_i1 = 0
        
        self.path_processor = PathProcessor()
        self.state_estimator = StateEstimator(self.state_source)
        
        # Shadow/GPS state
        self.shadow_topic = rospy.get_param('~shadow_topic', '/localization/gps_shadow')
        self.shadow_end_idx_margin = int(rospy.get_param('~shadow_end_idx_margin', 10))
        self.shadow_freeze_steering = float(rospy.get_param('~shadow_freeze_steering', 0.0))
        self.shadow_active = False
        self.shadow_freeze_active = False
        self.shadow_last_change = rospy.Time.now()

        # TF listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # Publishers
        self.control_info_pub = rospy.Publisher('/lateral_mpc/control_info', ControlInfo, queue_size=1, tcp_nodelay=True)
        self.debug_pub = rospy.Publisher('/lateral_mpc/debug', LateralMpcDebug, queue_size=1, tcp_nodelay=True)
        self.status_pub = rospy.Publisher('/lateral_mpc/status', LateralMpcStatus, queue_size=1, tcp_nodelay=True)
        self.timing_pub = rospy.Publisher('/lateral_mpc/status/verbose', LateralMpcTiming, queue_size=1, tcp_nodelay=True)
        # Visualization namespace for RViz-only topics
        vis_ns = rospy.get_param('~vis_namespace', '/vis')
        # Replaced: predicted_path (path-offset visualization)
        # New: predicted_trajectory (time-evolved kinematic trajectory from base)
        self.pred_traj_pub = rospy.Publisher(f'{vis_ns}/control/lateral_mpc/predicted_trajectory', Path, queue_size=1)
        # Separate visualization topics to toggle independently in RViz
        self.pred_speed_pub = rospy.Publisher(f'{vis_ns}/control/lateral_mpc/predicted_speed_pillars', MarkerArray, queue_size=1)
        self.pred_ctrl_pub = rospy.Publisher(f'{vis_ns}/control/lateral_mpc/predicted_steer_pillars', MarkerArray, queue_size=1)
        # Current index visualization (single cube marker)
        self.current_idx_marker_pub = rospy.Publisher(f'{vis_ns}/control/lateral_mpc/current_idx_marker', Marker, queue_size=1, tcp_nodelay=True)
        # Clamp window visualization (back/front boundary spheres on path)
        self.current_idx_window_pub = rospy.Publisher(f'{vis_ns}/control/lateral_mpc/current_idx_window', MarkerArray, queue_size=1, tcp_nodelay=True)
        # Reinit radius visualization (vehicle-centered disk)
        self.reinit_disk_pub = rospy.Publisher(f'{vis_ns}/control/lateral_mpc/reinit_disk', Marker, queue_size=1, tcp_nodelay=True)
        self.metrics_pub = rospy.Publisher('/lateral_mpc/metrics', Float32MultiArray, queue_size=1)

        # Shadow subscription
        self.shadow_sub = rospy.Subscriber(self.shadow_topic, Bool, self.shadow_callback, queue_size=1)
        
        # Subscribers
        global_path_topic = rospy.get_param('~global_path_topic', '/planning/global/path')
        self.path_sub = rospy.Subscriber(global_path_topic, Path, self.path_callback, queue_size=1)
        self.ego_sub = rospy.Subscriber('/Competition_topic', EgoVehicleStatus, self.ego_callback, queue_size=1)
        self.imu_sub = rospy.Subscriber('/imu', Imu, self.imu_callback, queue_size=1)

        # Optional: Predicted longitudinal velocities from longitudinal MPC (m/s)
        # Subscribe only if topic param is provided; otherwise disabled.
        self.long_predicted_velocity = None
        long_pred_vel_topic = rospy.get_param('~longitudinal_predicted_velocity_topic', None)
        self.long_pred_vel_sub = None
        if long_pred_vel_topic:
            self.long_pred_vel_sub = rospy.Subscriber(long_pred_vel_topic, Float32MultiArray, self.long_pred_vel_callback)
            rospy.loginfo(f"Subscribed to longitudinal predicted velocity: {long_pred_vel_topic}")
        else:
            rospy.loginfo("Longitudinal predicted velocity subscription disabled (no topic param set)")

        # Throttle heavy visualization to avoid starving control loop
        self.predicted_traj_rate = float(rospy.get_param('~predicted_trajectory_rate', 10.0))  # Hz
        self._next_predicted_pub_time = rospy.Time.now()
        
        # Low-speed steering hold parameters
        self.low_speed_freeze_enable = bool(rospy.get_param('~low_speed/freeze_enable', True))
        self.low_speed_freeze_kmph = float(rospy.get_param('~low_speed/freeze_below_kmph', 1.0))  # enter hold ≤ this
        self.low_speed_release_kmph = float(rospy.get_param('~low_speed/release_above_kmph', 2.0))  # exit hold ≥ this
        self.low_speed_publish_rate = float(rospy.get_param('~low_speed/publish_rate_hz', 10.0))  # hold-mode publish rate
        self._is_low_speed_hold = False
        self._next_low_speed_pub_time = rospy.Time.now()
        
        # Control timer (high-rate). Use precise period without extra work inside callback.
        self.control_timer = rospy.Timer(
            rospy.Duration.from_sec(max(1e-3, 1.0/float(self.control_rate))),
            self.control_loop,
            oneshot=False
        )
        
        # Status timer (1Hz)
        self.status_timer = rospy.Timer(
            rospy.Duration(1.0),
            self.publish_status
        )
        
        # State variables
        self.current_path = None
        self.current_speed = 0.0
        self.last_steering = 0.0
        self._last_heading_error = 0.0
        self._last_pd_time = rospy.Time.now()
        self.controller_active = False
        self.consecutive_failures = 0
        
        # Performance metrics
        self.metrics_buffer = {
            'lateral_errors': [],
            'heading_errors': [],
            'steering_commands': [],
            'solver_times': []
        }
        
        rospy.loginfo("Lateral MPC Controller initialized")
        rospy.loginfo(f"Control mode: {self.model_mode.capitalize()} Bicycle Model")
        rospy.loginfo(f"Control rate: {self.control_rate} Hz")

    def long_pred_vel_callback(self, msg: Float32MultiArray):
        """Store predicted longitudinal velocity sequence in m/s (length Np+1 preferred)."""
        try:
            self.long_predicted_velocity = list(msg.data)
        except Exception:
            self.long_predicted_velocity = None

    def shadow_callback(self, msg: Bool):
        if not msg:
            return
        new_state = bool(msg.data)
        if new_state != self.shadow_active:
            self.shadow_active = new_state
            self.shadow_last_change = rospy.Time.now()
            rospy.loginfo("Lateral MPC: GPS shadow %s", "ON" if new_state else "OFF")
            if not new_state:
                self.shadow_freeze_active = False
        else:
            self.shadow_active = new_state

    def _load_parameters(self):
        """Load parameters from ROS parameter server."""
        # Control parameters
        self.control_rate = rospy.get_param('~control_rate', 50.0)
        self.state_source = rospy.get_param('~state_source', 'tf')
        # Load model mode from config or parameter
        self.model_mode = rospy.get_param('~model_mode', 
                                         rospy.get_param('~model/mode', 'kinematic'))  # kinematic or dynamic
        
        # TF frame parameters
        self.ref_frame_id = rospy.get_param('~ref_frame_id', 'reference')
        self.base_frame_id = rospy.get_param('~base_frame_id', 'base')

        # Visualization scales for predicted speed pillars (match global speed profile)
        # Use same defaults as Decision/global_planner/scripts/global_speed_profile.py
        # - base XY scale (cylinder diameter)
        # - height scale per m/s
        self.vis_speed_pillar_base_xy = float(
            rospy.get_param('~vis/speed_pillars/base_scale_xy', 0.1)
        )
        self.vis_speed_pillar_height_per_mps = float(
            rospy.get_param('~vis/speed_pillars/height_scale_per_mps', 0.5)
        )
        
        # MPC parameters - adjust based on model type
        use_seconds_override = rospy.get_param('~mpc/use_seconds_override', False)
        if self.model_mode == 'kinematic':
            # Kinematic model uses fewer state weights
            self.control_params = {
                'Np': rospy.get_param('~mpc/prediction_horizon', 20),
                'Nc': rospy.get_param('~mpc/control_horizon', 5),
                'Ts': 1.0 / self.control_rate,
                'Q_kinematic': rospy.get_param('~mpc/Q_kinematic', [100, 50]),  # [e_y, e_psi]
                'P_kinematic': rospy.get_param('~mpc/P_kinematic', [1000, 500]),
                'R': rospy.get_param('~mpc/R', 1.0),
                'R_delta': rospy.get_param('~mpc/R_delta', 10.0),
                'delta_limits': rospy.get_param('~mpc/delta_limits', [-0.7, 0.7]),
                'delta_rate_max': rospy.get_param('~mpc/delta_rate_max', 0.01),
                'preview_distance': rospy.get_param('~mpc/preview_distance', 5.0),
                # OSQP solver tuning (speed-first defaults)
                'osqp_max_iter': rospy.get_param('~mpc/osqp_max_iter', 400),
                'osqp_eps_abs': rospy.get_param('~mpc/osqp_eps_abs', 1e-3),
                'osqp_eps_rel': rospy.get_param('~mpc/osqp_eps_rel', 1e-3),
                'osqp_polish': rospy.get_param('~mpc/osqp_polish', False),
                'osqp_time_limit': rospy.get_param('~mpc/osqp_time_limit', -1.0),
                # Solver backend selection
                'solver_backend': rospy.get_param('~mpc/solver_backend', 'cvxpy'),  # 'cvxpy' or 'osqp'
                # Nonlinear weighting features (band soft-constraint, IRLS, time-varying Q/P)
                'time_varying_weights_enable': rospy.get_param('~mpc/time_varying_weights_enable', False),
                'q_scale_front': rospy.get_param('~mpc/q_scale_front', 0.7),
                'q_scale_back': rospy.get_param('~mpc/q_scale_back', 1.3),
                'p_scale_end': rospy.get_param('~mpc/p_scale_end', 1.3),
                'band_base_width': rospy.get_param('~mpc/band_base_width', 0.2),
                'band_enable': rospy.get_param('~mpc/band_enable', False),
                'band_use_speed_curvature': rospy.get_param('~mpc/band_use_speed_curvature', True),
                'band_width_min': rospy.get_param('~mpc/band_width_min', 0.1),
                'band_width_max': rospy.get_param('~mpc/band_width_max', 0.35),
                'band_width_speed_gain': rospy.get_param('~mpc/band_width_speed_gain', 0.0),
                'band_width_curvature_gain': rospy.get_param('~mpc/band_width_curvature_gain', 0.0),
                'band_lambda': rospy.get_param('~mpc/band_lambda', 10.0 * float(rospy.get_param('~mpc/Q_kinematic', [100, 50])[0])),
                'irls_enable': rospy.get_param('~mpc/irls_enable', False),
                'irls_max_iters': rospy.get_param('~mpc/irls_max_iters', 1),
                'irls_kappa': rospy.get_param('~mpc/irls_kappa', 0.2),
                'irls_epsilon': rospy.get_param('~mpc/irls_epsilon', 1e-3),
                'auto_switch_backend': rospy.get_param('~mpc/auto_switch_backend', True),
                # In-band PD assist on heading (hotfix for band overshoot)
                'band_pd_enable': rospy.get_param('~mpc/band_pd_enable', True),
                'band_pd_kp_psi': rospy.get_param('~mpc/band_pd_kp_psi', 0.0),
                'band_pd_kd_psi': rospy.get_param('~mpc/band_pd_kd_psi', 0.6),
                'band_pd_alpha': rospy.get_param('~mpc/band_pd_alpha', 0.7),
                'band_pd_deadzone_rad': rospy.get_param('~mpc/band_pd_deadzone_rad', 0.02),
            }
            # Optional: override horizons using seconds parameters for consistency with other controllers
            if use_seconds_override:
                horizon_s = rospy.get_param('~mpc/horizon_seconds', None)
                control_horizon_s = rospy.get_param('~mpc/control_horizon_seconds', None)
                if isinstance(horizon_s, (int, float)) and horizon_s > 0:
                    self.control_params['Np'] = max(1, int(round(horizon_s * self.control_rate)))
                if isinstance(control_horizon_s, (int, float)) and control_horizon_s > 0:
                    self.control_params['Nc'] = max(1, int(round(control_horizon_s * self.control_rate)))
        else:
            # Dynamic model uses full state weights
            self.control_params = {
                'Np': rospy.get_param('~mpc/prediction_horizon', 20),
                'Nc': rospy.get_param('~mpc/control_horizon', 5),
                'Ts': 1.0 / self.control_rate,
                'Q': np.diag(self._parse_list(rospy.get_param('~mpc/Q', [100, 10, 50, 5]))),
                'R': rospy.get_param('~mpc/R', 1.0),
                'R_delta': rospy.get_param('~mpc/R_delta', 10.0),
                'delta_limits': rospy.get_param('~mpc/delta_limits', [-0.7, 0.7]),
                'delta_rate_max': rospy.get_param('~mpc/delta_rate_max', 0.01),
                'preview_distance': rospy.get_param('~mpc/preview_distance', 5.0),
                'osqp_max_iter': rospy.get_param('~mpc/osqp_max_iter', 400),
                'osqp_eps_abs': rospy.get_param('~mpc/osqp_eps_abs', 1e-3),
                'osqp_eps_rel': rospy.get_param('~mpc/osqp_eps_rel', 1e-3),
                'osqp_polish': rospy.get_param('~mpc/osqp_polish', False),
                'osqp_time_limit': rospy.get_param('~mpc/osqp_time_limit', -1.0),
                # Solver backend selection
                'solver_backend': rospy.get_param('~mpc/solver_backend', 'cvxpy'),  # 'cvxpy' or 'osqp'
                # Nonlinear weighting features (band soft-constraint, IRLS, time-varying Q/P)
                'time_varying_weights_enable': rospy.get_param('~mpc/time_varying_weights_enable', False),
                'q_scale_front': rospy.get_param('~mpc/q_scale_front', 0.7),
                'q_scale_back': rospy.get_param('~mpc/q_scale_back', 1.3),
                'p_scale_end': rospy.get_param('~mpc/p_scale_end', 1.3),
                'band_base_width': rospy.get_param('~mpc/band_base_width', 0.2),
                'band_enable': rospy.get_param('~mpc/band_enable', False),
                'band_use_speed_curvature': rospy.get_param('~mpc/band_use_speed_curvature', True),
                'band_width_min': rospy.get_param('~mpc/band_width_min', 0.1),
                'band_width_max': rospy.get_param('~mpc/band_width_max', 0.35),
                'band_width_speed_gain': rospy.get_param('~mpc/band_width_speed_gain', 0.0),
                'band_width_curvature_gain': rospy.get_param('~mpc/band_width_curvature_gain', 0.0),
                'band_lambda': rospy.get_param('~mpc/band_lambda', 10.0 * float(self._parse_list(rospy.get_param('~mpc/Q', [100, 10, 50, 5]))[0])),
                'irls_enable': rospy.get_param('~mpc/irls_enable', False),
                'irls_max_iters': rospy.get_param('~mpc/irls_max_iters', 1),
                'irls_kappa': rospy.get_param('~mpc/irls_kappa', 0.2),
                'irls_epsilon': rospy.get_param('~mpc/irls_epsilon', 1e-3),
                'auto_switch_backend': rospy.get_param('~mpc/auto_switch_backend', True),
                # In-band PD assist on heading (hotfix for band overshoot)
                'band_pd_enable': rospy.get_param('~mpc/band_pd_enable', True),
                'band_pd_kp_psi': rospy.get_param('~mpc/band_pd_kp_psi', 0.0),
                'band_pd_kd_psi': rospy.get_param('~mpc/band_pd_kd_psi', 0.6),
                'band_pd_alpha': rospy.get_param('~mpc/band_pd_alpha', 0.7),
                'band_pd_deadzone_rad': rospy.get_param('~mpc/band_pd_deadzone_rad', 0.02),
            }
            if use_seconds_override:
                horizon_s = rospy.get_param('~mpc/horizon_seconds', None)
                control_horizon_s = rospy.get_param('~mpc/control_horizon_seconds', None)
                if isinstance(horizon_s, (int, float)) and horizon_s > 0:
                    self.control_params['Np'] = max(1, int(round(horizon_s * self.control_rate)))
                if isinstance(control_horizon_s, (int, float)) and control_horizon_s > 0:
                    self.control_params['Nc'] = max(1, int(round(control_horizon_s * self.control_rate)))
        
        # Enforce Nc=1 as per requirement (stability)
        try:
            self.control_params['Nc'] = 1
        except Exception:
            pass

        # Model parameters based on model type
        if self.model_mode == 'kinematic':
            # Kinematic model only needs wheelbase
            self.model_params = {
                'L': rospy.get_param('~model/wheelbase', 3.0)  # Wheelbase [m]
            }
        else:
            # Dynamic model needs full parameters
            self.model_params = {
                'm': 1901.0,  # 차량 질량 [kg]
                'Iz': 2456.54,  # Iz_combined 사용 [kg·m²]
                'lf': 1.7,  # 전륜 거리 [m]
                'lr': 1.3,  # 후륜 거리 [m]
                'L': 3.0,  # 휠베이스 [m]
                'Caf': 50040.6,  # 전륜 코너링 강성 [N/rad]
                'Car': 198123.4,  # 후륜 코너링 강성 [N/rad]
                'Kv': 0.00684  # understeer gradient [rad/(m/s²)]
            }

        # If advanced weighting features are enabled but OSQP backend was selected,
        # optionally auto-switch to 'cvxpy' backend (which supports these features).
        # Do NOT auto-switch to cvxpy by default anymore; OSQP path is preferred for real-time
    
    def _parse_list(self, param):
        """Parse list parameter."""
        if isinstance(param, str):
            return eval(param)
        return param

    def _maybe_update_horizon(self, closest_idx: int):
        """Dynamically adjust prediction horizon (Np) based on path curvature in ROI.
        No dependence on speed. Rebuilds MPC core when change passes thresholds.
        """
        # Load dynamic horizon params on first use
        if not hasattr(self, 'horizon_dynamic_enable'):
            self.horizon_dynamic_enable = bool(rospy.get_param('~mpc/horizon_dynamic_enable', False))
            self.horizon_min_steps = int(rospy.get_param('~mpc/horizon_min_steps', 50))
            self.horizon_max_steps = int(rospy.get_param('~mpc/horizon_max_steps', 170))
            self.horizon_quantize_steps = int(rospy.get_param('~mpc/horizon_quantize_steps', 10))
            self.horizon_change_threshold_steps = int(rospy.get_param('~mpc/horizon_change_threshold_steps', 15))
            self.horizon_cooldown_sec = float(rospy.get_param('~mpc/horizon_cooldown_sec', 0.3))
            # Curvature-ROI (steps-only)
            self.horizon_roi_steps = int(rospy.get_param('~mpc/horizon_roi_steps', 80))
            self.horizon_kappa_agg = str(rospy.get_param('~mpc/horizon_kappa_agg', 'max_abs')).lower()  # 'max_abs' or 'rms'
            self.horizon_kappa_low = float(rospy.get_param('~mpc/horizon_kappa_low', 0.001))
            self.horizon_kappa_high = float(rospy.get_param('~mpc/horizon_kappa_high', 0.02))

        # Validate path
        if not self.path_processor or not self.path_processor.is_valid:
            return
        n_pts = len(self.path_processor.curvatures) if self.path_processor.curvatures is not None else 0
        if n_pts == 0 or closest_idx < 0 or closest_idx >= n_pts:
            return

        # Build ROI indices (steps-only)
        i0 = max(0, closest_idx)
        i1 = max(i0, min(n_pts - 1, closest_idx + int(self.horizon_roi_steps)))

        if i1 <= i0:
            return

        seg = np.array(self.path_processor.curvatures[i0:i1+1])
        if seg.size == 0:
            return

        # Aggregate curvature magnitude in ROI
        if self.horizon_kappa_agg == 'rms':
            kappa_metric = float(np.sqrt(np.mean(np.square(seg))))
        else:
            kappa_metric = float(np.max(np.abs(seg)))
        # Store for publishing on control_info
        self._last_roi_kappa_metric = float(kappa_metric)
        self._last_roi_kappa_agg = str(self.horizon_kappa_agg)
        self._last_roi_i0 = int(i0)
        self._last_roi_i1 = int(i1)

        # Store curvature metric for logging/telemetry (independent of enable)
        self._last_roi_kappa_metric = float(kappa_metric)
        self._last_roi_kappa_agg = str(self.horizon_kappa_agg)
        self._last_roi_i0 = int(i0)
        self._last_roi_i1 = int(i1)

        # If dynamic update is disabled, stop here after updating metrics
        if not self.horizon_dynamic_enable:
            return

        # Map curvature to Np (higher curvature -> shorter horizon)
        k_lo = float(self.horizon_kappa_low)
        k_hi = float(self.horizon_kappa_high)
        if k_hi <= k_lo:
            return
        t = (kappa_metric - k_lo) / (k_hi - k_lo)
        t = max(0.0, min(1.0, t))
        target_np = int(round(self.horizon_max_steps - t * (self.horizon_max_steps - self.horizon_min_steps)))

        # Quantize and clamp
        q = max(1, int(self.horizon_quantize_steps))
        target_np = int(round(float(target_np) / q) * q)
        target_np = int(min(self.horizon_max_steps, max(self.horizon_min_steps, target_np)))

        current_np = int(self.control_params.get('Np', target_np))
        if abs(target_np - current_np) < int(self.horizon_change_threshold_steps):
            return

        now = rospy.Time.now()
        if (now - self._last_horizon_update_time).to_sec() < float(self.horizon_cooldown_sec):
            return

        # Rebuild MPC core with new Np; keep Nc=1
        last_u = getattr(self.mpc_core, 'last_u', 0.0)
        self.control_params['Np'] = int(target_np)
        self.control_params['Nc'] = 1
        if self.model_mode == 'kinematic':
            self.mpc_core = KinematicMpcCore(self.model_params, self.control_params)
        else:
            self.mpc_core = LateralMpcCore(self.model_params, self.control_params)
        try:
            self.mpc_core.last_u = last_u
        except Exception:
            pass
        self._last_horizon_update_time = now
        self._last_applied_Np = int(target_np)
        rospy.loginfo(f"[Lateral MPC] Dynamic Np update: {current_np} -> {target_np} (kappa_{self._last_roi_kappa_agg}={kappa_metric:.5f} 1/m, roi=[{i0},{i1}])")

    
    def path_callback(self, msg):
        """Handle new path message."""
        self.path_processor.update_path(msg)
        self.current_path = msg
        
        if self.path_processor.is_valid:
            rospy.loginfo(f"Path updated: {len(self.path_processor.path_points)} points")
            self.controller_active = True
        else:
            rospy.logwarn("Invalid path received")
            self.controller_active = False
    
    def ego_callback(self, msg):
        """Handle ego vehicle status."""
        # Update speed
        self.current_speed = msg.velocity.x  # m/s
        
        # Update state estimator if using ego_status source
        if self.state_source == 'ego_status':
            self.state_estimator.update_from_ego_status(msg)
    
    def imu_callback(self, msg):
        """Handle IMU data."""
        self.state_estimator.update_from_imu(msg)
    
    def control_loop(self, event):
        """Main control loop."""
        loop_t0 = rospy.Time.now()
        # Check if controller is active
        if not self.controller_active or self.current_path is None:
            # Publish minimal timing so topic stays alive for debugging
            timing = LateralMpcTiming()
            timing.header.stamp = rospy.Time.now()
            timing.loop_period_ms = (rospy.Time.now() - loop_t0).to_sec() * 1000.0
            if timing.loop_period_ms > 0:
                timing.loop_frequency_hz = 1000.0 / timing.loop_period_ms
            timing.solver_status = 'inactive'
            try:
                self.timing_pub.publish(timing)
            except Exception:
                pass
            return
        
        try:
            # Update state estimation
            if self.state_source == 'tf':
                t0 = rospy.Time.now()
                success = self.state_estimator.update_from_tf(self.tf_buffer, 
                                                             self.ref_frame_id, 
                                                             self.base_frame_id)
                if not success:
                    # Publish minimal timing for TF failure and skip this tick
                    timing = LateralMpcTiming()
                    timing.header.stamp = rospy.Time.now()
                    timing.loop_period_ms = (rospy.Time.now() - loop_t0).to_sec() * 1000.0
                    if timing.loop_period_ms > 0:
                        timing.loop_frequency_hz = 1000.0 / timing.loop_period_ms
                    timing.t_state_estimation_ms = (rospy.Time.now() - t0).to_sec() * 1000.0
                    timing.solver_status = 'no_tf'
                    try:
                        self.timing_pub.publish(timing)
                    except Exception:
                        pass
                    return
                t_state = (rospy.Time.now() - t0).to_sec() * 1000.0
            else:
                t_state = 0.0
            
            # Low-speed steering hold with hysteresis
            if self.low_speed_freeze_enable:
                v_kmph = max(0.0, float(self.current_speed)) * 3.6
                now = rospy.Time.now()
                
                # Update hold state with hysteresis
                if self._is_low_speed_hold:
                    if v_kmph >= self.low_speed_release_kmph:
                        self._is_low_speed_hold = False
                        rospy.loginfo(f"[Lateral MPC] Exiting low-speed hold (speed={v_kmph:.1f} km/h)")
                    else:
                        # In hold: publish last steering at reduced rate and skip MPC
                        if now >= self._next_low_speed_pub_time:
                            try:
                                self.publish_control_command(self.last_steering)
                            except Exception:
                                pass
                            self._next_low_speed_pub_time = now + rospy.Duration.from_sec(1.0 / max(1e-3, self.low_speed_publish_rate))
                        
                        # Publish minimal timing info
                        timing = LateralMpcTiming()
                        timing.header.stamp = rospy.Time.now()
                        loop_ms = (rospy.Time.now() - loop_t0).to_sec() * 1000.0
                        timing.loop_period_ms = loop_ms
                        if loop_ms > 0:
                            timing.loop_frequency_hz = 1000.0 / loop_ms
                        timing.t_state_estimation_ms = t_state
                        timing.solver_status = 'low_speed_hold'
                        try:
                            self.timing_pub.publish(timing)
                        except Exception:
                            pass
                        return
                else:
                    if v_kmph <= self.low_speed_freeze_kmph:
                        # Enter hold immediately and publish once
                        self._is_low_speed_hold = True
                        rospy.loginfo(f"[Lateral MPC] Entering low-speed hold (speed={v_kmph:.1f} km/h, holding steering={self.last_steering:.3f} rad)")
                        try:
                            self.publish_control_command(self.last_steering)
                        except Exception:
                            pass
                        self._next_low_speed_pub_time = now + rospy.Duration.from_sec(1.0 / max(1e-3, self.low_speed_publish_rate))
                        
                        timing = LateralMpcTiming()
                        timing.header.stamp = rospy.Time.now()
                        loop_ms = (rospy.Time.now() - loop_t0).to_sec() * 1000.0
                        timing.loop_period_ms = loop_ms
                        if loop_ms > 0:
                            timing.loop_frequency_hz = 1000.0 / loop_ms
                        timing.t_state_estimation_ms = t_state
                        timing.solver_status = 'low_speed_hold'
                        try:
                            self.timing_pub.publish(timing)
                        except Exception:
                            pass
                        return
            
            # Get current pose
            t1 = rospy.Time.now()
            current_pose = {
                'x': self.state_estimator.current_state['x'],
                'y': self.state_estimator.current_state['y'],
                'heading': self.state_estimator.current_state['heading']
            }
            t_error = (rospy.Time.now() - t1).to_sec() * 1000.0  # negligible but measured
            
            # Calculate path errors
            t2 = rospy.Time.now()
            lateral_error, heading_error = self.path_processor.calculate_errors(
                current_pose,
                velocity=self.current_speed,
                Ts=self.control_params['Ts']
            )
            t_err_comp = (rospy.Time.now() - t2).to_sec() * 1000.0
            
            # Get error state for MPC
            t3 = rospy.Time.now()
            if self.model_mode == 'kinematic':
                # Kinematic model only needs [e_y, e_psi]
                error_state = np.array([lateral_error, heading_error])
            else:
                # Dynamic model needs full error state [e_y, e_y_dot, e_psi, e_psi_dot]
                error_state = self.state_estimator.get_error_state(
                    lateral_error, heading_error
                )
            t_ref_build0 = rospy.Time.now()
            
            # Get reference path information
            # Pass velocity and Ts for hysteresis with progress prediction
            closest_idx = self.path_processor.find_closest_point(
                current_pose, 
                velocity=self.current_speed,
                Ts=self.control_params['Ts']
            )
            # Curvature-based dynamic horizon update (no speed dependency)
            try:
                self._maybe_update_horizon(closest_idx)
            except Exception:
                pass
            Np = self.control_params['Np']
            
            # Get curvatures ahead
            curvatures = self.path_processor.get_curvatures_ahead(
                closest_idx, Np, 
                self.current_speed * self.control_params['Ts']
            )
            
            reference_info = {
                'curvatures': curvatures,
                'closest_idx': closest_idx
            }
            t_ref_build = (rospy.Time.now() - t_ref_build0).to_sec() * 1000.0

            # Publish current index marker for RViz visualization
            try:
                self.publish_current_idx_marker(closest_idx)
                # self.publish_current_idx_window(closest_idx)  # COMMENTED OUT - window visualization
                # self.publish_reinit_disk(current_pose)        # COMMENTED OUT - reinit disk visualization
            except Exception:
                pass
            
            # Determine whether to freeze steering near path end under GPS shadow
            shadow_freeze = False
            remaining_points = None
            if self.shadow_active and self.path_processor and getattr(self.path_processor, 'path_points', None):
                total_points = len(self.path_processor.path_points)
                if total_points > 0:
                    clamped_idx = max(0, min(closest_idx, total_points - 1))
                    remaining_points = total_points - 1 - clamped_idx
                    if remaining_points <= max(0, self.shadow_end_idx_margin):
                        shadow_freeze = True

            if shadow_freeze:
                if not self.shadow_freeze_active:
                    rospy.logwarn("Lateral MPC: Shadow freeze active (idx=%d, rem=%s)", closest_idx, remaining_points)
                self.shadow_freeze_active = True
                optimal_steering = self.shadow_freeze_steering
                Nc = int(self.control_params.get('Nc', 1))
                control_sequence = np.full((1, max(1, Nc)), optimal_steering, dtype=float)
                debug_info = {
                    'solver_status': 'shadow_freeze',
                    'solver_time': 0.0,
                    'solver_iterations': 0,
                    'cost': 0.0,
                    'control_sequence': control_sequence
                }
                self.consecutive_failures = 0
                t_mpc_total = 0.0
            else:
                if self.shadow_freeze_active:
                    rospy.loginfo("Lateral MPC: Shadow freeze released (idx=%d)", closest_idx)
                self.shadow_freeze_active = False
                t_mpc0 = rospy.Time.now()
                optimal_steering, debug_info = self.mpc_core.solve(
                    error_state, self.current_speed, reference_info
                )
                t_mpc_total = (rospy.Time.now() - t_mpc0).to_sec() * 1000.0

                # Check for solver failure
                if debug_info['solver_status'] != 'optimal':
                    self.consecutive_failures += 1
                    if self.consecutive_failures > 5:
                        rospy.logwarn("MPC solver failing repeatedly")
                else:
                    self.consecutive_failures = 0

            # Publish control command ASAP (keep publisher queue small)
            t_pub0 = rospy.Time.now()
            self.publish_control_command(optimal_steering)
            
            # Publish debug information
            self.publish_debug_info(
                error_state, optimal_steering, debug_info, 
                lateral_error, heading_error, curvatures[0] if len(curvatures) > 0 else 0.0
            )
            t_publish = (rospy.Time.now() - t_pub0).to_sec() * 1000.0
            
            # Publish predicted trajectory (time-evolved from base) using steering sequence and longitudinal prediction
            if debug_info.get('control_sequence') is not None and self.predicted_traj_rate > 0.0:
                now = rospy.Time.now()
                if now >= self._next_predicted_pub_time:
                    self.publish_predicted_trajectory(
                        debug_info['control_sequence'][0, :].tolist() if hasattr(debug_info['control_sequence'], 'shape') else list(debug_info['control_sequence']),
                        current_pose
                    )
                    self._next_predicted_pub_time = now + rospy.Duration.from_sec(1.0 / self.predicted_traj_rate)
            
            # Update metrics
            self.update_metrics(lateral_error, heading_error, 
                              optimal_steering, debug_info.get('solver_time', 0.0))
            
            # Store last steering
            self.last_steering = optimal_steering

            # Verbose timing publish
            loop_ms = (rospy.Time.now() - loop_t0).to_sec() * 1000.0
            timing = LateralMpcTiming()
            timing.header.stamp = rospy.Time.now()
            timing.loop_period_ms = loop_ms
            if loop_ms > 0:
                timing.loop_frequency_hz = 1000.0 / loop_ms
            timing.t_state_estimation_ms = locals().get('t_state', 0.0)
            timing.t_error_computation_ms = t_error + t_err_comp
            timing.t_reference_build_ms = t_ref_build
            timing.t_mpc_setup_ms = max(0.0, t_mpc_total - float(debug_info.get('solver_time', 0.0)))
            timing.t_mpc_solve_ms = float(debug_info.get('solver_time', 0.0))
            timing.t_publish_ms = t_publish
            timing.solver_status = debug_info.get('solver_status', '')
            timing.solver_cost = float(debug_info.get('cost', 0.0))
            timing.solver_iterations = int(debug_info.get('solver_iterations', 0) or 0)
            try:
                self.timing_pub.publish(timing)
            except Exception:
                pass
                
        except Exception as e:
            rospy.logwarn_throttle(1.0, f"Control loop error: {e}")
            # Publish fallback steering command to keep system alive
            try:
                self.publish_control_command(self.last_steering)
            except:
                pass

    def publish_current_idx_marker(self, closest_idx: int):
        """Publish a semi-transparent orange cube at the current index point."""
        if not self.path_processor or not self.path_processor.path_points:
            return
        if closest_idx < 0 or closest_idx >= len(self.path_processor.path_points):
            return

        p = self.path_processor.path_points[closest_idx]

        m = Marker()
        m.header.stamp = rospy.Time.now()
        m.header.frame_id = self.ref_frame_id
        m.ns = 'current_idx'
        m.id = 0
        m.type = Marker.CUBE
        m.action = Marker.ADD
        m.pose.position.x = float(p.get('x', 0.0))
        m.pose.position.y = float(p.get('y', 0.0))
        # Lift slightly above ground to avoid z-fighting
        m.pose.position.z = float(p.get('z', 0.0)) + 0.1
        m.pose.orientation.w = 1.0
        # Size ~0.6 m cube for visibility against grid
        m.scale.x = 0.6
        m.scale.y = 0.6
        m.scale.z = 0.6
        # Orange, semi-transparent
        m.color.r = 1.0
        m.color.g = 0.55
        m.color.b = 0.0
        m.color.a = 0.55
        # No automatic expiration; it will be refreshed at control rate
        m.lifetime = rospy.Duration(0.0)

        self.current_idx_marker_pub.publish(m)

    # # COMMENTED OUT - window visualization
    # def publish_current_idx_window(self, closest_idx: int):
    #     """Publish small light-orange spheres on every path index within the clamp window.
    #
    #     - Shows all indices in [last_idx - B_back, last_idx + F_fwd]
    #     - Each index is visualized as a small semi-transparent sphere
    #     - The current idx itself is also shown separately as a cube (unchanged)
    #     """
    #     if not self.path_processor or not self.path_processor.path_points:
    #         return
    #     n = len(self.path_processor.path_points)
    #     last_idx = int(getattr(self.path_processor, 'last_idx', closest_idx))
    #     B_back = int(getattr(self.path_processor, 'B_back', 3))
    #     F_fwd = int(getattr(self.path_processor, 'F_fwd', 12))
    #
    #     i_min = max(0, min(n - 1, last_idx - B_back))
    #     i_max = max(0, min(n - 1, last_idx + F_fwd))
    #
    #     arr = MarkerArray()
    #
    #     # Clear previous markers in this namespace to avoid stale spheres
    #     clear = Marker()
    #     clear.header.stamp = rospy.Time.now()
    #     clear.header.frame_id = self.ref_frame_id
    #     clear.ns = 'current_idx_window'
    #     clear.id = 0
    #     clear.action = Marker.DELETEALL
    #     arr.markers.append(clear)
    #
    #     # Add spheres for every index in the clamp window (make them more visible)
    #     marker_id = 1
    #     for i in range(i_min, i_max + 1):
    #         p = self.path_processor.path_points[i]
    #         m = Marker()
    #         m.header.stamp = rospy.Time.now()
    #         m.header.frame_id = self.ref_frame_id
    #         m.ns = 'current_idx_window'
    #         m.id = marker_id
    #         m.type = Marker.SPHERE
    #         m.action = Marker.ADD
    #         m.pose.position.x = float(p.get('x', 0.0))
    #         m.pose.position.y = float(p.get('y', 0.0))
    #         m.pose.position.z = float(p.get('z', 0.0)) + 0.1
    #         m.pose.orientation.w = 1.0
    #         # Slightly larger spheres along the path window
    #         m.scale.x = 0.5
    #         m.scale.y = 0.5
    #         m.scale.z = 0.5
    #         # Darker orange, more opaque for visibility
    #         m.color.r = 1.0
    #         m.color.g = 0.45
    #         m.color.b = 0.0
    #         m.color.a = 0.8
    #         m.lifetime = rospy.Duration(0.0)
    #         arr.markers.append(m)
    #         marker_id += 1
    #
    #     self.current_idx_window_pub.publish(arr)


    # # COMMENTED OUT - reinit disk visualization  
    # def publish_reinit_disk(self, current_pose):
    #     """Publish a vehicle-centered disk representing d_reinit radius (clamp hold radius)."""
    #     if not self.path_processor:
    #         return
    #     d_reinit = float(getattr(self.path_processor, 'd_reinit', 5.0))
    #
    #     m = Marker()
    #     m.header.stamp = rospy.Time.now()
    #     m.header.frame_id = self.ref_frame_id
    #     m.ns = 'reinit_disk'
    #     m.id = 0
    #     m.type = Marker.SPHERE
    #     m.action = Marker.ADD
    #     m.pose.position.x = float(current_pose.get('x', 0.0))
    #     m.pose.position.y = float(current_pose.get('y', 0.0))
    #     m.pose.position.z = 0.05
    #     m.pose.orientation.w = 1.0
    #     m.scale.x = 2.0 * d_reinit
    #     m.scale.y = 2.0 * d_reinit
    #     m.scale.z = 0.1
    #     # Soft, very light orange
    #     m.color.r = 1.0
    #     m.color.g = 0.6
    #     m.color.b = 0.0
    #     m.color.a = 0.12
    #     m.lifetime = rospy.Duration(0.0)
    #
    #     self.reinit_disk_pub.publish(m)
    
    def publish_predicted_trajectory(self, steering_sequence, current_pose):
        """Publish time-evolved kinematic trajectory, speed and steering pillars.

        Path (nav_msgs/Path) is published to /lateral_mpc/predicted_trajectory.
        Speed pillars (MarkerArray) to /lateral_mpc/predicted_speed_pillars.
        Steering pillars (MarkerArray) to /lateral_mpc/predicted_steer_pillars.
        """
        try:
            Ts = self.control_params['Ts']
            L = self.model_params['L'] if 'L' in self.model_params else 3.0

            # Prepare input sequences
            delta_seq = list(steering_sequence) if steering_sequence is not None else []

            # Determine desired steps (full prediction horizon)
            desired_steps = int(self.control_params['Np'])
            desired_steps = max(1, desired_steps)

            # Build longitudinal velocity sequence length = desired_steps + 1
            if self.long_predicted_velocity and len(self.long_predicted_velocity) > 0:
                v_seq_full = list(self.long_predicted_velocity)
                if len(v_seq_full) < desired_steps + 1:
                    v_seq_full = v_seq_full + [v_seq_full[-1]] * (desired_steps + 1 - len(v_seq_full))
                else:
                    v_seq_full = v_seq_full[:desired_steps + 1]
            else:
                # Fallback to constant current speed
                v_seq_full = [float(self.current_speed)] * (desired_steps + 1)

            # Build steering sequence length = desired_steps (hold-last padding)
            if len(delta_seq) == 0:
                # no steering info, assume zero steering
                delta_seq_full = [0.0] * desired_steps
            else:
                delta_seq_full = list(delta_seq)
                if len(delta_seq_full) < desired_steps:
                    delta_seq_full = delta_seq_full + [delta_seq_full[-1]] * (desired_steps - len(delta_seq_full))
                else:
                    delta_seq_full = delta_seq_full[:desired_steps]

            # Initial state from current_pose (reference frame, base = rear axle)
            x = current_pose['x']
            y = current_pose['y']
            psi = current_pose['heading']

            path_msg = Path()
            path_msg.header.stamp = rospy.Time.now()
            path_msg.header.frame_id = self.ref_frame_id

            speed_markers = MarkerArray()
            steer_markers = MarkerArray()
            now = rospy.Time.now()

            # Integrate trajectory for desired_steps
            for k in range(desired_steps):
                v = float(v_seq_full[k])
                delta = float(delta_seq_full[k])

                # Append pose BEFORE advancing (pose at step k)
                pose = PoseStamped()
                pose.header = path_msg.header
                pose.pose.position.x = x
                pose.pose.position.y = y
                pose.pose.position.z = 0.0
                # Orientation from psi (yaw-only quaternion)
                qz = np.sin(psi / 2.0)
                qw = np.cos(psi / 2.0)
                pose.pose.orientation.z = qz
                pose.pose.orientation.w = qw
                path_msg.poses.append(pose)

                # Speed pillar marker (blue cylinder)
                m_speed = Marker()
                m_speed.header.stamp = now
                m_speed.header.frame_id = self.ref_frame_id
                m_speed.ns = 'predicted_speed'
                m_speed.id = k
                m_speed.type = Marker.CYLINDER
                m_speed.action = Marker.ADD
                m_speed.pose.position.x = x
                m_speed.pose.position.y = y
                # Match global speed profile pillar scaling (m/s -> height)
                height_v = max(0.02, float(v) * float(self.vis_speed_pillar_height_per_mps))
                m_speed.pose.position.z = 0.5 * height_v
                m_speed.pose.orientation.w = 1.0
                base_xy = max(0.01, float(self.vis_speed_pillar_base_xy))
                m_speed.scale.x = base_xy
                m_speed.scale.y = base_xy
                m_speed.scale.z = height_v
                m_speed.color.r = 0.0
                m_speed.color.g = 0.4
                m_speed.color.b = 1.0
                m_speed.color.a = 0.4
                m_speed.lifetime = rospy.Duration(0.0)
                speed_markers.markers.append(m_speed)

                # Steering pillar marker (thin magenta cylinder) height proportional to |delta|
                m_steer = Marker()
                m_steer.header.stamp = now
                m_steer.header.frame_id = self.ref_frame_id
                m_steer.ns = 'predicted_steer'
                m_steer.id = k
                m_steer.type = Marker.CYLINDER
                m_steer.action = Marker.ADD
                m_steer.pose.position.x = x
                m_steer.pose.position.y = y
                height_d = max(0.02, min(4.0, abs(delta) * 5.0))  # scale rad -> height
                m_steer.pose.position.z = 0.5 * height_d
                m_steer.pose.orientation.w = 1.0
                m_steer.scale.x = 0.04
                m_steer.scale.y = 0.04
                m_steer.scale.z = height_d
                m_steer.color.r = 1.0
                m_steer.color.g = 0.0
                m_steer.color.b = 1.0
                m_steer.color.a = 0.45
                m_steer.lifetime = rospy.Duration(0.0)
                steer_markers.markers.append(m_steer)

                # Advance kinematics
                x = x + v * np.cos(psi) * Ts
                y = y + v * np.sin(psi) * Ts
                psi = psi + (v / max(1e-6, L)) * np.tan(delta) * Ts

            # Append final pose as well
            pose_end = PoseStamped()
            pose_end.header = path_msg.header
            pose_end.pose.position.x = x
            pose_end.pose.position.y = y
            pose_end.pose.position.z = 0.0
            qz = np.sin(psi / 2.0)
            qw = np.cos(psi / 2.0)
            pose_end.pose.orientation.z = qz
            pose_end.pose.orientation.w = qw
            path_msg.poses.append(pose_end)

            # Publish
            self.pred_traj_pub.publish(path_msg)
            self.pred_speed_pub.publish(speed_markers)
            self.pred_ctrl_pub.publish(steer_markers)
        except Exception as e:
            rospy.logwarn_throttle(1.0, f"Failed to publish predicted trajectory: {e}")
    
    def publish_control_command(self, steering_angle):
        """Publish steering command only for longitudinal MPC integration."""
        # Optional in-band PD assist to correct heading when inside band (steady input issue)
        assist_delta = 0.0
        try:
            # Freeze assist during low-speed hold or shadow freeze to keep steering constant
            if getattr(self, 'shadow_freeze_active', False):
                assist_delta = 0.0
            elif not getattr(self, '_is_low_speed_hold', False):
                if bool(self.control_params.get('band_pd_enable', False)):
                    # Heading error (rad)
                    heading_error = float(self.state_estimator.current_state.get('heading_error', 0.0))
                    # Deadzone to avoid chattering
                    dz = float(self.control_params.get('band_pd_deadzone_rad', 0.02))
                    if abs(heading_error) > dz:
                        # Discrete derivative
                        now = rospy.Time.now()
                        dt = max(1e-3, (now - self._last_pd_time).to_sec())
                        dpsi = (heading_error - self._last_heading_error) / dt
                        self._last_pd_time = now
                        self._last_heading_error = heading_error
                        kp = float(self.control_params.get('band_pd_kp_psi', 0.0))
                        kd = float(self.control_params.get('band_pd_kd_psi', 0.6))
                        raw = -kp * heading_error - kd * dpsi
                        alpha = float(self.control_params.get('band_pd_alpha', 0.7))
                        # Blend with MPC steering (low-pass on assist)
                        assist_delta = alpha * raw + (1.0 - alpha) * assist_delta
        except Exception:
            assist_delta = 0.0

        msg = ControlInfo()
        msg.steering = steering_angle + assist_delta  # Keep in radians
        # Add dynamic horizon and curvature ROI info
        try:
            msg.horizon_steps = int(self.control_params.get('Np', 0))
        except Exception:
            msg.horizon_steps = 0
        try:
            msg.roi_curvature = float(getattr(self, '_last_roi_kappa_metric', 0.0))
        except Exception:
            msg.roi_curvature = 0.0
        try:
            mode = str(getattr(self, '_last_roi_kappa_agg', ''))
            msg.roi_curvature_mode = mode
        except Exception:
            msg.roi_curvature_mode = ''
        
        # Find closest path point for current index
        if self.path_processor and self.path_processor.path_points:
            current_pose = {
                'x': self.state_estimator.current_state['x'],
                'y': self.state_estimator.current_state['y'],
                'heading': self.state_estimator.current_state.get('heading', 0.0)
            }
            closest_idx = self.path_processor.find_closest_point(
                current_pose,
                velocity=self.current_speed,
                Ts=self.control_params['Ts']
            )
            msg.current_idx = closest_idx
        else:
            msg.current_idx = 0
        
        self.control_info_pub.publish(msg)
    
    def publish_debug_info(self, error_state, steering, debug_info, 
                          lateral_error, heading_error, curvature):
        """Publish debug information."""
        msg = LateralMpcDebug()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "base"
        
        # Errors
        msg.lateral_error = lateral_error
        msg.heading_error = heading_error
        
        # Only set rates if using dynamic model
        if self.model_mode == 'dynamic' and len(error_state) >= 4:
            msg.lateral_error_rate = error_state[1]
            msg.heading_error_rate = error_state[3]
        else:
            msg.lateral_error_rate = 0.0
            msg.heading_error_rate = 0.0
        
        # Control
        msg.steering_command = steering
        msg.steering_rate = (steering - self.last_steering) * self.control_rate
        
        # Calculate feedforward component
        ff_steering = self.mpc_core.calculate_feedforward_steering(
            curvature, self.current_speed
        )
        msg.feedforward_steering = ff_steering
        msg.feedback_steering = steering - ff_steering
        
        # Solver info
        msg.cost_value = debug_info['cost']
        msg.solver_time = debug_info['solver_time']
        msg.solver_status = debug_info['solver_status']
        if 'solver_iterations' in debug_info and debug_info['solver_iterations'] is not None:
            msg.solver_iterations = int(debug_info['solver_iterations'])
        
        # Predictions
        if debug_info['predicted_states'] is not None:
            states = debug_info['predicted_states']
            msg.predicted_lateral_errors = states[0, :].tolist()
            if states.shape[0] >= 3:
                msg.predicted_heading_errors = states[2, :].tolist()
        
        if debug_info['control_sequence'] is not None:
            msg.predicted_steering = debug_info['control_sequence'][0, :].tolist()
        
        # Path info
        msg.path_curvature = curvature
        msg.preview_distance = self.control_params['preview_distance']
        msg.current_speed = self.current_speed
        msg.control_mode = self.model_mode  # kinematic or dynamic
        
        self.debug_pub.publish(msg)
    
    def publish_status(self, event):
        """Publish controller status."""
        msg = LateralMpcStatus()
        msg.header.stamp = rospy.Time.now()
        
        # Controller state
        msg.is_active = self.controller_active
        msg.is_initialized = self.state_estimator.is_initialized
        msg.control_mode = self.model_mode  # kinematic or dynamic
        
        # Performance metrics
        if self.metrics_buffer['lateral_errors']:
            msg.lateral_rmse = np.sqrt(np.mean(np.square(self.metrics_buffer['lateral_errors'])))
            msg.max_lateral_error = np.max(np.abs(self.metrics_buffer['lateral_errors']))
        
        if self.metrics_buffer['heading_errors']:
            msg.heading_rmse = np.sqrt(np.mean(np.square(self.metrics_buffer['heading_errors'])))
        
        if self.metrics_buffer['solver_times']:
            msg.avg_solver_time = np.mean(self.metrics_buffer['solver_times'])
        
        # Path status
        msg.path_available = self.path_processor.is_valid
        if self.path_processor.is_valid:
            current_pose = {
                'x': self.state_estimator.current_state['x'],
                'y': self.state_estimator.current_state['y'],
                'heading': self.state_estimator.current_state.get('heading', 0.0)
            }
            msg.closest_waypoint_idx = self.path_processor.find_closest_point(
                current_pose,
                velocity=self.current_speed,
                Ts=self.control_params['Ts']
            )
            msg.path_completion = self.path_processor.get_path_completion(msg.closest_waypoint_idx)
        
        # Health
        msg.solver_healthy = self.consecutive_failures < 3
        msg.state_estimation_ok = self.state_estimator.is_valid
        msg.consecutive_failures = self.consecutive_failures
        
        self.status_pub.publish(msg)
        
        # Clear metrics buffer
        self.metrics_buffer = {k: [] for k in self.metrics_buffer}
    
    def update_metrics(self, lateral_error, heading_error, steering, solver_time):
        """Update performance metrics."""
        self.metrics_buffer['lateral_errors'].append(lateral_error)
        self.metrics_buffer['heading_errors'].append(heading_error)
        self.metrics_buffer['steering_commands'].append(steering)
        self.metrics_buffer['solver_times'].append(solver_time)
        
        # Publish aggregated metrics periodically
        if len(self.metrics_buffer['lateral_errors']) >= self.control_rate:
            msg = Float32MultiArray()
            msg.data = [
                np.sqrt(np.mean(np.square(self.metrics_buffer['lateral_errors']))),  # RMSE
                np.max(np.abs(self.metrics_buffer['lateral_errors'])),  # Max error
                np.sqrt(np.mean(np.square(self.metrics_buffer['heading_errors']))),  # Heading RMSE
                np.std(np.diff(self.metrics_buffer['steering_commands']))  # Steering smoothness
            ]
            self.metrics_pub.publish(msg)


def main():
    """Main function."""
    try:
        node = LateralMpcControllerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Fatal error: {e}")
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main()
