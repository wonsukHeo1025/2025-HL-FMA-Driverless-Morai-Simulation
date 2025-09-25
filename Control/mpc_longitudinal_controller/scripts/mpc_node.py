#!/usr/bin/env python3

import rospy
import numpy as np

# Import from installed package (via setup.py and catkin_python_setup)
from mpc_longitudinal_controller import MpcCore
from morai_msgs.msg import CtrlCmd, EgoVehicleStatus
from std_msgs.msg import Float64, String, Float32MultiArray, Bool
from custom_interface.msg import LateralMpcTiming
from custom_interface.msg import ControlInfo


class SimpleMpcNode:
    """
    Simplified MPC longitudinal controller.
    Pure profile following without complex endgame logic.
    """
    
    def __init__(self):
        rospy.init_node('mpc_controller_node', anonymous=False)
        
        # Unit conversion constants
        self.MPS_TO_KMPH = 3.6
        self.KMPH_TO_MPS = 1.0 / 3.6
        
        # Load parameters
        self._load_parameters()
        
        # Initialize MPC core
        self.mpc = MpcCore(self.A, self.B, self.d, self.mpc_params)
        rospy.loginfo(f"MPC Core initialized with A={self.A}, B={self.B}, d={self.d}")
        
        # State variables
        self.current_velocity_mps = 0.0
        self.previous_input = 0.0
        self.last_update_time = rospy.Time.now()
        self.low_speed_active = False
        
        # Speed profile
        self.speed_profile = []
        self.current_path_idx = 0
        self.last_profile_time = rospy.Time(0)
        
        # Steering from lateral controller
        self.steering_rad = 0.0
        self.has_steering = False
        self.shadow_active = False
        self.shadow_freeze_active = False
        self.shadow_last_change = rospy.Time.now()
        
        # Publishers
        self.ctrl_cmd_pub = rospy.Publisher(
            self.topic_cmd, CtrlCmd, queue_size=1
        )
        
        # Debug publishers (global namespace under /mpc_longitudinal)
        mon_ns = rospy.get_param('~monitoring_namespace', '/mpc_longitudinal')
        if not mon_ns.startswith('/'):
            mon_ns = '/' + mon_ns
        mon_ns = mon_ns.rstrip('/')

        self.debug_velocity_pub = rospy.Publisher(
            f"{mon_ns}/current_velocity_kmph", Float64, queue_size=1
        )
        self.debug_target_pub = rospy.Publisher(
            f"{mon_ns}/target_velocity_kmph", Float64, queue_size=1
        )
        self.solver_status_pub = rospy.Publisher(
            f"{mon_ns}/solver_status", String, queue_size=1
        )
        # Verbose timing/status (reuse LateralMpcTiming message for consistency)
        self.timing_pub = rospy.Publisher(
            f"{mon_ns}/status/verbose", LateralMpcTiming, queue_size=1, tcp_nodelay=True
        )
        
        # Predicted velocity sequence publisher (for RViz visualization/integration)
        pred_vel_topic = rospy.get_param('~predicted_velocity_topic', '/mpc_longitudinal/predicted_velocity')
        self.pred_vel_pub = rospy.Publisher(pred_vel_topic, Float32MultiArray, queue_size=1)
        
        # Subscribers
        self.ego_sub = rospy.Subscriber(
            self.topic_ego, EgoVehicleStatus, self.ego_callback
        )
        
        # Speed profile subscription (consolidated)
        self.speed_profile_topic = rospy.get_param('~speed_profile_topic', '/planning/speed_profile/global')
        self.speed_profile_sub = rospy.Subscriber(
            self.speed_profile_topic, Float32MultiArray, self.speed_profile_callback
        )
        
        # Lateral control info
        lateral_control_topic = rospy.get_param('~lateral_control_topic', '/lateral_mpc/control_info')
        self.control_info_sub = rospy.Subscriber(
            lateral_control_topic, ControlInfo, self.control_info_callback
        )

        # Shadow status subscription
        self.shadow_sub = rospy.Subscriber(
            self.shadow_topic, Bool, self.shadow_callback, queue_size=1
        )
        
        # Control timer
        self.control_timer = rospy.Timer(
            rospy.Duration(1.0 / self.control_rate), self.control_loop
        )
        
        rospy.loginfo("Simplified MPC Controller Node initialized")
        rospy.loginfo(f"  Control rate: {self.control_rate} Hz")
        rospy.loginfo(f"  Np: {self.mpc_params['Np']}, Nc: {self.mpc_params['Nc']}")
        rospy.loginfo(f"  Publishing to: {self.topic_cmd}")
        
        if self.zero_throttle_window_enabled:
            rospy.loginfo(
                f"  Zero-throttle window: idx {self.zero_throttle_start_idx_05m}-{self.zero_throttle_end_idx_05m} (0.5m grid), "
                f"v<{self.zero_throttle_speed_kmph} km/h"
            )
        if self.stop_window_enabled:
            rospy.loginfo(
                f"  Stop-brake window: idx {self.stop_window_start_idx_05m}-{self.stop_window_end_idx_05m} (0.5m grid), "
                f"brake>={self.stop_brake_value} when v<{self.stop_brake_speed_kmph} km/h"
            )
    
    def _load_parameters(self):
        """Load parameters from ROS parameter server."""
        # System model
        self.A = rospy.get_param('~A', 0.9956)
        self.B = rospy.get_param('~B', 0.0779)
        self.d = rospy.get_param('~d', 0.0336)
        
        # MPC parameters
        # Base MPC parameters (used for normal-speed mode)
        self.mpc_params = {
            'Np': rospy.get_param('~Np', 77),
            'Nc': rospy.get_param('~Nc', 20),  # Fixed: was 77, should be 20
            'Q': rospy.get_param('~Q', 1.0),
            'R': rospy.get_param('~R', 1.0),  # Fixed: was 0.3, should be 1.0
            'R_delta': rospy.get_param('~R_delta', 0.25),  # Fixed: was 0.05, should be 0.25
            'v_min': rospy.get_param('~v_min_kmph', 0.0) * self.KMPH_TO_MPS,
            'v_max': rospy.get_param('~v_max_kmph', 40.0) * self.KMPH_TO_MPS,
            'u_min': rospy.get_param('~u_min', -1.0),
            'u_max': rospy.get_param('~u_max', 1.0),
            'delta_u_max': rospy.get_param('~delta_u_max', 0.10),  # Fixed: was 0.2, should be 0.10
            # OSQP solver tuning
            'max_iter': rospy.get_param('~max_iter', 1200),
            'eps_abs': rospy.get_param('~eps_abs', 1e-3),
            'eps_rel': rospy.get_param('~eps_rel', 1e-3),
            'time_limit': rospy.get_param('~time_limit', -1.0),  # seconds, <=0 to disable
            # Feasibility aids
            'use_state_slack': rospy.get_param('~use_state_slack', True),
            'slack_weight': rospy.get_param('~slack_weight', 1000.0),
            'slack_max': rospy.get_param('~slack_max', 0.5),
        }

        # Low-speed scheduling parameters
        self.low_speed_enabled = rospy.get_param('~low_speed_enabled', True)
        threshold_kmph = rospy.get_param('~low_speed_threshold_kmph', 6.0)
        hysteresis_kmph = rospy.get_param('~low_speed_hysteresis_kmph', 1.0)
        self.low_speed_threshold_mps = threshold_kmph * self.KMPH_TO_MPS
        self.low_speed_hysteresis_mps = hysteresis_kmph * self.KMPH_TO_MPS
        self.low_speed_params = {
            'Nc': rospy.get_param('~low_speed_Nc', 15),
            'Q': rospy.get_param('~low_speed_Q', 0.8),
            'R': rospy.get_param('~low_speed_R', 1.5),
            'R_delta': rospy.get_param('~low_speed_R_delta', 0.4),  # Added: more damping at low speed
            'u_max': rospy.get_param('~low_speed_u_max', 0.6),
            'delta_u_max': rospy.get_param('~low_speed_delta_u_max', 0.05),  # Added: stricter jerk limit
        }

        # Zero-throttle window (index based on 0.5 m path resolution)
        self.zero_throttle_window_enabled = rospy.get_param('~zero_throttle_window_enabled', True)
        self.zero_throttle_start_idx_05m = int(rospy.get_param('~zero_throttle_start_idx_05m', 1190))
        self.zero_throttle_end_idx_05m = int(rospy.get_param('~zero_throttle_end_idx_05m', 1203))
        self.zero_throttle_speed_kmph = float(rospy.get_param('~zero_throttle_speed_kmph', 3.0))
        # Convert to internal 0.05 m indexing and m/s speed
        self.zero_throttle_start_idx = self.zero_throttle_start_idx_05m * 10
        self.zero_throttle_end_idx = self.zero_throttle_end_idx_05m * 10
        self.zero_throttle_speed_mps = self.zero_throttle_speed_kmph * self.KMPH_TO_MPS

        # Stop-brake window (maintain a minimum brake when crawling)
        self.stop_window_enabled = rospy.get_param('~stop_window_enabled', True)
        self.stop_window_start_idx_05m = int(rospy.get_param('~stop_window_start_idx_05m', 1195))
        self.stop_window_end_idx_05m = int(rospy.get_param('~stop_window_end_idx_05m', 1203))
        self.stop_brake_speed_kmph = float(rospy.get_param('~stop_brake_speed_kmph', 1.0))
        self.stop_brake_value = float(rospy.get_param('~stop_brake_value', 0.1))
        # Convert to internal 0.05 m indexing and m/s speed
        self.stop_window_start_idx = self.stop_window_start_idx_05m * 10
        self.stop_window_end_idx = self.stop_window_end_idx_05m * 10
        self.stop_brake_speed_mps = self.stop_brake_speed_kmph * self.KMPH_TO_MPS
        
        # ROS parameters
        self.control_rate = rospy.get_param('~control_rate', 50.0)
        self.topic_ego = rospy.get_param('~topic_ego', '/Competition_topic')
        self.topic_cmd = rospy.get_param('~topic_cmd', '/ctrl_cmd')
        
        # Path resolution for distance-based indexing
        self.path_resolution_m = rospy.get_param('~path_resolution_m', 0.5)
        
        # Integration mode
        self.integrated_mode = rospy.get_param('~integrated_mode', False)
        self.steering_timeout = rospy.get_param('~steering_timeout', 0.5)
        
        # Safety
        self.timeout_duration = rospy.get_param('~timeout_duration', 1.0)

        # Dynamic stop hold near stopline (distance-based brake hold)
        # When distance to next zero-speed index (stopline) <= threshold, enforce minimum brake
        self.dynamic_stop_hold_enabled = rospy.get_param('~dynamic_stop_hold_enabled', True)
        self.dynamic_stop_hold_distance_m = rospy.get_param('~dynamic_stop_hold_distance_m', 1.0)
        self.dynamic_stop_hold_min_brake = rospy.get_param('~dynamic_stop_hold_min_brake', 0.1)

        # Shadow handling near path end
        self.shadow_topic = rospy.get_param('~shadow_topic', '/localization/gps_shadow')
        self.shadow_end_idx_margin = int(rospy.get_param('~shadow_end_idx_margin', 10))
        self.shadow_end_margin_upsampled = max(0, self.shadow_end_idx_margin) * 10
        self.shadow_disable_stop_windows = bool(rospy.get_param('~shadow_disable_stop_windows', True))
        self.shadow_disable_dynamic_stop_hold = bool(rospy.get_param('~shadow_disable_dynamic_stop_hold', True))
    
    def ego_callback(self, msg):
        """Extract current velocity from ego vehicle status."""
        self.current_velocity_mps = msg.velocity.x  # m/s from MORAI
        self.last_update_time = rospy.Time.now()
    
    def speed_profile_callback(self, msg):
        """Receive global speed profile and UPSAMPLE 10x for smoothness."""
        raw_profile = list(msg.data)
        self.speed_profile = self._upsample_profile(raw_profile)
        self.last_profile_time = rospy.Time.now()
        rospy.logdebug(f"Received speed profile: {len(raw_profile)} → {len(self.speed_profile)} points (10x)")
    
    def _upsample_profile(self, raw_profile, factor=10):
        """
        Upsample speed profile by interpolation.
        0.5m spacing → 0.05m spacing (10x denser).
        """
        if len(raw_profile) < 2:
            return raw_profile
        
        # Create high-resolution profile
        n_raw = len(raw_profile)
        n_upsampled = (n_raw - 1) * factor + 1
        upsampled = np.zeros(n_upsampled)
        
        # Linear interpolation between each pair of points
        for i in range(n_raw - 1):
            for j in range(factor):
                alpha = j / float(factor)
                idx = i * factor + j
                upsampled[idx] = (1 - alpha) * raw_profile[i] + alpha * raw_profile[i + 1]
        
        # Last point
        upsampled[-1] = raw_profile[-1]
        
        return upsampled.tolist()
    
    def control_info_callback(self, msg):
        """Extract steering and path index from lateral controller."""
        self.steering_rad = msg.steering
        self.has_steering = True
        # Convert 0.5m index to 0.05m index (10x)
        self.current_path_idx = msg.current_idx * 10
        rospy.logdebug(f"Control info: idx={msg.current_idx}→{self.current_path_idx}, steering={msg.steering:.3f}")

    def shadow_callback(self, msg: Bool):
        if not msg:
            return
        new_state = bool(msg.data)
        if new_state != self.shadow_active:
            self.shadow_active = new_state
            self.shadow_last_change = rospy.Time.now()
            rospy.loginfo("Longitudinal MPC: GPS shadow %s", "ON" if new_state else "OFF")
            if not new_state:
                self.shadow_freeze_active = False
        else:
            self.shadow_active = new_state

    def _distance_to_next_stop(self):
        """
        Find distance [m] from current_path_idx to the next index where the
        upsampled speed profile becomes zero (<= epsilon). Returns None if not found
        or if prerequisites are missing.
        """
        if not self.speed_profile or not isinstance(self.speed_profile, (list, tuple)):
            return None
        # Require control info for a valid index reference
        if not self.has_steering:
            return None
        n = len(self.speed_profile)
        cur = int(max(0, min(self.current_path_idx, n - 1)))
        # Small epsilon for zero detection
        eps = 1e-3
        # Search ahead up to a reasonable window to avoid long scans (e.g., 2000 pts = 100 m)
        max_ahead = min(n - cur, 2000)
        upsampled_resolution = float(self.path_resolution_m) / 10.0  # 0.05 m if base is 0.5 m
        for off in range(0, max_ahead):
            idx = cur + off
            try:
                if float(self.speed_profile[idx]) <= eps:
                    return off * upsampled_resolution
            except Exception:
                break
        return None
    
    def get_target_trajectory(self):
        """
        Extract future speed profile with LINEAR INTERPOLATION for smoothness.
        Avoids step changes from 0.5m grid resolution.
        """
        Np = self.mpc_params['Np']
        dt = 1.0 / self.control_rate
        target_trajectory = np.zeros(Np)
        
        if not self.speed_profile or self.current_path_idx >= len(self.speed_profile):
            # No profile available
            target_trajectory[:] = 10.0 * self.KMPH_TO_MPS  # Default 10 km/h
            return target_trajectory
        
        # Use current velocity to predict future distances
        v_current = max(0.1, self.current_velocity_mps)  # Avoid divide by zero
        
        for k in range(Np):
            # Predict distance traveled by step k
            distance_k = v_current * dt * k
            
            # Convert to upsampled index offset (0.05m spacing now)
            upsampled_resolution = self.path_resolution_m / 10.0  # 0.05m
            exact_idx_offset = distance_k / upsampled_resolution
            exact_future_idx = self.current_path_idx + exact_idx_offset
            
            # Bounds check
            if exact_future_idx >= len(self.speed_profile) - 1:
                target_trajectory[k] = self.speed_profile[-1]
            elif exact_future_idx <= 0:
                target_trajectory[k] = self.speed_profile[0]
            else:
                # Direct indexing - already upsampled, no need for interpolation
                idx = int(round(exact_future_idx))
                idx = min(idx, len(self.speed_profile) - 1)
                target_trajectory[k] = self.speed_profile[idx]
        
        return target_trajectory
    
    def control_loop(self, event):
        """Main control loop - simplified."""
        loop_t0 = rospy.Time.now()
        t_build_ms = 0.0
        t_pub_ms = 0.0
        # Check for timeout
        if (rospy.Time.now() - self.last_update_time).to_sec() > self.timeout_duration:
            rospy.logwarn_throttle(5.0, "No velocity update, emergency stop")
            self.publish_control(0.0, 1.0)
            # Publish minimal timing so topic stays alive
            try:
                timing = LateralMpcTiming()
                timing.header.stamp = rospy.Time.now()
                loop_ms = (rospy.Time.now() - loop_t0).to_sec() * 1000.0
                timing.loop_period_ms = loop_ms
                if loop_ms > 0:
                    timing.loop_frequency_hz = 1000.0 / loop_ms
                timing.solver_status = 'inactive'
                self.timing_pub.publish(timing)
            except Exception:
                pass
            return

        # Speed-scheduled parameter switching with hysteresis
        if self.low_speed_enabled:
            v = self.current_velocity_mps
            if not self.low_speed_active and v <= self.low_speed_threshold_mps:
                self._apply_low_speed_mode(True)
            elif self.low_speed_active and v >= (self.low_speed_threshold_mps + self.low_speed_hysteresis_mps):
                self._apply_low_speed_mode(False)
        
        # Get target trajectory from speed profile
        t_ref0 = rospy.Time.now()
        target_trajectory = self.get_target_trajectory()
        t_build_ms = (rospy.Time.now() - t_ref0).to_sec() * 1000.0
        
        # Solve MPC
        status = 'unknown'
        try:
            u_optimal, status = self.mpc.solve(
                self.current_velocity_mps,
                target_trajectory,
                self.previous_input
            )
            
            if status not in ["optimal", "optimal_inaccurate"]:
                rospy.logwarn(f"MPC solver status: {status}")
                u_optimal = 0.0
                
        except Exception as e:
            rospy.logerr(f"MPC solver exception: {e}")
            u_optimal = 0.0
        
        # Determine if shadow freeze window is active near path end
        shadow_freeze = False
        remaining_idx = None
        clamped_idx = None
        if self.shadow_active and self.speed_profile:
            total_idx = len(self.speed_profile) - 1
            clamped_idx = int(max(0, min(self.current_path_idx, total_idx)))
            remaining_idx = total_idx - clamped_idx
            if remaining_idx <= self.shadow_end_margin_upsampled:
                shadow_freeze = True

        if shadow_freeze != self.shadow_freeze_active:
            if shadow_freeze:
                rospy.logwarn("Longitudinal MPC: Shadow freeze active (idx=%s, rem=%s pts)", clamped_idx, remaining_idx)
            else:
                rospy.loginfo("Longitudinal MPC: Shadow freeze released")
        self.shadow_freeze_active = shadow_freeze

        # Enforce zero throttle in zero-throttle window at low speed
        if self.zero_throttle_window_enabled and not (self.shadow_freeze_active and self.shadow_disable_stop_windows):
            idx = self.current_path_idx
            if self.zero_throttle_start_idx <= idx <= self.zero_throttle_end_idx:
                if self.current_velocity_mps <= self.zero_throttle_speed_mps:
                    u_optimal = min(u_optimal, 0.0)

        # Enforce minimum brake in stop window at crawl speed
        if self.stop_window_enabled and not (self.shadow_freeze_active and self.shadow_disable_stop_windows):
            idx = self.current_path_idx
            if self.stop_window_start_idx <= idx <= self.stop_window_end_idx:
                if self.current_velocity_mps <= self.stop_brake_speed_mps:
                    # Clamp to ensure brake >= stop_brake_value
                    u_optimal = min(u_optimal, -self.stop_brake_value)

        # Dynamic stopline hold brake based on distance to next zero-speed index
        if self.dynamic_stop_hold_enabled and not (self.shadow_freeze_active and self.shadow_disable_dynamic_stop_hold):
            dist_to_stop_m = self._distance_to_next_stop()
            if dist_to_stop_m is not None and dist_to_stop_m <= float(self.dynamic_stop_hold_distance_m):
                u_optimal = min(u_optimal, -float(self.dynamic_stop_hold_min_brake))

        # Convert to accel/brake
        accel = max(0.0, min(1.0, u_optimal))
        brake = max(0.0, min(1.0, -u_optimal))
        
        # Publish control
        t_pub0 = rospy.Time.now()
        self.publish_control(accel, brake)
        t_pub_ms = (rospy.Time.now() - t_pub0).to_sec() * 1000.0
        
        # Update state
        self.previous_input = u_optimal
        
        # Debug info
        self.debug_velocity_pub.publish(Float64(self.current_velocity_mps * self.MPS_TO_KMPH))
        self.debug_target_pub.publish(Float64(target_trajectory[0] * self.MPS_TO_KMPH))
        self.solver_status_pub.publish(String(data=status))
        
        # Publish predicted velocity trajectory (m/s) for visualization
        try:
            x_pred = self.mpc.get_predicted_trajectory()
            if x_pred is not None and hasattr(x_pred, 'size') and x_pred.size > 0:
                arr = Float32MultiArray()
                arr.data = x_pred.tolist()
                self.pred_vel_pub.publish(arr)
        except Exception:
            pass
        
        # Log
        rospy.loginfo_throttle(
            10.0,
            f"V: {self.current_velocity_mps*self.MPS_TO_KMPH:.1f} km/h, "
            f"Target: {target_trajectory[0]*self.MPS_TO_KMPH:.1f} km/h, "
            f"Control: {u_optimal:.3f} (A:{accel:.2f}, B:{brake:.2f})"
        )

        # Publish verbose timing/status (aligned with lateral)
        try:
            timing = LateralMpcTiming()
            timing.header.stamp = rospy.Time.now()
            loop_ms = (rospy.Time.now() - loop_t0).to_sec() * 1000.0
            timing.loop_period_ms = loop_ms
            if loop_ms > 0:
                timing.loop_frequency_hz = 1000.0 / loop_ms
            timing.t_state_estimation_ms = 0.0
            timing.t_error_computation_ms = 0.0
            timing.t_reference_build_ms = t_build_ms
            timing.t_mpc_setup_ms = 0.0
            # Use core-reported solve time if available
            stats = self.mpc.get_last_solve_stats()
            timing.t_mpc_solve_ms = float(stats.get('solve_time_ms', 0.0))
            timing.t_publish_ms = t_pub_ms
            timing.solver_status = str(stats.get('status', 'unknown'))
            timing.solver_iterations = int(stats.get('num_iters', 0))
            self.timing_pub.publish(timing)
        except Exception:
            pass
    
    def publish_control(self, accel, brake):
        """Publish control command."""
        cmd = CtrlCmd()
        cmd.longlCmdType = 1
        cmd.accel = accel
        cmd.brake = brake
        
        # Add steering if integrated mode
        if self.integrated_mode and self.has_steering:
            if self.shadow_freeze_active:
                cmd.steering = 0.0
            else:
                cmd.steering = self.steering_rad
        else:
            cmd.steering = 0.0
        
        self.ctrl_cmd_pub.publish(cmd)

    def _apply_low_speed_mode(self, activate: bool):
        """Apply or restore low-speed-specific MPC parameters.

        Rebuilds the MPC problem only on mode changes.
        """
        if activate == self.low_speed_active:
            return

        # Prepare parameter set
        if activate:
            # Ensure Nc <= Np
            Nc_low = int(max(1, min(self.low_speed_params['Nc'], self.mpc_params['Np'])))
            new_params = {
                'Nc': Nc_low,
                'Q': float(self.low_speed_params['Q']),
                'R': float(self.low_speed_params['R']),
                'R_delta': float(self.low_speed_params.get('R_delta', self.mpc_params['R_delta'])),
                'u_max': float(self.low_speed_params['u_max']),
                'delta_u_max': float(self.low_speed_params.get('delta_u_max', self.mpc_params['delta_u_max'])),
            }
        else:
            new_params = {
                'Nc': int(self.mpc_params['Nc']),
                'Q': float(self.mpc_params['Q']),
                'R': float(self.mpc_params['R']),
                'R_delta': float(self.mpc_params['R_delta']),
                'u_max': float(self.mpc_params['u_max']),
                'delta_u_max': float(self.mpc_params['delta_u_max']),
            }

        # Apply to core (triggers rebuild)
        try:
            self.mpc.update_params(new_params)
            self.low_speed_active = activate
            rospy.loginfo(
                f"Low-speed mode {'ENABLED' if activate else 'DISABLED'}: "
                f"Nc={new_params['Nc']}, Q={new_params['Q']:.1f}, R={new_params['R']:.1f}, "
                f"R_delta={new_params['R_delta']:.2f}, u_max={new_params['u_max']:.1f}, "
                f"delta_u_max={new_params['delta_u_max']:.2f}"
            )
        except Exception as e:
            rospy.logerr(f"Failed to {'enable' if activate else 'disable'} low-speed mode: {e}")


def main():
    try:
        node = SimpleMpcNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Fatal error: {e}")
        raise


if __name__ == '__main__':
    main()
