#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64, Float32MultiArray
from custom_interface.msg import ControlInfo


class SpeedProfileAdapter:
    """
    Adapter node that extracts target velocity from speed profile
    based on current path index from Lateral MPC.
    
    This bridges between the array-based speed profile and the
    single target velocity that Longitudinal MPC expects.
    """
    
    def __init__(self):
        rospy.init_node('speed_profile_adapter', anonymous=False)
        
        # Parameters
        self.default_speed_kmph = rospy.get_param('~default_speed_kmph', 10.0)
        self.speed_scale_factor = rospy.get_param('~speed_scale_factor', 1.0)
        self.lookahead_offset = rospy.get_param('~lookahead_offset', 2)  # Points ahead of current position
        
        # State
        self.speed_profile = []
        self.latest_current_idx = 0
        self.has_control_info = False
        self.last_profile_time = rospy.Time(0)
        self.using_final_profile = False
        
        # Publishers
        self.target_vel_pub = rospy.Publisher(
            '/target_velocity', Float64, queue_size=1
        )
        
        # Debug publisher
        self.debug_pub = rospy.Publisher(
            '~debug_info', Float32MultiArray, queue_size=1
        )
        
        # Subscribers
        # Subscribe to both profiles - final_speed_profile has priority
        self.final_speed_sub = rospy.Subscriber(
            '/final_speed_profile', Float32MultiArray, self.final_speed_callback
        )
        self.desired_speed_sub = rospy.Subscriber(
            '/planning/speed_profile/global', Float32MultiArray, self.desired_speed_callback
        )
        
        # Subscribe to Lateral MPC control info
        control_info_topic = rospy.get_param('~control_info_topic', '/lateral_mpc/control_info')
        self.control_info_sub = rospy.Subscriber(
            control_info_topic, ControlInfo, self.control_info_callback
        )
        
        # Timer for publishing target velocity
        self.timer = rospy.Timer(rospy.Duration(1.0/50.0), self.publish_target)  # 50Hz
        
        rospy.loginfo("Speed Profile Adapter initialized")
        rospy.loginfo(f"  Default speed: {self.default_speed_kmph} km/h")
        rospy.loginfo(f"  Lookahead offset: {self.lookahead_offset} points")
        rospy.loginfo(f"  Speed scale factor: {self.speed_scale_factor}")
        rospy.loginfo(f"  Control info topic: {control_info_topic}")
        rospy.loginfo("  Subscribing to both /final_speed_profile (priority) and /desired_speed_profile")
    
    def final_speed_callback(self, msg):
        """
        Receive final speed profile from stopline_speed_controller.
        This has priority over desired_speed_profile.
        Values are in m/s.
        """
        self.speed_profile = list(msg.data)
        self.last_profile_time = rospy.Time.now()
        self.using_final_profile = True
        rospy.logdebug(f"Received FINAL speed profile with {len(self.speed_profile)} points")
    
    def desired_speed_callback(self, msg):
        """
        Receive desired speed profile from global_speed_planning.
        Only use if final_speed_profile is not available or stale.
        Values are in m/s.
        """
        # Only use desired profile if final profile is stale (>0.5s old)
        if (rospy.Time.now() - self.last_profile_time).to_sec() > 0.5:
            self.speed_profile = list(msg.data)
            self.using_final_profile = False
            rospy.logdebug(f"Using DESIRED speed profile with {len(self.speed_profile)} points")
    
    def control_info_callback(self, msg):
        """
        Receive control info from Lateral MPC including current index.
        """
        self.latest_current_idx = msg.current_idx
        self.has_control_info = True
        rospy.logdebug(f"Received control info: current_idx={msg.current_idx}, steering={msg.steering:.3f}")
    
    def get_target_velocity(self):
        """
        Extract target velocity based on current position with lookahead offset.
        Returns velocity in km/h for MPC.
        """
        if not self.speed_profile:
            # No speed profile available
            return self.default_speed_kmph
        
        if not self.has_control_info:
            # Waiting for Lateral MPC control info
            rospy.logwarn_throttle(5.0, "No control info from Lateral MPC, using default speed")
            return self.default_speed_kmph
        
        # Use current position with small offset ahead
        idx = self.latest_current_idx + self.lookahead_offset
        
        # Clamp index to valid range
        idx = max(0, min(idx, len(self.speed_profile) - 1))
        
        # Get speed in m/s from profile
        speed_mps = self.speed_profile[idx]
        
        # Convert to km/h and apply scale factor
        speed_kmph = speed_mps * 3.6 * self.speed_scale_factor
        
        # Safety clamp
        speed_kmph = max(0.0, speed_kmph)
        
        return speed_kmph
    
    def publish_target(self, event):
        """
        Publish target velocity at fixed rate.
        """
        target_vel_kmph = self.get_target_velocity()
        
        # Publish target velocity
        vel_msg = Float64()
        vel_msg.data = target_vel_kmph
        self.target_vel_pub.publish(vel_msg)
        
        # Publish debug info
        debug_msg = Float32MultiArray()
        debug_msg.data = [
            float(self.latest_current_idx),
            float(len(self.speed_profile)),
            float(target_vel_kmph),
            float(self.has_control_info),
            float(self.using_final_profile)  # 1.0 if using final, 0.0 if using desired
        ]
        self.debug_pub.publish(debug_msg)
        
        # Log profile source change
        if event and hasattr(self, '_last_using_final') and self._last_using_final != self.using_final_profile:
            rospy.loginfo(f"Speed profile source changed to: {'FINAL' if self.using_final_profile else 'DESIRED'}")
            self._last_using_final = self.using_final_profile
        
        # Log occasionally
        actual_idx = self.latest_current_idx + self.lookahead_offset
        rospy.loginfo_throttle(
            10.0,
            f"Target velocity: {target_vel_kmph:.1f} km/h "
            f"(current={self.latest_current_idx}, "
            f"using=current+{self.lookahead_offset}={actual_idx}/{len(self.speed_profile)})"
        )


def main():
    try:
        node = SpeedProfileAdapter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Fatal error in speed profile adapter: {e}")
        raise


if __name__ == '__main__':
    main()
