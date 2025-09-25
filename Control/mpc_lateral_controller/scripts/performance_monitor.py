#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Performance Monitor Node

Monitors and logs MPC controller performance metrics.
"""

import rospy
import numpy as np
import pandas as pd
from collections import deque
from datetime import datetime
import os

# Optional matplotlib import for plotting (remove if causing issues)
try:
    import matplotlib
    matplotlib.use('Agg')  # Non-interactive backend
    import matplotlib.pyplot as plt
    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False
    rospy.logwarn("matplotlib not available, plotting disabled")

from std_msgs.msg import Float32MultiArray
from custom_interface.msg import LateralMpcDebug, LateralMpcStatus


class PerformanceMonitor:
    """Monitor and analyze MPC performance."""
    
    def __init__(self):
        """Initialize monitor."""
        rospy.init_node('performance_monitor')
        
        # Parameters
        self.window_size = rospy.get_param('~window_size', 1000)
        self.log_dir = rospy.get_param('~log_dir', '/tmp/lateral_mpc_logs')
        self.save_interval = rospy.get_param('~save_interval', 60.0)  # seconds
        
        # Create log directory
        os.makedirs(self.log_dir, exist_ok=True)
        
        # Data buffers
        self.lateral_errors = deque(maxlen=self.window_size)
        self.heading_errors = deque(maxlen=self.window_size)
        self.steering_commands = deque(maxlen=self.window_size)
        self.solver_times = deque(maxlen=self.window_size)
        self.timestamps = deque(maxlen=self.window_size)
        
        # Statistics
        self.stats = {
            'total_samples': 0,
            'solver_failures': 0,
            'max_lateral_error': 0.0,
            'max_solver_time': 0.0
        }
        
        # Subscribers
        self.debug_sub = rospy.Subscriber('/lateral_mpc/debug', LateralMpcDebug, self.debug_callback)
        self.status_sub = rospy.Subscriber('/lateral_mpc/status', LateralMpcStatus, self.status_callback)
        
        # Save timer
        self.save_timer = rospy.Timer(rospy.Duration(self.save_interval), self.save_data)
        
        # Report timer (1Hz)
        self.report_timer = rospy.Timer(rospy.Duration(1.0), self.print_report)
        
        rospy.loginfo(f"Performance monitor initialized. Logs: {self.log_dir}")
    
    def debug_callback(self, msg):
        """Process debug message."""
        current_time = rospy.Time.now().to_sec()
        
        # Store data
        self.lateral_errors.append(msg.lateral_error)
        self.heading_errors.append(msg.heading_error)
        self.steering_commands.append(msg.steering_command)
        self.solver_times.append(msg.solver_time)
        self.timestamps.append(current_time)
        
        # Update statistics
        self.stats['total_samples'] += 1
        
        if msg.solver_status != "optimal":
            self.stats['solver_failures'] += 1
        
        if abs(msg.lateral_error) > self.stats['max_lateral_error']:
            self.stats['max_lateral_error'] = abs(msg.lateral_error)
        
        if msg.solver_time > self.stats['max_solver_time']:
            self.stats['max_solver_time'] = msg.solver_time
    
    def status_callback(self, msg):
        """Process status message."""
        # Could store high-level metrics here
        pass
    
    def calculate_metrics(self):
        """Calculate performance metrics."""
        if not self.lateral_errors:
            return {}
        
        lateral_errors = np.array(self.lateral_errors)
        heading_errors = np.array(self.heading_errors)
        steering_commands = np.array(self.steering_commands)
        solver_times = np.array(self.solver_times)
        
        metrics = {
            'lateral_rmse': np.sqrt(np.mean(lateral_errors**2)),
            'lateral_mean': np.mean(lateral_errors),
            'lateral_std': np.std(lateral_errors),
            'lateral_max': np.max(np.abs(lateral_errors)),
            
            'heading_rmse': np.sqrt(np.mean(heading_errors**2)),
            'heading_mean': np.mean(heading_errors),
            'heading_std': np.std(heading_errors),
            'heading_max': np.max(np.abs(heading_errors)),
            
            'steering_mean': np.mean(steering_commands),
            'steering_std': np.std(steering_commands),
            'steering_smoothness': np.std(np.diff(steering_commands)) if len(steering_commands) > 1 else 0,
            
            'solver_mean': np.mean(solver_times),
            'solver_max': np.max(solver_times),
            'solver_95th': np.percentile(solver_times, 95),
            
            'failure_rate': self.stats['solver_failures'] / max(1, self.stats['total_samples'])
        }
        
        return metrics
    
    def print_report(self, event):
        """Print performance report."""
        metrics = self.calculate_metrics()
        
        if not metrics:
            return
        
        rospy.loginfo("="*50)
        rospy.loginfo("MPC Performance Report")
        rospy.loginfo("-"*50)
        rospy.loginfo(f"Lateral RMSE: {metrics['lateral_rmse']:.3f} m")
        rospy.loginfo(f"Lateral Max:  {metrics['lateral_max']:.3f} m")
        rospy.loginfo(f"Heading RMSE: {np.degrees(metrics['heading_rmse']):.2f} deg")
        rospy.loginfo(f"Solver Time:  {metrics['solver_mean']:.1f} ms (95th: {metrics['solver_95th']:.1f} ms)")
        rospy.loginfo(f"Failure Rate: {metrics['failure_rate']*100:.1f}%")
        rospy.loginfo("="*50)
    
    def save_data(self, event):
        """Save data to CSV file."""
        if not self.lateral_errors:
            return
        
        # Create DataFrame
        data = {
            'timestamp': list(self.timestamps),
            'lateral_error': list(self.lateral_errors),
            'heading_error': list(self.heading_errors),
            'steering_command': list(self.steering_commands),
            'solver_time': list(self.solver_times)
        }
        
        df = pd.DataFrame(data)
        
        # Generate filename
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = os.path.join(self.log_dir, f"mpc_performance_{timestamp}.csv")
        
        # Save to CSV
        df.to_csv(filename, index=False)
        rospy.loginfo(f"Performance data saved to {filename}")
        
        # Also save summary statistics
        metrics = self.calculate_metrics()
        summary_file = os.path.join(self.log_dir, f"mpc_summary_{timestamp}.txt")
        
        with open(summary_file, 'w') as f:
            f.write("MPC Performance Summary\n")
            f.write("="*40 + "\n")
            f.write(f"Total Samples: {self.stats['total_samples']}\n")
            f.write(f"Duration: {(self.timestamps[-1] - self.timestamps[0]):.1f} seconds\n")
            f.write("-"*40 + "\n")
            
            for key, value in metrics.items():
                if 'heading' in key:
                    f.write(f"{key}: {np.degrees(value):.3f} deg\n")
                elif 'time' in key or 'solver' in key:
                    f.write(f"{key}: {value:.2f} ms\n")
                elif 'rate' in key:
                    f.write(f"{key}: {value*100:.2f}%\n")
                else:
                    f.write(f"{key}: {value:.4f}\n")
    
    def plot_performance(self):
        """Generate performance plots."""
        if not HAS_MATPLOTLIB:
            rospy.logdebug("Matplotlib not available, skipping plots")
            return
            
        if len(self.lateral_errors) < 10:
            return
        
        fig, axes = plt.subplots(2, 2, figsize=(12, 8))
        
        # Time axis
        time = np.array(self.timestamps) - self.timestamps[0]
        
        # Lateral error
        ax = axes[0, 0]
        ax.plot(time, self.lateral_errors, 'b-', alpha=0.7)
        ax.set_xlabel('Time [s]')
        ax.set_ylabel('Lateral Error [m]')
        ax.set_title('Lateral Tracking Error')
        ax.grid(True)
        
        # Heading error
        ax = axes[0, 1]
        ax.plot(time, np.degrees(np.array(self.heading_errors)), 'r-', alpha=0.7)
        ax.set_xlabel('Time [s]')
        ax.set_ylabel('Heading Error [deg]')
        ax.set_title('Heading Tracking Error')
        ax.grid(True)
        
        # Steering command
        ax = axes[1, 0]
        ax.plot(time, np.degrees(np.array(self.steering_commands)), 'g-', alpha=0.7)
        ax.set_xlabel('Time [s]')
        ax.set_ylabel('Steering [deg]')
        ax.set_title('Steering Command')
        ax.grid(True)
        
        # Solver time
        ax = axes[1, 1]
        ax.plot(time, self.solver_times, 'k-', alpha=0.7)
        ax.axhline(y=20, color='r', linestyle='--', label='20ms limit')
        ax.set_xlabel('Time [s]')
        ax.set_ylabel('Solver Time [ms]')
        ax.set_title('MPC Solver Time')
        ax.legend()
        ax.grid(True)
        
        plt.suptitle('MPC Performance Metrics')
        plt.tight_layout()
        
        # Save plot
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        plot_file = os.path.join(self.log_dir, f"mpc_performance_{timestamp}.png")
        plt.savefig(plot_file, dpi=150)
        rospy.loginfo(f"Performance plot saved to {plot_file}")
        
        plt.close()


def main():
    """Main function."""
    try:
        monitor = PerformanceMonitor()
        rospy.spin()
        
        # Save final data on shutdown
        monitor.save_data(None)
        monitor.plot_performance()
        
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()