#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""CLI handler for interactive user input."""

import rospy
import threading
from typing import Callable, Optional


class CLIHandler:
    """Handles command-line interface for user interaction."""
    
    def __init__(self, node_core):
        """Initialize CLI handler.
        
        Args:
            node_core: Reference to the node core instance
        """
        self.node_core = node_core
        self.input_thread = None
        self.running = False
    
    def start(self):
        """Start CLI input handler."""
        self.running = True
        self.input_thread = threading.Thread(target=self._input_loop)
        self.input_thread.daemon = True
        self.input_thread.start()
        
        # Show initial menu
        self.print_menu()
    
    def stop(self):
        """Stop CLI handler."""
        self.running = False
        if self.input_thread:
            self.input_thread.join(timeout=1.0)
    
    def _input_loop(self):
        """Main input loop."""
        while self.running and not rospy.is_shutdown():
            try:
                # Wait for node to be ready
                if self.node_core.state.value not in ['ready', 'complete']:
                    rospy.sleep(0.1)
                    continue
                
                # Get user input
                user_input = input().strip()
                if user_input:
                    self.process_command(user_input)
                    
            except EOFError:
                break
            except KeyboardInterrupt:
                rospy.signal_shutdown("User interrupted")
                break
    
    def process_command(self, command: str):
        """Process user command.
        
        Args:
            command: User input command
        """
        command = command.lower()
        
        # Scenario selection - Longitudinal
        if command == '1':
            self.node_core.start_scenario('step_accel')
        elif command == '2':
            self.node_core.start_scenario('step_accel_reset')
        elif command == '3':
            self.node_core.start_scenario('step_brake')
        elif command == '4':
            self.node_core.start_scenario('prbs')
        elif command == '5':
            self.node_core.start_scenario('chirp')
        
        # Scenario selection - Lateral
        elif command == '6':
            self.start_lateral_scenario('steady_state_cornering')
        elif command == '7':
            self.start_lateral_scenario('step_steer')
        elif command == '8':
            self.start_lateral_scenario('sine_sweep')
        elif command == '9':
            self.start_lateral_scenario('double_lane_change')
        
        # Control commands
        elif command == 'c':
            self.change_control_mode()
        
        elif command == 'i':
            self.show_status()
        
        elif command == 'h':
            self.print_help()
        
        elif command == 'q':
            # Handle quit based on current state
            if self.node_core.state.value in ['countdown', 'speed_building', 
                                             'speed_stabilizing', 'running']:
                self.node_core.stop_scenario()
                self.print_menu()
            else:
                rospy.loginfo("\nShutting down...")
                self.node_core.shutdown()
                rospy.signal_shutdown("User requested shutdown")
        
        else:
            rospy.logwarn("Invalid command: %s", command)
            self.print_menu()
    
    def print_menu(self):
        """Print main menu."""
        menu = """
============================================================
                  DATA COLLECTION SYSTEM
============================================================
LONGITUDINAL Control Scenarios:
  1. Step Acceleration Test (Continuous)
  2. Step Acceleration Test (Reset to Zero)
  3. Step Brake Test
  4. PRBS (Pseudo-Random Binary Sequence)
  5. Chirp Signal Test

LATERAL Control Scenarios:
  6. Steady-State Cornering (언더스티어 측정)
  7. Step Steer Response (과도 응답)
  8. Sine Sweep (주파수 응답)
  9. Double Lane Change (검증)

Other commands:
  c. Change control mode (current: {})
  i. Show vehicle status
  h. Show help
  q. Quit

Enter your choice and press Enter: 
============================================================
        """.format(self.node_core.control_mode)
        rospy.loginfo(menu)
    
    def print_help(self):
        """Print help information."""
        help_text = """
============================================================
                        HELP
============================================================
LONGITUDINAL Scenarios:
  1. Step Acceleration: Incremental acceleration steps (continuous)
  2. Step Acceleration Reset: Each step starts from zero speed
  3. Step Brake: Speed-based brake test at 50 km/h
  4. PRBS: Random binary switching for system ID
  5. Chirp: Frequency-sweep signal for dynamics analysis

LATERAL Scenarios:
  6. Steady-State Cornering: Constant steering at various angles
  7. Step Steer: Transient response to step steering input
  8. Sine Sweep: Frequency response characterization
  9. Double Lane Change: ISO 3888-2 style validation

Control Modes:
  - throttle: Direct accel/brake control (0-1)
  - velocity: Target velocity control (km/h) [횡방향 실험 권장]
  - acceleration: Target acceleration control (m/s²)

Tips for Lateral Testing:
  - Use 'velocity' mode for constant speed
  - Start with low speeds (20-40 km/h)
  - Keep steering angles < 15° for linearity

Press Enter to return to menu...
============================================================
        """
        rospy.loginfo(help_text)
        input()  # Wait for Enter
        self.print_menu()
    
    def show_status(self):
        """Show current vehicle status."""
        from ..utils import mps_to_kmph
        
        status = f"""
========================================
Current Status:
  State: {self.node_core.state.value}
  Scenario: {self.node_core.scenario_type or 'None'}
  Control Mode: {self.node_core.control_mode}
  
Vehicle Data:
  Velocity: {mps_to_kmph(self.node_core.vehicle_state.velocity_x):.1f} km/h
  Accel: {self.node_core.vehicle_state.accel:.2f}
  Brake: {self.node_core.vehicle_state.brake:.2f}
  Wheel Angle: {self.node_core.vehicle_state.wheel_angle:.2f}
========================================
        """
        rospy.loginfo(status)
        rospy.loginfo("Press Enter to continue...")
        input()
        self.print_menu()
    
    def change_control_mode(self):
        """Change control mode interactively."""
        rospy.loginfo("\nSelect control mode:")
        rospy.loginfo("1. Throttle/Brake")
        rospy.loginfo("2. Velocity")
        rospy.loginfo("3. Acceleration")
        
        mode_input = input("Enter choice: ").strip()
        
        if mode_input == '1':
            self.node_core.control_mode = 'throttle'
        elif mode_input == '2':
            self.node_core.control_mode = 'velocity'
        elif mode_input == '3':
            self.node_core.control_mode = 'acceleration'
        else:
            rospy.logwarn("Invalid choice")
        
        rospy.loginfo("Control mode: %s", self.node_core.control_mode)
        rospy.sleep(1.0)
        self.print_menu()
    
    def start_lateral_scenario(self, scenario_name: str):
        """Start lateral scenario with parameter input.
        
        Args:
            scenario_name: Name of the lateral scenario
        """
        import numpy as np
        
        # Get target speed for all lateral scenarios
        rospy.loginfo("\n" + "="*40)
        rospy.loginfo("Starting %s", scenario_name.replace('_', ' ').title())
        rospy.loginfo("="*40)
        
        # Force velocity mode for lateral scenarios
        self.node_core.control_mode = 'velocity'
        rospy.loginfo("Control mode: velocity (required for lateral scenarios)")
        
        # Get target speed
        rospy.loginfo("\n목표 속도 [km/h] (기본값: 40): ")
        speed_input = input().strip()
        target_speed = float(speed_input) if speed_input else 40.0
        
        # Scenario-specific parameters
        scenario_params = {'target_speed': target_speed / 3.6}  # Convert to m/s
        
        if scenario_name == 'steady_state_cornering':
            rospy.loginfo("\n조향각 목록 [도] (양수만 입력, 음수 자동 추가, 쉼표로 구분, 기본: 5,10,15): ")
            angles_input = input().strip()
            if angles_input:
                positive_angles = [float(a) for a in angles_input.split(',')]
                # Create alternating positive and negative angles
                angles = []
                for angle in positive_angles:
                    angles.append(angle)
                    angles.append(-angle)
            else:
                angles = [5, -5, 10, -10, 15, -15]
            rospy.loginfo("실제 테스트 각도: %s", angles)
            scenario_params['steering_angles'] = np.radians(angles)
            
            rospy.loginfo("유지 시간 [초] (기본: 10): ")
            hold_input = input().strip()
            scenario_params['hold_duration'] = float(hold_input) if hold_input else 10.0
            
        elif scenario_name == 'step_steer':
            rospy.loginfo("\n스텝 조향각 목록 [도] (쉼표로 구분, 기본: 10,-10,15,-15): ")
            angles_input = input().strip()
            if angles_input:
                angles = [float(a) for a in angles_input.split(',')]
            else:
                angles = [10, -10, 15, -15]
            scenario_params['step_angles'] = np.radians(angles)
            
            rospy.loginfo("스텝 유지 시간 [초] (기본: 5): ")
            hold_input = input().strip()
            scenario_params['step_duration'] = float(hold_input) if hold_input else 5.0
            
        elif scenario_name == 'sine_sweep':
            rospy.loginfo("\n시작 주파수 [Hz] (기본: 0.1): ")
            f_start_input = input().strip()
            scenario_params['freq_start'] = float(f_start_input) if f_start_input else 0.1
            
            rospy.loginfo("종료 주파수 [Hz] (기본: 2.0): ")
            f_end_input = input().strip()
            scenario_params['freq_end'] = float(f_end_input) if f_end_input else 2.0
            
            rospy.loginfo("스윕 시간 [초] (기본: 60): ")
            sweep_input = input().strip()
            scenario_params['sweep_duration'] = float(sweep_input) if sweep_input else 60.0
            
            rospy.loginfo("조향 진폭 [도] (기본: 5): ")
            amp_input = input().strip()
            amplitude = float(amp_input) if amp_input else 5.0
            scenario_params['amplitude'] = np.radians(amplitude)
            
        elif scenario_name == 'double_lane_change':
            rospy.loginfo("\n차선 폭 [m] (기본: 3.5): ")
            lane_input = input().strip()
            scenario_params['lane_width'] = float(lane_input) if lane_input else 3.5
            
            rospy.loginfo("기동 거리 [m] (기본: 125): ")
            dist_input = input().strip()
            scenario_params['maneuver_distance'] = float(dist_input) if dist_input else 125.0
        
        # Start scenario with parameters
        rospy.loginfo("\n" + "="*50)
        rospy.loginfo("시나리오 시작: %s", scenario_name)
        rospy.loginfo("목표 속도: %.1f km/h", target_speed)
        rospy.loginfo("파라미터: %s", scenario_params)
        rospy.loginfo("\n[단계]")
        rospy.loginfo("1. 목표 속도까지 가속")
        rospy.loginfo("2. 3초간 속도 안정화")
        rospy.loginfo("3. 시나리오 시작")
        rospy.loginfo("="*50)
        
        self.node_core.start_scenario(scenario_name, scenario_params)