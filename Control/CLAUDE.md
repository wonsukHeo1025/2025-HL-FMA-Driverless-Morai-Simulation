# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview
This is a ROS-based Model Predictive Control (MPC) system for autonomous vehicle longitudinal control. The project is structured as a Catkin workspace package with phased development approach.

## Build Commands
```bash
# Build the workspace
cd ~/catkin_ws
catkin_make
source devel/setup.bash

# Clean build
cd ~/catkin_ws
catkin_make clean
catkin_make
```

## Running the System
```bash
# Launch data collection with default scenario
roslaunch data_collection data_collection.launch

# Run specific scenarios
roslaunch data_collection data_collection.launch scenario:=step_brake
roslaunch data_collection data_collection.launch scenario:=prbs
roslaunch data_collection data_collection.launch scenario:=chirp

# Monitor system data
rostopic echo /control_system_data

# Real-time visualization
rosrun plotjuggler plotjuggler  # Subscribe to /control_system_data
```

## Testing
```bash
# Run Python tests (when implemented)
cd ~/catkin_ws/src/Control/data_collection
python -m pytest tests/

# Check ROS message generation
rosmsg show control_msgs/SystemIdData
```

## Code Architecture

### Package Structure
- **control_msgs/**: Custom ROS messages for system identification data
- **data_collection/**: Core data collection system with modular Python architecture
  - Node layer: `scripts/data_collection_node.py` - ROS interface
  - Core logic: `src/data_collection/data_collection_core.py` - Business logic
  - Data logging: `src/data_collection/data_logger.py` - CSV export
- **docs/**: Phased development PRDs (Phase 1-4 + Lateral MPC)

### Key Design Patterns
1. **Separation of Concerns**: ROS nodes wrap core Python logic for testability
2. **Sensor Fusion**: Combines IMU, GPS, and TF data for state estimation
3. **Scenario-Based Control**: Implements step, PRBS, and chirp test patterns
4. **Real-time Performance**: 50Hz control loop with buffered logging

### Development Phases
- Phase 1 (COMPLETED): Data collection system
- Phase 2 (PLANNED): MPC controller design  
- Phase 3 (PLANNED): ROS integration
- Phase 4 (PLANNED): Validation and tuning

### Key Dependencies
- ROS Noetic/Melodic
- morai_msgs (MORAI simulator integration)
- sensor_msgs, std_msgs, tf2_ros
- numpy, pandas for data processing

### Data Output
CSV files saved to: `~/catkin_ws/src/Control/data_collection/data/`
Format: `{scenario}_{YYYYMMDD}_{HHMMSS}.csv`