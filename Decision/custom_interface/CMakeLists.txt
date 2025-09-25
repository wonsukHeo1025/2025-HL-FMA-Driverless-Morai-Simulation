cmake_minimum_required(VERSION 3.0.2)
project(custom_interface)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  morai_msgs
  message_generation
)

# Generate messages
add_message_files(
  FILES
  # From control_msgs
  ControlInfo.msg
  LateralSystemData.msg
  SystemIdData.msg
  # From mpc_lateral_controller
  LateralMpcDebug.msg
  LateralMpcStatus.msg
  LateralMpcTiming.msg
  # From path_roi_planner
  Waypoint.msg
  WaypointArray.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  morai_msgs
)

catkin_package(
  CATKIN_DEPENDS 
    std_msgs 
    geometry_msgs
    nav_msgs
    sensor_msgs
    morai_msgs
    message_runtime
)