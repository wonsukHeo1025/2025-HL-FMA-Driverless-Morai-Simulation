cmake_minimum_required(VERSION 3.0.2)
project(mpc_lateral_controller)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  morai_msgs
  custom_interface
)

catkin_package(
  CATKIN_DEPENDS 
    rospy 
    std_msgs 
    geometry_msgs 
    nav_msgs 
    sensor_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    morai_msgs
    custom_interface
)

# Setup Python module installation
catkin_python_setup()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Install Python scripts
catkin_install_python(PROGRAMS
  scripts/lateral_mpc_controller_node.py
  scripts/path_visualizer.py
  scripts/performance_monitor.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
