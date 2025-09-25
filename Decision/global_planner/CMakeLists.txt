cmake_minimum_required(VERSION 3.0.2)
project(global_planner)

find_package(catkin REQUIRED COMPONENTS
  morai_msgs
  rospy
  sensor_msgs
  std_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
)

catkin_python_setup()

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/global_path_planner.py
  scripts/global_speed_profile.py
  scripts/path_occupancy_evaluator.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY data/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/data
  FILES_MATCHING PATTERN "*.txt" PATTERN "*.csv"
)

install(DIRECTORY docs/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/docs
  FILES_MATCHING PATTERN "*.md"
)
