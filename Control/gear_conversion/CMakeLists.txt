cmake_minimum_required(VERSION 3.0.2)
project(gear_conversion)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  morai_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs morai_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/gear_conversion_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  FILES_MATCHING PATTERN "*.yaml"
)