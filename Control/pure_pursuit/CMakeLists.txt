cmake_minimum_required(VERSION 3.0.2)
project(pure_pursuit)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  tf2
  tf2_ros
  morai_msgs
  custom_interface
)

catkin_package(
  CATKIN_DEPENDS
    roscpp std_msgs geometry_msgs nav_msgs visualization_msgs
    tf2 tf2_ros morai_msgs custom_interface
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(pure_pursuit_node src/pure_pursuit_node.cpp)
target_link_libraries(pure_pursuit_node ${catkin_LIBRARIES})
add_dependencies(pure_pursuit_node ${catkin_EXPORTED_TARGETS})

install(TARGETS pure_pursuit_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
