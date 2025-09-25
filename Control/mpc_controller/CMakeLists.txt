cmake_minimum_required(VERSION 3.10)
project(mpc_controller)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED COMPONENTS
  custom_interface
  geometry_msgs
  morai_msgs
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
)

find_package(Eigen3 REQUIRED)
find_package(OsqpEigen REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mpc_controller_core
  CATKIN_DEPENDS
    custom_interface
    geometry_msgs
    morai_msgs
    nav_msgs
    roscpp
    sensor_msgs
    std_msgs
    tf2_geometry_msgs
    tf2_ros
    visualization_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_library(mpc_controller_core
  src/debug_publisher.cpp
  src/lateral_mpc.cpp
  src/longitudinal_mpc.cpp
  src/path_manager.cpp
  src/speed_profile_buffer.cpp
  src/state_estimator.cpp
  src/utils.cpp
)

target_link_libraries(mpc_controller_core
  ${catkin_LIBRARIES}
  OsqpEigen::OsqpEigen
)

target_include_directories(mpc_controller_core PUBLIC
  include
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(mpc_controller_node
  src/controller_node.cpp
)

target_link_libraries(mpc_controller_node
  mpc_controller_core
  ${catkin_LIBRARIES}
)

add_dependencies(mpc_controller_core
  ${catkin_EXPORTED_TARGETS}
)

add_dependencies(mpc_controller_node
  ${catkin_EXPORTED_TARGETS}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(speed_profile_buffer_test
    tests/test_speed_profile_buffer.cpp)
  if(TARGET speed_profile_buffer_test)
    target_link_libraries(speed_profile_buffer_test
      mpc_controller_core
      ${catkin_LIBRARIES})
  endif()

  catkin_add_gtest(path_manager_test
    tests/test_path_manager.cpp)
  if(TARGET path_manager_test)
    target_link_libraries(path_manager_test
      mpc_controller_core
      ${catkin_LIBRARIES})
  endif()

  catkin_add_gtest(lateral_mpc_test
    tests/test_lateral_mpc.cpp)
  if(TARGET lateral_mpc_test)
    target_link_libraries(lateral_mpc_test
      mpc_controller_core
      ${catkin_LIBRARIES})
  endif()
endif()

install(TARGETS mpc_controller_core mpc_controller_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
