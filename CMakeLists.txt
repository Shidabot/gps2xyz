cmake_minimum_required(VERSION 3.0.2)
project(gps_to_xyz)

add_compile_options(-std=c++14 -Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp sensor_msgs
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(gps_to_xyz_node src/gps_to_xyz.cpp)
add_dependencies(gps_to_xyz_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(gps_to_xyz_node ${catkin_LIBRARIES})

install(TARGETS gps_to_xyz_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

