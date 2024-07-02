cmake_minimum_required(VERSION 3.0.2)
project(gps_to_xyz)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES gps_deal
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


add_executable(gps_to_xyz src/gps_to_xyz.cpp)
target_link_libraries(gps_to_xyz ${catkin_LIBRARIES})
