cmake_minimum_required(VERSION 3.0.2)
project(lidar_obstacle_avoid)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  tf2_geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  visualization_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
# DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES lidar_obstacle_avoid
  CATKIN_DEPENDS geometry_msgs tf2_geometry_msgs nav_msgs roscpp rospy sensor_msgs std_msgs tf visualization_msgs
  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

include_directories(include/lidar_obstacle_avoid)
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)


add_executable(lidar_obstacle_avoid_node src/lidar_obstacle_avoid_control.cpp)
add_dependencies(lidar_obstacle_avoid_node ${lidar_obstacle_avoid_node_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(lidar_obstacle_avoid_node ${catkin_LIBRARIES} )
