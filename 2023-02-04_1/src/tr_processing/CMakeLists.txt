cmake_minimum_required(VERSION 2.8.3)

add_compile_options(-std=c++11)

project(tr_processing)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_geometry
  laser_geometry
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
  darknet_ros_msgs
  geometry_msgs
)

find_package(OpenCV REQUIRED)

catkin_package(
    INCLUDE_DIRS include
#  LIBRARIES targetless
    CATKIN_DEPENDS cv_bridge image_geometry pcl_ros roscpp sensor_msgs std_msgs darknet_ros_msgs geometry_msgs
#  DEPENDS system_lib
)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

file(GLOB_RECURSE project_HEADERS
    "*.h"
    "*.hpp"
)

add_executable(
${PROJECT_NAME}
${project_HEADERS}
src/main.cpp 
src/traffic_light_2022_past.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
   ${OpenCV_LIBS}
)

