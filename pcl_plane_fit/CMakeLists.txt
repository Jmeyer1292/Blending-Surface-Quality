cmake_minimum_required(VERSION 2.8.3)
project(pcl_plane_fit)

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS 
    pcl_conversions
    pcl_ros
    roscpp
    sensor_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable (extract_plane src/extract_plane.cpp)

target_link_libraries (extract_plane 
  ${catkin_LIBRARIES}
)
