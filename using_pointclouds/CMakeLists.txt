cmake_minimum_required(VERSION 2.8)
project(using_pointclouds)

find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    pcl_ros
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable (publish_pointcloud src/publish_pointcloud.cpp)

target_link_libraries (publish_pointcloud 
  ${catkin_LIBRARIES}
)
