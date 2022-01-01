cmake_minimum_required(VERSION 3.0.2)
project(pcl_test)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pcl_test
#  CATKIN_DEPENDS roscpp rospy sensor_msgs
#  DEPENDS system_lib
)
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(relay_pointcloud src/relay_pointcloud.cpp)
target_link_libraries(relay_pointcloud ${catkin_LIBRARIES})

