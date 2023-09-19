cmake_minimum_required(VERSION 3.0.2)
project(elas_test)


add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  image_geometry
  image_transport
  libelas
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  stereo_msgs
  tf
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES elas_test
#  CATKIN_DEPENDS cv_bridge geometry_msgs image_geometry image_transport libelas pcl_conversions pcl_ros roscpp sensor_msgs stereo_msgs tf
#  DEPENDS system_lib
)


find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS signals)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

include_directories(src ${libelas_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})


add_executable(elas_test src/main.cpp)
target_link_libraries(elas_test ${OpenCV_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES})
