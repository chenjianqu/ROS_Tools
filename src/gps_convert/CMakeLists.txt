cmake_minimum_required(VERSION 3.0.2)
project(gps_convert)

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  sensor_msgs
		rosbag
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
	src/
)

find_package( OpenCV 3.4.16 REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )


add_library(GeographicLib SHARED
        src/GeographicLib/Constants.hpp
        src/GeographicLib/Geocentric.hpp
        src/GeographicLib/Geocentric.cpp
        src/GeographicLib/LocalCartesian.hpp
        src/GeographicLib/LocalCartesian.cpp
         )

set(THIRTY_LIBS
		${catkin_LIBRARIES}
		${OpenCV_LIBS}
		glog
)


add_executable(gps_pose src/gps_pose.cpp )
target_link_libraries(gps_pose ${THIRTY_LIBS} GeographicLib)

add_executable(convert_kitti_gps src/convert_kitti_gps.cpp )
target_link_libraries(convert_kitti_gps ${THIRTY_LIBS} GeographicLib)

add_executable(gps_record src/gps_record.cpp )
target_link_libraries(gps_record ${THIRTY_LIBS} GeographicLib)

add_executable(write_gps_from_bag src/write_gps_from_bag.cpp )
target_link_libraries(write_gps_from_bag ${THIRTY_LIBS} GeographicLib)



