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
# include
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


add_executable(gps_pose src/gps_pose.cpp )
target_link_libraries(gps_pose ${catkin_LIBRARIES} GeographicLib ${OpenCV_LIBS})

add_executable(gps_record src/gps_record.cpp )
target_link_libraries(gps_record ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(write_gps_from_bag src/write_gps_from_bag.cpp )
target_link_libraries(write_gps_from_bag ${catkin_LIBRARIES} ${OpenCV_LIBS} GeographicLib)



