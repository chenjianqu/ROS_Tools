cmake_minimum_required(VERSION 3.0.2)
project(lidar_to_image)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
        cv_bridge
        image_transport
        roscpp
        rospy
        rosbag
        )

find_package(OpenCV 3.4 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(spdlog REQUIRED)
include_directories(${spdlog_INCLUDE_DIRS})

catkin_package(
        #  INCLUDE_DIRS include
        #  LIBRARIES multi_image_test
        #  CATKIN_DEPENDS cv_bridge image_transport roscpp rospy
        #  DEPENDS system_lib
)
include_directories( ${catkin_INCLUDE_DIRS})
include_directories( src)


find_package(PCL 1.8 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(calib_manager)


set(THIRY_LIBS fmt
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        ${spdlog_LIBRARIES}
        -lglog
        -lgflags
        ${PCL_LIBRARIES}
        calib_manager
        )

set(${PROJECT_NAME}_lib
        src/io/bbox.cpp
        src/io/file_utils.cpp
        src/bbox_remover.cpp
        src/utils.cpp
        src/cloud_utils.cpp
        src/pcl_def.h
        )

add_executable(lidar_to_image_from_bag src/exe/lidar_to_image_from_bag.cpp ${${PROJECT_NAME}_lib} )
target_link_libraries(lidar_to_image_from_bag ${THIRY_LIBS})

add_executable(print_timestamp_from_bag src/exe/print_timestamp_from_bag.cpp ${${PROJECT_NAME}_lib} )
target_link_libraries(print_timestamp_from_bag ${THIRY_LIBS})

