cmake_minimum_required(VERSION 3.0.2)
project(rosbag_test)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
        cv_bridge
        image_transport
        roscpp
        rospy
        rosbag
        )

find_package(OpenCV 3.4 REQUIRED)

find_package(spdlog REQUIRED)
include_directories(${spdlog_INCLUDE_DIRS})
message("spdlog version:${spdlog_VERSION}")
message("spdlog dir:${spdlog_DIR}")


catkin_package(
        #  INCLUDE_DIRS include
        #  LIBRARIES multi_image_test
        #  CATKIN_DEPENDS cv_bridge image_transport roscpp rospy
        #  DEPENDS system_lib
)


include_directories(
        # include
        ${catkin_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
)


set(THIRY_LIBS fmt
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        ${spdlog_LIBRARIES}
        -lglog
        -lgflags
        )

add_executable(rosbag_test src/main.cpp)
target_link_libraries(rosbag_test ${THIRY_LIBS})


add_executable(get_image_from_bag src/get_image_from_bag.cpp)
target_link_libraries(get_image_from_bag ${THIRY_LIBS})


add_executable(write_image src/write_image.cpp)
target_link_libraries(write_image ${THIRY_LIBS})

add_executable(write_rgb_mask src/write_rgb_mask.cpp)
target_link_libraries(write_rgb_mask ${THIRY_LIBS})

add_executable(get_imu_from_bag src/get_imu_from_bag.cpp)
target_link_libraries(get_imu_from_bag ${THIRY_LIBS})

add_executable(get_imu_full_from_bag src/get_imu_full_from_bag.cpp)
target_link_libraries(get_imu_full_from_bag ${THIRY_LIBS})

add_executable(colmap_mask_preprocess src/colmap_mask_preprocess.cpp)
target_link_libraries(colmap_mask_preprocess ${THIRY_LIBS})


add_executable(get_odom_from_bag src/get_odom_from_bag.cpp)
target_link_libraries(get_odom_from_bag ${THIRY_LIBS})

add_executable(get_image_time_stamp src/get_image_time_stamp.cpp)
target_link_libraries(get_image_time_stamp ${THIRY_LIBS})


