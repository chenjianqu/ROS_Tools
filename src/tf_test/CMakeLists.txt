cmake_minimum_required(VERSION 3.0.2)
project(tf_test)

add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        tf
        tf_conversions
)

catkin_package(
)


set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3)
include_directories( ${EIGEN3_INCLUDE_DIR})

find_package(OpenCV 3 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
message("OpenCV DIR:  ${OpenCV_DIR}")
message("OpenCV_INCLUDE_DIRS:  ${OpenCV_INCLUDE_DIRS}")
message("OpenCV OpenCV_VERSION:  ${OpenCV_VERSION}")


include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)


set(Libs
        ${OpenCV_LIBS}
        )


add_executable(tf_test src/tf_test.cpp)
target_link_libraries(tf_test
   ${catkin_LIBRARIES})

add_executable(write_tf_once src/write_tf_once.cpp)
target_link_libraries(write_tf_once
        ${catkin_LIBRARIES}
        ${Libs})

add_executable(write_tf_stamp src/write_tf_stamp.cpp )
target_link_libraries(write_tf_stamp
        ${catkin_LIBRARIES}
        ${Libs})

add_executable(broadcast_tf src/broadcast_tf.cpp)
target_link_libraries(broadcast_tf
        ${catkin_LIBRARIES}
        ${Libs})

add_executable(cvt_tf src/cvt_tf.cpp)
target_link_libraries(cvt_tf
        ${catkin_LIBRARIES}
        ${Libs})