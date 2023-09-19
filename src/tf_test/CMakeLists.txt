cmake_minimum_required(VERSION 3.0.2)
project(tf_test)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
        tf_conversions
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES tf_test
#  CATKIN_DEPENDS roscpp rospy std_msgs tf
#  DEPENDS system_lib
)




find_package(OpenCV REQUIRED)
find_package(Eigen3 3 REQUIRED)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_LIBS}
        ${OpenCV_INCLUDE_DIRS}
        "/usr/local/include/eigen3/"
)


set(Libs
        ${OpenCV_LIBS}
        )


add_executable(tf_test src/tf_test.cpp)
target_link_libraries(tf_test
   ${catkin_LIBRARIES})

add_executable(write_tf src/write_tf.cpp)
target_link_libraries(write_tf
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