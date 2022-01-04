cmake_minimum_required(VERSION 3.0.2)
project(rosbag_test)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
        cv_bridge
        image_transport
        roscpp
        rospy
        rosbag
        )

find_package(OpenCV 3.4 REQUIRED)



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


add_executable(rosbag_test src/main.cpp)
target_link_libraries(rosbag_test ${catkin_LIBRARIES} ${OpenCV_LIBS})


add_executable(get_image_from_bag src/get_image_from_bag.cpp)
target_link_libraries(get_image_from_bag ${catkin_LIBRARIES} ${OpenCV_LIBS})


