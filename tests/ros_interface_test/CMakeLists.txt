cmake_minimum_required(VERSION 2.8.5)
project(mems_complementary_filter_ros)

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()

find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    tf
    tf_conversions
)

find_package(Eigen3 3.1.0 REQUIRED)

set(LIB_SRC ${PROJECT_SOURCE_DIR}/../../src)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${PROJECT_SOURCE_DIR}/../../include
    ${PROJECT_SOURCE_DIR}/../../src
    ${PROJECT_SOURCE_DIR}/include
)

add_library(mems_complementary_filter 
    ${LIB_SRC}/filter_implementary_helper.cpp
    ${LIB_SRC}/mems_complementary_filter.cpp
    ${LIB_SRC}/utility.cpp
)

add_executable(${PROJECT_NAME}
    ${PROJECT_SOURCE_DIR}/src/main.cpp
    ${PROJECT_SOURCE_DIR}/src/mems_complementary_filter_ros.cpp
)
target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    mems_complementary_filter
)