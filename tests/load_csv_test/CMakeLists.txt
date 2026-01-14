cmake_minimum_required(VERSION 2.8.5)
project(load_csv_test)

find_package(Eigen3 3.1.0 REQUIRED)

set(LIB_SRC ${PROJECT_SOURCE_DIR}/../../src)

include_directories(
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
)
target_link_libraries(${PROJECT_NAME}
    mems_complementary_filter
)