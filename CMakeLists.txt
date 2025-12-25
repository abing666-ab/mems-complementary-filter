cmake_minimum_required(VERSION 3.10...3.18)

project(mems_complementary_filter)

add_library(mems_complementary_filter SHARED)

target_sources(mems_complementary_filter
PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include/mems_complementary_filter_interface.h
PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src/filter_implementary_helper.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/filter_implementary_helper.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/mems_complementary_filter.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/mems_complementary_filter.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/utility.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/utility.cpp
)
target_include_directories(mems_complementary_filter PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
set_property(TARGET mems_complementary_filter PROPERTY INTERFACE_INCLUDE_DIRECTORIES ${CMAKE_CURRENT_SOURCE_DIR}/include)