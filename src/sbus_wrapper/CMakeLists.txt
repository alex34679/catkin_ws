cmake_minimum_required(VERSION 3.10)

project(sbus_wrapper)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

include_directories(
        include
        )

cs_add_library(sbus_wrapper
        src/sbus_wrapper.cpp
        )

cs_add_executable(sbus_wrapper_node
        src/sbus_wrapper_node.cpp
        )

target_link_libraries(sbus_wrapper_node
        sbus_wrapper
        )