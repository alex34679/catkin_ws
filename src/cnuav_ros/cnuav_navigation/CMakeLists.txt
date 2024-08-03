cmake_minimum_required(VERSION 3.16)

project(cnuav_navigation C CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        quadrotor_common
        trajectory_generation_helper
        )

catkin_package()

include_directories(
        ${catkin_INCLUDE_DIRS}
        include
        )


add_executable(eight_traj_gen_node
        src/node/eight_traj_gen_node.cpp
        )

target_link_libraries(eight_traj_gen_node
        ${catkin_LIBRARIES}
        )

add_executable(bowknot_gen_node
        src/node/bowknot_gen_node.cpp)

target_link_libraries(bowknot_gen_node
        ${catkin_LIBRARIES}
        )

add_executable(two_obstacles_node
        src/node/two_obstalces_node.cpp)

target_link_libraries(two_obstacles_node
        ${catkin_LIBRARIES}
        )

add_executable(one_obstacle_node
        src/node/one_obstacle_node.cpp)

target_link_libraries(one_obstacle_node
        ${catkin_LIBRARIES}
        )

