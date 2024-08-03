cmake_minimum_required(VERSION 3.16)

project(cnuav_control C CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        geometry_msgs
        nav_msgs
        sensor_msgs
        quadrotor_msgs
        eigen_catkin
        )

find_package(
        yaml-cpp
)

catkin_package()

include_directories(
        ${catkin_INCLUDE_DIRS}
        ${YAML_CPP_INCLUDE_DIRS}
        include
        mpc_codegen/quadrotor_mpc_codegen
        externals/qpoases
        externals/qpoases/SRC
        externals/qpoases/INCLUDE
        )


################# MPC lib #################

add_library(cnuav_qpoases_lib
        externals/qpoases/SRC/Bounds.cpp
        externals/qpoases/SRC/Constraints.cpp
        externals/qpoases/SRC/CyclingManager.cpp
        externals/qpoases/SRC/Indexlist.cpp
        externals/qpoases/SRC/MessageHandling.cpp
        externals/qpoases/SRC/QProblem.cpp
        externals/qpoases/SRC/QProblemB.cpp
        externals/qpoases/SRC/SubjectTo.cpp
        externals/qpoases/SRC/Utils.cpp
        externals/qpoases/SRC/EXTRAS/SolutionAnalysis.cpp
        )

target_include_directories(cnuav_qpoases_lib PUBLIC
        externals/qpoases
        externals/qpoases/INCLUDE
        externals/qpoases/SRC
        )

add_library(cnuav_mpc_solver
        mpc_codegen/quadrotor_mpc_codegen/acado_auxiliary_functions.c
        mpc_codegen/quadrotor_mpc_codegen/acado_integrator.c
        mpc_codegen/quadrotor_mpc_codegen/acado_qpoases_interface.cpp
        mpc_codegen/quadrotor_mpc_codegen/acado_solver.c
        )

target_include_directories(cnuav_mpc_solver PUBLIC
        mpc_codegen/quadrotor_mpc_codegen
        )

################# Simulation node #################

add_library(cnuav_cbf_clf_sim_lib
        src/autopilot.cpp
        src/roswrapper.cpp
        src/mpc/mpc.cpp
        src/velfilter.cpp
        )

target_link_libraries(cnuav_cbf_clf_sim_lib
        cnuav_mpc_solver
        cnuav_qpoases_lib
        )

target_compile_definitions(cnuav_cbf_clf_sim_lib PRIVATE
        SIMULATION
        )

add_executable(cnuav_cbf_clf_sim_node
        src/node/cnuav_mpc_node.cpp
        )

add_executable(cnuav_one
        src/node/uav1.cpp
        )     

add_executable(cnuav_two
        src/node/uav2.cpp
        )   

add_executable(cnuav_three
        src/node/uav3.cpp
        )     

target_link_libraries(cnuav_one
        cnuav_cbf_clf_sim_lib
        ${catkin_LIBRARIES}
        ${YAML_CPP_LIBRARIES}
        )

target_link_libraries(cnuav_two
        cnuav_cbf_clf_sim_lib
        ${catkin_LIBRARIES}
        ${YAML_CPP_LIBRARIES}
        )

target_link_libraries(cnuav_three
        cnuav_cbf_clf_sim_lib
        ${catkin_LIBRARIES}
        ${YAML_CPP_LIBRARIES}
        )

target_link_libraries(cnuav_cbf_clf_sim_node
        cnuav_cbf_clf_sim_lib
        ${catkin_LIBRARIES}
        ${YAML_CPP_LIBRARIES}
        )

################# Experiment node #################

add_library(cnuav_cbf_clf_exp_lib
        src/autopilot.cpp
        src/roswrapper.cpp
        src/mpc/mpc.cpp
        src/velfilter.cpp
        )

target_link_libraries(cnuav_cbf_clf_exp_lib
        cnuav_mpc_solver
        cnuav_qpoases_lib
        )

target_compile_definitions(cnuav_cbf_clf_exp_lib PRIVATE
        EXPERIMENT
        )

add_executable(cnuav_cbf_clf_exp_node
        src/node/cnuav_mpc_node.cpp
        )

add_executable(cnuav_exp_one
        src/node/uav1.cpp
        )

add_executable(cnuav_exp_two
        src/node/uav2.cpp
        )   

add_executable(cnuav_exp_three
        src/node/uav3.cpp
        )

add_executable(cnuav_exp_four
        src/node/uav4.cpp
        )

add_executable(cnuav_exp_five
src/node/uav5.cpp
)


target_link_libraries(cnuav_cbf_clf_exp_node
        cnuav_cbf_clf_exp_lib
        ${catkin_LIBRARIES}
        ${YAML_CPP_LIBRARIES}
        )

target_link_libraries(cnuav_exp_one
        cnuav_cbf_clf_exp_lib
        ${catkin_LIBRARIES}
        ${YAML_CPP_LIBRARIES}
        )

target_link_libraries(cnuav_exp_two
        cnuav_cbf_clf_exp_lib
        ${catkin_LIBRARIES}
        ${YAML_CPP_LIBRARIES}
        )

target_link_libraries(cnuav_exp_three
        cnuav_cbf_clf_exp_lib
        ${catkin_LIBRARIES}
        ${YAML_CPP_LIBRARIES}
        )

target_link_libraries(cnuav_exp_four
        cnuav_cbf_clf_exp_lib
        ${catkin_LIBRARIES}
        ${YAML_CPP_LIBRARIES}
        )


target_link_libraries(cnuav_exp_five
        cnuav_cbf_clf_exp_lib
        ${catkin_LIBRARIES}
        ${YAML_CPP_LIBRARIES}
        )