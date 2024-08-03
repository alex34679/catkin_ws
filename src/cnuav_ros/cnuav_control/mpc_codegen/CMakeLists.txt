cmake_minimum_required(VERSION 3.16)
project(codegen)

# set(CMAKE_CXX_STANDARD 14)
set( CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR} )

find_package(ACADO REQUIRED )

include_directories(${ACADO_INCLUDE_DIRS})

add_executable(mpc_codegen mpc_codegen.cpp)

target_link_libraries(mpc_codegen ${ACADO_SHARED_LIBRARIES})

set_target_properties(mpc_codegen
        PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} )