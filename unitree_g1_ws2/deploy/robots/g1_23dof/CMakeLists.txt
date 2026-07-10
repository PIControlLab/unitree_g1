cmake_minimum_required(VERSION 3.12)
project(g1_23dof_controller)

find_package(Boost REQUIRED COMPONENTS program_options)
find_package(yaml-cpp REQUIRED)
set(EXPORT_COMPILE_COMMANDS ON)
include_directories(
  /usr/include/eigen3
  /opt/unitree_robotics/include/ddscxx
  /opt/unitree_robotics/include
  include
  ${PROJECT_SOURCE_DIR}/../../thirdparty/onnxruntime-linux-x64-1.22.0/include
  ${PROJECT_SOURCE_DIR}/../../include/
)

link_directories(
  /opt/unitree_robotics/lib
)

link_libraries(
  unitree_sdk2 ddsc ddscxx rt pthread
  libboost_program_options.a libyaml-cpp.a fmt
  ${PROJECT_SOURCE_DIR}/../../thirdparty/onnxruntime-linux-x64-1.22.0/lib/libonnxruntime.so.1.22.0
)

file(GLOB_RECURSE ADD_SRC_LIST
  ${PROJECT_SOURCE_DIR}/src/*.cpp
)

add_library(${PROJECT_NAME}_lib ${ADD_SRC_LIST})
link_libraries(${PROJECT_NAME}_lib)

add_executable(g1_ctrl main.cpp)