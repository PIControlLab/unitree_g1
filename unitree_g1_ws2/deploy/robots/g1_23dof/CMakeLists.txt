cmake_minimum_required(VERSION 3.12)
project(g1_23dof_controller)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Boost REQUIRED COMPONENTS program_options)
find_package(yaml-cpp REQUIRED)
find_package(ZLIB REQUIRED)
set(EXPORT_COMPILE_COMMANDS ON)
include_directories(
  /usr/include/eigen3
  /opt/unitree_robotics/include/ddscxx
  /opt/unitree_robotics/include
  include
  ${PROJECT_SOURCE_DIR}/../../thirdparty/onnxruntime-linux-x64-1.22.0/include
  ${PROJECT_SOURCE_DIR}/../../include/
  ${PROJECT_SOURCE_DIR}/../../thirdparty/cnpy
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

add_library(${PROJECT_NAME}_lib
  ${ADD_SRC_LIST}
  ${PROJECT_SOURCE_DIR}/../../thirdparty/cnpy/cnpy.cpp
)
target_link_libraries(${PROJECT_NAME}_lib PUBLIC ZLIB::ZLIB)
link_libraries(${PROJECT_NAME}_lib)

add_executable(g1_ctrl main.cpp)
