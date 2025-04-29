cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(GazeboDrone)

set(AIRSIM_ROOT ..)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 11)
endif()

find_package(gazebo REQUIRED)

include_directories(
    ${GAZEBO_INCLUDE_DIRS}
    ${AIRSIM_ROOT}/AirLib/deps/eigen3
    ${AIRSIM_ROOT}/AirLib/deps/rpclib/include
    ${AIRSIM_ROOT}/AirLib/include
)

link_directories(
    ${GAZEBO_LIBRARY_DIRS}
    ../../build_release/output/lib
)

# GCC에서는 stdlib 옵션을 제거하고 stdc++fs 추가
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_executable(${PROJECT_NAME} src/main.cpp)

target_link_libraries(
    ${PROJECT_NAME} 
    ${GAZEBO_LIBRARIES}
    pthread
    AirLib
    rpc
    stdc++fs
)

