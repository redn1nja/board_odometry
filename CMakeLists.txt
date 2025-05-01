cmake_minimum_required(VERSION 3.15)
project(odometry)

set(LOCAL_BUILD ON)
set(CMAKE_CXX_STANDARD 20)



if(LOCAL_BUILD)
    add_subdirectory(local)
endif()

find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/odometry/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/serial/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/utils)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug CACHE STRING "Build type" FORCE)
endif()


set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g")

add_subdirectory(odometry)
add_subdirectory(serial)
add_subdirectory(app)
add_subdirectory(experiments)