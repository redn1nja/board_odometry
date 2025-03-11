cmake_minimum_required(VERSION 3.15)
project(odometry)

set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV REQUIRED)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib/video)

add_executable(odometry main.cpp)
target_include_directories(odometry PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(odometry video)
target_link_libraries(odometry ${OpenCV_LIBS})

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug CACHE STRING "Build type" FORCE)
endif()


set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g")

add_subdirectory(lib/video)
