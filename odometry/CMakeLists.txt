cmake_minimum_required(VERSION 3.15)
project(odometry_library)

set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV REQUIRED)

add_library(navigation SHARED
    src/odometry.cpp src/correction.cpp src/processor.cpp)

target_include_directories(navigation PRIVATE ${OpenCV_INCLUDE_DIRS})
include_directories(navigation PRIVATE include)
target_compile_definitions(navigation PRIVATE USE_MA)

target_link_libraries(navigation ${OpenCV_LIBS})


