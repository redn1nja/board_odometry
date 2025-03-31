cmake_minimum_required(VERSION 3.15)
project(odometry)

set(LOCAL_BUILD ON)
set(CMAKE_CXX_STANDARD 20)

add_subdirectory(lib/video)
add_subdirectory(odometry)
add_subdirectory(serial)


if(LOCAL_BUILD)
    add_subdirectory(local)
endif()

find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib/video)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/odometry/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/serial/include)

add_executable(vision_main main_v.cpp)
add_executable(local_main main_l.cpp)

target_include_directories(vision_main PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(vision_main video)
target_link_libraries(vision_main navigation)
target_link_libraries(vision_main ${OpenCV_LIBS})
target_link_libraries(vision_main serial)
target_link_libraries(vision_main Threads::Threads)

target_include_directories(local_main PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(local_main ${OpenCV_LIBS})
target_link_libraries(local_main navigation)
target_link_libraries(local_main serial)
target_link_libraries(local_main Threads::Threads)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug CACHE STRING "Build type" FORCE)
endif()


set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g")

