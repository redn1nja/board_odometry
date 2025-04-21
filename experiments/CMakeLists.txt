cmake_minimum_required(VERSION 3.15)

project(tests)

set(CMAKE_CXX_STANDARD 17)

add_executable(imu_tests test_madgwick.cpp)

set(DIR ${CMAKE_CURRENT_SOURCE_DIR}/../external/madgwick/Core/Src)

add_library(madgwick ${DIR}/madgwick.c ${DIR}/quat.c)
target_include_directories(madgwick PUBLIC ../external/madgwick/Core/Inc/imu_filter)
target_include_directories(madgwick PUBLIC ../utils)
target_compile_definitions(madgwick PUBLIC LOCAL_BUILD)

target_link_libraries(imu_tests madgwick)
target_link_libraries(imu_tests navigation)


add_subdirectory(runner)