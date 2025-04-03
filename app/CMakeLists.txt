cmake_minimum_required(VERSION 3.15)

project(app)

set(CMAKE_CXX_STANDARD 17)

add_library(app SHARED app.cpp)
target_include_directories(app PRIVATE include)

target_link_libraries(app navigation)

target_link_libraries(app serial)