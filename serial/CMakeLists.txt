cmake_minimum_required(VERSION 3.15)

project(serial_connection)

set(CMAKE_CXX_STANDARD 20)

find_package(Threads REQUIRED)
find_package(Boost REQUIRED COMPONENTS system regex)

add_library(serial SHARED
    serial.cpp)
target_include_directories(serial PRIVATE ${Boost_INCLUDE_DIRS})
include_directories(serial PRIVATE include)
target_link_libraries(serial Boost::system Boost::regex)
target_link_libraries(serial Threads::Threads)

