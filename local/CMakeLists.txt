cmake_minimum_required(VERSION 3.15)

project(local)

set(CMAKE_CXX_STANDARD 20)

find_package(gz-transport12 REQUIRED)
find_package(OpenCV REQUIRED)


add_executable(topics read_gz.cpp)
target_include_directories(topics PRIVATE ${OpenCV_INCLUDE_DIRS})
target_include_directories(topics PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../utils)
target_link_libraries(topics ${OpenCV_LIBS})
target_link_libraries(topics gz-transport12::core)

set (CMAKE_INSTALL_PREFIX ${CMAKE_CURRENT_SOURCE_DIR})
install(TARGETS topics DESTINATION ./bin)