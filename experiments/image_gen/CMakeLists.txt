cmake_minimum_required(VERSION 3.15)

project(image_gen)

add_executable(persp perspective_transform.cpp)
target_link_libraries(persp navigation)