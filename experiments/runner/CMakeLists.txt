cmake_minimum_required(VERSION 3.15)
project(rosbag_to_dataset)

set(CMAKE_CXX_STANDARD 17)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(runner_node runner.cpp)

target_include_directories(runner_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_include_directories(runner_node PRIVATE ${OpenCV_INCLUDE_DIRS})

target_link_libraries(runner_node
    ${rclcpp_LIBRARIES}
    ${OpenCV_LIBS}
)

target_link_libraries(runner_node madgwick)
target_link_libraries(runner_node navigation)

target_compile_definitions(runner_node PUBLIC
  "ROS_BUILD"
)


ament_target_dependencies(runner_node
    rclcpp
    rclcpp_components
    sensor_msgs
    nav_msgs
    tf2_geometry_msgs
)

ament_export_include_directories(include)
ament_export_dependencies(rclcpp rclcpp_components)
ament_export_libraries(runner_node)
ament_package()
