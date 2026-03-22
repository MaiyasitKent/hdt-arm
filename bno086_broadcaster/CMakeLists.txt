cmake_minimum_required(VERSION 3.8)
project(bno086_broadcaster)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(bno086_udp_publisher src/bno086_udp_publisher.cpp)
ament_target_dependencies(bno086_udp_publisher rclcpp sensor_msgs)

add_executable(bno086_serial_publisher src/bno086_serial_publisher.cpp)
ament_target_dependencies(bno086_serial_publisher rclcpp sensor_msgs)

install(TARGETS
  bno086_udp_publisher
  bno086_serial_publisher
  DESTINATION lib/${PROJECT_NAME})

ament_package()