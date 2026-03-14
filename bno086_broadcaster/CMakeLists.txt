cmake_minimum_required(VERSION 3.8)
project(bno086_broadcaster)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2 REQUIRED)

# ── bno086_publisher ──────────────────────────────────────────────────────────
add_executable(bno086_publisher src/bno086_publisher.cpp)
ament_target_dependencies(bno086_publisher rclcpp sensor_msgs)

# ── imu_csv_publisher ─────────────────────────────────────────────────────────
add_executable(imu_csv_publisher src/imu_csv_publisher.cpp)
ament_target_dependencies(imu_csv_publisher rclcpp sensor_msgs)

install(DIRECTORY data/
  DESTINATION share/${PROJECT_NAME}/data)

install(TARGETS
  bno086_publisher
  imu_csv_publisher
  DESTINATION lib/${PROJECT_NAME})

ament_package()
