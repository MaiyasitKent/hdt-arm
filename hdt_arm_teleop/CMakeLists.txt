cmake_minimum_required(VERSION 3.8)
project(hdt_arm_teleop)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(moveit_servo REQUIRED)

# ── imu_kinematics_node ───────────────────────────────────────────────────────
add_executable(imu_kinematics_node src/imu_kinematics_node.cpp)
ament_target_dependencies(imu_kinematics_node
  rclcpp
  rclcpp_action
  sensor_msgs
  control_msgs
  trajectory_msgs
  message_filters
  tf2
  std_msgs
  std_srvs
)

# ── test_joint_jog_node ───────────────────────────────────────────────────────
add_executable(test_joint_jog_node src/test_joint_jog_controller.cpp)
ament_target_dependencies(test_joint_jog_node
  rclcpp
  control_msgs
  std_msgs
  std_srvs
  moveit_servo
  sensor_msgs
  message_filters
  tf2
)

install(TARGETS
  imu_kinematics_node
  test_joint_jog_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()