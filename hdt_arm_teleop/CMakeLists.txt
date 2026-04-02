cmake_minimum_required(VERSION 3.8)
project(hdt_arm_teleop)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(controller_manager_msgs REQUIRED)   # เพิ่ม: สำหรับ hardware_recovery_node

# ── home_position ─────────────────────────────────────────────────────────────
add_executable(home_position src/home_position.cpp)
ament_target_dependencies(home_position
  rclcpp
  trajectory_msgs
)

# ── joint_state_bridge ────────────────────────────────────────────────────────
add_executable(joint_state_bridge src/joint_state_bridge.cpp)
ament_target_dependencies(joint_state_bridge
  rclcpp
  sensor_msgs
  trajectory_msgs
)

# ── imu_kinematics_node ───────────────────────────────────────────────────────
add_executable(imu_kinematics_node
  src/main.cpp
  src/imu_kinematics_node.cpp
)
target_include_directories(imu_kinematics_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(imu_kinematics_node PUBLIC cxx_std_17)
ament_target_dependencies(imu_kinematics_node
  rclcpp
  rclcpp_action
  sensor_msgs
  control_msgs
  trajectory_msgs
  std_srvs
  std_msgs
  tf2
)

# ── hardware_recovery_node ────────────────────────────────────────────────────
add_executable(hardware_recovery_node
  src/hardware_recovery_node.cpp
)
target_include_directories(hardware_recovery_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(hardware_recovery_node PUBLIC cxx_std_17)
ament_target_dependencies(hardware_recovery_node
  rclcpp
  std_msgs
  controller_manager_msgs
)

# ── joint_accuracy_monitor_node ───────────────────────────────────────────────
add_executable(joint_accuracy_monitor_node
  src/joint_accuracy_monitor_node.cpp
)
ament_target_dependencies(joint_accuracy_monitor_node
  rclcpp
  sensor_msgs
  std_msgs
  tf2
)

# ── install ───────────────────────────────────────────────────────────────────
install(DIRECTORY include/
  DESTINATION include
)
install(TARGETS
  home_position
  imu_kinematics_node
  joint_state_bridge
  joint_accuracy_monitor_node
  hardware_recovery_node          # เพิ่ม
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()