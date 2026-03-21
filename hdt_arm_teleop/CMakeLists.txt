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
find_package(geometry_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)

# home_position
add_executable(home_position src/home_position.cpp)
ament_target_dependencies(home_position rclcpp trajectory_msgs)

# imu_kinematics_node
add_executable(imu_kinematics_node src/imu_kinematics_node.cpp)
ament_target_dependencies(imu_kinematics_node
  rclcpp rclcpp_action sensor_msgs trajectory_msgs
  geometry_msgs control_msgs std_srvs std_msgs tf2)

# joint_state_bridge
add_executable(joint_state_bridge src/joint_state_bridge.cpp)
ament_target_dependencies(joint_state_bridge
  rclcpp sensor_msgs trajectory_msgs)

install(TARGETS
  home_position
  imu_kinematics_node
  joint_state_bridge
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()