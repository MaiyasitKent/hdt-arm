#include "imu_kinematics_node.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuKinematicsNode>());
  rclcpp::shutdown();
  return 0;
}
