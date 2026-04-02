#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>

class HardwareRecoveryNode : public rclcpp::Node
{
public:
  HardwareRecoveryNode();

private:
  void check_and_recover();
  void try_reactivate();

  rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedPtr  list_client_;
  rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr switch_client_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr                          pub_resync_;
  rclcpp::TimerBase::SharedPtr                                               timer_;

  bool controller_was_inactive_ = false;
  int  consecutive_failures_    = 0;
  static constexpr int MAX_FAILURES = 10;
};