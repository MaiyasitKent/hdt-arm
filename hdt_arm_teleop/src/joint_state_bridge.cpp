/**
 * joint_state_bridge.cpp
 * ======================
 * Bridge node สำหรับให้ Gazebo simulation follow ตาม real hardware
 *
 * Subscribe:  /real/joint_states  (sensor_msgs/msg/JointState)
 * Publish:    /sim/arm_controller/joint_trajectory
 *             (trajectory_msgs/msg/JointTrajectory)
 *
 * วิธีใช้:
 *   ros2 run hdt_arm_bringup joint_state_bridge
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <string>
#include <vector>

class JointStateBridge : public rclcpp::Node
{
public:
  JointStateBridge() : Node("joint_state_bridge")
  {
    this->declare_parameter("trajectory_duration", 0.1);
    this->declare_parameter("joint_names", std::vector<std::string>{
      "shoulder_pitch_joint",
      "shoulder_roll_joint",
      "upperarm_joint",
      "elbow_joint",
      "wrist_joint"
    });

    traj_duration_ = this->get_parameter("trajectory_duration").as_double();
    joint_names_   = this->get_parameter("joint_names").as_string_array();

    sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/real/joint_states",
      rclcpp::SensorDataQoS(),
      std::bind(&JointStateBridge::joint_state_cb, this, std::placeholders::_1)
    );

    pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/sim/arm_controller/joint_trajectory", 10
    );

    RCLCPP_INFO(this->get_logger(), "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
    RCLCPP_INFO(this->get_logger(), "  joint_state_bridge started");
    RCLCPP_INFO(this->get_logger(), "  SUB  /real/joint_states");
    RCLCPP_INFO(this->get_logger(), "  PUB  /sim/arm_controller/joint_trajectory");
    RCLCPP_INFO(this->get_logger(), "  traj duration: %.3f s", traj_duration_);
    RCLCPP_INFO(this->get_logger(), "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
  }

private:
  void joint_state_cb(const sensor_msgs::msg::JointState::ConstSharedPtr msg)
  {
    auto traj = trajectory_msgs::msg::JointTrajectory();
    traj.header.stamp    = this->now();
    traj.header.frame_id = "base_link";
    traj.joint_names     = joint_names_;

    trajectory_msgs::msg::JointTrajectoryPoint point;

    for (const auto & name : joint_names_) {
      double pos = 0.0;
      double vel = 0.0;
      for (size_t i = 0; i < msg->name.size(); i++) {
        if (msg->name[i] == name) {
          if (!msg->position.empty() && i < msg->position.size()) pos = msg->position[i];
          if (!msg->velocity.empty() && i < msg->velocity.size()) vel = msg->velocity[i];
          break;
        }
      }
      point.positions.push_back(pos);
      point.velocities.push_back(vel);
    }

    point.time_from_start = rclcpp::Duration::from_seconds(traj_duration_);
    traj.points.push_back(point);
    pub_->publish(traj);
  }

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_;
  double traj_duration_;
  std::vector<std::string> joint_names_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JointStateBridge>());
  rclcpp::shutdown();
  return 0;
}
