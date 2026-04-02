/**
 * home_position.cpp
 * =================
 * Node สำหรับส่งแขนกลไปที่ท่าเริ่มต้น (home position)
 * อ่านค่า joint positions จาก parameter (initial_positions.yaml)
 *
 * วิธีใช้:
 *   รันโดยตรง:
 *     ros2 run hdt_arm_teleop home_position
 *   หรือผ่าน launch file (hdt_arm_real.launch.py) จะเรียกให้อัตโนมัติ
 */

#include <chrono>
#include <memory>
#include <vector>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

using namespace std::chrono_literals;

class HomePositionNode : public rclcpp::Node
{
public:
  HomePositionNode()
  : Node("home_position_node")
  {
    // ชื่อ joint (ต้องตรงกับ ros2_controllers.yaml)
    joint_names_ = {
      "shoulder_pitch_joint",
      "shoulder_roll_joint",
      "upperarm_joint",
      "elbow_joint",
      "wrist_joint"
    };

    // อ่านค่า initial positions จาก parameter
    // ถ้าไม่มีก็ใช้ค่า default = 0.0
    for (const auto & name : joint_names_) {
      this->declare_parameter("initial_positions." + name, 0.0);
    }

    // Publisher ส่ง trajectory ไปที่ arm_controller
    publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/real/arm_controller/follow_joint_trajectory", 10);

    // รอให้ controller พร้อมก่อนส่ง (2 วินาที)
    timer_ = this->create_wall_timer(
      2000ms,
      std::bind(&HomePositionNode::send_home_trajectory, this));
  }

private:
  void send_home_trajectory()
  {
    // ยกเลิก timer ไม่ให้ส่งซ้ำ
    timer_->cancel();

    // สร้าง trajectory message
    auto msg = trajectory_msgs::msg::JointTrajectory();
    msg.joint_names = joint_names_;

    // จุดปลายทาง (home position)
    trajectory_msgs::msg::JointTrajectoryPoint point;
    for (const auto & name : joint_names_) {
      double pos = this->get_parameter("initial_positions." + name).as_double();
      point.positions.push_back(pos);
      point.velocities.push_back(0.0);
    }

    // ใช้เวลา 3 วินาทีในการเคลื่อนที่
    point.time_from_start = rclcpp::Duration::from_seconds(3.0);
    msg.points.push_back(point);

    RCLCPP_INFO(this->get_logger(), "Execut Home...");
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      RCLCPP_INFO(this->get_logger(), "  %s: %.4f rad",
        joint_names_[i].c_str(), point.positions[i]);
    }

    publisher_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Sent Home trajectory");

    // หน่วงแล้วปิด node
    rclcpp::sleep_for(4s);
    rclcpp::shutdown();
  }

  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<std::string> joint_names_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HomePositionNode>());
  rclcpp::shutdown();
  return 0;
}