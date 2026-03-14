#include <rclcpp/rclcpp.hpp>
#include <control_msgs/msg/joint_jog.hpp>
#include <std_srvs/srv/trigger.hpp>

// Joint indices for readability
enum JointIndex {
  SHOULDER_PITCH = 0,
  SHOULDER_ROLL  = 1,
  UPPERARM       = 2,
  ELBOW          = 3,
  WRIST          = 4,
  NUM_JOINTS
};

class JointJogController : public rclcpp::Node
{
public:
  JointJogController() : Node("test_joint_jog_controller")
  {
    publisher_ = this->create_publisher<control_msgs::msg::JointJog>(
      "/servo_node/delta_joint_cmds", 10);

    // Start servo service client
    start_servo_client_ = this->create_client<std_srvs::srv::Trigger>(
      "/servo_node/start_servo");

    // Wait for servo node
    while (!start_servo_client_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_INFO(this->get_logger(), "Waiting for /servo_node/start_servo...");
    }

    // Call start servo
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    start_servo_client_->async_send_request(request,
      [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
        auto result = future.get();
        if (result->success) {
          RCLCPP_INFO(this->get_logger(), "Servo started!");
          // Start publishing after servo is ready
          timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&JointJogController::publishCommand, this));
        } else {
          RCLCPP_ERROR(this->get_logger(), "Failed to start servo: %s", result->message.c_str());
        }
      });
  }

private:
  void publishCommand()
  {
    auto msg = control_msgs::msg::JointJog();
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = "base_link";

    msg.joint_names = {
      "shoulder_pitch_joint",
      "shoulder_roll_joint",
      "upperarm_joint",
      "elbow_joint",
      "wrist_joint"
    };

    // ตั้งค่าความเร็ว rad/s (เปลี่ยนตรงนี้ตามที่ต้องการ)
    msg.velocities.resize(NUM_JOINTS, 0.0);
    msg.velocities[SHOULDER_PITCH] = 0.1;   // หมุน shoulder_pitch
    msg.velocities[SHOULDER_ROLL]  = 0.0;
    msg.velocities[UPPERARM]       = 0.0;
    msg.velocities[ELBOW]          = 0.0;
    msg.velocities[WRIST]          = 0.0;

    publisher_->publish(msg);
  }

  rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr publisher_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr start_servo_client_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JointJogController>());
  rclcpp::shutdown();
  return 0;
}