#include "hardware_recovery_node.hpp"

// ── Constructor ──────────────────────────────────────────────────────────────

HardwareRecoveryNode::HardwareRecoveryNode()
: Node("hardware_recovery_node")
{
  list_client_ = create_client<controller_manager_msgs::srv::ListControllers>(
    "/real/controller_manager/list_controllers");

  switch_client_ = create_client<controller_manager_msgs::srv::SwitchController>(
    "/real/controller_manager/switch_controller");

  pub_resync_ = create_publisher<std_msgs::msg::Bool>(
    "/hw_recovery/resync", 10);

  timer_ = create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&HardwareRecoveryNode::check_and_recover, this));

  RCLCPP_INFO(get_logger(), "Hardware recovery node ready");
}

// ── Check & recover ──────────────────────────────────────────────────────────

void HardwareRecoveryNode::check_and_recover()
{
  if (!list_client_->service_is_ready()) return;

  auto req = std::make_shared<                                    // แก้: เพิ่ม < ที่หายไป
    controller_manager_msgs::srv::ListControllers::Request>();

  auto future = list_client_->async_send_request(req);

  if (future.wait_for(std::chrono::milliseconds(500))
      != std::future_status::ready) return;

  for (auto & ctrl : future.get()->controller) {
    if (ctrl.name != "arm_controller") continue;

    if (ctrl.state != "active") {
      // controller หลุด — log ครั้งแรกครั้งเดียว แล้วพยายาม reactivate
      if (!controller_was_inactive_) {
        RCLCPP_WARN(get_logger(),
          "arm_controller state='%s' — cable disconnected? retrying...",
          ctrl.state.c_str());
        controller_was_inactive_ = true;
      }
      try_reactivate();

    } else if (controller_was_inactive_) {
      // กลับมา active แล้ว — reset state และ publish resync
      RCLCPP_INFO(get_logger(),
        "arm_controller restored after %d attempt(s) — sending resync",
        consecutive_failures_);
      controller_was_inactive_ = false;
      consecutive_failures_    = 0;       // แก้: reset ต้องอยู่ก่อน publish

      auto msg = std_msgs::msg::Bool();
      msg.data = true;
      pub_resync_->publish(msg);
    }
    break;
  }
}                                                                  // แก้: เพิ่ม } ที่หายไป

// ── Try reactivate ───────────────────────────────────────────────────────────

void HardwareRecoveryNode::try_reactivate()
{
  consecutive_failures_++;

  if (consecutive_failures_ > MAX_FAILURES) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
      "Cannot recover after %d attempts — check cable physically",
      consecutive_failures_);
    return;
  }

  if (!switch_client_->service_is_ready()) return;

  auto req = std::make_shared<                                     // แก้: เพิ่ม < ที่หายไป
    controller_manager_msgs::srv::SwitchController::Request>();
  req->activate_controllers   = {"arm_controller"};
  req->deactivate_controllers = {};
  req->strictness =
    controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT;

  auto future = switch_client_->async_send_request(req);

  if (future.wait_for(std::chrono::seconds(1))
      != std::future_status::ready) return;

  if (!future.get()->ok) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "Reactivation attempt %d failed — hardware may still be down",
      consecutive_failures_);
  }
}

// ── Main ─────────────────────────────────────────────────────────────────────

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HardwareRecoveryNode>());
  rclcpp::shutdown();
}