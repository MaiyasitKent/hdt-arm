/**
 * joint_accuracy_monitor_node.cpp
 * ================================
 * Monitor node สำหรับวัดความแม่นยำของ HDT Arm
 *
 * Subscribe:
 *   /real/joint_states              → ค่าจริงจากหุ่นยนต์
 *   /sim/joint_states               → ค่าจาก digital twin
 *   /imu_kinematics/joint_angles    → data[0-4]=raw, data[5-9]=filtered (human)
 *   /imu/upperarm                   → quaternion ดิบจาก IMU ต้นแขน
 *   /imu/forearm                    → quaternion ดิบจาก IMU ปลายแขน
 *
 * Publish (สำหรับ rqt_plot):
 *   /accuracy/real_vs_sim           → Float64MultiArray [5] error real-sim   (deg)
 *   /accuracy/human_vs_robot        → Float64MultiArray [5] error human-robot (deg)
 *   /accuracy/joint_positions       → Float64MultiArray [15]
 *                                      [0-4]  = real  (rad)
 *                                      [5-9]  = sim   (rad)
 *                                      [10-14]= human filtered (rad)
 *   /imu/upperarm/rpy               → Float64MultiArray [3] roll/pitch/yaw (deg)
 *   /imu/forearm/rpy                → Float64MultiArray [3] roll/pitch/yaw (deg)
 *
 * วิธีใช้:
 *   ros2 run hdt_arm_teleop joint_accuracy_monitor_node
 *
 * rqt_plot ตัวอย่าง:
 *   rqt_plot /accuracy/joint_positions/data[0]:data[5]:data[10]
 *   rqt_plot /accuracy/real_vs_sim/data[0]:data[1]:data[2]:data[3]:data[4]
 *   rqt_plot /imu/upperarm/rpy/data[0]:data[1]:data[2]
 *   rqt_plot /imu/forearm/rpy/data[0]:data[1]:data[2]
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <array>
#include <cmath>
#include <mutex>

static constexpr int NUM_JOINTS = 5;

static const std::array<std::string, NUM_JOINTS> JOINT_NAMES = {{
  "shoulder_pitch_joint",
  "shoulder_roll_joint",
  "upperarm_joint",
  "elbow_joint",
  "wrist_joint",
}};

// helper: quaternion → [roll, pitch, yaw] หน่วย degree
static std::array<double, 3> quat_to_rpy_deg(
  double x, double y, double z, double w)
{
  tf2::Quaternion q(x, y, z, w);
  q.normalize();
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  return {roll  * 180.0 / M_PI,
          pitch * 180.0 / M_PI,
          yaw   * 180.0 / M_PI};
}

class JointAccuracyMonitor : public rclcpp::Node
{
public:
  JointAccuracyMonitor() : Node("joint_accuracy_monitor")
  {
    // ── Subscribers ──────────────────────────────────────────────────────────
    sub_real_ = create_subscription<sensor_msgs::msg::JointState>(
      "/real/joint_states", rclcpp::SensorDataQoS(),
      std::bind(&JointAccuracyMonitor::real_cb, this, std::placeholders::_1));

    sub_sim_ = create_subscription<sensor_msgs::msg::JointState>(
      "/sim/joint_states", rclcpp::SensorDataQoS(),
      std::bind(&JointAccuracyMonitor::sim_cb, this, std::placeholders::_1));

    sub_human_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "/imu_kinematics/joint_angles", 10,
      std::bind(&JointAccuracyMonitor::human_cb, this, std::placeholders::_1));

    // ใช้ RELIABLE ให้ตรงกับ udp_imu_node publisher
    auto reliable_qos = rclcpp::QoS(10).reliable();

    sub_imu_upper_ = create_subscription<sensor_msgs::msg::Imu>(
      "/imu/upperarm", reliable_qos,
      std::bind(&JointAccuracyMonitor::imu_upper_cb, this, std::placeholders::_1));

    sub_imu_fore_ = create_subscription<sensor_msgs::msg::Imu>(
      "/imu/forearm", reliable_qos,
      std::bind(&JointAccuracyMonitor::imu_fore_cb, this, std::placeholders::_1));

    //     // subscribe joint_states เพื่อรู้ตำแหน่งจริงของ servo
    // sub_joint_states_ = create_subscription<sensor_msgs::msg::JointState>(
    //   "/real/joint_states", rclcpp::SensorDataQoS(),
    //   [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
    //     if (msg->position.size() < NUM_JOINTS) return;
    //     std::lock_guard<std::mutex> lock(sync_mutex_);
    //     for (int i = 0; i < NUM_JOINTS; i++)
    //       synced_pos_[i] = msg->position[i];
    //   });

    // // รับสัญญาณ resync จาก HardwareRecoveryNode
    // sub_resync_ = create_subscription<std_msgs::msg::Bool>(
    //   "/hw_recovery/resync", 10,
    //   [this](const std_msgs::msg::Bool::SharedPtr) {
    //     std::lock_guard<std::mutex> lock(sync_mutex_);
    //     pending_resync_ = true;
    //   });

    // ── Publishers ───────────────────────────────────────────────────────────
    pub_real_vs_sim_    = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/accuracy/real_vs_sim", 10);
    pub_human_vs_robot_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/accuracy/human_vs_robot", 10);
    pub_positions_      = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/accuracy/joint_positions", 10);
    pub_rpy_upper_      = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/imu/upperarm/rpy", 10);
    pub_rpy_fore_       = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/imu/forearm/rpy", 10);

    // ── Timer 50 Hz ──────────────────────────────────────────────────────────
    timer_ = create_wall_timer(
      std::chrono::milliseconds(20),
      std::bind(&JointAccuracyMonitor::timer_cb, this));

    real_pos_.fill(0.0);
    sim_pos_.fill(0.0);
    human_pos_.fill(0.0);
    rpy_upper_.fill(0.0);
    rpy_fore_.fill(0.0);

    RCLCPP_INFO(get_logger(), "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
    RCLCPP_INFO(get_logger(), "  joint_accuracy_monitor started  [50 Hz]");
    RCLCPP_INFO(get_logger(), "  SUB  /real/joint_states");
    RCLCPP_INFO(get_logger(), "  SUB  /sim/joint_states");
    RCLCPP_INFO(get_logger(), "  SUB  /imu_kinematics/joint_angles");
    RCLCPP_INFO(get_logger(), "  SUB  /imu/upperarm  /imu/forearm");
    RCLCPP_INFO(get_logger(), "  PUB  /accuracy/real_vs_sim        (deg)");
    RCLCPP_INFO(get_logger(), "  PUB  /accuracy/human_vs_robot     (deg)");
    RCLCPP_INFO(get_logger(), "  PUB  /accuracy/joint_positions    (rad)");
    RCLCPP_INFO(get_logger(), "  PUB  /imu/upperarm/rpy  [roll,pitch,yaw] (deg)");
    RCLCPP_INFO(get_logger(), "  PUB  /imu/forearm/rpy   [roll,pitch,yaw] (deg)");
    RCLCPP_INFO(get_logger(), "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
  }

private:
  // ── IMU callbacks ─────────────────────────────────────────────────────────

  void imu_upper_cb(const sensor_msgs::msg::Imu::ConstSharedPtr msg)
  {
    auto rpy = quat_to_rpy_deg(
      msg->orientation.x, msg->orientation.y,
      msg->orientation.z, msg->orientation.w);
    std::lock_guard<std::mutex> lock(mutex_);
    rpy_upper_     = rpy;
    has_imu_upper_ = true;
  }

  void imu_fore_cb(const sensor_msgs::msg::Imu::ConstSharedPtr msg)
  {
    auto rpy = quat_to_rpy_deg(
      msg->orientation.x, msg->orientation.y,
      msg->orientation.z, msg->orientation.w);
    std::lock_guard<std::mutex> lock(mutex_);
    rpy_fore_     = rpy;
    has_imu_fore_ = true;
  }

  // ── Joint state callbacks ─────────────────────────────────────────────────

  void real_cb(const sensor_msgs::msg::JointState::ConstSharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (int i = 0; i < NUM_JOINTS; i++) {
      for (size_t j = 0; j < msg->name.size(); j++) {
        if (msg->name[j] == JOINT_NAMES[i] && j < msg->position.size()) {
          real_pos_[i] = msg->position[j];
          break;
        }
      }
    }
    has_real_ = true;
  }

  void sim_cb(const sensor_msgs::msg::JointState::ConstSharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (int i = 0; i < NUM_JOINTS; i++) {
      for (size_t j = 0; j < msg->name.size(); j++) {
        if (msg->name[j] == JOINT_NAMES[i] && j < msg->position.size()) {
          sim_pos_[i] = msg->position[j];
          break;
        }
      }
    }
    has_sim_ = true;
  }

  void human_cb(const std_msgs::msg::Float64MultiArray::ConstSharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    // data[5-9] = filtered (deg) → แปลงเป็น rad
    if (msg->data.size() >= 10) {
      for (int i = 0; i < NUM_JOINTS; i++) {
        human_pos_[i] = msg->data[i + NUM_JOINTS] * M_PI / 180.0;
      }
      has_human_ = true;
    }
  }

  // ── Timer: publish ทุก topic ──────────────────────────────────────────────

  void timer_cb()
  {
    std::array<double, NUM_JOINTS> real, sim, human;
    std::array<double, 3> rpy_u, rpy_f;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      real  = real_pos_;
      sim   = sim_pos_;
      human = human_pos_;
      rpy_u = rpy_upper_;
      rpy_f = rpy_fore_;
    }

    // ── IMU RPY — publish เสมอ ไม่ขึ้นกับ joint data ─────────────────────
    if (has_imu_upper_) {
      std_msgs::msg::Float64MultiArray m;
      m.data = {rpy_u[0], rpy_u[1], rpy_u[2]};  // roll, pitch, yaw (deg)
      pub_rpy_upper_->publish(m);
    }
    if (has_imu_fore_) {
      std_msgs::msg::Float64MultiArray m;
      m.data = {rpy_f[0], rpy_f[1], rpy_f[2]};
      pub_rpy_fore_->publish(m);
    }

    // ── Joint accuracy — ต้องมีครบทั้ง 3 source ──────────────────────────
    if (!has_real_ || !has_sim_ || !has_human_) return;

    // 1. joint_positions [15]: [0-4]=real [5-9]=sim [10-14]=human (rad)
    std_msgs::msg::Float64MultiArray pos_msg;
    pos_msg.data.resize(NUM_JOINTS * 3);
    for (int i = 0; i < NUM_JOINTS; i++) {
      pos_msg.data[i]                  = real[i];
      pos_msg.data[i + NUM_JOINTS]     = sim[i];
      pos_msg.data[i + NUM_JOINTS * 2] = human[i];
    }
    pub_positions_->publish(pos_msg);

    // 2. real_vs_sim error (deg)
    std_msgs::msg::Float64MultiArray rvs_msg;
    rvs_msg.data.resize(NUM_JOINTS);
    for (int i = 0; i < NUM_JOINTS; i++) {
      rvs_msg.data[i] = (real[i] - sim[i]) * 180.0 / M_PI;
    }
    pub_real_vs_sim_->publish(rvs_msg);

    // 3. human_vs_robot error (deg)
    std_msgs::msg::Float64MultiArray hvr_msg;
    hvr_msg.data.resize(NUM_JOINTS);
    for (int i = 0; i < NUM_JOINTS; i++) {
      hvr_msg.data[i] = (human[i] - real[i]) * 180.0 / M_PI;
    }
    pub_human_vs_robot_->publish(hvr_msg);
  }

  // ── Members ───────────────────────────────────────────────────────────────
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr     sub_real_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr     sub_sim_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_human_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr            sub_imu_upper_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr            sub_imu_fore_;

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr    pub_real_vs_sim_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr    pub_human_vs_robot_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr    pub_positions_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr    pub_rpy_upper_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr    pub_rpy_fore_;

  rclcpp::TimerBase::SharedPtr timer_;
  std::mutex mutex_;

  std::array<double, NUM_JOINTS> real_pos_;
  std::array<double, NUM_JOINTS> sim_pos_;
  std::array<double, NUM_JOINTS> human_pos_;
  std::array<double, 3>          rpy_upper_;
  std::array<double, 3>          rpy_fore_;

  bool has_real_      = false;
  bool has_sim_       = false;
  bool has_human_     = false;
  bool has_imu_upper_ = false;
  bool has_imu_fore_  = false;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JointAccuracyMonitor>());
  rclcpp::shutdown();
  return 0;
}