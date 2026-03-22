/**
 * udp_imu_broadcaster.cpp
 *
 * รับ UDP packet จาก ESP32 firmware และ publish เป็น ROS2 IMU topic
 *
 * ====================================================
 * Changes from original
 * ====================================================
 * [B1] Binary packet format (16 bytes) แทน ASCII (64+ bytes)
 *      struct { float qw, qx, qy, qz }
 *      → ลด parse overhead, ลด WiFi payload
 *
 * [B2] ลบ LPF ออกจาก broadcaster
 *      เดิม: broadcaster กรอง (alpha=0.3) + node กรอง (alpha=0.2) = delay สองชั้น
 *      ใหม่: pass-through quaternion ตรงๆ ให้ node จัดการเอง
 *
 * [B3] ลบ Zero Motion Detection ออก
 *      gyro data ไม่ได้ส่งมาแล้วใน firmware ใหม่
 *      zero motion ใน broadcaster ไม่ช่วยลด jitter จริง
 *
 * [B4] timer 10ms → 1ms (1000Hz polling)
 *      ทำให้ packet ที่มาถึงถูก forward ออกเร็วขึ้น
 *      UDP buffer drain loop ยังคงอยู่ → drain ทุก packet ใน buffer
 *
 * [B5] ลบ angular_velocity และ linear_acceleration ออกจาก message
 *      firmware ใหม่ไม่ส่งค่าเหล่านี้แล้ว set เป็น 0 เพื่อความสะอาด
 *
 * UDP Packet Format ที่รับ (16 bytes, little-endian):
 *   [0-3]   float qw
 *   [4-7]   float qx
 *   [8-11]  float qy
 *   [12-15] float qz
 * ====================================================
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <fcntl.h>
#include <cmath>
#include <cstring>

// [B1] Binary packet struct ต้องตรงกับ firmware
struct __attribute__((packed)) ImuPacket {
  float qw, qx, qy, qz;
};

class DualUdpImuNode : public rclcpp::Node {
public:
  DualUdpImuNode() : Node("udp_imu_node") {
    pub_upperarm_ = create_publisher<sensor_msgs::msg::Imu>("/imu/upperarm", 10);
    pub_forearm_  = create_publisher<sensor_msgs::msg::Imu>("/imu/forearm",  10);

    sock_upper_   = setup_udp_socket(8888);
    sock_forearm_ = setup_udp_socket(8889);

    // [B4] poll ทุก 1ms แทน 10ms → forward packet เร็วขึ้น
    timer_ = create_wall_timer(
      std::chrono::milliseconds(1),
      std::bind(&DualUdpImuNode::timer_callback, this));

    RCLCPP_INFO(get_logger(), "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
    RCLCPP_INFO(get_logger(), "  udp_imu_broadcaster  [binary format, no LPF]");
    RCLCPP_INFO(get_logger(), "  Port 8888 → /imu/upperarm");
    RCLCPP_INFO(get_logger(), "  Port 8889 → /imu/forearm");
    RCLCPP_INFO(get_logger(), "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
  }

  ~DualUdpImuNode() {
    close(sock_upper_);
    close(sock_forearm_);
  }

private:
  int sock_upper_, sock_forearm_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_upperarm_, pub_forearm_;
  rclcpp::TimerBase::SharedPtr timer_;

  int setup_udp_socket(int port) {
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
      RCLCPP_ERROR(get_logger(), "Failed to create socket for port %d", port);
      return -1;
    }
    sockaddr_in addr{};
    addr.sin_family      = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port        = htons(port);
    bind(sock, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr));
    fcntl(sock, F_SETFL, O_NONBLOCK);
    return sock;
  }

  // [B1][B2][B3] process_socket: binary parse, no LPF, no zero-motion
  void process_socket(
    int sock,
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr & pub,
    const char* frame_id)
  {
    ImuPacket pkt;

    // drain ทุก packet ที่รออยู่ใน buffer (ดีกว่ารับทีละ packet)
    while (true) {
      int len = recvfrom(sock, &pkt, sizeof(pkt), 0, nullptr, nullptr);
      if (len != sizeof(ImuPacket)) break;  // หมด buffer หรือ packet ผิด size

      // normalize quaternion (ป้องกัน floating point drift จาก WiFi)
      float norm = std::sqrt(pkt.qw*pkt.qw + pkt.qx*pkt.qx +
                             pkt.qy*pkt.qy + pkt.qz*pkt.qz);
      if (norm < 1e-6f) continue;  // packet เสีย ทิ้ง
      pkt.qw /= norm; pkt.qx /= norm;
      pkt.qy /= norm; pkt.qz /= norm;

      // publish
      sensor_msgs::msg::Imu msg;
      msg.header.stamp    = now();
      msg.header.frame_id = frame_id;
      msg.orientation.w   = pkt.qw;
      msg.orientation.x   = pkt.qx;
      msg.orientation.y   = pkt.qy;
      msg.orientation.z   = pkt.qz;
      // [B5] angular_velocity และ linear_acceleration = 0 (ไม่ส่งมาแล้ว)

      pub->publish(msg);
    }
  }

  void timer_callback() {
    process_socket(sock_upper_,   pub_upperarm_, "imu_upperarm_link");
    process_socket(sock_forearm_, pub_forearm_,  "imu_forearm_link");
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DualUdpImuNode>());
  rclcpp::shutdown();
  return 0;
}