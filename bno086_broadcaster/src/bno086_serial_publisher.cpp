/**
 * serial_imu_broadcaster.cpp
 *
 * รับ Serial packet จาก ESP32 สองตัวพร้อมกัน และ publish เป็น ROS2 IMU topic
 * โครงสร้างเดียวกับ udp_imu_broadcaster.cpp — เปลี่ยนแค่ transport layer
 *
 * Serial Packet Format (22 bytes, little-endian):
 *   [0-1]   uint8_t  magic   { 0xAA, 0x55 }
 *   [2-5]   uint32_t seq
 *   [6-9]   float    qw
 *   [10-13] float    qx
 *   [14-17] float    qy
 *   [18-21] float    qz
 *
 * Parameters:
 *   upper_port   (string)  default: /dev/ttyUSB0
 *   forearm_port (string)  default: /dev/ttyUSB1
 *   baud         (int)     default: 115200
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <cmath>

struct __attribute__((packed)) SerialPacket {
  uint8_t  magic[2];   // { 0xAA, 0x55 }
  uint32_t seq;
  float    qw, qx, qy, qz;
};

static constexpr size_t  PKT_SIZE = sizeof(SerialPacket);  // 22 bytes
static constexpr size_t  BUF_SIZE = PKT_SIZE * 8;
static constexpr uint8_t MAGIC0   = 0xAA;
static constexpr uint8_t MAGIC1   = 0x55;

struct SerialChannel {
  std::string port;
  std::string frame_id;
  std::string topic;
  int         fd        = -1;

  uint8_t     buf[BUF_SIZE];
  size_t      buf_len   = 0;

  uint32_t    prev_seq  = 0;
  bool        first_pkt = true;

  uint64_t    total_pkts   = 0;
  uint64_t    total_drops  = 0;
  uint64_t    window_pkts  = 0;
  uint64_t    window_drops = 0;

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr                  pub_imu;
};

class DualSerialImuNode : public rclcpp::Node {
public:
  DualSerialImuNode() : Node("serial_imu_node") {

    declare_parameter("upper_port",   "/dev/ttyUSB0");
    declare_parameter("forearm_port", "/dev/ttyUSB1");
    declare_parameter("baud",         115200);

    int baud = get_parameter("baud").as_int();

    upper_.port     = get_parameter("upper_port").as_string();
    upper_.frame_id = "imu_upperarm_link";
    upper_.topic    = "/imu/upperarm";
    upper_.pub_imu  = create_publisher<sensor_msgs::msg::Imu>(upper_.topic, 10);
    upper_.fd       = open_serial(upper_.port, baud);

    fore_.port     = get_parameter("forearm_port").as_string();
    fore_.frame_id = "imu_forearm_link";
    fore_.topic    = "/imu/forearm";
    fore_.pub_imu  = create_publisher<sensor_msgs::msg::Imu>(fore_.topic, 10);
    fore_.fd       = open_serial(fore_.port, baud);

    if (upper_.fd < 0 || fore_.fd < 0) {
      RCLCPP_FATAL(get_logger(), "เปิด serial port ไม่สำเร็จ — ตรวจสอบ device และสิทธิ์");
      rclcpp::shutdown();
      return;
    }

    timer_ = create_wall_timer(
      std::chrono::milliseconds(1),
      std::bind(&DualSerialImuNode::timer_callback, this));

    RCLCPP_INFO(get_logger(), "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
    RCLCPP_INFO(get_logger(), "  serial_imu_broadcaster  [dual port, magic sync]");
    RCLCPP_INFO(get_logger(), "  %s -> /imu/upperarm", upper_.port.c_str());
    RCLCPP_INFO(get_logger(), "  %s -> /imu/forearm",  fore_.port.c_str());
    RCLCPP_INFO(get_logger(), "  baud: %d  packet: %zu bytes", baud, PKT_SIZE);
    RCLCPP_INFO(get_logger(), "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
  }

  ~DualSerialImuNode() {
    if (upper_.fd >= 0) close(upper_.fd);
    if (fore_.fd  >= 0) close(fore_.fd);
  }

private:
  SerialChannel upper_, fore_;
  rclcpp::TimerBase::SharedPtr timer_, diag_timer_;

  int open_serial(const std::string & port, int baud) {
    int fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
      RCLCPP_ERROR(get_logger(), "open(%s) failed: %s", port.c_str(), strerror(errno));
      return -1;
    }

    termios tty{};
    if (tcgetattr(fd, &tty) < 0) {
      RCLCPP_ERROR(get_logger(), "tcgetattr(%s) failed: %s", port.c_str(), strerror(errno));
      close(fd);
      return -1;
    }

    speed_t speed = B115200;
    if      (baud == 921600) speed = B921600;
    else if (baud == 460800) speed = B460800;
    else if (baud == 230400) speed = B230400;
    else if (baud == 115200) speed = B115200;
    else RCLCPP_WARN(get_logger(), "baud %d ไม่รู้จัก — ใช้ 115200", baud);

    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);
    cfmakeraw(&tty);
    tty.c_cflag    |= CLOCAL | CREAD;
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 0;
    tcsetattr(fd, TCSANOW, &tty);
    tcflush(fd, TCIFLUSH);

    RCLCPP_INFO(get_logger(), "เปิด %s @ %d baud สำเร็จ", port.c_str(), baud);
    return fd;
  }

  void process_channel(SerialChannel & ch) {
    if (ch.fd < 0) return;

    ssize_t n = read(ch.fd, ch.buf + ch.buf_len, BUF_SIZE - ch.buf_len);
    if (n > 0) ch.buf_len += static_cast<size_t>(n);

    while (ch.buf_len >= PKT_SIZE) {
      if (ch.buf[0] != MAGIC0) { memmove(ch.buf, ch.buf + 1, --ch.buf_len); continue; }
      if (ch.buf[1] != MAGIC1) { memmove(ch.buf, ch.buf + 1, --ch.buf_len); continue; }

      SerialPacket pkt;
      memcpy(&pkt, ch.buf, PKT_SIZE);
      ch.buf_len -= PKT_SIZE;
      memmove(ch.buf, ch.buf + PKT_SIZE, ch.buf_len);

      if (!ch.first_pkt) {
        uint32_t gap = pkt.seq - (ch.prev_seq + 1);
        if (gap > 0) {
          ch.total_drops  += gap;
          ch.window_drops += gap;
          RCLCPP_WARN(get_logger(), "[%s] seq gap: expect %u got %u (drop %u)",
            ch.port.c_str(), ch.prev_seq + 1, pkt.seq, gap);
        }
      }
      ch.prev_seq  = pkt.seq;
      ch.first_pkt = false;
      ch.total_pkts++;
      ch.window_pkts++;

      float norm = std::sqrt(pkt.qw*pkt.qw + pkt.qx*pkt.qx +
                             pkt.qy*pkt.qy + pkt.qz*pkt.qz);
      if (norm < 1e-6f) continue;
      pkt.qw /= norm;  pkt.qx /= norm;
      pkt.qy /= norm;  pkt.qz /= norm;

      sensor_msgs::msg::Imu msg;
      msg.header.stamp    = now();
      msg.header.frame_id = ch.frame_id;
      msg.orientation.w   = pkt.qw;
      msg.orientation.x   = pkt.qx;
      msg.orientation.y   = pkt.qy;
      msg.orientation.z   = pkt.qz;
      ch.pub_imu->publish(msg);
    }
  }

  void timer_callback() {
    process_channel(upper_);
    process_channel(fore_);
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DualSerialImuNode>());
  rclcpp::shutdown();
  return 0;
}