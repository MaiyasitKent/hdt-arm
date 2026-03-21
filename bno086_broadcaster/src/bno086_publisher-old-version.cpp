#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <fcntl.h>
#include <cmath>
 
class DualUdpImuNode : public rclcpp::Node {
public:
    DualUdpImuNode() : Node("udp_imu_node") {
        // สร้าง Publisher 2 ตัวแยกกัน
        pub_forearm_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/forearm", 10);
        pub_uperarm = this->create_publisher<sensor_msgs::msg::Imu>("/imu/upperarm", 10);
        
        // เปิด Socket รอรับ 2 Port พร้อมกัน
        sock_upper_   = setup_udp_socket(8888);
        sock_forearm_ = setup_udp_socket(8889);
        
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&DualUdpImuNode::timer_callback, this)
        );
        RCLCPP_INFO(this->get_logger(), "Dual IMU Node Started!");
        RCLCPP_INFO(this->get_logger(), "Listening on Port 8888 (Forearm) & 8889 (Wrist)");
    }
 
    ~DualUdpImuNode() {
        close(sock_forearm_);
        close(sock_upper_);
    }
 
private:
    int sock_forearm_, sock_upper_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_forearm_, pub_uperarm;
    rclcpp::TimerBase::SharedPtr timer_;
 
    // --- แยกตัวแปร Smoothing ของแขน และ ข้อมือ ออกจากกัน ---
    float f_qw=1.0, f_qx=0.0, f_qy=0.0, f_qz=0.0; // Forearm
    float w_qw=1.0, w_qx=0.0, w_qy=0.0, w_qz=0.0; // Wrist
    
    // ตั้งค่า Filter และ Zero Motion (ปรับแก้ได้ที่นี่)
    const float alpha = 0.3;
    const float gyro_threshold = 0.03;
 
    int setup_udp_socket(int port) {
        int sock = socket(AF_INET, SOCK_DGRAM, 0);
        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = INADDR_ANY;
        addr.sin_port = htons(port);
        bind(sock, (struct sockaddr*)&addr, sizeof(addr));
        fcntl(sock, F_SETFL, O_NONBLOCK); // สำคัญมาก ทำให้โค้ดไม่ค้างถ้ารอข้อมูล
        return sock;
    }
 
    // ฟังก์ชันจัดการข้อมูล (ใช้ร่วมกันได้ทั้งแขนและข้อมือ)
    void process_socket(int sock, rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub,
                        const std::string& frame_id, float& sqw, float& sqx, float& sqy, float& sqz) {
        char buffer[512];
        while (true) {
            int len = recvfrom(sock, buffer, sizeof(buffer) - 1, 0, nullptr, nullptr);
            if (len > 0) {
                buffer[len] = '\0';
                float qw, qx, qy, qz, ax, ay, az, gx, gy, gz, mx, my, mz, lx, ly, lz;
                
                // แกะ 16 ค่า
                if (sscanf(buffer, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f",
                           &qw, &qx, &qy, &qz, &ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz, &lx, &ly, &lz) == 16) {
                    
                    // 1. ตรวจจับการหยุดนิ่ง (Zero Motion)
                    bool is_zero_motion = (std::abs(gx) < gyro_threshold &&
                                           std::abs(gy) < gyro_threshold &&
                                           std::abs(gz) < gyro_threshold);
 
                    // 2. กรองข้อมูล (Low-Pass Filter)
                    if (!is_zero_motion) {
                        sqw = (alpha * qw) + ((1.0 - alpha) * sqw);
                        sqx = (alpha * qx) + ((1.0 - alpha) * sqx);
                        sqy = (alpha * qy) + ((1.0 - alpha) * sqy);
                        sqz = (alpha * qz) + ((1.0 - alpha) * sqz);
                        // Normalization
                        float norm = std::sqrt(sqw*sqw + sqx*sqx + sqy*sqy + sqz*sqz);
                        sqw /= norm; sqx /= norm; sqy /= norm; sqz /= norm;
                    }
 
                    // 3. แพ็กลง ROS2 Message
                    auto msg = sensor_msgs::msg::Imu();
                    msg.header.stamp = this->now();
                    msg.header.frame_id = frame_id;
 
                    msg.orientation.w = sqw; msg.orientation.x = sqx;
                    msg.orientation.y = sqy; msg.orientation.z = sqz;
                    
                    msg.angular_velocity.x = gx; msg.angular_velocity.y = gy; msg.angular_velocity.z = gz;
                    msg.linear_acceleration.x = ax; msg.linear_acceleration.y = ay; msg.linear_acceleration.z = az;
 
                    pub->publish(msg);
                }
            } else {
                break; // หมดข้อมูลใน Buffer รอบนี้แล้ว
            }
        }
    }
 
    void timer_callback() {
        // เช็คและอัปเดตข้อมูลทั้ง 2 เซนเซอร์พร้อมกัน
        process_socket(sock_forearm_, pub_forearm_, "imu_forearm_link", f_qw, f_qx, f_qy, f_qz);
        process_socket(sock_upper_, pub_uperarm, "imu_upperarm_link", w_qw, w_qx, w_qy, w_qz);
    }
};
 
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DualUdpImuNode>());
    rclcpp::shutdown();
    return 0;
}