/**
 * imu_csv_publisher.cpp
 * อ่านข้อมูล IMU จาก CSV แล้ว publish เป็น sensor_msgs/Imu
 *
 * CSV columns:
 *   Time(s), smooth_qw, smooth_qx, smooth_qy, smooth_qz,
 *   ax, ay, az, gx, gy, gz, mx, my, mz, lx, ly, lz
 *
 * Publishes:
 *   /imu/upperarm  (sensor_msgs/Imu)
 *   /imu/forearm   (sensor_msgs/Imu)
 *
 * CSV files อยู่ที่:
 *   install/bno086_broadcaster/share/bno086_broadcaster/data/
 *   (หา path อัตโนมัติผ่าน ament_index ไม่ต้องส่ง argument)
 *
 * Optional Parameters:
 *   loop         : วนซ้ำ (default: true)
 *   speed_factor : 1.0=realtime, 2.0=2x (default: 1.0)
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <stdexcept>

struct ImuRow {
  double time;
  double qw, qx, qy, qz;
  double ax, ay, az;
  double gx, gy, gz;
};

std::vector<ImuRow> load_csv(const std::string & path)
{
  std::vector<ImuRow> rows;
  std::ifstream file(path);
  if (!file.is_open())
    throw std::runtime_error("Cannot open CSV: " + path);

  std::string line;
  std::getline(file, line); // skip header

  while (std::getline(file, line)) {
    if (line.empty()) continue;
    if (!line.empty() && line.back() == '\r') line.pop_back();

    std::stringstream ss(line);
    std::string token;
    std::vector<double> v;
    while (std::getline(ss, token, ',')) {
      try { v.push_back(std::stod(token)); }
      catch (...) { v.push_back(0.0); }
    }
    if (v.size() < 11) continue;

    rows.push_back({v[0], v[1],v[2],v[3],v[4], v[5],v[6],v[7], v[8],v[9],v[10]});
  }
  return rows;
}

class ImuCsvPublisher : public rclcpp::Node
{
public:
  ImuCsvPublisher() : Node("imu_csv_publisher")
  {
    // ── Parameters (optional) ─────────────────────────────────────────────────
    declare_parameter("loop",         true);
    declare_parameter("speed_factor", 1.0);

    loop_         = get_parameter("loop").as_bool();
    speed_factor_ = get_parameter("speed_factor").as_double();

    // ── Auto-find CSV path ────────────────────────────────────────────────────
    // ไม่ต้องส่ง path มาเอง — หาจาก package share directory อัตโนมัติ
    const std::string pkg_share =
      ament_index_cpp::get_package_share_directory("bno086_broadcaster");

    const std::string upper_path = pkg_share + "/data/imu_upperarm_8888.csv";
    const std::string fore_path  = pkg_share + "/data/imu_forearm_8889.csv";

    // ── Load CSV ──────────────────────────────────────────────────────────────
    RCLCPP_INFO(get_logger(), "Loading upperarm: %s", upper_path.c_str());
    upper_rows_ = load_csv(upper_path);

    RCLCPP_INFO(get_logger(), "Loading forearm:  %s", fore_path.c_str());
    fore_rows_  = load_csv(fore_path);

    RCLCPP_INFO(get_logger(), "Loaded %zu upperarm + %zu forearm rows",
      upper_rows_.size(), fore_rows_.size());

    // ── Publishers ────────────────────────────────────────────────────────────
    pub_upper_ = create_publisher<sensor_msgs::msg::Imu>("/imu/upperarm", 10);
    pub_fore_  = create_publisher<sensor_msgs::msg::Imu>("/imu/forearm",  10);

    csv_start_  = std::min(upper_rows_.front().time, fore_rows_.front().time);
    play_start_ = now();

    timer_ = create_wall_timer(
      std::chrono::milliseconds(1),
      std::bind(&ImuCsvPublisher::tick, this));

    RCLCPP_INFO(get_logger(),
      "Playback started  speed=%.1fx  loop=%s",
      speed_factor_, loop_ ? "true" : "false");
  }

private:
  std::vector<ImuRow> upper_rows_, fore_rows_;
  size_t upper_idx_{0}, fore_idx_{0};
  bool   loop_{true};
  double speed_factor_{1.0};
  double csv_start_{0.0};
  rclcpp::Time play_start_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_upper_, pub_fore_;

  sensor_msgs::msg::Imu make_msg(const ImuRow & r, const std::string & frame)
  {
    sensor_msgs::msg::Imu msg;
    msg.header.stamp    = get_clock()->now();
    msg.header.frame_id = frame;

    msg.orientation.w = r.qw;
    msg.orientation.x = r.qx;
    msg.orientation.y = r.qy;
    msg.orientation.z = r.qz;
    msg.orientation_covariance     = {0.01,0,0, 0,0.01,0, 0,0,0.01};

    msg.angular_velocity.x = r.gx;
    msg.angular_velocity.y = r.gy;
    msg.angular_velocity.z = r.gz;
    msg.angular_velocity_covariance = {0.01,0,0, 0,0.01,0, 0,0,0.01};

    msg.linear_acceleration.x = r.ax;
    msg.linear_acceleration.y = r.ay;
    msg.linear_acceleration.z = r.az;
    msg.linear_acceleration_covariance = {0.1,0,0, 0,0.1,0, 0,0,0.1};

    return msg;
  }

  void tick()
  {
    double elapsed = (now() - play_start_).seconds() * speed_factor_;

    while (upper_idx_ < upper_rows_.size() &&
           (upper_rows_[upper_idx_].time - csv_start_) <= elapsed) {
      pub_upper_->publish(make_msg(upper_rows_[upper_idx_], "imu_upperarm"));
      upper_idx_++;
    }

    while (fore_idx_ < fore_rows_.size() &&
           (fore_rows_[fore_idx_].time - csv_start_) <= elapsed) {
      pub_fore_->publish(make_msg(fore_rows_[fore_idx_], "imu_forearm"));
      fore_idx_++;
    }

    if (upper_idx_ >= upper_rows_.size() && fore_idx_ >= fore_rows_.size()) {
      if (loop_) {
        RCLCPP_INFO(get_logger(), "Looping...");
        upper_idx_  = 0;
        fore_idx_   = 0;
        play_start_ = now();
      } else {
        RCLCPP_INFO(get_logger(), "Playback done.");
        timer_->cancel();
      }
    }
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuCsvPublisher>());
  rclcpp::shutdown();
  return 0;
}
