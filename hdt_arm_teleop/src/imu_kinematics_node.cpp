/**
 * imu_kinematics_node.cpp  —  Position Control via Action (preempt mode)
 *
 * Subscribes:  /imu/upperarm, /imu/forearm  (sensor_msgs/Imu)
 * Action:      /real/arm_controller/follow_joint_trajectory
 * Publishes:   /imu_kinematics/joint_angles  (debug, Float64MultiArray)
 * Services:    /imu_kinematics/calibrate (std_srvs/Trigger)
 *              /imu_kinematics/estop     (std_srvs/SetBool)
 */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>

#include <array>
#include <atomic>
#include <cmath>
#include <mutex>
#include <string>

using ImuMsg     = sensor_msgs::msg::Imu;
using FollowJT   = control_msgs::action::FollowJointTrajectory;
using GoalHandle = rclcpp_action::ClientGoalHandle<FollowJT>;

enum JIdx { PITCH=0, ROLL=1, TWIST=2, ELBOW=3, WRIST=4, NUM_JOINTS=5 };

static const std::array<const char*, NUM_JOINTS> JOINT_NAMES = {
  "shoulder_pitch_joint", "shoulder_roll_joint",
  "upperarm_joint", "elbow_joint", "wrist_joint"
};

static const std::array<double, NUM_JOINTS> VEL_LIMITS   = { 2.5, 2.5, 3.0, 2.5, 3.5 };
static const std::array<double, NUM_JOINTS> HARD_LOWER   = { -1.57, -0.1,  -1.57, -1.9,  -1.57 };
static const std::array<double, NUM_JOINTS> HARD_UPPER   = {  3.142, 2.967,  1.57,  0.0,   1.57 };
static const std::array<double, NUM_JOINTS> JOINT_SCALE  = { 1.0, -1.0, 1.0, -1.0, 1.0 };

class LowPassFilter {
public:
  explicit LowPassFilter(double alpha = 0.3) : alpha_(alpha) {}
  double update(double x) { v_ = alpha_*x + (1.0-alpha_)*v_; return v_; }
  void   reset(double v = 0.0) { v_ = v; }
private:
  double alpha_, v_{0.0};
};

class ImuKinematicsNode : public rclcpp::Node
{
public:
  ImuKinematicsNode() : Node("imu_kinematics_node")
  {
    // ============================================================
    // [แก้ 1] declare use_sim_time ก่อน parameter อื่นทั้งหมด
    //         ทำให้ this->now() และ get_clock() ใช้ clock ที่ถูกต้อง
    //         ทั้งใน real mode และ sim mode
    // ============================================================
    // this->declare_parameter_if_not_declared("use_sim_time", rclcpp::ParameterValue(false));

    declare_parameter("auto_calibrate",        true);
    declare_parameter("calibration_samples",   50);
    declare_parameter("lpf_alpha",             0.2);
    declare_parameter("send_hz",               10.0);
    declare_parameter("trajectory_duration",   0.15);
    declare_parameter("imu_timeout_ms",        1000.0);
    declare_parameter("max_jump_deg",          50.0);
    declare_parameter("soft_limit_margin_deg", 3.0);

    auto_calibrate_ = get_parameter("auto_calibrate").as_bool();
    calib_samples_  = get_parameter("calibration_samples").as_int();
    traj_duration_  = get_parameter("trajectory_duration").as_double();
    imu_timeout_s_  = get_parameter("imu_timeout_ms").as_double() / 1000.0;
    max_jump_rad_   = get_parameter("max_jump_deg").as_double() * M_PI / 180.0;
    use_sim_time_   = get_parameter("use_sim_time").as_bool();

    const double alpha   = get_parameter("lpf_alpha").as_double();
    const double send_hz = get_parameter("send_hz").as_double();
    const double margin  = get_parameter("soft_limit_margin_deg").as_double() * M_PI / 180.0;

    for (int i = 0; i < NUM_JOINTS; i++) {
      soft_min_[i] = HARD_LOWER[i] + margin;
      soft_max_[i] = HARD_UPPER[i] - margin;
      lpf_[i]      = LowPassFilter(alpha);
    }

    latest_pos_.fill(0.0);
    prev_pos_.fill(0.0);
    qu_latest_ = tf2::Quaternion(0, 0, 0, 1);
    qf_latest_ = tf2::Quaternion(0, 0, 0, 1);

    // [H1] init timestamps
    const auto clock_type = this->get_clock()->get_clock_type();
    last_upper_time_    = rclcpp::Time(0, 0, clock_type);
    last_fore_time_     = rclcpp::Time(0, 0, clock_type);
    imu_upper_received_ = false;
    imu_fore_received_  = false;

    if (auto_calibrate_) {
      calibrating_ = true;
      RCLCPP_INFO(get_logger(),
        "Auto-calibrate ON — collecting %d frames, hold arm hanging down...",
        calib_samples_);
    }

    // ============================================================
    // [แก้ 2] เพิ่ม namespace /real ให้ตรงกับ launch file
    // ============================================================
    action_client_ = rclcpp_action::create_client<FollowJT>(
      this, "/real/arm_controller/follow_joint_trajectory");

    pub_debug_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/imu_kinematics/joint_angles", 10);

    srv_calib_ = create_service<std_srvs::srv::Trigger>(
      "/imu_kinematics/calibrate",
      std::bind(&ImuKinematicsNode::calib_cb, this,
                std::placeholders::_1, std::placeholders::_2));

    srv_estop_ = create_service<std_srvs::srv::SetBool>(
      "/imu_kinematics/estop",
      std::bind(&ImuKinematicsNode::estop_cb, this,
                std::placeholders::_1, std::placeholders::_2));

    sub_upper_ = create_subscription<ImuMsg>(
      "/imu/upperarm", rclcpp::SensorDataQoS(),
      std::bind(&ImuKinematicsNode::upper_cb, this, std::placeholders::_1));
    sub_fore_ = create_subscription<ImuMsg>(
      "/imu/forearm", rclcpp::SensorDataQoS(),
      std::bind(&ImuKinematicsNode::fore_cb, this, std::placeholders::_1));

    const auto ms = std::chrono::milliseconds(static_cast<int>(1000.0 / send_hz));
    send_timer_ = create_wall_timer(ms, std::bind(&ImuKinematicsNode::timer_cb, this));

    RCLCPP_INFO(get_logger(), "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
    RCLCPP_INFO(get_logger(), "  imu_kinematics_node v7  [%.0fHz / traj=%.3fs]",
      send_hz, traj_duration_);
    RCLCPP_INFO(get_logger(), "  use_sim_time : %s", use_sim_time_ ? "true" : "false");
    RCLCPP_INFO(get_logger(), "  action       : /real/arm_controller/follow_joint_trajectory");
    RCLCPP_INFO(get_logger(), "  [H1] watchdog   : %.0f ms", imu_timeout_s_*1000);
    RCLCPP_INFO(get_logger(), "  [H2] jump check : %.1f deg", max_jump_rad_*180/M_PI);
    RCLCPP_INFO(get_logger(), "  [H3] vel hints  : enabled");
    RCLCPP_INFO(get_logger(), "  [H4] soft limits: margin=%.1f deg", margin*180/M_PI);
    RCLCPP_INFO(get_logger(), "  [H5] e-stop     : ros2 service call /imu_kinematics/estop std_srvs/srv/SetBool \"{data: true}\"");
    RCLCPP_INFO(get_logger(), "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
  }

private:
  bool   auto_calibrate_{true};
  bool   use_sim_time_{false};
  int    calib_samples_;
  double traj_duration_;
  double imu_timeout_s_;
  double max_jump_rad_;

  std::array<double, NUM_JOINTS>        soft_min_, soft_max_;
  std::array<LowPassFilter, NUM_JOINTS> lpf_;
  double ref_elbow_{0.0};
  std::array<double, NUM_JOINTS> latest_pos_, prev_pos_;

  tf2::Quaternion qu_latest_, qf_latest_;
  std::mutex      imu_mutex_;

  rclcpp::Time last_upper_time_, last_fore_time_;
  bool         imu_upper_received_{false};
  bool         imu_fore_received_{false};

  std::atomic<bool> estop_{false};

  std::mutex     calib_mutex_;
  bool           calibrating_{false};
  int            calib_count_{0};
  double         calib_elbow_accum_{0.0};
  bool           ready_{false};
  tf2::Matrix3x3 R_calib_upper_, R_calib_fore_;
  tf2::Matrix3x3 calib_R_upper_accum_, calib_R_fore_accum_;

  rclcpp_action::Client<FollowJT>::SharedPtr          action_client_;
  GoalHandle::SharedPtr                               current_gh_{nullptr};
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_debug_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr  srv_calib_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr  srv_estop_;
  rclcpp::TimerBase::SharedPtr                        send_timer_;
  rclcpp::Subscription<ImuMsg>::SharedPtr             sub_upper_, sub_fore_;

  void upper_cb(const ImuMsg::ConstSharedPtr & msg)
  {
    tf2::Quaternion q(msg->orientation.x, msg->orientation.y,
                      msg->orientation.z, msg->orientation.w);
    q.normalize();
    std::lock_guard<std::mutex> lock(imu_mutex_);
    qu_latest_          = q;
    last_upper_time_    = now();
    imu_upper_received_ = true;
  }

  void fore_cb(const ImuMsg::ConstSharedPtr & msg)
  {
    tf2::Quaternion q(msg->orientation.x, msg->orientation.y,
                      msg->orientation.z, msg->orientation.w);
    q.normalize();
    std::lock_guard<std::mutex> lock(imu_mutex_);
    qf_latest_         = q;
    last_fore_time_    = now();
    imu_fore_received_ = true;
  }

  void estop_cb(
    const std_srvs::srv::SetBool::Request::SharedPtr  req,
    std_srvs::srv::SetBool::Response::SharedPtr       res)
  {
    estop_.store(req->data);
    if (req->data) {
      if (current_gh_) {
        action_client_->async_cancel_goal(current_gh_);
        current_gh_ = nullptr;
      }
      RCLCPP_WARN(get_logger(), "━━━ EMERGENCY STOP ACTIVATED ━━━");
      res->message = "E-stop activated";
    } else {
      RCLCPP_INFO(get_logger(), "━━━ E-stop released ━━━");
      res->message = "E-stop released";
    }
    res->success = true;
  }

  void timer_cb()
  {
    if (estop_.load()) return;

    tf2::Quaternion qu, qf;
    rclcpp::Time t_upper, t_fore;
    {
      std::lock_guard<std::mutex> lock(imu_mutex_);
      qu      = qu_latest_;
      qf      = qf_latest_;
      t_upper = last_upper_time_;
      t_fore  = last_fore_time_;
    }

    if (imu_upper_received_ && imu_fore_received_) {
      const double age_u = (now() - t_upper).seconds();
      const double age_f = (now() - t_fore).seconds();
      if (age_u > imu_timeout_s_ || age_f > imu_timeout_s_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
          "IMU timeout! upper=%.1fms fore=%.1fms — holding position",
          age_u*1000, age_f*1000);
        return;
      }
    }

    {
      std::lock_guard<std::mutex> lock(calib_mutex_);
      if (calibrating_) {
        const tf2::Matrix3x3 Ru(qu), Rf(qf);
        const tf2::Vector3 bu = Ru * tf2::Vector3(1,0,0);
        const tf2::Vector3 bf = Rf * tf2::Vector3(1,0,0);
        calib_R_upper_accum_ = mat_add(calib_R_upper_accum_, Ru);
        calib_R_fore_accum_  = mat_add(calib_R_fore_accum_,  Rf);
        calib_elbow_accum_  += std::acos(std::clamp(bu.dot(bf), -1.0, 1.0));
        if (++calib_count_ >= calib_samples_) {
          R_calib_upper_ = mat_div(calib_R_upper_accum_, calib_count_);
          R_calib_fore_  = mat_div(calib_R_fore_accum_,  calib_count_);
          ref_elbow_     = calib_elbow_accum_ / calib_count_;
          calibrating_   = false;
          ready_         = true;
          for (auto & f : lpf_) f.reset(0.0);
          prev_pos_.fill(0.0);
          RCLCPP_INFO(get_logger(), "━━━ Calibration complete! ━━━");
          RCLCPP_INFO(get_logger(), "  elbow ref = %.3f rad (%.1f deg)",
            ref_elbow_, ref_elbow_*180/M_PI);
        }
        return;
      }
      if (!ready_) return;
    }

    const tf2::Matrix3x3 Ru(qu), Rf(qf);
    const tf2::Vector3 bu = Ru * tf2::Vector3(1,0,0);
    const tf2::Vector3 bf = Rf * tf2::Vector3(1,0,0);

    double pitch = 0.0, roll = 0.0;
    compute_swing_pitch_roll(Ru, R_calib_upper_, pitch, roll);

    const std::array<double, NUM_JOINTS> angles = {{
      pitch,
      roll,
      compute_twist_swing(Ru, R_calib_upper_),
      std::acos(std::clamp(bu.dot(bf), -1.0, 1.0)) - ref_elbow_,
      compute_twist_swing(Rf, R_calib_fore_)
    }};

    std::array<double, NUM_JOINTS> new_pos;
    for (int i = 0; i < NUM_JOINTS; i++) {
      const double filtered = lpf_[i].update(angles[i] * JOINT_SCALE[i]);
      new_pos[i] = std::clamp(filtered, soft_min_[i], soft_max_[i]);
    }

    for (int i = 0; i < NUM_JOINTS; i++) {
      const double delta = new_pos[i] - latest_pos_[i];
      if (std::abs(delta) > max_jump_rad_) {
        new_pos[i] = latest_pos_[i] + std::copysign(max_jump_rad_, delta);
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 500,
          "Rate limit joint %d: %.1f->%.1f deg",
          i, latest_pos_[i]*180/M_PI, new_pos[i]*180/M_PI);
      }
    }

    std_msgs::msg::Float64MultiArray dbg;
    dbg.data.resize(NUM_JOINTS * 2);
    for (int i = 0; i < NUM_JOINTS; i++) {
      dbg.data[i]              = angles[i] * 180.0/M_PI;
      dbg.data[i + NUM_JOINTS] = new_pos[i] * 180.0/M_PI;
    }
    pub_debug_->publish(dbg);

    prev_pos_   = latest_pos_;
    latest_pos_ = new_pos;
    send_goal_now();
  }

  void send_goal_now()
  {
    if (!action_client_->action_server_is_ready()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "Action server not ready");
      return;
    }

    if (current_gh_) {
      auto status = current_gh_->get_status();
      if (status == rclcpp_action::GoalStatus::STATUS_ACCEPTED ||
          status == rclcpp_action::GoalStatus::STATUS_EXECUTING) {
        action_client_->async_cancel_goal(current_gh_);
      }
      current_gh_ = nullptr;
    }

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions.assign(latest_pos_.begin(), latest_pos_.end());
    point.time_from_start = rclcpp::Duration::from_seconds(traj_duration_);

    point.velocities.resize(NUM_JOINTS);
    for (int i = 0; i < NUM_JOINTS; i++) {
      double vel = (latest_pos_[i] - prev_pos_[i]) / traj_duration_;
      point.velocities[i] = std::clamp(vel, -VEL_LIMITS[i], VEL_LIMITS[i]);
    }

    FollowJT::Goal goal;

    // ============================================================
    // [แก้ 3] timestamp
    //   use_sim_time=false → stamp=0  (execute ทันที, real clock)
    //   use_sim_time=true  → stamp=now() (ใช้ sim clock จาก Gazebo)
    //                        ถ้าใช้ 0 controller จะ reject goal
    //                        เพราะ timestamp ไม่ sync กับ /clock
    // ============================================================
    if (use_sim_time_) {
      goal.trajectory.header.stamp = this->now();
    } else {
      goal.trajectory.header.stamp =
        rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
    }

    goal.trajectory.header.frame_id = "base_link";
    goal.trajectory.joint_names.assign(JOINT_NAMES.begin(), JOINT_NAMES.end());
    goal.trajectory.points.push_back(point);

    auto opts = rclcpp_action::Client<FollowJT>::SendGoalOptions();
    opts.goal_response_callback =
      [this](const GoalHandle::SharedPtr & gh) {
        if (!gh) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Goal rejected");
          return;
        }
        current_gh_ = gh;
      };
    opts.result_callback =
      [this](const GoalHandle::WrappedResult & result) {
        if (result.code == rclcpp_action::ResultCode::ABORTED)
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
            "Goal aborted: %s", result.result->error_string.c_str());
        current_gh_ = nullptr;
      };
    action_client_->async_send_goal(goal, opts);
  }

  void calib_cb(
    const std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr res)
  {
    if (estop_.load()) {
      res->success = false;
      res->message = "Cannot calibrate while e-stop is active";
      return;
    }
    std::lock_guard<std::mutex> lock(calib_mutex_);
    calib_elbow_accum_   = 0.0;
    calib_R_upper_accum_ = tf2::Matrix3x3(0,0,0, 0,0,0, 0,0,0);
    calib_R_fore_accum_  = tf2::Matrix3x3(0,0,0, 0,0,0, 0,0,0);
    calib_count_ = 0;
    calibrating_ = true;
    ready_       = false;
    RCLCPP_INFO(get_logger(), "Re-calibration started — hold arm hanging down...");
    res->success = true;
    res->message = "Calibrating, hold reference pose (arm hanging down)";
  }

  static tf2::Matrix3x3 mat_add(const tf2::Matrix3x3 & A, const tf2::Matrix3x3 & B) {
    tf2::Matrix3x3 C;
    for (int i=0;i<3;i++) for (int j=0;j<3;j++) C[i][j]=A[i][j]+B[i][j];
    return C;
  }

  static tf2::Matrix3x3 mat_div(const tf2::Matrix3x3 & A, double s) {
    tf2::Matrix3x3 C;
    for (int i=0;i<3;i++) for (int j=0;j<3;j++) C[i][j]=A[i][j]/s;
    return C;
  }

  static void compute_swing_pitch_roll(
    const tf2::Matrix3x3 & R_now,
    const tf2::Matrix3x3 & R_calib,
    double & pitch, double & roll)
  {
    const tf2::Vector3 bu_c = R_calib.transpose() * (R_now * tf2::Vector3(1,0,0));
    const tf2::Vector3 home(1.0, 0.0, 0.0);
    tf2::Vector3 sa = home.cross(bu_c);
    const double sa_len = sa.length();
    if (sa_len < 1e-6) { pitch = 0.0; roll = 0.0; return; }
    sa /= sa_len;
    const double angle = std::acos(std::clamp(home.dot(bu_c), -1.0, 1.0));
    pitch = angle * sa.z();
    roll  = angle * sa.y();
  }

  static double compute_twist_swing(
    const tf2::Matrix3x3 & R_now,
    const tf2::Matrix3x3 & R_calib)
  {
    const tf2::Vector3 bc = R_calib * tf2::Vector3(1,0,0);
    const tf2::Vector3 bn = R_now   * tf2::Vector3(1,0,0);
    tf2::Vector3 sa = bc.cross(bn);
    const double sa_len = sa.length();
    const double cos_a  = std::clamp(bc.dot(bn), -1.0, 1.0);
    tf2::Matrix3x3 Rs;
    if (sa_len < 1e-6) {
      Rs = (cos_a > 0.0) ? R_calib : [&]() {
        tf2::Vector3 ax = std::abs(bc.x()) < 0.9 ?
          tf2::Vector3(1,0,0) : tf2::Vector3(0,1,0);
        tf2::Vector3 a = bc.cross(ax); a /= a.length();
        return tf2::Matrix3x3(
          2*a.x()*a.x()-1, 2*a.x()*a.y(),   2*a.x()*a.z(),
          2*a.y()*a.x(),   2*a.y()*a.y()-1, 2*a.y()*a.z(),
          2*a.z()*a.x(),   2*a.z()*a.y(),   2*a.z()*a.z()-1) * R_calib;
      }();
    } else {
      sa /= sa_len;
      const double ang = std::atan2(sa_len, cos_a);
      const double s=std::sin(ang), c=std::cos(ang), t=1.0-c;
      const double x=sa.x(), y=sa.y(), z=sa.z();
      Rs = tf2::Matrix3x3(
        t*x*x+c,   t*x*y-s*z, t*x*z+s*y,
        t*x*y+s*z, t*y*y+c,   t*y*z-s*x,
        t*x*z-s*y, t*y*z+s*x, t*z*z+c) * R_calib;
    }
    const tf2::Matrix3x3 Rt = Rs.transpose() * R_now;
    return std::atan2(Rt[2][1], Rt[2][2]);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuKinematicsNode>());
  rclcpp::shutdown();
  return 0;
}