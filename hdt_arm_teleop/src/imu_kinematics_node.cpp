/**
 * imu_kinematics_node.cpp  —  Position Control via Action (preempt mode)
 *
 * Subscribes:  /imu/upperarm, /imu/forearm  (sensor_msgs/Imu)
 * Action:      /arm_controller/follow_joint_trajectory
 * Publishes:   /imu_kinematics/joint_angles  (debug, Float64MultiArray)
 *                data[0-4] = human relative angles (deg)
 *                data[5-9] = robot target positions (deg)
 * Services:    /imu_kinematics/calibrate (std_srvs/Trigger)
 *
 * Strategy:
 *   timer 10Hz → ส่ง action goal ใหม่ทับ goal เก่า (preempt)
 *   controller จะ CANCEL goal เก่าและเริ่ม goal ใหม่ทันที
 *   ไม่มี drift เพราะส่ง position โดยตรง
 *
 * Auto-calibration:
 *   calibrate อัตโนมัติจาก 50 frame แรก
 *   re-calibrate ได้ด้วย: ros2 service call /imu_kinematics/calibrate std_srvs/srv/Trigger {}
 */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>

#include <cmath>
#include <map>
#include <string>
#include <vector>
#include <mutex>

using ImuMsg     = sensor_msgs::msg::Imu;
using FollowJT   = control_msgs::action::FollowJointTrajectory;
using GoalHandle = rclcpp_action::ClientGoalHandle<FollowJT>;
using ApproxSync = message_filters::sync_policies::ApproximateTime<ImuMsg, ImuMsg>;

// ─── Low-pass filter ─────────────────────────────────────────────────────────
class LowPassFilter {
public:
  explicit LowPassFilter(double alpha = 0.2) : alpha_(alpha) {}
  double update(double x) { v_ = alpha_ * x + (1.0 - alpha_) * v_; return v_; }
  void reset(double v = 0.0) { v_ = v; }
private:
  double alpha_, v_{0.0};
};

// ─── Joint config ─────────────────────────────────────────────────────────────
struct JointConfig {
  double robot_min, robot_max, scale;
  LowPassFilter lpf;
  JointConfig() : robot_min(-1.57), robot_max(1.57), scale(1.0), lpf(0.2) {}
  JointConfig(double mn, double mx, double sc, double alpha = 0.2)
    : robot_min(mn), robot_max(mx), scale(sc), lpf(alpha) {}
};

// ─── Node ─────────────────────────────────────────────────────────────────────
class ImuKinematicsNode : public rclcpp::Node
{
public:
  ImuKinematicsNode() : Node("imu_kinematics_node")
  {
    declare_parameter("auto_calibrate",      true);
    declare_parameter("calibration_samples", 50);
    declare_parameter("lpf_alpha",           0.2);
    declare_parameter("sync_tolerance_ms",   10.0);
    declare_parameter("send_hz",             10.0);   // ส่ง goal กี่ครั้ง/วินาที
    declare_parameter("trajectory_duration", 0.2);    // ควรมากกว่า 1/send_hz

    auto_calibrate_ = get_parameter("auto_calibrate").as_bool();
    calib_samples_  = get_parameter("calibration_samples").as_int();
    traj_duration_  = get_parameter("trajectory_duration").as_double();
    double alpha    = get_parameter("lpf_alpha").as_double();
    double send_hz  = get_parameter("send_hz").as_double();

    joint_names_ = {
      "shoulder_pitch_joint",
      "shoulder_roll_joint",
      "upperarm_joint",
      "elbow_joint",
      "wrist_joint"
    };

    // scale: human_rel * scale = robot_target (rad)
    // sign verified from URDF + CSV analysis
    joint_configs_["shoulder_pitch_joint"] = JointConfig(-1.57,  3.142, -1.0,  alpha);
    joint_configs_["shoulder_roll_joint"]  = JointConfig(-0.1,   2.356, +1.0,  alpha);
    joint_configs_["upperarm_joint"]       = JointConfig(-1.57,  1.57,  +0.55, alpha);
    joint_configs_["elbow_joint"]          = JointConfig(-1.9,   0.0,   -0.82, alpha);
    joint_configs_["wrist_joint"]          = JointConfig(-1.57,  1.57,  +0.92, alpha);

    for (const auto & n : joint_names_) {
      ref_angles_[n]       = 0.0;
      calib_accum_[n]      = 0.0;
      latest_positions_[n] = 0.0;
    }

    if (auto_calibrate_) {
      calibrating_ = true;
      RCLCPP_INFO(get_logger(),
        "Auto-calibrate ON — collecting %d frames, hold arm hanging down...",
        calib_samples_);
    }

    // ── Action client ─────────────────────────────────────────────────────────
    action_client_ = rclcpp_action::create_client<FollowJT>(
      this, "/arm_controller/follow_joint_trajectory");

    // ── Debug publisher ───────────────────────────────────────────────────────
    pub_debug_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/imu_kinematics/joint_angles", 10);

    // ── Calibration service ───────────────────────────────────────────────────
    srv_calib_ = create_service<std_srvs::srv::Trigger>(
      "/imu_kinematics/calibrate",
      std::bind(&ImuKinematicsNode::calib_cb, this,
                std::placeholders::_1, std::placeholders::_2));

    // ── IMU synchronized subscribers ──────────────────────────────────────────
    sub_upper_.subscribe(this, "/imu/upperarm");
    sub_fore_.subscribe(this,  "/imu/forearm");
    double tol = get_parameter("sync_tolerance_ms").as_double();
    sync_ = std::make_shared<message_filters::Synchronizer<ApproxSync>>(
      ApproxSync(20), sub_upper_, sub_fore_);
    sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(tol / 1000.0));
    sync_->registerCallback(std::bind(&ImuKinematicsNode::imu_cb, this,
      std::placeholders::_1, std::placeholders::_2));

    // ── Send timer — ส่ง goal ใหม่ทุก 1/send_hz วินาที ───────────────────────
    auto ms = std::chrono::milliseconds(static_cast<int>(1000.0 / send_hz));
    send_timer_ = create_wall_timer(ms,
      std::bind(&ImuKinematicsNode::send_goal, this));

    RCLCPP_INFO(get_logger(), "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
    RCLCPP_INFO(get_logger(), "  imu_kinematics_node  [Action preempt %.0fHz]", send_hz);
    RCLCPP_INFO(get_logger(), "  → /arm_controller/follow_joint_trajectory");
    RCLCPP_INFO(get_logger(), "  Re-calibrate: ros2 service call /imu_kinematics/calibrate std_srvs/srv/Trigger {}");
    RCLCPP_INFO(get_logger(), "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
  }

private:
  bool   auto_calibrate_{true};
  int    calib_samples_;
  double traj_duration_;

  std::vector<std::string>           joint_names_;
  std::map<std::string, JointConfig> joint_configs_;
  std::map<std::string, double>      ref_angles_;
  std::map<std::string, double>      latest_positions_;

  std::mutex calib_mutex_;
  bool       calibrating_{false};
  int        calib_count_{0};
  std::map<std::string, double> calib_accum_;
  bool       ready_{false};

  rclcpp_action::Client<FollowJT>::SharedPtr action_client_;
  GoalHandle::SharedPtr                      current_gh_{nullptr};

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_debug_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr             srv_calib_;
  rclcpp::TimerBase::SharedPtr                                   send_timer_;
  message_filters::Subscriber<ImuMsg>                            sub_upper_, sub_fore_;
  std::shared_ptr<message_filters::Synchronizer<ApproxSync>>     sync_;

  // ── Send goal (timer callback) ────────────────────────────────────────────
  void send_goal()
  {
    if (!ready_) return;
    if (!action_client_->action_server_is_ready()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
        "Action server not ready");
      return;
    }

    // ── Build trajectory point ────────────────────────────────────────────────
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.velocities.assign(joint_names_.size(), 0.0);
    point.time_from_start = rclcpp::Duration::from_seconds(traj_duration_);
    {
      std::lock_guard<std::mutex> lock(calib_mutex_);
      for (const auto & n : joint_names_)
        point.positions.push_back(latest_positions_[n]);
    }

    // ── Build goal ────────────────────────────────────────────────────────────
    FollowJT::Goal goal;
    goal.trajectory.header.stamp    = now();
    goal.trajectory.header.frame_id = "base_link";
    goal.trajectory.joint_names     = joint_names_;
    goal.trajectory.points.push_back(point);

    // ── Send — controller จะ preempt goal เก่าอัตโนมัติ ─────────────────────
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
        // CANCELED = preempted by next goal — ปกติ ไม่ต้อง log
        // SUCCEEDED = ถึง target ก่อน goal ถัดไปมา
        if (result.code == rclcpp_action::ResultCode::ABORTED) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
            "Goal aborted: %s", result.result->error_string.c_str());
        }
        current_gh_ = nullptr;
      };

    action_client_->async_send_goal(goal, opts);
  }

  // ── Calibration service ───────────────────────────────────────────────────
  void calib_cb(
    const std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr res)
  {
    std::lock_guard<std::mutex> lock(calib_mutex_);
    for (const auto & n : joint_names_) calib_accum_[n] = 0.0;
    calib_count_ = 0;
    calibrating_ = true;
    ready_       = false;
    RCLCPP_INFO(get_logger(), "Re-calibration started — hold arm hanging down...");
    res->success = true;
    res->message = "Calibrating, hold reference pose (arm hanging down)";
  }

  // ── Helpers ───────────────────────────────────────────────────────────────
  static double wrap_pi(double a) {
    while (a >  M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
  }

  static double compute_twist(const tf2::Matrix3x3 & R) {
    tf2::Vector3 bone   = R * tf2::Vector3(1, 0, 0);
    tf2::Vector3 z_sens = R * tf2::Vector3(0, 0, 1);
    tf2::Vector3 z_perp = z_sens - bone.dot(z_sens) * bone;
    if (z_perp.length() < 1e-6) return 0.0;
    z_perp /= z_perp.length();
    tf2::Vector3 ref(0, 1, 0);  // Y_world (IMU Z หมุน 90° รอบ X)
    if (std::abs(bone.dot(ref)) > 0.95) ref = tf2::Vector3(0, 0, 1);
    tf2::Vector3 ref_perp = ref - bone.dot(ref) * bone;
    if (ref_perp.length() < 1e-6) return 0.0;
    ref_perp /= ref_perp.length();
    double sign = (ref_perp.cross(z_perp).dot(bone) >= 0.0) ? 1.0 : -1.0;
    return sign * std::acos(std::clamp(ref_perp.dot(z_perp), -1.0, 1.0));
  }

  std::map<std::string, double> compute_joint_angles(
    const tf2::Quaternion & qu, const tf2::Quaternion & qf)
  {
    tf2::Matrix3x3 Ru(qu), Rf(qf);
    tf2::Vector3 bu = Ru * tf2::Vector3(1, 0, 0);
    tf2::Vector3 bf = Rf * tf2::Vector3(1, 0, 0);
    return {
      {"shoulder_pitch_joint", std::atan2(-bu.z(), bu.x())},
      {"shoulder_roll_joint",  std::asin(std::clamp( bu.y(), -1.0, 1.0))},
      {"upperarm_joint",       compute_twist(Ru)},
      {"elbow_joint",          std::acos(std::clamp(bu.dot(bf), -1.0, 1.0))},
      {"wrist_joint",          compute_twist(Rf)}
    };
  }

  // ── IMU callback (100Hz) — คำนวณและเก็บ latest_positions_ ────────────────
  void imu_cb(
    const ImuMsg::ConstSharedPtr & upper,
    const ImuMsg::ConstSharedPtr & fore)
  {
    tf2::Quaternion qu(upper->orientation.x, upper->orientation.y,
                       upper->orientation.z, upper->orientation.w);
    tf2::Quaternion qf(fore->orientation.x,  fore->orientation.y,
                       fore->orientation.z,  fore->orientation.w);
    qu.normalize(); qf.normalize();

    auto angles = compute_joint_angles(qu, qf);

    std::lock_guard<std::mutex> lock(calib_mutex_);

    // ── Calibration ───────────────────────────────────────────────────────────
    if (calibrating_) {
      for (const auto & n : joint_names_) calib_accum_[n] += angles[n];
      if (++calib_count_ >= calib_samples_) {
        for (const auto & n : joint_names_)
          ref_angles_[n] = calib_accum_[n] / calib_count_;
        calibrating_ = false;
        ready_       = true;
        for (auto & [n, cfg] : joint_configs_) cfg.lpf.reset(0.0);
        RCLCPP_INFO(get_logger(), "━━━ Calibration complete! ━━━");
        for (const auto & n : joint_names_)
          RCLCPP_INFO(get_logger(), "  %-25s ref = %6.3f rad (%5.1f deg)",
            n.c_str(), ref_angles_[n], ref_angles_[n] * 180.0 / M_PI);
        RCLCPP_INFO(get_logger(), "Robot will now follow arm movement.");
      }
      return;
    }

    if (!ready_) return;

    // ── Compute robot positions ───────────────────────────────────────────────
    std_msgs::msg::Float64MultiArray dbg;
    for (const auto & n : joint_names_) {
      double rel = (n == "elbow_joint")
        ? angles[n] - ref_angles_[n]
        : wrap_pi(angles[n] - ref_angles_[n]);

      auto & cfg = joint_configs_.at(n);
      double filtered = cfg.lpf.update(rel * cfg.scale);
      latest_positions_[n] = std::clamp(filtered, cfg.robot_min, cfg.robot_max);

      dbg.data.push_back(rel * 180.0 / M_PI);                      // [0-4] human deg
    }
    for (const auto & n : joint_names_)
      dbg.data.push_back(latest_positions_[n] * 180.0 / M_PI);     // [5-9] robot deg

    pub_debug_->publish(dbg);
    // goal จะถูกส่งโดย send_timer_ ไม่ใช่ที่นี่
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuKinematicsNode>());
  rclcpp::shutdown();
  return 0;
}