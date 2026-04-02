#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>        // เพิ่ม: สำหรับ sub_joint_states_
#include <std_msgs/msg/bool.hpp>                  // เพิ่ม: สำหรับ sub_resync_
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include "joint_constants.hpp"
#include "low_pass_filter.hpp"

#include <array>
#include <atomic>
#include <mutex>
#include <string>

using ImuMsg     = sensor_msgs::msg::Imu;
using FollowJT   = control_msgs::action::FollowJointTrajectory;
using GoalHandle = rclcpp_action::ClientGoalHandle<FollowJT>;

class ImuKinematicsNode : public rclcpp::Node
{
public:
  explicit ImuKinematicsNode();

private:
  // ── parameters ──────────────────────────────────────────────
  bool        auto_calibrate_{true};
  bool        use_sim_time_{false};
  int         calib_samples_;
  double      traj_duration_;
  double      imu_timeout_s_;
  double      max_jump_rad_;
  std::string action_name_;

  // ── joint state ─────────────────────────────────────────────
  std::array<double, NUM_JOINTS>        soft_min_, soft_max_;
  std::array<LowPassFilter, NUM_JOINTS> lpf_;
  std::array<double, NUM_JOINTS>        latest_pos_, prev_pos_;

  // ── IMU state ────────────────────────────────────────────────
  tf2::Quaternion qu_latest_, qf_latest_;
  std::mutex      imu_mutex_;
  rclcpp::Time    last_upper_time_, last_fore_time_;
  bool            imu_upper_received_{false};
  bool            imu_fore_received_{false};

  // ── calibration ──────────────────────────────────────────────
  std::mutex     calib_mutex_;
  bool           calibrating_{false};
  int            calib_count_{0};
  double         calib_elbow_accum_{0.0};
  bool           ready_{false};
  double         ref_elbow_{0.0};
  tf2::Matrix3x3 R_calib_upper_, R_calib_fore_;
  tf2::Matrix3x3 calib_R_upper_accum_, calib_R_fore_accum_;

  // ── safety ───────────────────────────────────────────────────
  std::atomic<bool> estop_{false};

  // ── recovery ─────────────────────────────────────────────────
  std::mutex                     sync_mutex_;
  std::array<double, NUM_JOINTS> synced_pos_{};     // ตำแหน่งจริงจาก feedback
  bool                           pending_resync_{false};
  bool                           joint_states_received_{false}; // เพิ่ม: guard ป้องกัน sync ค่า zero

  // ── ROS2 interfaces ──────────────────────────────────────────
  rclcpp_action::Client<FollowJT>::SharedPtr                     action_client_;
  GoalHandle::SharedPtr                                          current_gh_{nullptr};
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_debug_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr             srv_calib_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr             srv_estop_;
  rclcpp::TimerBase::SharedPtr                                   send_timer_;
  rclcpp::Subscription<ImuMsg>::SharedPtr                        sub_upper_, sub_fore_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr  sub_joint_states_; // เพิ่ม
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr           sub_resync_;        // เพิ่ม

  // ── callbacks ────────────────────────────────────────────────
  void upper_cb(const ImuMsg::ConstSharedPtr & msg);
  void fore_cb(const ImuMsg::ConstSharedPtr & msg);
  void timer_cb();
  void send_goal_now();

  void calib_cb(
    const std_srvs::srv::Trigger::Request::SharedPtr  req,
    std_srvs::srv::Trigger::Response::SharedPtr       res);

  void estop_cb(
    const std_srvs::srv::SetBool::Request::SharedPtr  req,
    std_srvs::srv::SetBool::Response::SharedPtr       res);
};