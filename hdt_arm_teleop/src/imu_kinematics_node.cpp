#include "imu_kinematics_node.hpp"
#include "kinematics_math.hpp"

#include <tf2/LinearMath/Vector3.h>
#include <cmath>
#include <algorithm>

// ── Constructor ──────────────────────────────────────────────────────────────

ImuKinematicsNode::ImuKinematicsNode()
: Node("imu_kinematics_node")
{
  declare_parameter("auto_calibrate",        true);
  declare_parameter("calibration_samples",   50);
  declare_parameter("lpf_alpha",             0.3);
  declare_parameter("send_hz",               10.0);
  declare_parameter("trajectory_duration",   0.07);
  declare_parameter("imu_timeout_ms",        200.0);
  declare_parameter("max_jump_deg",          45.0);
  declare_parameter("soft_limit_margin_deg", 10.0);
  declare_parameter("action_name",
    std::string("/real/arm_controller/follow_joint_trajectory"));

  auto_calibrate_ = get_parameter("auto_calibrate").as_bool();
  calib_samples_  = get_parameter("calibration_samples").as_int();
  traj_duration_  = get_parameter("trajectory_duration").as_double();
  imu_timeout_s_  = get_parameter("imu_timeout_ms").as_double() / 1000.0;
  max_jump_rad_   = get_parameter("max_jump_deg").as_double() * M_PI / 180.0;
  use_sim_time_   = get_parameter("use_sim_time").as_bool();
  action_name_    = get_parameter("action_name").as_string();

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

  const auto clock_type = this->get_clock()->get_clock_type();
  last_upper_time_ = rclcpp::Time(0, 0, clock_type);
  last_fore_time_  = rclcpp::Time(0, 0, clock_type);

  if (auto_calibrate_) {
    calibrating_ = true;
    RCLCPP_INFO(get_logger(),
      "Auto-calibrate ON — collecting %d frames, hold arm hanging down...",
      calib_samples_);
  }

  action_client_ = rclcpp_action::create_client<FollowJT>(this, action_name_);

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
  send_timer_ = create_wall_timer(
    ms, std::bind(&ImuKinematicsNode::timer_cb, this));

  RCLCPP_INFO(get_logger(), "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
  RCLCPP_INFO(get_logger(), "  imu_kinematics_node  [%.0fHz / traj=%.3fs]",
    send_hz, traj_duration_);
  RCLCPP_INFO(get_logger(), "  use_sim_time : %s", use_sim_time_ ? "true" : "false");
  RCLCPP_INFO(get_logger(), "  action       : %s", action_name_.c_str());
  RCLCPP_INFO(get_logger(), "  watchdog     : %.0f ms", imu_timeout_s_ * 1000);
  RCLCPP_INFO(get_logger(), "  jump limit   : %.1f deg", max_jump_rad_ * 180 / M_PI);
  RCLCPP_INFO(get_logger(), "  soft margin  : %.1f deg", margin * 180 / M_PI);
  RCLCPP_INFO(get_logger(), "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
}

// ── IMU callbacks ────────────────────────────────────────────────────────────

void ImuKinematicsNode::upper_cb(const ImuMsg::ConstSharedPtr & msg)
{
  tf2::Quaternion q(msg->orientation.x, msg->orientation.y,
                    msg->orientation.z, msg->orientation.w);
  q.normalize();
  std::lock_guard<std::mutex> lock(imu_mutex_);
  qu_latest_          = q;
  last_upper_time_    = now();
  imu_upper_received_ = true;
}

void ImuKinematicsNode::fore_cb(const ImuMsg::ConstSharedPtr & msg)
{
  tf2::Quaternion q(msg->orientation.x, msg->orientation.y,
                    msg->orientation.z, msg->orientation.w);
  q.normalize();
  std::lock_guard<std::mutex> lock(imu_mutex_);
  qf_latest_         = q;
  last_fore_time_    = now();
  imu_fore_received_ = true;
}

// ── Service callbacks ────────────────────────────────────────────────────────

void ImuKinematicsNode::estop_cb(
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

void ImuKinematicsNode::calib_cb(
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

// ── Main timer loop ──────────────────────────────────────────────────────────

void ImuKinematicsNode::timer_cb()
{
  if (estop_.load()) return;

  // 1. copy IMU state under lock
  tf2::Quaternion qu, qf;
  rclcpp::Time t_upper, t_fore;
  {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    qu      = qu_latest_;
    qf      = qf_latest_;
    t_upper = last_upper_time_;
    t_fore  = last_fore_time_;
  }

  // 2. watchdog
  if (imu_upper_received_ && imu_fore_received_) {
    const double age_u = (now() - t_upper).seconds();
    const double age_f = (now() - t_fore).seconds();
    if (age_u > imu_timeout_s_ || age_f > imu_timeout_s_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
        "IMU timeout! upper=%.1fms fore=%.1fms — holding position",
        age_u * 1000, age_f * 1000);
      return;
    }
  }

  // 3. calibration accumulation
  {
    std::lock_guard<std::mutex> lock(calib_mutex_);
    if (calibrating_) {
      const tf2::Matrix3x3 Ru(qu), Rf(qf);
      const tf2::Vector3 bu = Ru * tf2::Vector3(1, 0, 0);
      const tf2::Vector3 bf = Rf * tf2::Vector3(1, 0, 0);

      calib_R_upper_accum_ = kinematics::mat_add(calib_R_upper_accum_, Ru);
      calib_R_fore_accum_  = kinematics::mat_add(calib_R_fore_accum_,  Rf);
      calib_elbow_accum_  += std::acos(std::clamp(bu.dot(bf), -1.0, 1.0));

      if (++calib_count_ >= calib_samples_) {
        R_calib_upper_ = kinematics::mat_div(calib_R_upper_accum_, calib_count_);
        R_calib_fore_  = kinematics::mat_div(calib_R_fore_accum_,  calib_count_);
        ref_elbow_     = calib_elbow_accum_ / calib_count_;
        calibrating_   = false;
        ready_         = true;
        for (auto & f : lpf_) f.reset(0.0);
        prev_pos_.fill(0.0);
        RCLCPP_INFO(get_logger(), "━━━ Calibration complete! ━━━");
        RCLCPP_INFO(get_logger(), "  elbow ref = %.3f rad (%.1f deg)",
          ref_elbow_, ref_elbow_ * 180 / M_PI);
      }
      return;
    }
    if (!ready_) return;
  }

  // 4. swing-twist decomposition
  const tf2::Matrix3x3 Ru(qu), Rf(qf);
  const tf2::Vector3 bu = Ru * tf2::Vector3(1, 0, 0);
  const tf2::Vector3 bf = Rf * tf2::Vector3(1, 0, 0);

  double pitch = 0.0, roll = 0.0;
  kinematics::compute_swing_pitch_roll(Ru, R_calib_upper_, pitch, roll);

  const std::array<double, NUM_JOINTS> angles = {{
    pitch,
    roll,
    kinematics::compute_twist_swing(Ru, R_calib_upper_),
    std::acos(std::clamp(bu.dot(bf), -1.0, 1.0)) - ref_elbow_,
    kinematics::compute_twist_swing(Rf, R_calib_fore_)
  }};

  // 5. LPF + soft clamp
  std::array<double, NUM_JOINTS> new_pos;
  for (int i = 0; i < NUM_JOINTS; i++) {
    const double filtered = lpf_[i].update(angles[i] * JOINT_SCALE[i]);
    new_pos[i] = std::clamp(filtered, soft_min_[i], soft_max_[i]);
  }

  // 6. jump limit
  for (int i = 0; i < NUM_JOINTS; i++) {
    const double delta = new_pos[i] - latest_pos_[i];
    if (std::abs(delta) > max_jump_rad_) {
      new_pos[i] = latest_pos_[i] + std::copysign(max_jump_rad_, delta);
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 500,
        "Rate limit joint %d: %.1f->%.1f deg",
        i, latest_pos_[i] * 180 / M_PI, new_pos[i] * 180 / M_PI);
    }
  }

  // 7. publish debug
  std_msgs::msg::Float64MultiArray dbg;
  dbg.data.resize(NUM_JOINTS * 2);
  for (int i = 0; i < NUM_JOINTS; i++) {
    dbg.data[i]              = angles[i]  * 180.0 / M_PI;
    dbg.data[i + NUM_JOINTS] = new_pos[i] * 180.0 / M_PI;
  }
  pub_debug_->publish(dbg);

  prev_pos_   = latest_pos_;
  latest_pos_ = new_pos;
  send_goal_now();
}

// ── Action goal ──────────────────────────────────────────────────────────────

void ImuKinematicsNode::send_goal_now()
{
  if (!action_client_->action_server_is_ready()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "Action server not ready");
    return;
  }

  // ถ้า goal เดิมยังรันอยู่ให้ข้ามไป ไม่ cancel ไม่ส่งซ้อน
  if (current_gh_) {
    const auto status = current_gh_->get_status();
    if (status == rclcpp_action::GoalStatus::STATUS_ACCEPTED ||
        status == rclcpp_action::GoalStatus::STATUS_EXECUTING) {
      return;
    }
    current_gh_ = nullptr;
  }

  // build trajectory point
  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions.assign(latest_pos_.begin(), latest_pos_.end());
  point.time_from_start = rclcpp::Duration::from_seconds(traj_duration_);

  point.velocities.resize(NUM_JOINTS);
  for (int i = 0; i < NUM_JOINTS; i++) {
    double vel = (latest_pos_[i] - prev_pos_[i]) / traj_duration_;
    point.velocities[i] = std::clamp(vel, -VEL_LIMITS[i], VEL_LIMITS[i]);
  }

  // build goal
  FollowJT::Goal goal;
  if (use_sim_time_) {
    goal.trajectory.header.stamp = this->now();
  } else {
    goal.trajectory.header.stamp =
      rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
  }
  goal.trajectory.header.frame_id = "base_link";
  goal.trajectory.joint_names.assign(JOINT_NAMES.begin(), JOINT_NAMES.end());
  goal.trajectory.points.push_back(point);

  // send
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
