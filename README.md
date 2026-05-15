# HDT Arm — Humanoid Digital Twin Arm

> **Senior Project** | ROS2 Humble · Ubuntu 22.04 · Gazebo Ignition Fortress 6.x  
> ระบบ Teleoperation แขนหุ่นยนต์ 5-DOF ด้วย IMU พร้อม Digital Twin

---

## ภาพรวม / Overview

**ภาษาไทย**  
HDT Arm เป็นระบบควบคุมแขนหุ่นยนต์ 5-DOF แบบ Teleoperation โดยใช้ IMU sensor (BNO086) ที่ติดบนแขนผู้ใช้ 2 ตัว คือที่ต้นแขน (upperarm) และปลายแขน (forearm) ระบบคำนวณมุม joint ด้วย Swing-Twist Decomposition แล้วส่งคำสั่งไปยังหุ่นยนต์จริง (Dynamixel MX-series) ผ่าน ROS2 Control พร้อมสร้าง Digital Twin ใน Gazebo Ignition Fortress เพื่อ visualize และ monitor การเคลื่อนไหวแบบ real-time

**English**  
HDT Arm is a 5-DOF robot arm teleoperation system driven by two BNO086 IMU sensors worn on the operator's upper arm and forearm. Joint angles are computed via Swing-Twist Decomposition with Low-Pass Filtering and sent to real Dynamixel MX-series hardware through ROS2 Control. A Gazebo Ignition Fortress Digital Twin mirrors every movement in real-time for visualization and accuracy monitoring.

---

## System Architecture / สถาปัตยกรรมระบบ

```
┌──────────────────────────────────────────────────────────────────┐
│  OPERATOR                                                        │
│  IMU upperarm ──┐                                                │
│  IMU forearm  ──┴──► imu_kinematics_node                         │
│                       ├─ Swing-Twist Decomposition               │
│                       ├─ Low-Pass Filter (α=0.3)                 │
│                       ├─ Soft Clamp + Jump Limit                 │
│                       └─ /real/arm_controller/follow_joint_traj  │
│                                     │                            │
│                          ┌──────────▼──────────┐                │
│                          │    REAL ROBOT        │                │
│                          │  Dynamixel MX-106×4  │                │
│                          │  Dynamixel MX-64×1   │                │
│                          └──────────┬──────────┘                │
│                                     │ /real/joint_states         │
│                          ┌──────────▼──────────┐                │
│                          │  joint_state_bridge  │                │
│                          └──────────┬──────────┘                │
│                                     │ /sim/arm_controller/...    │
│                          ┌──────────▼──────────┐                │
│                          │   GAZEBO SIM         │                │
│                          │  (Digital Twin)      │                │
│                          └─────────────────────┘                │
│                                                                  │
│  joint_accuracy_monitor_node                                     │
│  ├─ /accuracy/real_vs_sim      (error deg)                       │
│  ├─ /accuracy/human_vs_robot   (error deg)                       │
│  ├─ /accuracy/joint_positions  (real/sim/human rad)              │
│  ├─ /imu/upperarm/rpy          (roll/pitch/yaw deg)              │
│  └─ /imu/forearm/rpy           (roll/pitch/yaw deg)              │
└──────────────────────────────────────────────────────────────────┘
```

---

## Hardware / ฮาร์ดแวร์

| Component | Spec |
|---|---|
| Robot Arm | 5-DOF — Dynamixel MX-106 × 4, MX-64 × 1 |
| IMU Sensor | BNO086 × 2 (upperarm, forearm) — UDP |
| Power Supply | 12V / 20A (≥ 240W, safety margin 25%) |
| Wiring | AWG 16+, Fuse 22A, Bulk Cap 1000–4700µF |
| Compute | Ubuntu 22.04 LTS, ROS2 Humble |

---

## Software Packages / แพ็กเกจซอฟต์แวร์

```
hdt_arm_ws/src/
├── hdt_arm_description/          # URDF/xacro, Gazebo world, controller config
│   ├── urdf/
│   │   ├── hdt_arm.urdf.xacro
│   │   └── hdt_arm.ros2_control.xacro
│   ├── config/
│   │   ├── ros2_controllers_real.yaml   # real: update_rate 20Hz
│   │   ├── ros2_controllers_sim.yaml    # sim:  update_rate 1000Hz
│   │   ├── joint_limits.yaml
│   │   └── home_position_params.yaml
│   └── worlds/default.sdf
│
├── hdt_arm_bringup/              # Launch files
│   ├── hdt_arm_real.launch.py        # รันหุ่นยนต์จริง
│   ├── hdt_arm_sim.launch.py         # รัน Gazebo Digital Twin
│   └── hdt_arm_test.launch.py        # ทดสอบการ teleop จาก imu
│
├── hdt_arm_teleop/               # Control & monitor nodes
│   ├── src/
│   │   ├── imu_kinematics_node.cpp       # IMU → joint angles → real robot
│   │   ├── joint_state_bridge.cpp        # real joint_states → sim mirror
│   │   ├── joint_accuracy_monitor_node.cpp # accuracy + IMU RPY publisher
│   │   ├── home_position.cpp
│   │   └── hardware_recovery_node.cpp
│   └── include/
│       ├── joint_constants.hpp           # joint names, limits, scales
│       ├── kinematics_math.hpp
│       └── low_pass_filter.hpp
│
├── bno086_broadcaster/           # IMU driver (UDP)
│   └── src/
│       ├── bno086_udp_publisher.cpp # เชื่อมต่อผ่าน WiFi
│       └── bno086_serial_publisher.cpp # เชื่อมต่อผ่านสาย USB (testing)
│
└── dynamixel_hardware_interface/ # ROS2 Control hardware plugin
```

---

## Joint Configuration / การตั้งค่า Joint

| ID|      Joint Name      | Lower (rad) | Upper (rad) | Vel Limit (rad/s) |
|---|----------------------|-------------|-------------|-------------------|
| 0 | shoulder_pitch_joint | -1.57       | 3.142       | 2.5               |
| 1 | shoulder_roll_joint  | -0.1        | 2.967       | 2.5               |
| 2 | upperarm_joint       | -1.57       | 1.57        | 3.0               |
| 3 | elbow_joint          | -1.9        | 0.0         | 2.5               |
| 4 | wrist_joint          | -1.57       | 1.57        | 3.5               |

> Soft limit margin: 10° จาก hard limit ทุก joint  
> Joint scale: shoulder_roll และ elbow ใช้ `-1.0` (invert direction)

---

## Dependencies / การติดตั้ง

```bash
# ROS2 Humble
sudo apt install ros-humble-desktop

# Gazebo Ignition Fortress (official pair กับ ROS2 Humble)
sudo apt install ignition-fortress
sudo apt install ros-humble-ros-ign-bridge
sudo apt install ros-humble-ros-ign-gazebo
sudo apt install ros-humble-gz-ros2-control

# ROS2 Control
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers

# TF2
sudo apt install ros-humble-tf2-ros

# Visualization & Monitoring
sudo apt install ros-humble-rqt ros-humble-rqt-plot
sudo apt install ros-humble-plotjuggler-ros   # แนะนำสำหรับ multi-panel

# rosdep
rosdep update
```

---

## Build / การ Build

```bash
cd ~/hdt_arm_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

---

## Usage / วิธีใช้งาน

### 1. Real Robot Only / หุ่นยนต์จริงอย่างเดียว

```bash
ros2 launch hdt_arm_bringup hdt_arm_real.launch.py
```

|      Argument      |     Default    |         Description       |
|--------------------|----------------|---------------------------|
| `port_name`        | `/dev/ttyUSB0` | Serial port ของ Dynamixel |
| `start_rviz`       | `true`         | เปิด RViz                  |
| `init_position`    | `true`         | ส่ง home position ตอนเปิด  |
| `start_imu_teleop` | `false`        | เปิด IMU teleoperation     |

### 2. IMU Teleoperation / ควบคุมด้วย IMU

```bash
# Terminal 1 — รัน IMU Driver ก่อน
ros2 run bno086_broadcaster bno086_udp_publisher

# Terminal 2 — รัน Real Robot + imu teleop -> ก่อนรันให้ถือแขนห้อยลงตรงๆ สำหรับ auto-calibration
ros2 launch hdt_arm_bringup hdt_arm_real.launch.py start_imu_teleop:=true
```

### 3. Digital Twin Mode / โหมด Digital Twin

```bash
# Terminal 3 — รัน Gazebo + bridge
ros2 launch hdt_arm_bringup hdt_arm_sim.launch.py start_bridge:=true
```

### 4. Test Simulation Only / Gazebo อย่างเดียว

```bash
ros2 launch hdt_arm_bringup hdt_arm_test.launch.py start_imu_teleop:=true
```

### 5. Accuracy Monitoring / วัดความแม่นยำ

```bash
# รัน monitor node
ros2 run hdt_arm_teleop joint_accuracy_monitor_node

# เทียบ real vs sim vs human — ทีละ joint
rqt_plot /accuracy/joint_positions/data[0]:data[5]:data[10]   # shoulder_pitch
rqt_plot /accuracy/joint_positions/data[1]:data[6]:data[11]   # shoulder_roll
rqt_plot /accuracy/joint_positions/data[2]:data[7]:data[12]   # upperarm
rqt_plot /accuracy/joint_positions/data[3]:data[8]:data[13]   # elbow
rqt_plot /accuracy/joint_positions/data[4]:data[9]:data[14]   # wrist

# tracking error ทุก joint
rqt_plot /accuracy/human_vs_robot/data[0]:data[1]:data[2]:data[3]:data[4]

# Digital Twin sync error
rqt_plot /accuracy/real_vs_sim/data[0]:data[1]:data[2]:data[3]:data[4]

# IMU raw RPY (องศา) — ก่อนคำนวณ kinematics
rqt_plot /imu/upperarm/rpy/data[0]:data[1]:data[2]   # roll/pitch/yaw
rqt_plot /imu/forearm/rpy/data[0]:data[1]:data[2]
```

---

## ROS2 Topics / Topics ที่สำคัญ

| Topic | Type | Description |
|---|---|---|
| `/imu/upperarm` | `sensor_msgs/Imu` | IMU ต้นแขน (RELIABLE QoS) |
| `/imu/forearm` | `sensor_msgs/Imu` | IMU ปลายแขน (RELIABLE QoS) |
| `/imu/upperarm/rpy` | `Float64MultiArray` | [roll, pitch, yaw] (deg) |
| `/imu/forearm/rpy` | `Float64MultiArray` | [roll, pitch, yaw] (deg) |
| `/imu_kinematics/joint_angles` | `Float64MultiArray` | [0-4]=raw, [5-9]=filtered (deg) |
| `/real/joint_states` | `sensor_msgs/JointState` | สถานะ joint หุ่นจริง |
| `/sim/joint_states` | `sensor_msgs/JointState` | สถานะ joint ใน Gazebo |
| `/accuracy/joint_positions` | `Float64MultiArray` | [0-4]=real, [5-9]=sim, [10-14]=human (rad) |
| `/accuracy/real_vs_sim` | `Float64MultiArray` | Error real−sim ทุก joint (deg) |
| `/accuracy/human_vs_robot` | `Float64MultiArray` | Error human−robot ทุก joint (deg) |

---

## Services / Services ที่มี

```bash
# Calibrate IMU — ถือแขนห้อยลงตรงๆ ก่อนเรียก
ros2 service call /imu_kinematics/calibrate std_srvs/srv/Trigger

# Emergency Stop
ros2 service call /imu_kinematics/estop std_srvs/srv/SetBool "{data: true}"

# Release E-Stop
ros2 service call /imu_kinematics/estop std_srvs/srv/SetBool "{data: false}"
```

---

## Key Parameters / Parameters ที่ปรับได้

```yaml
imu_kinematics_node:
  auto_calibrate:        true    # calibrate อัตโนมัติตอนเปิด
  calibration_samples:   50      # จำนวน frame สำหรับ calibrate
  lpf_alpha:             0.3     # Low-pass filter (0=smooth, 1=raw)
  send_hz:               10.0    # ความถี่ส่ง command (Hz)
  trajectory_duration:   0.07    # ระยะเวลา trajectory (s)
  imu_timeout_ms:        200.0   # Watchdog timeout (ms)
  max_jump_deg:          45.0    # Jump limit ต่อ cycle (deg)
  soft_limit_margin_deg: 10.0    # Margin จาก hard limit (deg)
  action_name:           /real/arm_controller/follow_joint_trajectory
```

---

## Recorded Test Data / ข้อมูลการทดสอบ

ข้อมูล rosbag2 ที่บันทึกจากการทดสอบจริง บันทึก topic `/accuracy/joint_positions`:
> เล่น rosbag: `ros2 bag play src/<teat_folder>/<joint_folder>/`
> ตัวอย่าง: `ros2 bag play src/test_1/shoulder_pitch/`

---

## Troubleshooting / แก้ปัญหา

**Sim ไม่ขยับตาม real**
```bash
ros2 node list | grep bridge        # ต้องเห็น /joint_state_bridge
ros2 topic hz /real/joint_states    # ต้องได้ ~20 Hz
ros2 topic hz /sim/arm_controller/joint_trajectory  # ต้องได้ ~20 Hz
```

**IMU ไม่ได้รับข้อมูล (QoS mismatch)**
```bash
# ตรวจสอบ QoS ของ publisher
ros2 topic info /imu/upperarm --verbose
# udp_imu_node ใช้ RELIABLE → subscriber ต้องใช้ QoS(10).reliable()
```

**E-Stop ติดค้าง**
```bash
ros2 service call /imu_kinematics/estop std_srvs/srv/SetBool "{data: false}"
```

**Calibration ผิดพลาด**
```bash
# recalibrate — ถือแขนห้อยลงตรงๆ ก่อน
ros2 service call /imu_kinematics/calibrate std_srvs/srv/Trigger
```

**Build error: tf2/LinearMath/Quaternion.h not found**
```cmake
# CMakeLists.txt — ต้องใส่ tf2 ใน ament_target_dependencies
ament_target_dependencies(joint_accuracy_monitor_node
  rclcpp sensor_msgs std_msgs tf2)
```
## Demo / ตัวอย่างการใช้งาน

| Description | Link |
|---|---|
| HDT ARM Demo | [YouTube](https://www.youtube.com/watch?v=avqWdkfJH2M&t=41s) |
---

## License

MIT License — feel free to use and modify with attribution.
