#pragma once
#include <array>

enum JIdx { PITCH = 0, ROLL = 1, TWIST = 2, ELBOW = 3, WRIST = 4, NUM_JOINTS = 5 };

static const std::array<const char*, NUM_JOINTS> JOINT_NAMES = {
  "shoulder_pitch_joint",
  "shoulder_roll_joint",
  "upperarm_joint",
  "elbow_joint",
  "wrist_joint"
};

static const std::array<double, NUM_JOINTS> VEL_LIMITS  = { 2.5, 2.5, 3.0, 2.5, 3.5 };
static const std::array<double, NUM_JOINTS> HARD_LOWER  = { -1.57, -0.1,  -1.57, -1.9,  -1.57 };
static const std::array<double, NUM_JOINTS> HARD_UPPER  = {  3.142, 2.967,  1.57,  0.0,   1.57 };
static const std::array<double, NUM_JOINTS> JOINT_SCALE = {  1.0,  -1.0,   1.0,  -1.0,   1.0  };
