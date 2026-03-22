#pragma once
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>
#include <cmath>
#include <algorithm>

namespace kinematics {

/**
 * mat_add / mat_div — helper สำหรับ accumulate rotation matrix ตอน calibrate
 */
inline tf2::Matrix3x3 mat_add(const tf2::Matrix3x3 & A, const tf2::Matrix3x3 & B)
{
  tf2::Matrix3x3 C;
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      C[i][j] = A[i][j] + B[i][j];
  return C;
}

inline tf2::Matrix3x3 mat_div(const tf2::Matrix3x3 & A, double s)
{
  tf2::Matrix3x3 C;
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      C[i][j] = A[i][j] / s;
  return C;
}

/**
 * compute_swing_pitch_roll
 *
 * แยก swing ของ bone vector ออกเป็น pitch และ roll
 * โดย project มุม swing ลงบน z-axis (pitch) และ y-axis (roll)
 * ทำงานใน calibration frame (R_calib.T × R_now)
 */
inline void compute_swing_pitch_roll(
  const tf2::Matrix3x3 & R_now,
  const tf2::Matrix3x3 & R_calib,
  double & pitch,
  double & roll)
{
  const tf2::Vector3 bu_c = R_calib.transpose() * (R_now * tf2::Vector3(1, 0, 0));
  const tf2::Vector3 home(1.0, 0.0, 0.0);

  tf2::Vector3 sa = home.cross(bu_c);
  const double sa_len = sa.length();

  if (sa_len < 1e-6) {
    pitch = 0.0;
    roll  = 0.0;
    return;
  }

  sa /= sa_len;
  const double angle = std::acos(std::clamp(home.dot(bu_c), -1.0, 1.0));
  pitch = angle * sa.z();
  roll  = angle * sa.y();
}

/**
 * compute_twist_swing
 *
 * คำนวณ twist รอบแกน bone เอง โดย:
 *   1. หา Rs = rotation ที่พา calib bone ไปยัง current bone (swing only)
 *   2. Rt = Rs.T × R_now → เหลือแต่ twist component
 *   3. อ่านค่า twist ด้วย atan2 จาก row ที่ 2 ของ Rt
 */
inline double compute_twist_swing(
  const tf2::Matrix3x3 & R_now,
  const tf2::Matrix3x3 & R_calib)
{
  const tf2::Vector3 bc = R_calib * tf2::Vector3(1, 0, 0);
  const tf2::Vector3 bn = R_now   * tf2::Vector3(1, 0, 0);

  tf2::Vector3 sa    = bc.cross(bn);
  const double sa_len = sa.length();
  const double cos_a  = std::clamp(bc.dot(bn), -1.0, 1.0);

  tf2::Matrix3x3 Rs;

  if (sa_len < 1e-6) {
    // bone ขนาน → ไม่มี swing axis → ใช้ rotation 0 หรือ 180 องศา
    if (cos_a > 0.0) {
      Rs = R_calib;
    } else {
      tf2::Vector3 ax = std::abs(bc.x()) < 0.9 ?
        tf2::Vector3(1, 0, 0) : tf2::Vector3(0, 1, 0);
      tf2::Vector3 a = bc.cross(ax);
      a /= a.length();
      Rs = tf2::Matrix3x3(
        2*a.x()*a.x()-1, 2*a.x()*a.y(),   2*a.x()*a.z(),
        2*a.y()*a.x(),   2*a.y()*a.y()-1, 2*a.y()*a.z(),
        2*a.z()*a.x(),   2*a.z()*a.y(),   2*a.z()*a.z()-1) * R_calib;
    }
  } else {
    sa /= sa_len;
    const double ang = std::atan2(sa_len, cos_a);
    const double s = std::sin(ang), c = std::cos(ang), t = 1.0 - c;
    const double x = sa.x(), y = sa.y(), z = sa.z();
    Rs = tf2::Matrix3x3(
      t*x*x+c,   t*x*y-s*z, t*x*z+s*y,
      t*x*y+s*z, t*y*y+c,   t*y*z-s*x,
      t*x*z-s*y, t*y*z+s*x, t*z*z+c) * R_calib;
  }

  const tf2::Matrix3x3 Rt = Rs.transpose() * R_now;
  return std::atan2(Rt[2][1], Rt[2][2]);
}

}  // namespace kinematics
