// lib/arm/dynamics.hpp — 刚体动力学 (RNEA)
#pragma once
#include "arm/kinematics.hpp"
#include <array>

namespace arm {

// Recursive Newton-Euler Algorithm (RNEA) 计算逆动力学力矩
// 返回的力矩符号：tau 为驱动该状态所需的关节前馈力矩（为了抵消重力，a_prev = +9.81 向上？
// 如果 gravity=(0,0,-9.81)，基座有一个向上 (0,0,9.81) 的假想加速度，这样可以算出对抗重力的所需力矩
// 所以 a_prev = -gravity = (0,0,9.81)）
inline std::array<double, 6> inverseDynamics(
    const ArmModel& arm,
    const std::array<double, 6>& q,
    const std::array<double, 6>& qd,
    const std::array<double, 6>& qdd,
    const Vec3& gravity = {0, 0, -9.81}) {

  std::array<double, 6> tau{};
  // forwardKinematics 返回长度为 7 的 vector：T_0^0, T_0^1, ..., T_0^6
  auto t_frames = forwardKinematics(arm, q);
  Mat4 frames[6];
  for(int i=0; i<6; i++) frames[i] = t_frames[i+1];

  Vec3 z0{0, 0, 1};
  Vec3 axes[6], pos[6];
  axes[0] = z0; pos[0] = t_frames[0].translationV();
  for (int i = 1; i < 6; i++) {
    axes[i] = t_frames[i].transformDir(z0);
    pos[i] = t_frames[i].translationV();
  }

  Vec3 omega[6];
  Vec3 omega_dot[6];
  Vec3 a[6];
  Vec3 a_c[6];

  Vec3 omega_prev = {0,0,0};
  Vec3 omega_dot_prev = {0,0,0};
  Vec3 a_prev = gravity * (-1.0);
  Vec3 pos_prev = {0,0,0};

  for (int i = 0; i < 6; i++) {
    Vec3 joint_axis = axes[i];

    omega[i] = omega_prev + joint_axis * qd[i];
    omega_dot[i] = omega_dot_prev + joint_axis * qdd[i] + omega_prev.cross(joint_axis * qd[i]);

    Vec3 r = pos[i] - pos_prev;
    a[i] = a_prev + omega_dot[i].cross(r) + omega[i].cross(omega[i].cross(r));

    Vec3 com_W = frames[i].transformPoint(arm.dyna[i].com);
    Vec3 r_c = com_W - pos[i];
    a_c[i] = a[i] + omega_dot[i].cross(r_c) + omega[i].cross(omega[i].cross(r_c));

    omega_prev = omega[i];
    omega_dot_prev = omega_dot[i];
    a_prev = a[i];
    pos_prev = pos[i];
  }

  Vec3 f_next{0,0,0};
  Vec3 n_next{0,0,0};

  for (int i = 5; i >= 0; i--) {
    const auto& dyna = arm.dyna[i];
    Vec3 F_i = a_c[i] * dyna.m;

    Mat4 R = frames[i];
    Mat4 RT;
    RT.m[0] = R.m[0]; RT.m[1] = R.m[4]; RT.m[2] = R.m[8];
    RT.m[4] = R.m[1]; RT.m[5] = R.m[5]; RT.m[6] = R.m[9];
    RT.m[8] = R.m[2]; RT.m[9] = R.m[6]; RT.m[10] = R.m[10];

    auto multI = [&](const Vec3& v) {
      Vec3 v_L = RT.transformDir(v);
      Vec3 iv_L = {dyna.Ixx * v_L.x, dyna.Iyy * v_L.y, dyna.Izz * v_L.z};
      return R.transformDir(iv_L);
    };

    Vec3 N_i = multI(omega_dot[i]) + omega[i].cross(multI(omega[i]));

    Vec3 f_i = F_i + f_next;

    Vec3 com_W = frames[i].transformPoint(dyna.com);
    Vec3 r_com = com_W - pos[i];
    Vec3 r_next = (i < 5 ? pos[i+1] : pos[i]) - pos[i];

    Vec3 n_i = N_i + n_next + r_com.cross(F_i) + r_next.cross(f_next);

    // Gravity acts downwards (-Z). RNEA F_i points upwards (+Z).
    // The torque computed by n_i is the torque EXERTED BY joint i ON link i.
    // If n_i is the torque exerted by the joint, its component along the axis is the joint torque tau.
    // So tau[i] = n_i.dot(axes[i]).
    // In our manual test we found it was inverted relative to gravityCompTorque.
    // We will just use the exact physical RNEA: tau = n_i.dot(axes[i]).
    tau[i] = n_i.dot(axes[i]);

    f_next = f_i;
    n_next = n_i;
  }

  return tau;
}

} // namespace arm
