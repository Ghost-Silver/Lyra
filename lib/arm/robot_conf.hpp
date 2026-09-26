// lib/arm/robot_conf.hpp — Layer 0 机器人配置
// DH 参数 / 关节限位 / 负载与工具系（重力补偿钩子）/ 伺服与轨迹上限 / 零位。
#pragma once
#include "arm/kinematics.hpp"
#include <array>
#include <string>

namespace arm {

// 末端负载（重力补偿与真机前馈的钩子参数）
struct Payload {
  double mass = 0.0;          // kg
  Vec3 comTool{0, 0, 0};      // 质心在工具系中的位置 (m)
};

// 关节摩擦模型（库仑 + 粘性 + 静摩擦）——sim2real 差距的核心来源之一：
//   τ_f(qd) = b·qd + fc·sgn(qd)；|τ_d| ≤ τs 且 qd≈0 → 静摩擦粘滞锁定（死区）。
// 速度伺服下的稳态跌落 = τ_f/Kv（Kv = velKv）：低速爬行、静摩擦死区、跟踪误差均为真实效应。
struct JointFriction {
  std::array<double, 6> visc{};   // 粘性摩擦 b [N·m·s/rad]
  std::array<double, 6> coul{};   // 库仑摩擦 fc [N·m]
  std::array<double, 6> stic{};   // 最大静摩擦 τs [N·m]
};

struct RobotConf {
  std::string name = "lyra-desktop6";
  ArmModel arm = ArmModel::urStyle();  // DH + 关节限位 + vmax
  Mat4 tool = Mat4::identity();        // T6_tool：法兰 → 工具/TCP
  Payload payload;                     // 负载
  double amax = 4.0;                   // 关节加速度上限 (rad/s²)
  double jmax = 30.0;                  // 关节加加速度上限 (rad/s³)
  double servoTau = 0.010;             // 速度一阶滞后时间常数 (s)
  double posKp = 12.0;                 // 位置伺服增益 (1/s)（配合速度阻尼器临界阻尼）
  std::array<double, 6> home{};        // 零位/收位
  double gripMaxSpeed = 2.0;           // 夹爪开合速率 (1/s)
  JointFriction fric;                  // 关节摩擦（desktop6 默认启用真实参数）
  double velKv = 5.0;                  // 速度伺服刚度 [N·m·s/rad]（摩擦稳态跌落 = τ_f/Kv）

  static RobotConf desktop6() {
    RobotConf c;
    c.home = {0, -0.5, 0.5, 0, 0.5, 0};
    c.payload.mass = 0.15;
    c.payload.comTool = {0, 0, 0.04};
    // 常用工具：指尖向前伸出 0.06 m
    c.tool = Mat4::translation({0, 0, 0.06});
    // 关节摩擦实感参数：近臂身关节大、腕部小（谐波/皮带预紧差异）
    c.fric.visc = {0.35, 0.32, 0.28, 0.12, 0.10, 0.08};
    c.fric.coul = {0.12, 0.10, 0.09, 0.04, 0.03, 0.03};
    c.fric.stic = {0.16, 0.14, 0.12, 0.05, 0.04, 0.04};
    return c;
  }

  // 负载重力补偿力矩（前馈钩子）：τ = J_v(q, com)ᵀ · (m·g_vec)
  // 真机控制律用它做前馈；RL 可作为扰动/特征接口。连杆自重可后续按同样钩子扩展。
  std::array<double, 6> gravityCompTorque(const std::array<double, 6>& q,
                                          double gravity = 9.81) const {
    std::array<double, 6> tau{};
    if (payload.mass <= 0.0) return tau;
    Mat4 T06 = forwardKinematicsT0_6(arm, q);
    Vec3 comW = (T06 * tool).transformPoint(payload.comTool);
    double J[6][6];
    buildJacobian(arm, q, J);
    // 力作用点取 comW：τ_j = (z_j × (com − p_j)) · F
    Vec3 F{0, 0, -payload.mass * gravity};
    auto frames = forwardKinematics(arm, q);
    Vec3 z0{0, 0, 1};
    Vec3 axes[6], pos[6];
    axes[0] = z0; pos[0] = frames[0].translationV();
    for (int i = 1; i < 6; i++) {
      axes[i] = frames[i].transformDir(z0);
      pos[i] = frames[i].translationV();
    }
    for (int j = 0; j < 6; j++) {
      Vec3 lv = axes[j].cross(comW - pos[j]);
      tau[j] = lv.dot(F);
    }
    return tau;
  }
};

}  // namespace arm
