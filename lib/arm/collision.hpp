// lib/arm/collision.hpp — 3D 碰撞检测与防碰撞
#pragma once
#include "arm/kinematics.hpp"
#include <cmath>
#include <algorithm>

namespace arm {

// 计算两个线段之间的最短距离 (用于胶囊体碰撞)
// 线段1: P1 到 Q1, 线段2: P2 到 Q2
// 核心思想：求解两条射线的公垂线或端点投影
inline double segmentSegmentDistance(const Vec3& p1, const Vec3& q1,
                                     const Vec3& p2, const Vec3& q2) {
  Vec3 d1 = q1 - p1;
  Vec3 d2 = q2 - p2;
  Vec3 r = p1 - p2;
  double a = d1.dot(d1); // 线段1长度平方
  double e = d2.dot(d2); // 线段2长度平方
  double f = d2.dot(r);
  double c = d1.dot(r);

  double s = 0.0, t = 0.0;
  double b = d1.dot(d2);
  double denom = a * e - b * b; // 0 if parallel

  if (denom > 1e-8) {
      s = std::clamp((b * f - c * e) / denom, 0.0, 1.0);
  } else {
      s = 0.0; // 平行时，随便取一点，通常取端点
  }

  // 计算线段2上的参数 t
  if (e != 0.0) {
      t = (b * s + f) / e;
  } else {
      t = 0.0;
  }

  // 处理 t 超出线段范围的情况 (或者 e==0 即线段2退化为点的情况)
  if (t <= 0.0) {
      t = 0.0;
      if (a != 0.0) s = std::clamp(-c / a, 0.0, 1.0);
      else s = 0.0;
  } else if (t >= 1.0) {
      t = 1.0;
      if (a != 0.0) s = std::clamp((b - c) / a, 0.0, 1.0);
      else s = 0.0;
  }

  Vec3 c1 = p1 + d1 * s;
  Vec3 c2 = p2 + d2 * t;
  return (c1 - c2).norm();
}

// 胶囊体定义
struct Capsule {
  Vec3 p1, p2;
  double radius;
};

// 球体定义
struct Sphere {
  Vec3 center;
  double radius;
};

// 检测胶囊体与胶囊体是否碰撞 (带安全余量)
inline bool checkCapsuleCollision(const Capsule& c1, const Capsule& c2, double margin = 0.0) {
  double dist = segmentSegmentDistance(c1.p1, c1.p2, c2.p1, c2.p2);
  return dist <= (c1.radius + c2.radius + margin);
}

// 检测胶囊体与球体是否碰撞
inline bool checkCapsuleSphereCollision(const Capsule& c, const Sphere& s, double margin = 0.0) {
  // 球心与线段的距离，相当于 segmentSegmentDistance 退化
  double dist = segmentSegmentDistance(c.p1, c.p2, s.center, s.center);
  return dist <= (c.radius + s.radius + margin);
}

} // namespace arm

// ---------- 机械臂碰撞建模 ----------

namespace arm {

// 桌面级 6 轴机械臂的简化防碰撞包围盒（胶囊体序列）
// 由于连杆较细，我们将每个连杆抽象为 1-2 个胶囊体
inline std::vector<Capsule> buildArmCollisionModel(const ArmModel& arm, const std::array<double, 6>& q) {
  std::vector<Capsule> caps;
  auto frames = forwardKinematics(arm, q);

  // forwardKinematics returns std::vector<Mat4> of size 7:
  // frames[0] = T_0^0 (Identity)
  // frames[1] = T_0^1
  // ...
  // frames[6] = T_0^6 (End Effector TCP flange)
  Vec3 p0 = frames[0].translationV();
  Vec3 p1 = frames[1].translationV();
  Vec3 p2 = frames[2].translationV();
  Vec3 p3 = frames[3].translationV();
  Vec3 p4 = frames[4].translationV();
  Vec3 p5 = frames[5].translationV();
  Vec3 p6 = frames[6].translationV(); // Fix: include the true end effector

  // J0 -> J1 (Base Link)
  caps.push_back({p0, p1, 0.05});
  // J1 -> J2 (Shoulder)
  caps.push_back({p1, p2, 0.045});
  // J2 -> J3 (Upper Arm)
  caps.push_back({p2, p3, 0.04});
  // J3 -> J4 (Forearm)
  caps.push_back({p3, p4, 0.035});
  // J4 -> J5 (Wrist 1)
  caps.push_back({p4, p5, 0.03});
  // J5 -> J6 (Wrist 2 & TCP)
  caps.push_back({p5, p6, 0.03});

  return caps;
}

// 自碰撞检测
inline bool checkSelfCollision(const std::vector<Capsule>& armCaps, double margin = 0.01) {
  // 相邻或隔一个关节的胶囊体天然相交，不需要检测（通过关节限位防涉）
  // 我们只检测相隔 >= 2 的胶囊体（例如 Base 与 Forearm, Upper Arm 与 Wrist2 等）
  int n = armCaps.size();
  for (int i = 0; i < n; i++) {
    for (int j = i + 2; j < n; j++) {
      // 稍微放宽对 i, i+2 的检测，因为关节物理体积可能重叠。
      // 对于相隔刚好为 2 的，稍微缩小半径或增大容差
      double m = (j == i + 2) ? (margin - 0.01) : margin;
      if (checkCapsuleCollision(armCaps[i], armCaps[j], m)) {
        return true;
      }
    }
  }
  return false;
}

} // namespace arm
