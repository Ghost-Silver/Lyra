// SlotA_planning.hpp
// 轨迹/路径规划：关节 PTP、笛卡尔直线/圆弧路径、势场避障、抓取动作编排。
// header-only。
#pragma once
#include "arm/kinematics.hpp"
#include <algorithm>
#include <vector>
#include <cmath>

namespace arm {

// 由 平移 + RPY(roll,pitch,yaw) 构造位姿  (RPY 为 ZYX 内旋: Rz*Ry*Rx)
inline Mat4 fromRPY(const Vec3& p, double roll, double pitch, double yaw) {
  return Mat4::translation(p) * Mat4::rotationZ(yaw) * Mat4::rotationY(pitch) * Mat4::rotationX(roll);
}

// 提取 RPY
inline void toRPY(const Mat4& T, double& roll, double& pitch, double& yaw) {
  pitch = std::atan2(-T.m[8], std::sqrt(T.m[0] * T.m[0] + T.m[4] * T.m[4]));
  roll  = std::atan2(T.m[9], T.m[10]);
  yaw   = std::atan2(T.m[4], T.m[0]);
}

// ---------- 时间离散的关节轨迹 ----------
struct JointTrajectory {
  std::vector<double> ts;
  std::vector<std::array<double, 6>> qs;
  bool empty() const { return qs.empty(); }
};

// 梯形速度曲线采样
inline std::vector<double> trapezoidProfile(double dist, double vmax, double amax, double dt) {
  std::vector<double> out;
  double accel_len = vmax * vmax / amax;              // 加减速段距离(对称)
  double total;
  if (std::abs(dist) <= accel_len) {
    // 达不到最高速：三角曲线
    double tAcc = std::sqrt(std::abs(dist) / amax);
    total = 2 * tAcc;
    double vm = amax * tAcc;
    double t = 0;
    while (t < total + 1e-9) { out.push_back(t); t += dt; }
    return out;
  } else {
    double tAcc = vmax / amax;
    double tCruise = (std::abs(dist) - accel_len) / vmax;
    total = 2 * tAcc + tCruise;
    double t = 0;
    while (t < total + 1e-9) { out.push_back(t); t += dt; }
    return out;
  }
}

// 关节空间点到点轨迹（各轴按最快轴整型缩放梯形曲线）
inline JointTrajectory jointPTP(const ArmModel& arm,
                                const std::array<double, 6>& q0,
                                const std::array<double, 6>& qf,
                                double dt = 0.002) {
  JointTrajectory res;
  double amax = 4.0;  // rad/s^2
  double vscale = 1.0;
  // 选出耗时最长的轴作为基准
  for (int i = 0; i < 6; i++) {
    double d = std::abs(qf[i] - q0[i]);
    double vm = arm.vmax[i];
    double tA = vm / amax;
    double tc = vscale * d;  // tentative
    (void)tc; (void)tA;
  }
  // 简化为均匀梯形：速度与加速度统一，按最大角位移缩放
  double maxd = 0;
  for (int i = 0; i < 6; i++) maxd = std::max(maxd, std::abs(qf[i] - q0[i]));
  double vm = arm.vmax[0];  // 统一基准
  amax = 4.0;
  double tAcc = vm / amax;
  bool triangular = false;
  double T;
  if (maxd < 1e-9) { T = 0.0; }
  else if (maxd <= vm * tAcc) { triangular = true; T = 2 * std::sqrt(maxd / amax); }
  else T = 2 * tAcc + (maxd - amax * tAcc * tAcc) / vm;

  int n = (int)std::ceil(T / dt) + 1;
  if (n <= 0) return res;
  res.ts.resize(n); res.qs.resize(n);
  auto profile = [&](double t) {
    if (T <= 1e-9) return 1.0;
    double s;
    if (!triangular) {
      double tCr = std::max(0.0, T - 2 * tAcc);
      if (t <= tAcc) s = 0.5 * amax * t * t;
      else if (t <= tAcc + tCr) s = amax * tAcc * (t - tAcc) + 0.5 * amax * tAcc * tAcc;
      else { double u = T - t; s = maxd - 0.5 * amax * u * u; }
    } else {
      double tP = std::sqrt(maxd / amax);
      if (t <= tP) s = 0.5 * amax * t * t;
      else { double u = T - t; s = maxd - 0.5 * amax * u * u; }
    }
    return std::clamp(s / maxd, 0.0, 1.0);
  };
  for (int i = 0; i < n; i++) {
    double t = double(i) * dt;
    res.ts[i] = t;
    double s = profile(t);
    for (int j = 0; j < 6; j++) res.qs[i][j] = q0[j] + (qf[j] - q0[j]) * s;
  }
  return res;
}

// ---------- 笛卡尔路径 ----------
struct CartesianWaypoint {
  Vec3 pos;
  double roll, pitch, yaw;  // 工具姿态
};

inline std::vector<Vec3> linePoints(const Vec3& a, const Vec3& b, int n) {
  std::vector<Vec3> pts;
  for (int i = 0; i <= n; i++) { double t = double(i) / n; pts.push_back(a + (b - a) * t); }
  return pts;
}

inline std::vector<Vec3> circlePoints(const Vec3& center, const Vec3& normal, double radius,
                                      double startAngle, double sweep, int n) {
  std::vector<Vec3> pts;
  Vec3 nrm = normal.normalized();
  // 建立平面坐标系 (u,v) 正交于 normal
  Vec3 ref = (std::abs(nrm.x) < 0.9) ? Vec3{1, 0, 0} : Vec3{0, 1, 0};
  Vec3 u = ref.cross(nrm).normalized();
  Vec3 v = nrm.cross(u);
  for (int i = 0; i <= n; i++) {
    double ang = startAngle + sweep * double(i) / n;
    pts.push_back(center + (u * std::cos(ang) + v * std::sin(ang)) * radius);
  }
  return pts;
}

// 势场式避障：把路径点横向推开球体障碍（保持高度不变）
inline std::vector<Vec3> repelObstacle(const std::vector<Vec3>& pts,
                                       const Vec3& obs, double radius,
                                       double strength = 0.6) {
  std::vector<Vec3> out = pts;
  for (auto& p : out) {
    Vec3 d = p - obs;
    double dist = d.norm();
    if (dist < radius && dist > 1e-6) {
      Vec3 lateral = Vec3{d.x, d.y, 0};           // 只在水平面避让
      double l = lateral.norm();
      if (l > 1e-6) {
        double push = (radius - dist) * strength;
        Vec3 dir = lateral * (1.0 / l);
        p = p + dir * push;
      }
    }
  }
  return out;
}

// 直线(姿态等间隔内插)的笛卡尔轨迹 → 落到关节空间
inline JointTrajectory cartesianLineTraj(const ArmModel& arm,
                                         const Mat4& T0, const Mat4& Tf,
                                         const std::array<double, 6>& ikCurrent,
                                         double dt = 0.002, double vLin = 0.12) {
  JointTrajectory res;
  Vec3 pa = T0.translationV(), pb = Tf.translationV();
  double dist = (pb - pa).norm();
  if (dist < 1e-9) return {};
  double T = dist / vLin;
  int n = (int)std::ceil(T / dt) + 1;

  double r0, p0, y0, rf, pf, yf;
  toRPY(T0, r0, p0, y0);
  toRPY(Tf, rf, pf, yf);

  // 先估算中间点姿态，避免欧拉角跳变
  std::array<double, 6> q = ikCurrent;
  res.ts.resize(n); res.qs.resize(n);
  for (int i = 0; i < n; i++) {
    double t = double(i) / n;
    Vec3 pos = pa + (pb - pa) * t;
    double r = r0 + (rf - r0) * t;
    double p = p0 + (pf - p0) * t;
    double y = y0 + (yf - y0) * t;
    Mat4 Tw = fromRPY(pos, r, p, y);
    std::array<double, 6> qnew;
    if (!inverseKinematics(arm, Tw, q, qnew)) return {};  // 中途不可达则整条失败
    res.qs[i] = qnew;
    res.ts[i] = double(i) * dt;
    q = qnew;  // 增量续解
  }
  return res;
}

// 圆弧笛卡尔轨迹
inline std::vector<Mat4> circlePoses(const Vec3& center, const Vec3& normal, double radius,
                                     double startAngle, double sweep, int n,
                                     const Mat4& baseOrientation) {
  std::vector<Vec3> pts = circlePoints(center, normal, radius, startAngle, sweep, n);
  std::vector<Mat4> out;
  double r, p, y; toRPY(baseOrientation, r, p, y);
  for (auto& pos : pts) out.push_back(fromRPY(pos, r, p, y));
  return out;
}

// ---------- 抓取动作编排 ----------
// 给定目标物体中心与高度、爪深，生成"上方接近→下降抓取→上提离开"笛卡尔路径
struct GraspPlan {
  std::vector<Mat4> targets;   // 依序执行
  std::vector<std::string> names;
};
inline GraspPlan graspPlan(const Vec3& objectPos, double objectHeight,
                           double approachD, double gripperZ,
                           const Mat4& baseOrientation) {
  GraspPlan gp;
  Vec3 top = objectPos + Vec3{0, 0, objectHeight};
  double r, p, y; toRPY(baseOrientation, r, p, y);
  Mat4 approach = fromRPY(top + Vec3{0, 0, approachD}, r, p, y);   // 末端悬停于物体上方
  Mat4 grasp    = fromRPY(top - Vec3{0, 0, gripperZ}, r, p, y);    // 下降到抓取位
  gp.targets = {approach, grasp};
  gp.names = {"approach", "grasp"};
  return gp;
}

}  // namespace arm