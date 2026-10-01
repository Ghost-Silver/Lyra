// lib/arm/planning.hpp — Layer 0 轨迹/路径规划
// 关节 PTP（S 形时间律、多轴同步、逐轴限速）、笛卡尔直线/圆弧路径（姿态 slerp、
// 奇异软降速、关节限速时间缩放）、水平面势场避障、抓取「接近→下降→离开」编排。
// header-only。include 根目录为 lib/。
#pragma once
#include "arm/kinematics.hpp"
#include "arm/trajectory.hpp"
#include <algorithm>
#include <string>
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
  size_t size() const { return qs.size(); }
};

// 关节空间点到点轨迹：各轴独立 S 曲线，取最长轴时长做平滑时间缩放对齐（同步到达），
// 每轴均满足 自身 vmax 与 共享 amax/jmax。speedScale ∈ (0,1] 整体降速。
inline JointTrajectory jointPTP(const ArmModel& arm,
                                const std::array<double, 6>& q0,
                                const std::array<double, 6>& qf,
                                double dt = 0.002,
                                double speedScale = 1.0,
                                double amax = 4.0,
                                double jmax = 30.0) {
  JointTrajectory res;
  speedScale = std::clamp(speedScale, 1e-3, 1.0);

  SCurveProfile prof[6];
  double T = 0;
  for (int i = 0; i < 6; i++) {
    double vmax = std::max(arm.vmax[i] * speedScale, 1e-6);
    if (!scurvePlan(qf[i] - q0[i], vmax, amax, jmax, prof[i])) return res;
    T = std::max(T, prof[i].T);
  }
  // 平滑时间缩放到同步时长
  for (int i = 0; i < 6; i++)
    if (T > prof[i].T && prof[i].T > 0) {
      if (!scurvePlan(qf[i] - q0[i], arm.vmax[i] * speedScale, amax, jmax, prof[i], T)) return res;
    }

  int n = (int)std::ceil(T / dt) + 1;
  if (T < 1e-12) n = 1;
  res.ts.resize(n);
  res.qs.resize(n);
  // prof.s(t) 已含位移符号；ts 与 qs 必须同步写入——播放端按 (ts,qs) 对做时间轴
  // 推进/插值/完成判定（t >= ts.back()），ts 缺失会让 n>1 的轨迹在首个控制周期即判完成。
  for (int i = 0; i < n; i++) {
    double t = std::min(double(i) * dt, T);
    res.ts[i] = t;
    for (int k = 0; k < 6; k++) res.qs[i][k] = q0[k] + prof[k].s(t);
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
    if (dist < radius) {
      Vec3 lateral = Vec3{d.x, d.y, 0};           // 只在水平面避让
      double l = lateral.norm();
      if (l < 1e-6) lateral = Vec3{1, 0, 0}, l = 1.0;  // 正对障碍中心：取任意横向
      double push = (radius - dist) * strength;
      Vec3 dir = lateral * (1.0 / l);
      p = p + dir * push;
    }
  }
  return out;
}

// 笛卡尔直线轨迹：位置直线 + 姿态 slerp（绕相对旋转轴等角速度，无欧拉角跳变）。
// 时间律：路径弧长用 S 曲线（vLin/aLin/jLin），再做两重时间缩放——
//   ① 奇异软处理：σ_min 低于阈值时末端按 singularityScale 降速；
//   ② 关节限速：段间所需时间不低于任一关节 |Δq|/vmax。
// 中途 IK 失败则整条返回空。
inline JointTrajectory cartesianLineTraj(const ArmModel& arm,
                                         const Mat4& T0, const Mat4& Tf,
                                         const std::array<double, 6>& ikCurrent,
                                         double dt = 0.002, double vLin = 0.12,
                                         double aLin = 0.6, double jLin = 3.0,
                                         double amaxJoint = 4.0) {
  JointTrajectory res;
  Vec3 pa = T0.translationV(), pb = Tf.translationV();
  double dist = (pb - pa).norm();
  if (dist < 1e-9) return {};

  // 路径弧长 S 曲线时间律
  SCurveProfile law;
  if (!scurvePlan(dist, vLin, aLin, jLin, law)) return {};
  double T = law.T;
  int n = std::max(2, (int)std::ceil(T / dt) + 1);

  // 契约：输出 (ts, qs) 是**完整时间重参数化**后的分段线性关节轨迹——qs[i] 是到达时刻
  // ts[i] 的路点，段内速度 = |Δq|/Δts（限速见下）。播放端必须按 ts 域插值（main.cpp playTraj
  // 即如此），禁止按未缩放的路径时刻 i·dt 索引。路点按路径时刻均匀采样，时间轴经 ①奇异性
  // 降速 ②关节限速 ③加速度一致 三重拉伸后自洽：任一段平均速度 ≤ vmax，相邻段平均速度跳变
  // 受 amax 约束（dti ≥ |Δv|/amax 不动点一次即得，只依赖已定稿的前段）。限位为硬约束：
  // inverseKinematics 已过滤越限解，任一路点无合规解即返回空。
  std::array<double, 6> q = ikCurrent;
  res.ts.resize(n);
  res.qs.resize(n);
  double t_acc = 0.0;          // 缩放后的累计时间
  std::vector<double> ts_raw(n);
  for (int i = 0; i < n; i++) {
    double t = std::min(double(i) * dt, T);
    double s = law.s(t);       // 0..dist
    double lambda = dist > 1e-12 ? s / dist : 0.0;
    Vec3 pos = pa + (pb - pa) * lambda;
    Mat4 Tw = Mat4::translation(pos) * rotationSlerp(T0, Tf, lambda);
    std::array<double, 6> qnew;
    if (!inverseKinematics(arm, Tw, q, qnew)) return {};
    if (i > 0) {
      // ② 关节限速下限：段平均速度 ≤ vmax
      double dtq = 0, dqa = 0;
      for (int k = 0; k < 6; k++) {
        dtq = std::max(dtq, std::abs(qnew[k] - res.qs[i - 1][k]) / std::max(arm.vmax[k], 1e-6));
        dqa = std::max(dqa, std::sqrt(2.0 * std::abs(qnew[k] - res.qs[i - 1][k]) /
                                     std::max(amaxJoint, 1e-6)));
      }
      // ① 奇异软处理：降速 → 时间膨胀
      double sc = singularityScale(arm, qnew);
      double dti = std::max({double(dt), dtq, dqa}) / std::max(sc, 0.05);
      // ③ 加速度一致：相邻段平均速度跳变 |Δv| ≤ amax·dti（依赖已定稿前段，一次成型）
      if (i > 1) {
        double dtPrev = ts_raw[i - 1] - ts_raw[i - 2];
        double need = 0;
        for (int k = 0; k < 6; k++) {
          double vPrev = (res.qs[i - 1][k] - res.qs[i - 2][k]) / std::max(dtPrev, 1e-9);
          double vCur = (qnew[k] - res.qs[i - 1][k]) / std::max(dti, 1e-9);
          need = std::max(need, std::abs(vCur - vPrev) / std::max(amaxJoint, 1e-6));
        }
        dti = std::max(dti, need);
      }
      t_acc += dti;
    }
    ts_raw[i] = t_acc;
    res.qs[i] = qnew;
    q = qnew;
  }
  for (int i = 1; i < n; i++)       // 严格单调（完成判定/插值除法的前提）
    if (ts_raw[i] <= ts_raw[i - 1]) ts_raw[i] = ts_raw[i - 1] + 1e-6;
  res.ts = ts_raw;
  return res;
}

// ---------- 笛卡尔连续折线轨迹 (多段直线平滑融合) ----------
struct PathSegment {
  enum Type { LINE, ARC };
  Type type;
  double length;

  // LINE
  Vec3 startPos, endPos;
  Mat4 startRot, endRot;

  // ARC
  Vec3 center;
  Vec3 u, v;
  double radius;
  double startAngle, sweepAngle;
  Mat4 arcRot;

  Mat4 eval(double s) const {
    if (type == LINE) {
      double lambda = length > 1e-9 ? s / length : 0.0;
      Vec3 p = startPos + (endPos - startPos) * lambda;
      Mat4 T_rot = rotationSlerp(startRot, endRot, lambda);
      T_rot.m[3] = p.x; T_rot.m[7] = p.y; T_rot.m[11] = p.z;
      return T_rot;
    } else {
      double lambda = length > 1e-9 ? s / length : 0.0;
      double ang = startAngle + sweepAngle * lambda;
      Vec3 p = center + (u * std::cos(ang) + v * std::sin(ang)) * radius;
      Mat4 T = arcRot;
      T.m[3] = p.x; T.m[7] = p.y; T.m[11] = p.z;
      return T;
    }
  }
};

inline JointTrajectory cartesianBlendPathTraj(const ArmModel& arm,
                                              const std::vector<Mat4>& waypoints,
                                              const std::array<double, 6>& ikCurrent,
                                              double blendRadius = 0.02,
                                              double dt = 0.002, double vLin = 0.12,
                                              double aLin = 0.6, double jLin = 3.0,
                                              double amaxJoint = 4.0) {
  JointTrajectory res;
  if (waypoints.size() < 2) return res;

  std::vector<PathSegment> segments;
  double totalLength = 0;

  std::vector<Vec3> pts;
  for(auto& w : waypoints) pts.push_back(w.translationV());

  std::vector<double> distsToCorner(waypoints.size(), 0.0);
  for (size_t i = 1; i < waypoints.size() - 1; i++) {
    Vec3 d1 = (pts[i] - pts[i-1]).normalized();
    Vec3 d2 = (pts[i+1] - pts[i]).normalized();
    double dot = d1.dot(d2);
    if (dot > -0.999 && dot < 0.999) {
      double angle = std::acos(std::clamp(dot, -1.0, 1.0));
      double halfAngle = angle / 2.0;
      double dist = blendRadius / std::tan(halfAngle);
      double l1 = (pts[i] - pts[i-1]).norm();
      double l2 = (pts[i+1] - pts[i]).norm();
      double maxDist = std::min(l1/2.0, l2/2.0);

      if (dist > maxDist) {
        dist = maxDist;
      }
      distsToCorner[i] = dist;
    }
  }

  for (size_t i = 0; i < waypoints.size() - 1; i++) {
    Vec3 p1 = pts[i];
    Vec3 p2 = pts[i+1];

    Vec3 lineStart = p1;
    if (i > 0) lineStart = p1 + (p2 - p1).normalized() * distsToCorner[i];

    Vec3 lineEnd = p2;
    if (i < waypoints.size() - 2) lineEnd = p2 - (p2 - p1).normalized() * distsToCorner[i+1];

    PathSegment lineSeg;
    lineSeg.type = PathSegment::LINE;
    lineSeg.startPos = lineStart;
    lineSeg.endPos = lineEnd;
    lineSeg.length = (lineEnd - lineStart).norm();

    lineSeg.startRot = waypoints[i];
    lineSeg.startRot.m[3] = 0; lineSeg.startRot.m[7] = 0; lineSeg.startRot.m[11] = 0;
    lineSeg.endRot = waypoints[i+1];
    lineSeg.endRot.m[3] = 0; lineSeg.endRot.m[7] = 0; lineSeg.endRot.m[11] = 0;

    if (lineSeg.length > 1e-6) {
       segments.push_back(lineSeg);
       totalLength += lineSeg.length;
    }

    if (i < waypoints.size() - 2) {
      double d = distsToCorner[i+1];
      if (d > 1e-6) {
        Vec3 corner = pts[i+1];
        Vec3 d1 = (p2 - p1).normalized();
        Vec3 d2 = (pts[i+2] - corner).normalized();

        Vec3 c1 = corner - d1 * d;
        Vec3 c2 = corner + d2 * d;

        double dot = d1.dot(d2);
        double angle = std::acos(std::clamp(dot, -1.0, 1.0));
        double halfAngle = angle / 2.0;

        double effectiveRadius = d * std::tan(halfAngle);

        Vec3 centerDir = (d2 - d1).normalized();
        // Since center is on the bisector, dist to center from corner is d / cos(halfAngle)
        // Wait, tan(half) = r/d, sin(half) = r / dist_to_center
        // dist_to_center = r / sin(half) = d * tan(half) / sin(half) = d / cos(half)
        double distToCenter = d / std::cos(halfAngle);
        Vec3 arcCenter = corner + centerDir * distToCenter;

        Vec3 normal = d1.cross(d2).normalized();

        PathSegment arcSeg;
        arcSeg.type = PathSegment::ARC;
        arcSeg.center = arcCenter;
        arcSeg.radius = effectiveRadius;

        arcSeg.u = (c1 - arcCenter).normalized();
        arcSeg.v = normal.cross(arcSeg.u).normalized();

        arcSeg.startAngle = 0;
        double sweep = M_PI - angle;

        Vec3 expectedC2 = arcCenter + (arcSeg.u * std::cos(sweep) + arcSeg.v * std::sin(sweep)) * effectiveRadius;
        if ((expectedC2 - c2).norm() > 1e-6) {
           sweep = -sweep;
        }

        arcSeg.sweepAngle = sweep;
        arcSeg.length = std::abs(sweep) * effectiveRadius;

        arcSeg.arcRot = waypoints[i+1];
        arcSeg.arcRot.m[3] = 0; arcSeg.arcRot.m[7] = 0; arcSeg.arcRot.m[11] = 0;

        if (arcSeg.length > 1e-6) {
          segments.push_back(arcSeg);
          totalLength += arcSeg.length;
        }
      }
    }
  }

  if (totalLength < 1e-9) return res;

  SCurveProfile law;
  if (!scurvePlan(totalLength, vLin, aLin, jLin, law)) return {};
  double T = law.T;
  int n = std::max(2, (int)std::ceil(T / dt) + 1);

  std::array<double, 6> q = ikCurrent;
  res.ts.resize(n);
  res.qs.resize(n);
  double t_acc = 0.0;
  std::vector<double> ts_raw(n);

  for (int i = 0; i < n; i++) {
    double t = std::min(double(i) * dt, T);
    double s = law.s(t);

    double s_acc = 0;
    Mat4 Tw;
    bool found = false;
    for (auto& seg : segments) {
      if (s <= s_acc + seg.length + 1e-9) {
        Tw = seg.eval(s - s_acc);
        found = true;
        break;
      }
      s_acc += seg.length;
    }
    if (!found) Tw = segments.back().eval(segments.back().length);

    std::array<double, 6> qnew;
    if (!inverseKinematics(arm, Tw, q, qnew)) return {};
    if (i > 0) {
      double dtq = 0, dqa = 0;
      for (int k = 0; k < 6; k++) {
        dtq = std::max(dtq, std::abs(qnew[k] - res.qs[i - 1][k]) / std::max(arm.vmax[k], 1e-6));
        dqa = std::max(dqa, std::sqrt(2.0 * std::abs(qnew[k] - res.qs[i - 1][k]) /
                                     std::max(amaxJoint, 1e-6)));
      }
      double sc = singularityScale(arm, qnew);
      double dti = std::max({double(dt), dtq, dqa}) / std::max(sc, 0.05);
      if (i > 1) {
        double dtPrev = ts_raw[i - 1] - ts_raw[i - 2];
        double need = 0;
        for (int k = 0; k < 6; k++) {
          double vPrev = (res.qs[i - 1][k] - res.qs[i - 2][k]) / std::max(dtPrev, 1e-9);
          double vCur = (qnew[k] - res.qs[i - 1][k]) / std::max(dti, 1e-9);
          need = std::max(need, std::abs(vCur - vPrev) / std::max(amaxJoint, 1e-6));
        }
        dti = std::max(dti, need);
      }
      t_acc += dti;
    }
    ts_raw[i] = t_acc;
    res.qs[i] = qnew;
    q = qnew;
  }
  for (int i = 1; i < n; i++)
    if (ts_raw[i] <= ts_raw[i - 1]) ts_raw[i] = ts_raw[i - 1] + 1e-6;
  res.ts = ts_raw;
  return res;
}

// 笛卡尔圆弧轨迹：时间参数化，姿态恒定
inline JointTrajectory cartesianCircleTraj(const ArmModel& arm,
                                           const Vec3& center, const Vec3& normal, double radius,
                                           double startAngle, double sweep,
                                           const Mat4& baseOrientation,
                                           const std::array<double, 6>& ikCurrent,
                                           double dt = 0.002, double vLin = 0.12,
                                           double aLin = 0.6, double jLin = 3.0,
                                           double amaxJoint = 4.0) {
  JointTrajectory res;
  double dist = std::abs(sweep) * radius;
  if (dist < 1e-9) return {};

  SCurveProfile law;
  if (!scurvePlan(dist, vLin, aLin, jLin, law)) return {};
  double T = law.T;
  int n = std::max(2, (int)std::ceil(T / dt) + 1);

  Vec3 nrm = normal.normalized();
  Vec3 ref = (std::abs(nrm.x) < 0.9) ? Vec3{1, 0, 0} : Vec3{0, 1, 0};
  Vec3 u = ref.cross(nrm).normalized();
  Vec3 v = nrm.cross(u);

  std::array<double, 6> q = ikCurrent;
  res.ts.resize(n);
  res.qs.resize(n);
  double t_acc = 0.0;
  std::vector<double> ts_raw(n);
  for (int i = 0; i < n; i++) {
    double t = std::min(double(i) * dt, T);
    double s = law.s(t);       // 0..dist
    double lambda = dist > 1e-12 ? s / dist : 0.0;
    double ang = startAngle + lambda * sweep;
    Vec3 pos = center + (u * std::cos(ang) + v * std::sin(ang)) * radius;
    Mat4 Tw = baseOrientation;
    Tw.m[3] = pos.x; Tw.m[7] = pos.y; Tw.m[11] = pos.z;

    std::array<double, 6> qnew;
    if (!inverseKinematics(arm, Tw, q, qnew)) return {};
    if (i > 0) {
      double dtq = 0, dqa = 0;
      for (int k = 0; k < 6; k++) {
        dtq = std::max(dtq, std::abs(qnew[k] - res.qs[i - 1][k]) / std::max(arm.vmax[k], 1e-6));
        dqa = std::max(dqa, std::sqrt(2.0 * std::abs(qnew[k] - res.qs[i - 1][k]) /
                                     std::max(amaxJoint, 1e-6)));
      }
      double sc = singularityScale(arm, qnew);
      double dti = std::max({double(dt), dtq, dqa}) / std::max(sc, 0.05);
      if (i > 1) {
        double dtPrev = ts_raw[i - 1] - ts_raw[i - 2];
        double need = 0;
        for (int k = 0; k < 6; k++) {
          double vPrev = (res.qs[i - 1][k] - res.qs[i - 2][k]) / std::max(dtPrev, 1e-9);
          double vCur = (qnew[k] - res.qs[i - 1][k]) / std::max(dti, 1e-9);
          need = std::max(need, std::abs(vCur - vPrev) / std::max(amaxJoint, 1e-6));
        }
        dti = std::max(dti, need);
      }
      t_acc += dti;
    }
    ts_raw[i] = t_acc;
    res.qs[i] = qnew;
    q = qnew;
  }
  for (int i = 1; i < n; i++)
    if (ts_raw[i] <= ts_raw[i - 1]) ts_raw[i] = ts_raw[i - 1] + 1e-6;
  res.ts = ts_raw;
  return res;
}

// 圆弧笛卡尔轨迹（姿态恒定 slerp 基准）→ 位姿序列；用 cartesianLineTraj 逐段衔接或直接 IK 求解
inline std::vector<Mat4> circlePoses(const Vec3& center, const Vec3& normal, double radius,
                                     double startAngle, double sweep, int n,
                                     const Mat4& baseOrientation) {
  std::vector<Vec3> pts = circlePoints(center, normal, radius, startAngle, sweep, n);
  std::vector<Mat4> out;
  for (auto& pos : pts) {
    Mat4 T = baseOrientation;              // 旋转 = 基准姿态
    T.m[3] = pos.x; T.m[7] = pos.y; T.m[11] = pos.z;  // 平移 = 圆弧点
    out.push_back(T);
  }
  return out;
}

// ---------- 抓取动作编排 ----------
// 给定目标物体中心与高度、爪深，生成「上方接近 → 下降抓取 → 上提离开」笛卡尔路径
struct GraspPlan {
  std::vector<Mat4> targets;   // 依序执行
  std::vector<std::string> names;
};
inline GraspPlan graspPlan(const Vec3& objectPos, double objectHeight,
                           double approachD, double gripperZ,
                           const Mat4& baseOrientation, double leaveLift = -1.0) {
  GraspPlan gp;
  Vec3 top = objectPos + Vec3{0, 0, objectHeight};
  double r, p, y; toRPY(baseOrientation, r, p, y);
  Mat4 approach = fromRPY(top + Vec3{0, 0, approachD}, r, p, y);   // 末端悬停于物体上方
  Mat4 grasp    = fromRPY(top - Vec3{0, 0, gripperZ}, r, p, y);    // 下降到抓取位
  // 离开段：闭合后抬升至 approach 之上（默认额外抬升 approachD/2，至少 2cm），
  // 与 approach 严格区分——旧版 leave == approach 只是原路退回、无独立离开段。
  // 抬升量 leaveLift 可配（>=0 直接生效；<0 取默认）。门形搬运以此为净空起点。
  double lift = leaveLift >= 0 ? leaveLift : std::max(0.5 * approachD, 0.02);
  Mat4 leave = fromRPY(top + Vec3{0, 0, approachD + lift}, r, p, y);
  gp.targets = {approach, grasp, leave};
  gp.names = {"approach", "grasp", "leave"};
  return gp;
}

}  // namespace arm
