// lib/arm/trajectory.hpp — Layer 0 轨迹时间律
// 对称 7 段 S 形速度曲线（jerk 受限 double-S）：[+j, 0, −j, 0, −j, 0, +j]，
// 全解析分段积分采样 s/v/a/j；支持「平滑时间缩放」（强制时长 T ≥ T_min，
// 按 k = T/T_min 拉伸时间轴，jerk 除以 k³，速度/加速度/加加速度上限自动满足）。
#pragma once
#include <array>
#include <cmath>
#include <vector>
#include <algorithm>

namespace arm {

struct SCurveProfile {
  double dist = 0;            // 总位移（带符号）
  double T = 0;               // 总时长
  std::array<double, 8> ts{}; // 7 段边界 ts[0..7]，ts[0]=0, ts[7]=T
  std::array<double, 7> jk{}; // 各段恒定 jerk
  // 各段起点状态（s, v, a）
  std::array<double, 7> s0{}, v0{}, a0{};

  int locate(double t) const {
    if (t <= 0) return 0;
    if (t >= T) return 6;
    for (int i = 0; i < 7; i++) if (t < ts[i + 1]) return i;
    return 6;
  }
  void eval(double t, double& s, double& v, double& a) const {
    t = std::clamp(t, 0.0, T);
    int i = locate(t);
    double u = t - ts[i];
    double j = jk[i];
    a = a0[i] + j * u;
    v = v0[i] + a0[i] * u + 0.5 * j * u * u;
    s = s0[i] + v0[i] * u + 0.5 * a0[i] * u * u + j * u * u * u / 6.0;
  }
  double s(double t) const { double s_, v_, a_; eval(t, s_, v_, a_); return s_; }
  double v(double t) const { double s_, v_, a_; eval(t, s_, v_, a_); return v_; }
  double a(double t) const { double s_, v_, a_; eval(t, s_, v_, a_); return a_; }
  double j(double t) const { t = std::clamp(t, 0.0, T); return jk[locate(t)]; }
};

// 内部：由 (Tj, Ta, Tv, dist 符号) 生成 profile（jerk 取 ±jmax）
inline SCurveProfile scurveBuild(double dist, double jmax,
                                 double Tj, double Ta, double Tv) {
  SCurveProfile p;
  double sg = dist >= 0 ? 1.0 : -1.0;
  p.dist = dist;
  p.ts = {0, Tj, Tj + Ta, 2 * Tj + Ta, 2 * Tj + Ta + Tv,
          3 * Tj + Ta + Tv, 3 * Tj + 2 * Ta + Tv, 4 * Tj + 2 * Ta + Tv};
  p.T = p.ts[7];
  p.jk = {sg * jmax, 0, -sg * jmax, 0, -sg * jmax, 0, sg * jmax};
  double s = 0, v = 0, a = 0;
  for (int i = 0; i < 7; i++) {
    p.s0[i] = s; p.v0[i] = v; p.a0[i] = a;
    double dur = p.ts[i + 1] - p.ts[i];
    s += v * dur + 0.5 * a * dur * dur + p.jk[i] * dur * dur * dur / 6.0;
    v += a * dur + 0.5 * p.jk[i] * dur * dur;
    a += p.jk[i] * dur;
  }
  return p;
}

// 规划单轴 S 曲线：
//   dist   位移 (rad / m / 任意单位)
//   vmax / amax / jmax  上限
//   T_forced > 0 时强制拉长到该时长（≥ 最优时长），实现平滑时间缩放
// 返回 false 表示参数非法（负上限 / T_forced < 最优时长）
inline bool scurvePlan(double dist, double vmax, double amax, double jmax,
                       SCurveProfile& out, double T_forced = 0.0) {
  if (vmax <= 0 || amax <= 0 || jmax <= 0) return false;
  double D = std::abs(dist);
  if (D < 1e-12) {
    out = scurveBuild(0.0, jmax, 0, 0, 0);
    return true;
  }

  double Tj, Ta, Tv;
  double Tj_a = amax / jmax;              // 达到 amax 所需 jerk 段时长
  double Tj_v = std::sqrt(vmax / jmax);   // 三角加速度达 vmax 的 jerk 段时长

  // x3(Tj, Ta)：加速段（j-a-j）末位移
  //   x3 = j*Tj³  +  1.5*j*Tj²*Ta  +  0.5*j*Tj*Ta²
  auto x3 = [&](double Tj, double Ta) {
    return jmax * Tj * Tj * Tj + 1.5 * jmax * Tj * Tj * Ta + 0.5 * jmax * Tj * Ta * Ta;
  };

  if (vmax >= amax * amax / jmax) {
    // 能达到 amax：Tj = Tj_a，先看能否达到 vmax
    Tj = Tj_a;
    Ta = vmax / amax - Tj;                 // ≥ 0
    double D1 = 2.0 * x3(Tj, Ta);          // 刚好达到 vmax 与 amax（Tv=0）
    double D2 = 2.0 * x3(Tj, 0.0);         // 只达到 amax（Ta=Tv=0）
    if (D >= D1 - 1e-15) {
      Tv = (D - D1) / vmax;
    } else if (D >= D2 - 1e-15) {
      // R2：Tv = 0，解 Ta 使 2*x3(Tj,Ta) = D（x3 对 Ta 单调）
      Tv = 0;
      double A2 = 0.5 * jmax * Tj;
      double B2 = 1.5 * jmax * Tj * Tj;
      double C2 = jmax * Tj * Tj * Tj;
      double disc = B2 * B2 - 4.0 * A2 * (C2 - 0.5 * D);
      if (disc < 0) disc = 0;
      Ta = (-B2 + std::sqrt(disc)) / (2.0 * A2);
    } else {
      // R3：纯 jerk 三角形，D = 2*j*Tj³
      Tj = std::cbrt(D / (2.0 * jmax));
      Ta = 0; Tv = 0;
    }
  } else {
    // 达不到 amax（vmax 较小）：Tj = Tj_v，Ta = 0
    Tj = Tj_v;
    Ta = 0;
    double D0 = 2.0 * jmax * Tj * Tj * Tj;
    if (D >= D0 - 1e-15) {
      Tv = (D - D0) / vmax;
    } else {
      Tj = std::cbrt(D / (2.0 * jmax));
      Tv = 0;
    }
  }

  out = scurveBuild(dist, jmax, std::max(Tj, 0.0), std::max(Ta, 0.0), std::max(Tv, 0.0));

  // 平滑时间缩放：拉长时间轴 k 倍，jerk 除以 k³
  if (T_forced > 0 && out.T > 0) {
    if (T_forced < out.T - 1e-9) return false;   // 不允许比最优更短（会超限）
    double k = T_forced / out.T;
    if (std::abs(k - 1.0) > 1e-12) {
      double j3 = 1.0 / (k * k * k);
      for (int i = 0; i < 7; i++) out.ts[i] *= k, out.jk[i] *= j3;
      out.ts[7] *= k;
      out.T *= k;
      double s = 0, v = 0, a = 0;
      for (int i = 0; i < 7; i++) {
        out.s0[i] = s; out.v0[i] = v; out.a0[i] = a;
        double dur = out.ts[i + 1] - out.ts[i];
        s += v * dur + 0.5 * a * dur * dur + out.jk[i] * dur * dur * dur / 6.0;
        v += a * dur + 0.5 * out.jk[i] * dur * dur;
        a += out.jk[i] * dur;
      }
    }
  }
  return true;
}

}  // namespace arm
