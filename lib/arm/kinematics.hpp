// SlotA_kinematics.hpp
// 纯 C++ 运动学引擎：DH 建模、正运动学(FK)、解析逆运动学(IK)、
// 数值 IK 回退、几何雅可比、奇异性检测。
// 全部 header-only，便于单个翻译单元编译。
#pragma once
#include <array>
#include <cmath>
#include <limits>
#include <vector>
#include <algorithm>
#include <cstdio>

namespace arm {

constexpr double kPi = 3.14159265358979323846;
constexpr double kD2R = kPi / 180.0;
constexpr double kR2D = 180.0 / kPi;
constexpr double kTol = 1e-9;

// ---------- 3D 向量 ----------
struct Vec3 {
  double x = 0, y = 0, z = 0;
  double& operator[](int i) { return i == 0 ? x : (i == 1 ? y : z); }
  double  operator[](int i) const { return i == 0 ? x : (i == 1 ? y : z); }
  Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
  Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
  Vec3 operator*(double s) const { return {x * s, y * s, z * s}; }
  double dot(const Vec3& o) const { return x * o.x + y * o.y + z * o.z; }
  Vec3 cross(const Vec3& o) const { return {y * o.z - z * o.y, z * o.x - x * o.z, x * o.y - y * o.x}; }
  double norm() const { return std::sqrt(x * x + y * y + z * z); }
  Vec3 normalized() const { double n = norm(); return n > kTol ? *this * (1.0 / n) : Vec3{}; }
};

// ---------- 4x4 齐次变换（行优先存储, m[16], 每行4个元素） ----------
struct Mat4 {
  double m[16];
  static Mat4 identity() {
    Mat4 r{};
    for (int i = 0; i < 4; i++) r.m[i * 4 + i] = 1.0;
    return r;
  }
  // 平移
  static Mat4 translation(const Vec3& t) {
    Mat4 r = identity();
    r.m[3] = t.x; r.m[7] = t.y; r.m[11] = t.z;
    return r;
  }
  // 绕轴旋转
  static Mat4 rotationX(double a) {
    Mat4 r = identity();
    double c = std::cos(a), s = std::sin(a);
    r.m[5] = c; r.m[6] = -s; r.m[9] = s; r.m[10] = c;
    return r;
  }
  static Mat4 rotationY(double a) {
    Mat4 r = identity();
    double c = std::cos(a), s = std::sin(a);
    r.m[0] = c; r.m[2] = s; r.m[8] = -s; r.m[10] = c;
    return r;
  }
  static Mat4 rotationZ(double a) {
    Mat4 r = identity();
    double c = std::cos(a), s = std::sin(a);
    r.m[0] = c; r.m[1] = -s; r.m[4] = s; r.m[5] = c;
    return r;
  }
  Vec3 rotationCol(int c) const { return {m[c], m[4 + c], m[8 + c]}; }
  Vec3 translationV() const { return {m[3], m[7], m[11]}; }
  Mat4 operator*(const Mat4& o) const {
    Mat4 r{};
    for (int row = 0; row < 4; row++)
      for (int col = 0; col < 4; col++) {
        double s = 0;
        for (int k = 0; k < 4; k++) s += m[row * 4 + k] * o.m[k * 4 + col];
        r.m[row * 4 + col] = s;
      }
    return r;
  }
  // 刚体变换求逆（旋转转置 + 平移回代）
  Mat4 inverse() const {
    Mat4 r = identity();
    for (int row = 0; row < 3; row++)
      for (int col = 0; col < 3; col++) r.m[row * 4 + col] = m[col * 4 + row];
    Vec3 t = translationV();
    r.m[3] = -(r.m[0] * t.x + r.m[4] * t.y + r.m[8] * t.z);
    r.m[7] = -(r.m[1] * t.x + r.m[5] * t.y + r.m[9] * t.z);
    r.m[11]= -(r.m[2] * t.x + r.m[6] * t.y + r.m[10]* t.z);
    return r;
  }
  Vec3 transformPoint(const Vec3& p) const {
    return {m[0]*p.x + m[1]*p.y + m[2]*p.z + m[3],
            m[4]*p.x + m[5]*p.y + m[6]*p.z + m[7],
            m[8]*p.x + m[9]*p.y + m[10]*p.z + m[11]};
  }
  Vec3 transformDir(const Vec3& d) const {
    return {m[0]*d.x + m[1]*d.y + m[2]*d.z,
            m[4]*d.x + m[5]*d.y + m[6]*d.z,
            m[8]*d.x + m[9]*d.y + m[10]*d.z};
  }
};

// ---------- DH 参数（标准 DH） ----------
struct LinkDH {
  double a = 0, alpha = 0, d = 0;  // a: 连杆长, alpha: 扭转, d: 偏置
  double thetaOffset = 0;          // 零位偏置
};

// 一个桌面 6 轴的机械臂模型（UR 风格几何）
struct ArmModel {
  std::array<LinkDH, 6> links{};
  std::array<double, 6> qmin{}, qmax{};   // 关节限位 (rad)
  std::array<double, 6> vmax{};           // 关节最大速度 (rad/s)

  static ArmModel urStyle() {  // 类和宽度贴合 6 轴桌面臂（单位: m）
    ArmModel m;
    m.links[0] = {0.000,  90, 0.1625, 0.0};
    m.links[1] = {-0.425,  0, 0.0000, -90.0 * kD2R};
    m.links[2] = {-0.392,  0, 0.0000, 0.0};
    m.links[3] = {0.000,  90, 0.1333, 0.0};
    m.links[4] = {0.000, -90, 0.0997, 0.0};
    m.links[5] = {0.000,   0, 0.0996, 0.0};
    for (int i = 0; i < 6; i++) {
      m.qmin[i] = -2.967; m.qmax[i] = 2.967;
      m.vmax[i] = 2.5;
    }
    return m;
  }
};

// 将关节角转成标准 DH 的 theta
inline double DHTheta(const LinkDH& l, double q) { return q + l.thetaOffset; }

// 单个 DH 变换矩阵
inline Mat4 DHTransform(const LinkDH& l, double q) {
  double th = DHTheta(l, q);
  double ca = std::cos(l.alpha * kD2R), sa = std::sin(l.alpha * kD2R);
  double cth = std::cos(th), sth = std::sin(th);
  Mat4 T{};
  T.m[0] = cth;  T.m[1] = -sth * ca; T.m[2] =  sth * sa; T.m[3] = l.a * cth;
  T.m[4] = sth;  T.m[5] =  cth * ca; T.m[6] = -cth * sa; T.m[7] = l.a * sth;
  T.m[8] = 0;    T.m[9] =  sa;       T.m[10]=  ca;       T.m[11]= l.d;
  return T;
}

// 前向运动学：返回所有关节坐标系相对基座的变换，末端为 T0_6
inline std::vector<Mat4> forwardKinematics(const ArmModel& arm,
                                           const std::array<double, 6>& q) {
  std::vector<Mat4> frames(7);
  frames[0] = Mat4::identity();
  for (int i = 0; i < 6; i++) frames[i + 1] = frames[i] * DHTransform(arm.links[i], q[i]);
  return frames;
}

inline Mat4 forwardKinematicsT0_6(const ArmModel& arm, const std::array<double, 6>& q) {
  Mat4 T = Mat4::identity();
  for (int i = 0; i < 6; i++) T = T * DHTransform(arm.links[i], q[i]);
  return T;
}

// 几何雅可比 (6x6), 列 j = [ (z_i x (p6 - p_i)), z_i ]^T
inline void buildJacobian(const ArmModel& arm, const std::array<double, 6>& q,
                          double J[6][6]) {
  std::vector<Mat4> T = forwardKinematics(arm, q);
  Vec3 p6 = T[6].translationV();
  Vec3 z0 = {0, 0, 1};
  Vec3 axes[6], pos[6];  // 关节 i 轴单位向量与原点（i=0..5 对应 T[i]）
  axes[0] = z0; pos[0] = T[0].translationV();
  for (int i = 1; i < 6; i++) { axes[i] = T[i].transformDir(z0); pos[i] = T[i].translationV(); }
  for (int j = 0; j < 6; j++) {
    Vec3 v = axes[j].cross(p6 - pos[j]);
    for (int r = 0; r < 3; r++) { J[r][j] = v[r]; J[3 + r][j] = axes[j][r]; }
  }
}

// 雅可比行列式 → 奇异性度量（取最小奇异值近似）
inline double manipulability(const ArmModel& arm, const std::array<double, 6>& q) {
  double J[6][6];
  buildJacobian(arm, q, J);
  // 计算 J*J^T 的迹的平方根作为粗粒度可操控度
  double s = 0;
  for (int r = 0; r < 6; r++)
    for (int c = 0; c < 6; c++) {
      double acc = 0;
      for (int k = 0; k < 6; k++) acc += J[r][k] * J[c][k];
      s += acc * acc;
    }
  return std::sqrt(std::max(s, 0.0));
}
inline bool isSingularNear(const ArmModel& arm, const std::array<double, 6>& q,
                           double threshold = 0.05) {
  return manipulability(arm, q) < threshold;
}

// ---------- 数值 IK：阻尼最小二乘 (线性化 Newton) ----------
inline bool numericIK(const ArmModel& arm, const Mat4& target,
                      std::array<double, 6>& q, int iters = 60) {
  for (int it = 0; it < iters; it++) {
    Mat4 cur = forwardKinematicsT0_6(arm, q);
    Mat4 err = target.inverse() * cur;  // 误差在目标系
    Vec3 pErr = err.translationV();
    // 旋转误差向量（轴角近似）
    Vec3 r = {err.m[4] - err.m[1],  // 3-6 ; 2-8
              err.m[2] - err.m[8],
              err.m[6] - err.m[4]};
    Vec3 e[6];
    for (int i = 0; i < 3; i++) e[i] = {pErr[i], 0, 0};
    // 合并成 6 维误差
    double E[6] = {pErr.x, pErr.y, pErr.z, r.x * 0.5, r.y * 0.5, r.z * 0.5};
    if (std::abs(pErr.x) + std::abs(pErr.y) + std::abs(pErr.z) < 1e-8) return true;

    double J[6][6];
    buildJacobian(arm, q, J);
    // damped least squares: dq = (J^T J + λI)^-1 J^T E
    double A[6][6]{};
    for (int i = 0; i < 6; i++)
      for (int j = 0; j < 6; j++) {
        double acc = 0;
        for (int k = 0; k < 6; k++) acc += J[k][i] * J[k][j];
        A[i][j] = acc;
      }
    double lambda = 1e-3;
    double rhs[6]{};
    for (int i = 0; i < 6; i++) {
      for (int k = 0; k < 6; k++) rhs[i] += J[k][i] * E[k];
      A[i][i] += lambda;
    }
    // Gaussian elimination
    double M[6][7] ;
    for (int i = 0; i < 6; i++) { for (int j = 0; j < 6; j++) M[i][j] = A[i][j]; M[i][6] = rhs[i]; }
    for (int c = 0; c < 6; c++) {
      int piv = c;
      for (int r = c + 1; r < 6; r++) if (std::abs(M[r][c]) > std::abs(M[piv][c])) piv = r;
      if (piv != c) for (int j = 0; j < 7; j++) std::swap(M[c][j], M[piv][j]);
      double d = M[c][c];
      if (std::abs(d) < 1e-12) return false;
      for (int j = 0; j < 7; j++) M[c][j] /= d;
      for (int r = 0; r < 6; r++) {
        if (r == c) continue;
        double f = M[r][c];
        for (int j = 0; j < 7; j++) M[r][j] -= f * M[c][j];
      }
    }
    std::array<double, 6> dq{};
    for (int i = 0; i < 6; i++) dq[i] = M[i][6];
    for (int i = 0; i < 6; i++) q[i] += dq[i];
  }
  return false;
}

// 解校验：FK 后与目标位姿对比误差
inline Vec3 poseErrorV(const Mat4& a, const Mat4& b) {
  Vec3 p = a.translationV() - b.translationV();
  double ang = std::acos(std::clamp(
      (a.m[0]*b.m[0]+a.m[1]*b.m[1]+a.m[2]*b.m[2] +
       a.m[4]*b.m[4]+a.m[5]*b.m[5]+a.m[6]*b.m[6] +
       a.m[8]*b.m[8]+a.m[9]*b.m[9]+a.m[10]*b.m[10]) / 3.0, -1.0, 1.0));
  return {p.norm(), ang, 0};
}

// 逆运动学：多初值（当前位形 + 若干随机扰动）跑阻尼最小二乘，
// 再用 FK 自校验，取误差最小且满足限位的解。
inline bool inverseKinematics(const ArmModel& arm, const Mat4& target,
                              const std::array<double, 6>& cur,
                              std::array<double, 6>& out) {
  std::vector<std::array<double, 6>> seeds;
  seeds.push_back(cur);
  // 若干确定性扰动初值，规避局部极小
  std::array<std::array<double, 6>, 6> pert{{
      {0.5, 0.5, 0.5, 0, 0, 0},
      {-0.5, 0.0, 0.0, 0.8, 0.0, 0.0},
      {0.0, 1.0, -1.0, 0.5, 0.5, 0.0},
      {2.2, 0.0, 0.0, 0.0, 0.0, 0.0},
      {0.0, 0.0, 0.0, 0.3, -0.6, 1.0},
      {1.8, -1.2, 1.2, 1.5, 1.0, -0.8},
  }};
  for (auto& p : pert) {
    auto s = cur;
    for (int i = 0; i < 6; i++) s[i] += p[i];
    seeds.push_back(s);
  }

  double best = std::numeric_limits<double>::max();
  bool found = false;
  for (auto& seed : seeds) {
    auto q = seed;
    if (!numericIK(arm, target, q)) continue;
    bool ok = true;
    for (int i = 0; i < 6; i++)
      if (q[i] < arm.qmin[i] - 1e-6 || q[i] > arm.qmax[i] + 1e-6) ok = false;
    if (!ok) continue;
    Vec3 err = poseErrorV(forwardKinematicsT0_6(arm, q), target);
    double score = err.x + err.y;
    if (score < best) { best = score; out = q; found = true; }
  }
  return found;
}

}  // namespace arm