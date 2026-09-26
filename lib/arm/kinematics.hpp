// lib/arm/kinematics.hpp — Layer 0 运动学引擎
// 纯 C++ 运动学引擎：DH 建模、正运动学(FK)、解析逆运动学(IK，8 分支闭式解)、
// 数值 IK 回退（阻尼最小二乘）、几何雅可比、奇异性检测（σ_min / 可操控度）。
// 全部 header-only，便于单个翻译单元编译。
//
// 模型结构（urStyle 桌面 6 轴，标准 DH）：
//   j1 竖直；j2 ∥ j3 ∥ j4（水平肩轴系）；z4 ⊥ 腕轴系且随 j4 滚转；
//   z5 ⊥ z4；工具轴 z6 = z5。腕部带横向偏置 d5（z4 方向），非球腕，
//   但可解析解耦：θ1 由 ŵ·W = d4 闭式求出（W = p − d6·n），
//   随后 z4 = ±(ŵ×n)/|ŵ×n| → O4 平面 2R 解 (θ2,θ3) → θ4 → (θ5,θ6)。
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

// ---------- 4x4 齐次变换（行优先存储, m[16], 每行4个元素；末行恒为 0001） ----------
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
  // 轴角构造旋转（Rodrigues）
  static Mat4 rotationAxisAngle(const Vec3& axis, double ang) {
    Vec3 u = axis.normalized();
    double c = std::cos(ang), s = std::sin(ang), t = 1.0 - c;
    Mat4 r = identity();
    r.m[0] = t*u.x*u.x + c;     r.m[1] = t*u.x*u.y - s*u.z; r.m[2]  = t*u.x*u.z + s*u.y;
    r.m[4] = t*u.x*u.y + s*u.z; r.m[5] = t*u.y*u.y + c;     r.m[6]  = t*u.y*u.z - s*u.x;
    r.m[8] = t*u.x*u.z - s*u.y; r.m[9] = t*u.y*u.z + s*u.x; r.m[10] = t*u.z*u.z + c;
    return r;
  }
  Vec3 rotationCol(int c) const { return {m[c], m[4 + c], m[8 + c]}; }
  Vec3 rotationRow(int r) const { return {m[r * 4], m[r * 4 + 1], m[r * 4 + 2]}; }
  Vec3 translationV() const { return {m[3], m[7], m[11]}; }
  double trace3() const { return m[0] + m[5] + m[10]; }
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

// 旋转插值：R(s) = A_rot · exp(s · log(A_rotᵀB_rot))，绕相对旋转轴等角速度，s∈[0,1]
// 只用两者的旋转部分；返回纯旋转（平移为零），可与 translation(pos) 直接复合。
inline Mat4 rotationSlerp(const Mat4& A, const Mat4& B, double s) {
  Mat4 Arot = A;
  Arot.m[3] = Arot.m[7] = Arot.m[11] = 0;
  Mat4 Brot = B;
  Brot.m[3] = Brot.m[7] = Brot.m[11] = 0;
  Mat4 C = Arot.inverse() * Brot;   // 相对旋转
  double tr = C.trace3();
  double cosang = std::clamp((tr - 1.0) * 0.5, -1.0, 1.0);
  double ang = std::acos(cosang);
  if (ang < 1e-9) return Arot;
  if (ang > kPi - 1e-6) {
    // 近 180°：从 C 的对角取稳定轴
    Vec3 diag{C.m[0], C.m[5], C.m[10]};
    int big = 0;
    if (diag[1] > diag[big]) big = 1;
    if (diag[2] > diag[big]) big = 2;
    Vec3 est{}; est[big] = 1.0;
    Vec3 col = C.rotationCol(big);
    Vec3 n = (est + col).normalized();
    if (n.norm() < 0.5) n = Vec3{1, 0, 0};
    return Arot * Mat4::rotationAxisAngle(n, ang * s);
  }
  Vec3 axis{C.m[9] - C.m[6], C.m[2] - C.m[8], C.m[4] - C.m[1]};  // vee(R−Rᵀ)
  return Arot * Mat4::rotationAxisAngle(axis, ang * s);
}

// ---------- DH 参数（标准 DH） ----------
struct LinkDH {
  double a = 0, alpha = 0, d = 0;  // a: 连杆长, alpha: 扭转(°), d: 偏置
  double thetaOffset = 0;          // 零位偏置 (rad)
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

// 单个 DH 变换矩阵：Rz(θ)·Tz(d)·Tx(a)·Rx(α)
inline Mat4 DHTransform(const LinkDH& l, double q) {
  double th = DHTheta(l, q);
  double ca = std::cos(l.alpha * kD2R), sa = std::sin(l.alpha * kD2R);
  double cth = std::cos(th), sth = std::sin(th);
  Mat4 T = Mat4::identity();
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

// 几何雅可比 (6x6), 列 j = [ (z_j × (p6 − p_j)), z_j ]ᵀ（基座系，角部映射 dq→ω_base）
inline void buildJacobian(const ArmModel& arm, const std::array<double, 6>& q,
                          double J[6][6]) {
  std::vector<Mat4> T = forwardKinematics(arm, q);
  Vec3 p6 = T[6].translationV();
  Vec3 z0 = {0, 0, 1};
  Vec3 axes[6], pos[6];  // 关节 j 轴单位向量与原点（j=0..5 对应 T[j]，关节 j+1 绕 z_j）
  axes[0] = z0; pos[0] = T[0].translationV();
  for (int i = 1; i < 6; i++) { axes[i] = T[i].transformDir(z0); pos[i] = T[i].translationV(); }
  for (int j = 0; j < 6; j++) {
    Vec3 v = axes[j].cross(p6 - pos[j]);
    for (int r = 0; r < 3; r++) { J[r][j] = v[r]; J[3 + r][j] = axes[j][r]; }
  }
}

// ---------- 6x6 线性代数小工具 ----------
// 列主元高斯消元解 Ax=b；奇异返回 false
inline bool solveLinear6(double A[6][6], double b[6], double x[6]) {
  double M[6][7];
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 6; j++) M[i][j] = A[i][j];
    M[i][6] = b[i];
  }
  for (int c = 0; c < 6; c++) {
    int piv = c;
    for (int r = c + 1; r < 6; r++) if (std::abs(M[r][c]) > std::abs(M[piv][c])) piv = r;
    if (piv != c) for (int j = c; j < 7; j++) std::swap(M[c][j], M[piv][j]);
    double d = M[c][c];
    if (std::abs(d) < 1e-12) return false;
    for (int j = c; j < 7; j++) M[c][j] /= d;
    for (int r = 0; r < 6; r++) {
      if (r == c) continue;
      double f = M[r][c];
      for (int j = c; j < 7; j++) M[r][j] -= f * M[c][j];
    }
  }
  for (int i = 0; i < 6; i++) x[i] = M[i][6];
  return true;
}

// 行列式（列主元消元，带符号）
inline double det6(const double A0[6][6]) {
  double M[6][6];
  for (int i = 0; i < 6; i++) for (int j = 0; j < 6; j++) M[i][j] = A0[i][j];
  double det = 1.0;
  for (int c = 0; c < 6; c++) {
    int piv = c;
    for (int r = c + 1; r < 6; r++) if (std::abs(M[r][c]) > std::abs(M[piv][c])) piv = r;
    if (std::abs(M[piv][c]) < 1e-18) return 0.0;
    if (piv != c) { for (int j = 0; j < 6; j++) std::swap(M[c][j], M[piv][j]); det = -det; }
    det *= M[c][c];
    for (int r = c + 1; r < 6; r++) {
      double f = M[r][c] / M[c][c];
      for (int j = c; j < 6; j++) M[r][j] -= f * M[c][j];
    }
  }
  return det;
}

// 对称 6x6 全部特征值（循环 Jacobi），返回升序排列
inline std::array<double, 6> eigenSymmetric6(double A0[6][6]) {
  double A[6][6];
  for (int i = 0; i < 6; i++) for (int j = 0; j < 6; j++) A[i][j] = 0.5 * (A0[i][j] + A0[j][i]);
  for (int sweep = 0; sweep < 32; sweep++) {
    double off = 0;
    for (int p = 0; p < 6; p++) for (int q = p + 1; q < 6; q++) off += A[p][q] * A[p][q];
    if (off < 1e-24) break;
    for (int p = 0; p < 6; p++) {
      for (int q = p + 1; q < 6; q++) {
        if (std::abs(A[p][q]) < 1e-30) continue;
        double theta = (A[q][q] - A[p][p]) / (2.0 * A[p][q]);
        double t = (theta >= 0 ? 1.0 : -1.0) / (std::abs(theta) + std::sqrt(theta * theta + 1.0));
        double c = 1.0 / std::sqrt(t * t + 1.0), s = t * c;
        for (int k = 0; k < 6; k++) {
          double akp = A[k][p], akq = A[k][q];
          A[k][p] = c * akp - s * akq;
          A[k][q] = s * akp + c * akq;
        }
        for (int k = 0; k < 6; k++) {
          double apk = A[p][k], aqk = A[q][k];
          A[p][k] = c * apk - s * aqk;
          A[q][k] = s * apk + c * aqk;
        }
      }
    }
  }
  std::array<double, 6> ev{};
  for (int i = 0; i < 6; i++) ev[i] = A[i][i];
  std::sort(ev.begin(), ev.end());
  return ev;
}

// ---------- 奇异性度量 ----------
// 可操控度（无量纲）：w̃ = sqrt(det(J̃J̃ᵀ)) = |det J̃|，J̃ 为 Lchar 归一后的无量纲雅可比
// （线速度行 ÷ Lchar、角速度行不变，与 singularityIndex 同一行标定）。量纲 [1]。
inline double manipulability(const ArmModel& arm, const std::array<double, 6>& q) {
  constexpr double Lchar = 0.5;   // 特征长度 = 半臂展 (m)，与 singularityIndex 一致
  double J[6][6];
  buildJacobian(arm, q, J);
  double JJt[6][6]{};
  for (int r = 0; r < 6; r++)
    for (int c = 0; c < 6; c++) {
      double acc = 0;
      for (int k = 0; k < 6; k++) {
        double jr = (r < 3 ? J[r][k] / Lchar : J[r][k]);
        double jc = (c < 3 ? J[c][k] / Lchar : J[c][k]);
        acc += jr * jc;
      }
      JJt[r][c] = acc;
    }
  return std::sqrt(std::abs(det6(JJt)));
}

// 最小奇异值 σ_min(J)：JJᵀ 的最小特征值开方（真·奇异度量，越小越贴近奇异）
inline double minSingularValue(const ArmModel& arm, const std::array<double, 6>& q) {
  double J[6][6];
  buildJacobian(arm, q, J);
  double JJt[6][6]{};
  for (int r = 0; r < 6; r++)
    for (int c = 0; c < 6; c++) {
      double acc = 0;
      for (int k = 0; k < 6; k++) acc += J[r][k] * J[c][k];
      JJt[r][c] = acc;
    }
  auto ev = eigenSymmetric6(JJt);
  return std::sqrt(std::max(ev[0], 0.0));
}

// 无量纲奇异指标 η = σ_min/σ_max ∈ (0,1]：1 = 灵活各向同性，0 = 奇异。
// 对雅可比做行标定（线部 ÷ 特征长度 L，角部不变）消除量纲混合后取条件比。
inline double singularityIndex(const ArmModel& arm, const std::array<double, 6>& q) {
  constexpr double Lchar = 0.5;   // 特征长度 = 半臂展 (m)
  double J[6][6];
  buildJacobian(arm, q, J);
  double JJt[6][6]{};
  for (int r = 0; r < 6; r++)
    for (int c = 0; c < 6; c++) {
      double acc = 0;
      for (int k = 0; k < 6; k++) {
        double jr = (r < 3 ? J[r][k] / Lchar : J[r][k]);
        double jc = (c < 3 ? J[c][k] / Lchar : J[c][k]);
        acc += jr * jc;
      }
      JJt[r][c] = acc;
    }
  auto ev = eigenSymmetric6(JJt);
  double smax = std::sqrt(std::max(ev[5], 0.0));
  double smin = std::sqrt(std::max(ev[0], 0.0));
  return smax > 1e-12 ? smin / smax : 0.0;
}

inline bool isSingularNear(const ArmModel& arm, const std::array<double, 6>& q,
                           double threshold = 0.02) {
  return singularityIndex(arm, q) < threshold;
}

// 奇异软处理系数：η 平滑降速 (floor..1)，接近奇异时末端减速。
// etaFull 按本臂 η 分布重标（3000 随机位形实测 p5=0.0059/中位 0.0753/p95=0.1953）：
// 0.12 会让 70.9% 位形进入降速区 → 0.05 后 34.0%，且 smoothstep 平滑（η≥0.04 时 scale≥0.92）。
inline double singularityScale(const ArmModel& arm, const std::array<double, 6>& q,
                               double etaFull = 0.05, double floorScale = 0.2) {
  double u = singularityIndex(arm, q);
  if (u >= etaFull) return 1.0;
  if (u <= 1e-9) return floorScale;
  double r = u / etaFull;                      // 0..1
  double smooth = r * r * (3.0 - 2.0 * r);     // smoothstep
  return floorScale + (1.0 - floorScale) * smooth;
}

// ---------- 位姿误差 ----------
// 返回 {位置误差范数, 姿态误差角(rad), 0}；角度公式 θ = acos(clamp((tr(RaᵀRb)−1)/2))
inline Vec3 poseErrorV(const Mat4& a, const Mat4& b) {
  Vec3 p = a.translationV() - b.translationV();
  double tr = a.m[0]*b.m[0] + a.m[1]*b.m[1] + a.m[2]*b.m[2] +
              a.m[4]*b.m[4] + a.m[5]*b.m[5] + a.m[6]*b.m[6] +
              a.m[8]*b.m[8] + a.m[9]*b.m[9] + a.m[10]*b.m[10];  // = tr(Raᵀ Rb)
  double ang = std::acos(std::clamp((tr - 1.0) * 0.5, -1.0, 1.0));
  return {p.norm(), ang, 0};
}

// 基座系位姿误差向量：e[0:3] = p_target − p_cur；e[3:6] = 精确轴角旋转向量误差（cur → target）
inline void poseErrorVec(const Mat4& cur, const Mat4& target, double e[6]) {
  Vec3 dp = target.translationV() - cur.translationV();
  e[0] = dp.x; e[1] = dp.y; e[2] = dp.z;
  // A = Rt Rcᵀ 的轴角：vee(A − Aᵀ)/2 = sinθ·n → 归一化后乘 θ 得精确旋转向量（空间系）
  Mat4 A = target * cur.inverse();
  double cosang = std::clamp((A.trace3() - 1.0) * 0.5, -1.0, 1.0);
  double ang = std::acos(cosang);
  Vec3 v{0.5 * (A.m[9] - A.m[6]), 0.5 * (A.m[2] - A.m[8]), 0.5 * (A.m[4] - A.m[1])};
  double s = v.norm();
  if (s < 1e-12 || ang < 1e-12) {
    e[3] = e[4] = e[5] = 0;
  } else {
    Vec3 n = v * (ang / s);        // = θ·n（|v| = sinθ）
    e[3] = n.x; e[4] = n.y; e[5] = n.z;
  }
}

// ---------- 数值 IK：阻尼最小二乘 ----------
// 误差与雅可比统一在基座系；位置 + 姿态同时判收敛。
inline bool numericIK(const ArmModel& arm, const Mat4& target,
                      std::array<double, 6>& q, int iters = 80) {
  const double lambda = 1e-3;
  for (int it = 0; it < iters; it++) {
    Mat4 cur = forwardKinematicsT0_6(arm, q);
    double E[6];
    poseErrorVec(cur, target, E);
    double posErr = std::sqrt(E[0]*E[0] + E[1]*E[1] + E[2]*E[2]);
    double rotErr = std::sqrt(E[3]*E[3] + E[4]*E[4] + E[5]*E[5]);
    if (posErr < 1e-9 && rotErr < 1e-9) return true;
    double J[6][6];
    buildJacobian(arm, q, J);
    // damped least squares: dq = (JᵀJ + λI)⁻¹ Jᵀ E
    double A[6][6]{};
    for (int i = 0; i < 6; i++)
      for (int j = 0; j < 6; j++) {
        double acc = 0;
        for (int k = 0; k < 6; k++) acc += J[k][i] * J[k][j];
        A[i][j] = acc;
      }
    double rhs[6]{};
    for (int i = 0; i < 6; i++) {
      for (int k = 0; k < 6; k++) rhs[i] += J[k][i] * E[k];
      A[i][i] += lambda;
    }
    std::array<double, 6> dq{};
    if (!solveLinear6(A, rhs, dq.data())) return false;
    for (int i = 0; i < 6; i++)
      q[i] = std::clamp(q[i] + dq[i], arm.qmin[i], arm.qmax[i]);   // 关节限位投影（每步迭代）
  }
  // 迭代耗尽：按最终误差判成败（宽松一档）；越限解一律不接受
  Mat4 cur = forwardKinematicsT0_6(arm, q);
  Vec3 err = poseErrorV(cur, target);
  bool inLim = true;
  for (int i = 0; i < 6; i++)
    if (q[i] < arm.qmin[i] - 1e-6 || q[i] > arm.qmax[i] + 1e-6) inLim = false;
  return err.x < 1e-6 && err.y < 1e-6 && inLim;
}

// ---------- 解析 IK（闭式 8 分支） ----------
// 解耦推导（标准 DH，urStyle 结构）：
//   n  = z5 = 末端工具轴 (= z6)，W = p − d6·n = O5（j5、j6 轴交点）
//   (1) θ1:  ŵ(θ1)·W = d4，ŵ = (sinθ1, −cosθ1, 0) —— 与腕部分支无关
//   (2) z4 = σ(ŵ×n)/|ŵ×n|，σ=±1 为腕翻转分支（n∥ŵ 时奇异，走数值回退）
//   (3) O4 = W − d5·z4；平面投影 Π(O4−O1) = a2·x̂2(θ2) + a3·x̂3(θ2+θ3) 标准 2R 反解（肘 2 分支）
//   (4) θ4 = atan2(z4·x3, −z4·y3)；M = R03ᵀR → θ5 = atan2(s5, M22)，
//       s5 = −(M02·c4 + M12·s4)；θ6 = atan2(−M21, M20)（|s5|≈0 退化时按耦合解）
inline int analyticIKAll(const ArmModel& arm, const Mat4& target,
                         std::vector<std::array<double, 6>>& out) {
  out.clear();
  const double d1 = arm.links[0].d;
  const double a2 = arm.links[1].a, a3 = arm.links[2].a;
  const double d4 = arm.links[3].d, d5 = arm.links[4].d, d6 = arm.links[5].d;
  const double off1 = arm.links[0].thetaOffset, off2 = arm.links[1].thetaOffset;
  const double off3 = arm.links[2].thetaOffset, off4 = arm.links[3].thetaOffset;
  const double off5 = arm.links[4].thetaOffset, off6 = arm.links[5].thetaOffset;

  Vec3 n = target.rotationCol(2);                 // 工具轴 z6 = z5
  Vec3 W = target.translationV() - n * d6;        // O5

  // (1) θ1：Wx s1 − Wy c1 = d4
  double R1 = std::hypot(W.x, W.y);
  if (R1 < std::abs(d4) - 1e-12) return 0;        // 目标横向距离不足
  double beta = std::atan2(-W.y, W.x);
  double ratio = std::clamp(d4 / std::max(R1, 1e-12), -1.0, 1.0);
  double th1_c[2] = {-beta + std::asin(ratio), -beta + kPi - std::asin(ratio)};

  for (int i1 = 0; i1 < 2; i1++) {
    double th1 = th1_c[i1];
    double c1 = std::cos(th1), s1 = std::sin(th1);
    Vec3 w_hat{s1, -c1, 0.0};

    // (2) z4 = σ(ŵ×n)/|ŵ×n|
    Vec3 cross_wn = w_hat.cross(n);
    double sw = cross_wn.norm();                  // = |sin θ5|（腕部奇异判据）
    if (sw < 1e-8) continue;                      // 退化 → 数值回退
    for (int sig = 0; sig < 2; sig++) {
      Vec3 z4 = cross_wn * ((sig == 0 ? 1.0 : -1.0) / sw);

      // (3) 平面 2R 反解 (θ2, θ3)
      Vec3 O1{0, 0, d1};
      Vec3 O4 = W - z4 * d5;
      Vec3 t = O4 - O1;
      Vec3 e1{c1, s1, 0.0};
      double t1 = t.dot(e1), t2 = t.dot(Vec3{0, 0, 1});
      double c3 = (t1 * t1 + t2 * t2 - a2 * a2 - a3 * a3) / (2.0 * a2 * a3);
      if (c3 < -1.0 - 1e-9 || c3 > 1.0 + 1e-9) continue;  // 不可达
      c3 = std::clamp(c3, -1.0, 1.0);
      for (int elbow = 0; elbow < 2; elbow++) {
        double th3 = (elbow == 0 ? 1.0 : -1.0) * std::acos(c3);
        double s3 = std::sin(th3);
        double th2 = std::atan2(t2, t1) - std::atan2(a3 * s3, a2 + a3 * c3);
        double th23 = th2 + th3;

        // (4) R03 与腕角
        double c23 = std::cos(th23), s23 = std::sin(th23);
        // R03 = Rz(θ1)·Rx(90°)·Rz(θ23)
        //   Rx(90)·Rz(θ23) = [[c23, −s23, 0], [0, 0, −1], [s23, c23, 0]]
        //   Rz(θ1) 左乘：
        double R03[3][3] = {
            {c1 * c23, -c1 * s23, s1},
            {s1 * c23, -s1 * s23, -c1},
            {s23, c23, 0}};
        double x3[3] = {R03[0][0], R03[1][0], R03[2][0]};
        double y3[3] = {R03[0][1], R03[1][1], R03[2][1]};

        // θ4：z4 = s4·x3 − c4·y3
        double z4x3 = z4.x * x3[0] + z4.y * x3[1] + z4.z * x3[2];
        double z4y3 = z4.x * y3[0] + z4.y * y3[1] + z4.z * y3[2];
        double th4 = std::atan2(z4x3, -z4y3);
        double c4 = std::cos(th4), s4 = std::sin(th4);

        // M = R03ᵀ · R
        double M[3][3];
        for (int r = 0; r < 3; r++)
          for (int c = 0; c < 3; c++)
            M[r][c] = R03[0][r] * target.m[c] + R03[1][r] * target.m[4 + c] + R03[2][r] * target.m[8 + c];

        double s5 = -(M[0][2] * c4 + M[1][2] * s4);
        double c5 = M[2][2];
        double th5, th6;
        if (std::hypot(s5, c5) < 1e-12) continue;
        th5 = std::atan2(s5, c5);
        if (std::abs(s5) > 1e-8) {
          th6 = std::atan2(-M[2][1], M[2][0]);
          if (sig == 1) th6 += kPi;   // 腕翻转分支：z4 取反翻转 R06 的 x6/y6 平面，θ6 补 π
        } else {
          // 腕奇异：θ4 已由 z4 定死，θ6 从剩余自由度闭式解
          double c5s = c5 >= 0 ? 1.0 : -1.0;      // θ5 = 0 或 π
          // Q = Rz(−θ4)·M = Rx(90)Rz(θ5)Rx(−90)Rz(θ6)
          double Q[3][3];
          double rc[2] = {c4, -s4}, rs[2] = {s4, c4};  // Rz(−θ4) 行合成用
          for (int r = 0; r < 3; r++)
            for (int c = 0; c < 3; c++)
              Q[r][c] = (r == 2 ? (r == 2 ? 1.0 : 0.0) * M[2][c]
                                : rc[r] * M[0][c] + rs[r] * M[1][c]);
          Q[2][0] = M[2][0]; Q[2][1] = M[2][1]; Q[2][2] = M[2][2];
          if (c5s > 0) th6 = std::atan2(Q[1][0], Q[0][0]);       // θ5=0:  Q=Rz(θ6)
          else        th6 = std::atan2(-Q[1][0], -Q[0][0]);      // θ5=π
        }

        std::array<double, 6> qq{th1 - off1, th2 - off2, th3 - off3,
                                 th4 - off4, th5 - off5, th6 - off6};
        // FK 自校验 + 去重
        Vec3 err = poseErrorV(forwardKinematicsT0_6(arm, qq), target);
        if (err.x > 1e-6 || err.y > 1e-6) continue;
        bool dup = false;
        for (auto& s : out) {
          double dmax = 0;
          for (int i = 0; i < 6; i++) dmax = std::max(dmax, std::abs(s[i] - qq[i]));
          if (dmax < 1e-7) { dup = true; break; }
        }
        if (!dup) out.push_back(qq);
      }
    }
  }
  return static_cast<int>(out.size());
}

// 按「限位优先 + 与当前构型连续」排序取最优
inline bool analyticIK(const ArmModel& arm, const Mat4& target,
                       const std::array<double, 6>& cur, std::array<double, 6>& out) {
  std::vector<std::array<double, 6>> sols;
  if (analyticIKAll(arm, target, sols) == 0) return false;
  double best = std::numeric_limits<double>::max();
  bool found = false;
  for (auto& s : sols) {
    bool ok = true;
    double dev = 0;
    for (int i = 0; i < 6; i++) {
      if (s[i] < arm.qmin[i] - 1e-6 || s[i] > arm.qmax[i] + 1e-6) ok = false;
      dev += std::abs(s[i] - cur[i]);
    }
    if (!ok) continue;                           // 限位硬过滤：越限解一律不返回
    if (dev < best) { best = dev; out = s; found = true; }
  }
  return found;                                  // 无合规候选 → false（宁缺勿滥）
}

// 统一 IK 入口：解析优先（全分支 + 连续性择优），失败回退多初值数值 IK，
// 全部解经 FK 自校验；限位为硬约束——越限解一律过滤，全越限时返回 false。
inline bool inverseKinematics(const ArmModel& arm, const Mat4& target,
                              const std::array<double, 6>& cur,
                              std::array<double, 6>& out) {
  struct Cand { std::array<double, 6> q; double score; bool inLimit; };
  std::vector<Cand> cands;

  // 解析分支
  {
    std::vector<std::array<double, 6>> sols;
    analyticIKAll(arm, target, sols);
    for (auto& s : sols) cands.push_back({s, 0.0, true});
  }

  // 数值多初值（确定性扰动）：解析无解 **或全越限** 时回退（numericIK 自带限位投影）
  bool hasOk = false;
  for (auto& c : cands) {
    bool ok = true;
    for (int i = 0; i < 6; i++)
      if (c.q[i] < arm.qmin[i] - 1e-6 || c.q[i] > arm.qmax[i] + 1e-6) ok = false;
    if (ok) { hasOk = true; break; }
  }
  if (!hasOk) {
    std::vector<std::array<double, 6>> seeds;
    seeds.push_back(cur);
    const std::array<std::array<double, 6>, 6> pert{{
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
    for (auto& seed : seeds) {
      auto q = seed;
      if (!numericIK(arm, target, q)) continue;
      Vec3 err = poseErrorV(forwardKinematicsT0_6(arm, q), target);
      if (err.x > 1e-5 || err.y > 1e-5) continue;
      cands.push_back({q, err.x + err.y, true});
    }
  }

  double best = std::numeric_limits<double>::max();
  bool found = false;
  for (auto& c : cands) {
    bool ok = true;
    double dev = 0;
    for (int i = 0; i < 6; i++) {
      if (c.q[i] < arm.qmin[i] - 1e-6 || c.q[i] > arm.qmax[i] + 1e-6) ok = false;
      dev += std::abs(c.q[i] - cur[i]);
    }
    if (!ok) continue;                           // 限位硬过滤：越限解一律不返回
    Vec3 err = poseErrorV(forwardKinematicsT0_6(arm, c.q), target);
    double score = dev + 10.0 * (err.x + err.y);
    if (score < best) { best = score; out = c.q; found = true; }
  }
  return found;                                  // 全部越限 → false
}

}  // namespace arm
