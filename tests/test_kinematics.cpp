// tests/test_kinematics.cpp — 运动学引擎单元测试
// 覆盖：FK 已知位形、齐次矩阵末行回归（历史 bug）、θ1 解耦方程、
// 解析 IK 8 分支往返、数值 IK 回归、雅可比数值校验、σ_min/可操控度、位姿误差公式。
#include "arm/kinematics.hpp"
#include <algorithm>
#include <cstdio>
#include <random>
#include <string>
#include <vector>
using namespace arm;

static int g_fail = 0;
#define CHECK(cond, ...)                                                     \
  do {                                                                       \
    if (!(cond)) {                                                           \
      std::printf("FAIL %s:%d  ", __FILE__, __LINE__);                       \
      std::printf(__VA_ARGS__);                                              \
      std::printf("\n");                                                     \
      g_fail++;                                                              \
    }                                                                        \
  } while (0)

int main(int argc, char** argv) {
  bool etaStats = false;
  for (int i = 1; i < argc; i++)
    if (std::string(argv[i]) == "--eta-stats") etaStats = true;
  auto m = ArmModel::urStyle();

  // ---- 1. DH 变换末行恒为 0001（回归：历史 bug 导致链式平移丢失）----
  {
    Mat4 T = DHTransform(m.links[0], 0.3);
    CHECK(std::abs(T.m[12]) < 1e-12 && std::abs(T.m[13]) < 1e-12 &&
              std::abs(T.m[14]) < 1e-12 && std::abs(T.m[15] - 1.0) < 1e-12,
          "DHTransform bottom row != 0001");
  }

  // ---- 2. FK 已知位形 q=0：手算 O1..O4 与 z1 ----
  {
    std::array<double, 6> q{};
    auto T = forwardKinematics(m, q);
    Vec3 O1 = T[1].translationV(), O2 = T[2].translationV();
    Vec3 O3 = T[3].translationV(), O4 = T[4].translationV();
    CHECK((O1 - Vec3{0, 0, 0.1625}).norm() < 1e-12, "O1=%g %g %g", O1.x, O1.y, O1.z);
    CHECK((O2 - Vec3{0, 0, 0.5875}).norm() < 1e-12, "O2=%g %g %g", O2.x, O2.y, O2.z);
    CHECK((O3 - Vec3{0, 0, 0.9795}).norm() < 1e-12, "O3=%g %g %g", O3.x, O3.y, O3.z);
    CHECK((O4 - Vec3{0, -0.1333, 0.9795}).norm() < 1e-12, "O4=%g %g %g", O4.x, O4.y, O4.z);
    Vec3 z1 = T[1].rotationCol(2);
    CHECK((z1 - Vec3{0, -1, 0}).norm() < 1e-12, "z1=%g %g %g", z1.x, z1.y, z1.z);
    // z5 == z6
    CHECK((T[5].rotationCol(2) - T[6].rotationCol(2)).norm() < 1e-12, "z5 != z6");
  }

  // ---- 3. 结构恒等式 + θ1 解耦方程（解析 IK 地基）----
  {
    std::mt19937 rng(11);
    std::uniform_real_distribution<double> U(-2.5, 2.5);
    double d4 = m.links[3].d, d6 = m.links[5].d;
    for (int k = 0; k < 200; k++) {
      std::array<double, 6> q;
      for (auto& x : q) x = U(rng);
      auto T = forwardKinematics(m, q);
      Vec3 w1 = T[1].rotationCol(2), w2 = T[2].rotationCol(2), w3 = T[3].rotationCol(2);
      CHECK((w1 - w2).norm() < 1e-12 && (w2 - w3).norm() < 1e-12, "j2/j3/j4 轴不平行");
      Vec3 n = T[5].rotationCol(2);
      Vec3 W = T[6].translationV() - n * d6;
      CHECK(std::abs(w1.dot(W) - d4) < 1e-9, "θ1 解耦方程残差 %.3e", w1.dot(W) - d4);
    }
  }

  // ---- 4. 解析 IK 全分支往返 ----
  {
    std::mt19937 rng(23);
    std::uniform_real_distribution<double> U(-2.2, 2.2);
    int solved = 0, nsols_total = 0;
    for (int k = 0; k < 300; k++) {
      std::array<double, 6> q;
      for (auto& x : q) x = U(rng);
      Mat4 T = forwardKinematicsT0_6(m, q);
      std::vector<std::array<double, 6>> sols;
      int n = analyticIKAll(m, T, sols);
      if (n > 0) {
        solved++;
        nsols_total += n;
        // 至少有一个解 FK 复现目标
        double best = 1e9;
        for (auto& s : sols) {
          Vec3 e = poseErrorV(forwardKinematicsT0_6(m, s), T);
          best = std::min(best, e.x + e.y);
        }
        CHECK(best < 1e-6, "解析解未复现目标, best=%.2e", best);
      }
    }
    std::printf("[kin] analytic IK: %d/300 solved, avg branches %.1f\n",
                solved, solved ? double(nsols_total) / solved : 0.0);
    CHECK(solved >= 295, "解析 IK 覆盖率过低 %d（θ6 腕翻转修复后应 300/300）", solved);
  }

  // ---- 5. inverseKinematics（解析优先 + 连续性）----
  {
    std::mt19937 rng(31);
    std::uniform_real_distribution<double> U(-2.0, 2.0);
    int ok = 0;
    for (int k = 0; k < 100; k++) {
      std::array<double, 6> q;
      for (auto& x : q) x = U(rng);
      Mat4 T = forwardKinematicsT0_6(m, q);
      std::array<double, 6> out{};
      std::array<double, 6> cur{};
      if (inverseKinematics(m, T, cur, out)) {
        Vec3 e = poseErrorV(forwardKinematicsT0_6(m, out), T);
        CHECK(e.x < 1e-5 && e.y < 1e-5, "IK 解误差 %.2e/%.2e", e.x, e.y);
        bool inLim = true;   // P0-补-2：返回解必须限位内（硬过滤）
        for (int i = 0; i < 6; i++)
          if (out[i] < m.qmin[i] - 1e-6 || out[i] > m.qmax[i] + 1e-6) inLim = false;
        CHECK(inLim, "IK 返回越限解");
        ok++;
      }
    }
    CHECK(ok >= 95, "IK 成功率过低 %d", ok);
  }

  // ---- 6. 数值 IK 回归（旋转误差提取 + 收敛判据 + 坐标系一致）----
  {
    std::array<double, 6> q{0.4, -0.9, 1.0, 0.6, -0.5, 0.8};
    Mat4 T = forwardKinematicsT0_6(m, q);
    std::array<double, 6> qs = q;
    for (int i = 0; i < 6; i++) qs[i] += 0.15;   // 扰动初值
    CHECK(numericIK(m, T, qs), "数值 IK 不收敛");
    Vec3 e = poseErrorV(forwardKinematicsT0_6(m, qs), T);
    CHECK(e.x < 1e-6 && e.y < 1e-6, "数值 IK 残差 %.2e/%.2e", e.x, e.y);
    // P0-补-2：numericIK 每步限位投影——任何返回解不得越限（修复前 136/200 越限）
    bool inLim = true;
    for (int i = 0; i < 6; i++)
      if (qs[i] < m.qmin[i] - 1e-6 || qs[i] > m.qmax[i] + 1e-6) inLim = false;
    CHECK(inLim, "numericIK 返回越限解");
  }
  {
    // 远离限位的随机扰动回归：200 次数值 IK 全部限位内
    std::mt19937 rng(77);
    std::uniform_real_distribution<double> U(-1.2, 1.2);
    for (int k = 0; k < 200; k++) {
      std::array<double, 6> q;
      for (auto& x : q) x = U(rng);
      Mat4 T = forwardKinematicsT0_6(m, q);
      std::array<double, 6> qs = q;
      for (int i = 0; i < 6; i++) qs[i] += 0.3;
      if (!numericIK(m, T, qs)) continue;
      for (int i = 0; i < 6; i++)
        CHECK(qs[i] >= m.qmin[i] - 1e-6 && qs[i] <= m.qmax[i] + 1e-6, "numericIK 越限 k=%d i=%d", k, i);
    }
  }

  // ---- 7. 雅可比 vs 数值微分 ----
  {
    std::array<double, 6> q{0.3, -0.7, 0.9, 0.2, 0.6, -0.3};
    double J[6][6];
    buildJacobian(m, q, J);
    double h = 1e-7;
    for (int j = 0; j < 6; j++) {
      auto qp = q, qm = q;
      qp[j] += h; qm[j] -= h;
      Mat4 Tp = forwardKinematicsT0_6(m, qp), Tm = forwardKinematicsT0_6(m, qm);
      Vec3 dp = (Tp.translationV() - Tm.translationV()) * (1.0 / (2 * h));
      // 角速度数值差分（空间系）：vee(Ṙ Rᵀ)
      double Rp[3][3], Rm[3][3], R[3][3];
      for (int r = 0; r < 3; r++)
        for (int c = 0; c < 3; c++) {
          Rp[r][c] = Tp.m[r * 4 + c];
          Rm[r][c] = Tm.m[r * 4 + c];
          R[r][c] = forwardKinematicsT0_6(m, q).m[r * 4 + c];
        }
      double w[3] = {0, 0, 0};
      // ω = vee(Ṙ Rᵀ) 的分量公式
      double dRdt[3][3];
      for (int r = 0; r < 3; r++)
        for (int c = 0; c < 3; c++) dRdt[r][c] = (Rp[r][c] - Rm[r][c]) / (2 * h);
      double S[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
      for (int r = 0; r < 3; r++)
        for (int c = 0; c < 3; c++) {
          double acc = 0;
          for (int a2 = 0; a2 < 3; a2++) acc += dRdt[r][a2] * R[c][a2];  // dRdt * Rᵀ
          S[r][c] = acc;
        }
      w[0] = 0.5 * (S[2][1] - S[1][2]);
      w[1] = 0.5 * (S[0][2] - S[2][0]);
      w[2] = 0.5 * (S[1][0] - S[0][1]);
      for (int r = 0; r < 3; r++) {
        CHECK(std::abs(J[r][j] - (r == 0 ? dp.x : r == 1 ? dp.y : dp.z)) < 1e-4,
              "J 线部 [%d][%d]", r, j);
        CHECK(std::abs(J[3 + r][j] - w[r]) < 1e-4, "J 角部 [%d][%d]", 3 + r, j);
      }
    }
  }

  // ---- 8. 位姿误差公式（回归：trace/3 错误）----
  {
    Mat4 A = Mat4::identity();
    Mat4 B = Mat4::rotationZ(kPi / 2);
    Vec3 e = poseErrorV(A, B);
    CHECK(std::abs(e.y - kPi / 2) < 1e-9, "90° 误差角 = %g", e.y);
    B = Mat4::rotationZ(kPi);
    e = poseErrorV(A, B);
    CHECK(std::abs(e.y - kPi) < 1e-6, "180° 误差角 = %g", e.y);
  }

  // ---- 9. 奇异度量：腕奇异（θ5=0）附近 σ_min 塌缩 ----
  {
    std::array<double, 6> qSing{0.3, -0.8, 0.7, 0.4, 0.0, 0.2};  // θ5 = q5 = 0
    std::array<double, 6> qGood{0.3, -0.8, 0.7, 0.4, 0.7, 0.2};
    double s1 = minSingularValue(m, qSing), s2 = minSingularValue(m, qGood);
    CHECK(s1 < 0.05 && s1 < 0.5 * s2, "σ_min 奇异=%.4f 正常=%.4f", s1, s2);
    double e1 = singularityIndex(m, qSing), e2 = singularityIndex(m, qGood);
    CHECK(e1 < 0.05 && e1 < 0.5 * e2, "η 奇异=%.4f 正常=%.4f", e1, e2);
    CHECK(isSingularNear(m, qSing), "isSingularNear 漏报");
    CHECK(!isSingularNear(m, qGood), "isSingularNear 误报");
    double w1 = manipulability(m, qSing), w2 = manipulability(m, qGood);
    CHECK(w1 < 0.1 * w2 + 1e-9, "可操控度 w 奇异=%.4f 正常=%.4f", w1, w2);
    double sc = singularityScale(m, qSing);
    CHECK(sc < 0.5 && sc >= 0.2, "奇异软降速系数 %.2f", sc);
  }

  // ---- 10. η 分布统计（README「奇异软降速」数字的复现入口；固定 seed 可复现）----
  // 采样：各关节在**限位内**均匀（真实可达构型），3000 个位形。
  if (etaStats) {
    const int N = 3000;
    std::mt19937 rng(20260926u);
    std::vector<double> etas;
    etas.reserve(N);
    for (int k = 0; k < N; k++) {
      std::array<double, 6> q{};
      for (int i = 0; i < 6; i++) {
        std::uniform_real_distribution<double> Ui(m.qmin[i], m.qmax[i]);
        q[i] = Ui(rng);
      }
      etas.push_back(singularityIndex(m, q));
    }
    std::vector<double> sorted = etas;
    std::sort(sorted.begin(), sorted.end());
    auto pctile = [&](double p) { return sorted[size_t(p * double(N - 1))]; };
    auto share = [&](double thr) {
      int c = 0;
      for (double e : etas) c += (e < thr);
      return 100.0 * double(c) / double(N);
    };
    int ge04 = 0;
    for (double e : etas) ge04 += (e >= 0.04);
    double mean = 0;
    for (double e : etas) mean += e;
    mean /= double(N);
    std::printf("[eta-stats] N=%d seed=20260926（关节限位内均匀采样）均=%.4f\n", N, mean);
    std::printf("  p5=%.4f 中位=%.4f p95=%.4f  min=%.4f max=%.4f\n", pctile(0.05),
                pctile(0.50), pctile(0.95), sorted.front(), sorted.back());
    std::printf("  旧阈值 0.12 降速占比 %.1f%% | 现阈值 0.05 占比 %.1f%% | 深度降速(η<0.02) %.1f%%"
                " | η≥0.04(scale≥0.92) %.1f%%\n",
                share(0.12), share(0.05), share(0.02), 100.0 * double(ge04) / double(N));
  }

  if (g_fail == 0) std::printf("test_kinematics PASS\n");
  else std::printf("test_kinematics FAIL (%d)\n", g_fail);
  return g_fail ? 1 : 0;
}
