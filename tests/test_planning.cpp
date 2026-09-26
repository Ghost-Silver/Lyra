// tests/test_planning.cpp — 规划与时间律单元测试
#include "arm/planning.hpp"
#include "arm/trajectory.hpp"
#include <cstdio>
#include <random>
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

// 数值检查 S 曲线各阶导数上限
static void checkLimits(const SCurveProfile& p, double vmax, double amax, double jmax,
                        const char* tag) {
  double h = 1e-4;
  double peakV = 0, peakA = 0, peakJ = 0;
  for (double t = 0; t <= p.T; t += h) {
    peakV = std::max(peakV, std::abs(p.v(t)));
    peakA = std::max(peakA, std::abs(p.a(t)));
    peakJ = std::max(peakJ, std::abs(p.j(t)));
  }
  CHECK(peakV <= vmax * 1.02 + 1e-9, "%s v峰值 %.4f > %.4f", tag, peakV, vmax);
  CHECK(peakA <= amax * 1.02 + 1e-9, "%s a峰值 %.4f > %.4f", tag, peakA, amax);
  CHECK(peakJ <= jmax * 1.02 + 1e-9, "%s j峰值 %.4f > %.4f", tag, peakJ, jmax);
  CHECK(std::abs(p.s(p.T) - p.dist) < 1e-9, "%s 末位移 %.6f != %.6f", tag, p.s(p.T), p.dist);
  CHECK(std::abs(p.s(0)) < 1e-12, "%s 初位移非 0", tag);
}

int main() {
  // ---- 1. S 曲线全工况限幅 ----
  {
    SCurveProfile p;
    // 长行程：v/a/j 全达限
    CHECK(scurvePlan(2.0, 2.0, 4.0, 30.0, p), "R1 规划失败");
    checkLimits(p, 2.0, 4.0, 30.0, "R1");
    // 中行程：达 a 不达 v
    CHECK(scurvePlan(0.15, 2.0, 4.0, 30.0, p), "R2 规划失败");
    checkLimits(p, 2.0, 4.0, 30.0, "R2");
    // 短行程：纯 jerk 三角
    CHECK(scurvePlan(0.005, 2.0, 4.0, 30.0, p), "R3 规划失败");
    checkLimits(p, 2.0, 4.0, 30.0, "R3");
    // 小 vmax（达 v 不达 a）
    CHECK(scurvePlan(1.5, 0.3, 10.0, 30.0, p), "R1' 规划失败");
    checkLimits(p, 0.3, 10.0, 30.0, "R1'");
    // 负方向
    CHECK(scurvePlan(-0.7, 2.0, 4.0, 30.0, p), "负向规划失败");
    checkLimits(p, 2.0, 4.0, 30.0, "neg");
    // 零位移
    CHECK(scurvePlan(0.0, 2.0, 4.0, 30.0, p), "零位移失败");
    CHECK(p.T < 1e-12, "零位移应零时长");
  }

  // ---- 2. 平滑时间缩放（强制时长）----
  {
    SCurveProfile p;
    CHECK(scurvePlan(1.0, 2.0, 4.0, 30.0, p), "规划失败");
    double Topt = p.T;
    CHECK(scurvePlan(1.0, 2.0, 4.0, 30.0, p, Topt * 2.5), "缩放失败");
    checkLimits(p, 2.0, 4.0, 30.0, "scaled");
    CHECK(std::abs(p.T - Topt * 2.5) < 1e-9, "缩放后时长 %.4f", p.T);
    CHECK(!scurvePlan(1.0, 2.0, 4.0, 30.0, p, Topt * 0.5), "不应允许比最优更快");
  }

  // ---- 3. jointPTP：同步到达 + 逐轴限速 ----
  {
    auto m = ArmModel::urStyle();
    std::array<double, 6> q0{0, 0, 0, 0, 0, 0};
    std::array<double, 6> qf{1.2, -2.0, 0.4, 1.9, -1.1, 2.3};
    auto traj = jointPTP(m, q0, qf, 0.002, 1.0, 4.0, 30.0);
    CHECK(!traj.empty(), "PTP 空");
    for (int k = 0; k < 6; k++)
      CHECK(std::abs(traj.qs.back()[k] - qf[k]) < 1e-9, "PTP 终点偏差 k=%d", k);
    double peak[6] = {};
    for (size_t i = 1; i < traj.size(); i++)
      for (int k = 0; k < 6; k++) {
        double dt = traj.ts[i] - traj.ts[i - 1];
        if (dt > 0) peak[k] = std::max(peak[k], std::abs(traj.qs[i][k] - traj.qs[i - 1][k]) / dt);
      }
    for (int k = 0; k < 6; k++)
      CHECK(peak[k] <= m.vmax[k] * 1.05 + 1e-6, "轴 %d 超速 %.3f", k, peak[k]);
  }

  // ---- 4. 笛卡尔直线：FK 落在直线上 + 姿态 slerp ----
  {
    auto m = ArmModel::urStyle();
    std::array<double, 6> q0{0.2, -0.9, 0.9, 0.3, 0.6, 0.0};
    Mat4 T0 = forwardKinematicsT0_6(m, q0);
    Vec3 p0 = T0.translationV();
    Vec3 pf = p0 + Vec3{0.06, 0.03, -0.05};
    // 目标姿态 = T0 再绕世界 z 转 0.5 rad；位置 = pf（slerp 现为纯旋转，可直接复合平移）
    Mat4 Tf = Mat4::translation(pf) * rotationSlerp(T0, Mat4::rotationZ(0.5) * T0, 1.0);
    auto traj = cartesianLineTraj(m, T0, Tf, q0, 0.002, 0.15);
    CHECK(!traj.empty(), "直线轨迹空（IK 失败?）");
    if (!traj.empty()) {
      double maxOff = 0;
      Vec3 dir = (pf - p0).normalized();
      for (auto& q : traj.qs) {
        Vec3 p = forwardKinematicsT0_6(m, q).translationV();
        Vec3 d = p - p0;
        Vec3 off = d - dir * d.dot(dir);
        maxOff = std::max(maxOff, off.norm());
        CHECK(d.dot(dir) >= -1e-6 && d.dot(dir) <= (pf - p0).norm() + 1e-6, "直线越界");
      }
      CHECK(maxOff < 1e-4, "偏离直线 %.2e", maxOff);
      Vec3 eEnd = poseErrorV(forwardKinematicsT0_6(m, traj.qs.back()), Tf);
      CHECK(eEnd.x < 1e-5 && eEnd.y < 1e-5, "终点误差 %.2e/%.2e", eEnd.x, eEnd.y);
    }
  }

  // ---- 5. 圆弧 / 势场 / 抓取编排 ----
  {
    auto pts = circlePoints({0, 0, 0.3}, {0, 0, 1}, 0.2, 0, kPi, 16);
    CHECK(pts.size() == 17, "圆弧点数 %zu", pts.size());
    for (auto& p : pts)
      CHECK(std::abs(std::hypot(p.x, p.y) - 0.2) < 1e-12 && std::abs(p.z - 0.3) < 1e-12,
            "圆弧点不在圆上");

    std::vector<Vec3> line{{0.1, 0, 0.2}, {0.2, 0.02, 0.2}, {0.3, 0, 0.2}};
    auto pushed = repelObstacle(line, {0.2, 0.0, 0.2}, 0.08, 1.0);
    // 中间点 (0.2, 0.02, 0.2) 在球内，横向推开后应离障碍中心 ≥ radius·(1−strength)+原距离...
    Vec3 d = pushed[1] - Vec3{0.2, 0.0, 0.2};
    CHECK(std::hypot(d.x, d.y) >= 0.08 - 1e-9 || d.y > 0.02,
          "障碍点未被推开: %g %g %g", pushed[1].x, pushed[1].y, pushed[1].z);
    CHECK(std::abs(pushed[1].z - 0.2) < 1e-12, "避让应保持高度");

    Mat4 base = Mat4::identity();
    GraspPlan gp = graspPlan({0.3, 0.1, 0}, 0.05, 0.1, 0.03, base);
    CHECK(gp.targets.size() == 3, "抓取应 3 段，实际 %zu", gp.targets.size());
    CHECK(gp.names[0] == "approach" && gp.names[1] == "grasp" && gp.names[2] == "leave",
          "抓取段命名错");
    CHECK(gp.targets[0].translationV().z > gp.targets[1].translationV().z, "approach 应高于 grasp");
    CHECK(gp.targets[2].translationV().z > gp.targets[1].translationV().z, "leave 应高于 grasp");

    // fromRPY/toRPY 往返
    double r = 0.3, p = -0.5, y = 1.2;
    Mat4 T = fromRPY({0.1, 0.2, 0.3}, r, p, y);
    double r2, p2, y2;
    toRPY(T, r2, p2, y2);
    CHECK(std::abs(r - r2) + std::abs(p - p2) + std::abs(y - y2) < 1e-9, "RPY 往返");
  }

  if (g_fail == 0) std::printf("test_planning PASS\n");
  else std::printf("test_planning FAIL (%d)\n", g_fail);
  return g_fail ? 1 : 0;
}
