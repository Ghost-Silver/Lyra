// tests/test_longrun.cpp — G2：数值长时稳定性（数百万拍，位置模式持续运行）
//
// 与秒级/分钟级测试不同，这里检查「跑一晚上」才会暴露的问题：
//   ① 关节角漂移：收敛后是否**随时间增长**偏离（用分桶包络区分「有界慢收敛/粘滑波动」
//      与「线性漂移」——实测本臂为前者：静差 ≈ τ_f/Kv ≈ 2e-3 rad，包络不增长）
//   ② 时间累加精度：t += dt 在数百万次后的相对误差，是否影响轨迹时间域插值
//   ③ 安全层误报：数值噪声是否触发位置/速度/加速度审计计数（应为 0）
//   ④ 大时间基下的轨迹播放：t≈数千秒时载入轨迹仍按 (now−t0) 正确推进（B2 回归）
//   ⑤ 速度包络：长时间运行不出现自激振荡（|qd| 有界）
//
// 用法： ./test_longrun [steps]（默认 2,000,000 拍 = 4000 s 仿真；--huge = 10,000,000 拍）
#include "arm/control/safety_monitor.hpp"
#include "arm/planning.hpp"
#include "arm/sim.hpp"
#include "arm/traj_player.hpp"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>
#include <string>

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
  long steps = 2000000;
  for (int i = 1; i < argc; i++) {
    if (std::string(argv[i]) == "--huge") steps = 10000000;
    else steps = std::atol(argv[i]);
  }
  RobotConf conf = RobotConf::desktop6();
  const auto& m = conf.arm;
  const double dt = 0.002;
  ArmSim sim(conf, dt);
  SafetyMonitor mon(m);
  sim.setJointPositionTarget(conf.home);

  const long settle = 20000;               // 前 40 s 用于收敛（不计入漂移统计）
  const int kBuckets = 8;
  std::array<double, 6> qSettle{};
  std::vector<double> envelope(size_t(kBuckets), 0.0);
  double maxDev = 0, maxV = 0, maxDriftSinceSettle = 0;
  long span = std::max<long>(1, (steps - settle) / kBuckets);
  auto t0 = std::chrono::steady_clock::now();
  for (long k = 0; k < steps; k++) {
    sim.step();
    mon.postStep(sim, dt);
    if (k == settle) {
      qSettle = sim.state().q;
    } else if (k > settle) {
      size_t b = size_t(std::min<long>((k - settle) / span, kBuckets - 1));
      for (int i = 0; i < 6; i++) {
        maxDev = std::max(maxDev, std::abs(sim.state().q[i] - conf.home[i]));
        double d = std::abs(sim.state().q[i] - qSettle[i]);
        envelope[b] = std::max(envelope[b], d);
        maxDriftSinceSettle = std::max(maxDriftSinceSettle, d);
        maxV = std::max(maxV, std::abs(sim.state().qd[i]));
      }
    }
  }
  double wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  const double tEnd = sim.state().t;
  const double tExact = double(steps) * dt;

  // ① 收敛后**无时间增长性漂移**：包络（各分桶相对收敛位形的最大偏移）不得随分桶序号增大，
  //    即排除线性漂移；允许有界慢收敛/粘滑波动（量级与摩擦静差 τ_f/Kv 一致）
  double envFirst = envelope.front(), envLast = envelope.back();
  double envMax = 0;
  for (double e : envelope) envMax = std::max(envMax, e);
  CHECK(envLast <= 1.5 * envFirst + 2e-4,
        "偏移包络随时间增长（首桶 %.3e → 末桶 %.3e）：存在时间性漂移", envFirst, envLast);
  CHECK(envMax < 5e-3, "偏移包络超界：%.3e rad（限 5e-3）", envMax);
  // ② 稳态偏差有界（摩擦/重力引起的静差，量级须与 Kv 折算一致）
  CHECK(maxDev < 5e-3, "稳态偏差过大：%.3e rad（限 5e-3）", maxDev);
  // ③ 时间累加精度（每个 dt 累加一次；相对误差须远小于轨迹插值分辨率）
  double relT = std::abs(tEnd - tExact) / tExact;
  CHECK(relT < 1e-9, "时间累加相对误差 %.3e（%ld 拍后 t=%.6f, 理论 %.6f）", relT, steps, tEnd,
        tExact);
  // ④ 速度包络有界（不自激）
  CHECK(maxV <= m.vmax[0] * 1.01, "速度包络越界：max|qd|=%.4f（vmax=%.4f）", maxV, m.vmax[0]);
  // ⑤ 安全层零误报（数值噪声不得触发任何审计计数）
  const auto& c = mon.counters();
  CHECK(c.total() == 0, "安全层误报：total=%d(pos=%d vel=%d acc=%d step=%d)",
        c.total(), c.posClamps + c.posState, c.velClamps + c.velState, c.accState, c.stepJump);

  std::printf("[G2] %ld 拍（仿真 %.1f s = %.2f h）耗时 %.1f s（%.2f us/拍）\n", steps, tEnd,
              tEnd / 3600.0, wall, wall / double(steps) * 1e6);
  std::printf("  ① 收敛后偏移包络（相对收敛位形，%d 桶）= [", kBuckets);
  for (int b = 0; b < kBuckets; b++) std::printf("%.1e%s", envelope[size_t(b)], b + 1 < kBuckets ? " " : "");
  std::printf("] → 首桶 %.1e vs 末桶 %.1e（不随时间增长；非漂移，量级≈摩擦静差）\n", envFirst,
              envLast);
  std::printf("  ② 稳态偏差 max|q−q_home| = %.3e rad；max|qd| = %.4f（vmax=%.4f）\n", maxDev, maxV,
              m.vmax[0]);
  std::printf("  ③ 时间基：t=%.6f vs 理论 %.6f（相对误差 %.2e）\n", tEnd, tExact, relT);
  std::printf("  ⑤ 安全层计数：total=%d（位置 %d / 速度 %d / 加速度 %d / 跃变 %d）\n", c.total(),
              c.posClamps + c.posState, c.velClamps + c.velState, c.accState, c.stepJump);

  // ---------- ④ 大时间基下的轨迹播放（B2 回归在 t≈数千秒处）----------
  {
    std::array<double, 6> qf = conf.home;
    qf[0] += 0.8;
    qf[2] -= 0.5;
    JointTrajectory traj = jointPTP(m, sim.state().q, qf, dt, 0.6, conf.amax, conf.jmax);
    TrajPlayer p;
    std::string err;
    CHECK(p.start(traj, sim.state().t, &err), "大时间基下载入轨迹失败: %s", err.c_str());
    std::array<double, 6> q{};
    bool fin = false;
    double devFirst = 0;
    CHECK(p.step(sim.state().t + dt, q, fin) && !fin, "首拍不应判完成（大时间基）");
    for (int i = 0; i < 6; i++) devFirst = std::max(devFirst, std::abs(q[i] - sim.state().q[i]));
    CHECK(devFirst < 1e-6, "首拍应贴近起点：偏差 %.3e", devFirst);
    const double T = traj.ts.back();
    CHECK(p.step(sim.state().t + T * 0.5, q, fin) && !fin, "中点不应判完成");
    CHECK(p.step(sim.state().t + T + 1e-9, q, fin) && fin, "超时应完成");
    double devEnd = 0;
    for (int i = 0; i < 6; i++) devEnd = std::max(devEnd, std::abs(q[i] - qf[i]));
    CHECK(devEnd < 1e-9, "终点偏差 %.3e", devEnd);
    std::printf("  ④ t=%.1f s 处播放 PTP（T=%.3f s）：首拍偏差 %.2e、中点不误判、终点偏差 %.2e\n",
                sim.state().t, T, devFirst, devEnd);
  }

  if (g_fail == 0) std::printf("test_longrun PASS\n");
  else std::printf("test_longrun FAIL (%d)\n", g_fail);
  return g_fail ? 1 : 0;
}
