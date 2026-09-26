// tests/test_safety.cpp — B1 独立安全监控层 + B2 轨迹播放防御性校验
// 覆盖：越限目标拦截（位置/速度）、状态侧审计（位置/速度/加速度/单拍跃变）、
//       急停策略、旧版规划器输出（越限轨迹）不泄漏到底层、时间基准漂移回归。
#include "arm/control/safety_monitor.hpp"
#include "arm/traj_player.hpp"

#include <cmath>
#include <cstdio>
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

static JointTrajectory makeValidTraj(const ArmModel& m, double dt = 0.002) {
  std::array<double, 6> q0{}, qf{};
  for (int i = 0; i < 6; i++) { q0[i] = 0.0; qf[i] = 0.5; }
  return jointPTP(m, q0, qf, dt, 1.0, 4.0, 30.0);
}

int main() {
  RobotConf conf = RobotConf::desktop6();
  const ArmModel& m = conf.arm;

  // ================= B2：TrajPlayer 防御性校验 =================
  {
    // 合法轨迹
    auto t = makeValidTraj(m);
    CHECK(!t.empty(), "PTP 轨迹应非空");
    CHECK(TrajPlayer::validate(t).empty(), "合法轨迹不应被拒: %s", TrajPlayer::validate(t).c_str());

    // 非法：空 / ts 未写入 / 非单调 / 末值 <= 0 / 长度不一致 / NaN
    JointTrajectory bad;
    CHECK(!TrajPlayer::validate(bad).empty(), "空轨迹应被拒");
    bad.ts = {0.0, 0.0, 0.0};
    bad.qs = {{0,0,0,0,0,0}, {0,0,0,0,0,0}, {0,0,0,0,0,0}};
    CHECK(TrajPlayer::validate(bad).find("末值") != std::string::npos, "ts 全零应报『末值<=0』");
    bad.ts = {0.0, 0.5, 0.4};
    CHECK(TrajPlayer::validate(bad).find("单调") != std::string::npos, "非单调应被拒");
    bad.ts = {0.0, 0.5, 1.0};
    bad.qs.pop_back();
    CHECK(TrajPlayer::validate(bad).find("长度") != std::string::npos, "长度不一致应被拒");
    bad.qs.push_back({0,0,0,0,0,0});
    bad.ts[1] = std::nan("");
    CHECK(TrajPlayer::validate(bad).find("有限") != std::string::npos, "NaN ts 应被拒");

    // start() 拒绝：不计入成功、拒播计数递增、错误可读（**不静默跳终点**）
    TrajPlayer p;
    std::string err;
    JointTrajectory zeroTs;
    zeroTs.ts = {0.0, 0.0};
    zeroTs.qs = {{0,0,0,0,0,0}, {1,1,1,1,1,1}};
    CHECK(!p.start(zeroTs, 0.0, &err), "非法轨迹 start 应失败");
    CHECK(p.rejects() == 1 && p.samples() == 0, "拒播计数=%d 成功=%zu", p.rejects(), p.samples());
    std::printf("[B2] 非法时间基准（空/全零 ts/非单调/NaN/长度不符 共 5 例）→ 全部拒播，rejects=%zu samples=%zu（不静默跳终点）\n",
                size_t(p.rejects()), p.samples());
    CHECK(!err.empty() && !p.playing(), "应给出原因且未进入播放");
  }

  // ================= B2：时间基准漂移回归（旧 bug 触发条件）=================
  {
    // 场景：sim 时间已跑到 300 s 后下发新轨迹 —— 旧实现会因时间原点缺失首拍判完成
    auto t = makeValidTraj(m);            // T ≈ 0.5 s 量级
    double T = t.ts.back();
    TrajPlayer p;
    double now0 = 300.0;
    CHECK(p.start(t, now0), "合法轨迹应载入");
    CHECK(std::abs(p.t0() - now0) < 1e-12, "时间原点应为载入时刻（唯一入口）");

    std::array<double, 6> q{};
    bool fin = false;
    // 第一拍：t = now0 + dt
    CHECK(p.step(now0 + 0.002, q, fin), "首拍应可推进");
    CHECK(!fin, "首拍不得判完成（旧 bug 的特征）");
    double devFirst = 0;
    for (int i = 0; i < 6; i++) devFirst = std::max(devFirst, std::abs(q[i] - t.qs[0][i]));
    CHECK(devFirst < 1e-6, "首拍应在起点附近，实际偏差 %.4f", devFirst);

    // 中途：约 50% 时长处应处于行程中段（远离终点）
    CHECK(p.step(now0 + T * 0.5, q, fin) && !fin, "中点不得判完成");
    CHECK(q[0] > 0.05 && q[0] < 0.45, "中点位置应在行程内，实际 q0=%.3f", q[0]);

    // 结束：仅在 t ≥ T 时完成，且给出终点
    CHECK(p.step(now0 + T + 1e-6, q, fin) && fin, "超过总时长应完成");
    CHECK(std::abs(q[0] - t.qs.back()[0]) < 1e-9, "完成时应给终点，实际 %.4f", q[0]);
    bool fin2 = false;
    CHECK(!p.step(now0 + T + 0.1, q, fin2), "完成后不再推进");
    std::printf("[B2] t=300 s 后载入新轨迹：首拍不判完成、中点 q0=%.3f 在行程内、按时长完成且给终点（时间原点唯一入口 t0=%.1f）\n",
                q[0], p.t0());
  }

  // ================= B1：下发前目标拦截 =================
  {
    ArmSim sim(conf, 0.002);
    SafetyMonitor mon(m);
    // 位置目标越限 → 夹回 + 计数
    std::array<double, 6> bad;
    for (int i = 0; i < 6; i++) bad[i] = m.qmax[i] + 5.0;
    sim.setJointPositionTarget(bad);
    bool estop = mon.preDispatch(sim, sim.dt());
    CHECK(!estop, "默认（不联急停）不应触发急停");
    const auto& tgt = sim.pendingQTarget();
    bool clamped = true;
    for (int i = 0; i < 6; i++)
      if (tgt[i] > m.qmax[i] + 1e-12 || tgt[i] < m.qmin[i] - 1e-12) clamped = false;
    CHECK(clamped, "越限目标应被夹回限位内");
    CHECK(mon.counters().posClamps == 1, "位置拦截计数=%d", mon.counters().posClamps);
    std::printf("[B1] 注入越限目标 q = qmax+5.0 rad → 下发前夹回限位内，posClamps=%d（状态 JSON/health 可见）\n",
                mon.counters().posClamps);

    // 速度目标越限 → 夹回 + 计数
    std::array<double, 6> v;
    for (int i = 0; i < 6; i++) v[i] = m.vmax[i] * 10.0;
    sim.setJointVelocityTarget(v);
    mon.preDispatch(sim, sim.dt());
    const auto& vt = sim.pendingVTarget();
    bool vclamped = true;
    for (int i = 0; i < 6; i++)
      if (std::abs(vt[i]) > m.vmax[i] + 1e-12) vclamped = false;
    CHECK(vclamped, "越限速度目标应被夹回");
    CHECK(mon.counters().velClamps == 1, "速度拦截计数=%d", mon.counters().velClamps);

    // 独立拦截入口：不经过任何规划器也能拦
    std::array<double, 6> direct;
    for (int i = 0; i < 6; i++) direct[i] = m.qmin[i] - 1.0;
    CHECK(mon.guardTarget(direct), "guardTarget 应报告拦截");
    CHECK(mon.counters().posClamps == 2, "独立入口计数=%d", mon.counters().posClamps);
  }

  // ================= B1：积分后状态审计 =================
  {
    ArmSim sim(conf, 0.002);
    sim.reset(conf.home);
    SafetyMonitor mon(m);
    mon.postStep(sim, sim.dt());          // 建立基线

    // (a) 单拍位置跃变（P0-1 类 bug 的特征）→ 回滚 + 计数
    auto jump = conf.home;
    jump[0] += 1.0;                        // 1 rad 单拍跃变（远超 vmax·dt·1.5 = 0.0075）
    sim.injectFaultState(jump, {0, 0, 0, 0, 0, 0});
    mon.postStep(sim, sim.dt());
    CHECK(mon.counters().stepJump == 1, "单拍跃变未检出（计数=%d）", mon.counters().stepJump);
    CHECK(std::abs(sim.state().q[0] - conf.home[0]) < 1e-12, "跃变应被回滚，实际 %.4f",
          sim.state().q[0]);

    // (b) 位置越限 → 投影回安全域
    auto oob = conf.home;
    oob[2] = m.qmax[2] + 0.3;
    sim.injectFaultState(oob, {0, 0, 0, 0, 0, 0});
    mon.postStep(sim, sim.dt());
    CHECK(mon.counters().posState >= 1, "状态越限未检出");
    CHECK(sim.state().q[2] <= m.qmax[2] + 1e-12, "越限状态应被投影，实际 %.4f", sim.state().q[2]);

    // (c) 观测速度越限 → 清零 + 计数
    auto qok = conf.home;
    std::array<double, 6> vbad{};
    vbad[1] = m.vmax[1] * 3.0;
    sim.injectFaultState(qok, vbad);
    mon.postStep(sim, sim.dt());
    CHECK(mon.counters().velState == 1, "观测速度越限未检出（计数=%d）", mon.counters().velState);
    CHECK(std::abs(sim.state().qd[1]) < 1e-12, "越限速度应被清零");

    // (d) 观测加速度越限（速度在限内，仅加速度超）→ 计数
    ArmSim sim2(conf, 0.0005);
    sim2.reset(conf.home);
    SafetyMonitor mon2(m);
    mon2.postStep(sim2, sim2.dt());
    std::array<double, 6> v0{};
    sim2.injectFaultState(conf.home, v0);
    mon2.postStep(sim2, sim2.dt());
    std::array<double, 6> v1{};
    v1[0] = 0.05;                          // a = 0.05/0.0005 = 100 rad/s² ≫ amax=4
    sim2.injectFaultState(conf.home, v1);
    mon2.postStep(sim2, sim2.dt());
    CHECK(mon2.counters().accState == 1, "观测加速度越限未检出（计数=%d）", mon2.counters().accState);
  }

  // ================= B1：越限即急停（可选策略）=================
  {
    ArmSim sim(conf, 0.002);
    SafetyMonitor::Config cfg;
    cfg.estopOnViolation = true;
    SafetyMonitor mon(m, cfg);
    std::array<double, 6> bad;
    for (int i = 0; i < 6; i++) bad[i] = m.qmin[i] - 2.0;
    sim.setJointPositionTarget(bad);
    bool estop = mon.preDispatch(sim, sim.dt());
    CHECK(estop, "开启策略时越限应请求急停");
    CHECK(sim.eStop(), "急停应锁存到仿真状态");
    CHECK(mon.counters().estopTriggers == 1, "急停计数=%d", mon.counters().estopTriggers);
    mon.preDispatch(sim, sim.dt());
    CHECK(mon.counters().estopTriggers == 1, "已锁存时不应重复计数");
  }

  // ================= B1：关闭开关 = 完全旁路（默认不改变行为）=================
  {
    ArmSim sim(conf, 0.002);
    SafetyMonitor mon(m);
    mon.setEnabled(false);
    std::array<double, 6> bad;
    for (int i = 0; i < 6; i++) bad[i] = m.qmax[i] + 1.0;
    sim.setJointPositionTarget(bad);
    CHECK(!mon.preDispatch(sim, sim.dt()), "关闭时应旁路");
    CHECK(mon.counters().total() == 0, "关闭时不应计数");
    CHECK(std::abs(sim.pendingQTarget()[0] - bad[0]) < 1e-12, "关闭时目标不应被改写（语义不变）");
  }

  // ================= 端到端：旧版规划器输出（越限轨迹）不泄漏到底层 =================
  {
    // 用旧版 cartesianLineTraj 的失效模式构造：轨迹 qs 含越限点（修复前实测 43% 轨迹含越限）
    JointTrajectory legacy;
    double T = 0.4, dt = 0.002;
    for (int i = 0; i <= int(T / dt); i++) {
      legacy.ts.push_back(double(i) * dt);
      std::array<double, 6> q{};
      // 路径前半正常，后半冲入越限区（模拟无限位校验的 IK 解直接进轨迹）
      double frac = double(i) * dt / T;
      q[3] = (frac < 0.5) ? 0.3 * frac : 0.15 + (frac - 0.5) * 8.0;
      legacy.qs.push_back(q);
    }
    legacy.qs.back()[3] = 4.0;             // ≫ qmax[3]
    TrajPlayer p;
    ArmSim sim(conf, 0.002);
    SafetyMonitor mon(m);
    CHECK(p.start(legacy, sim.state().t), "旧轨迹本身时间轴合法（会被播放）");

    int leaked = 0;
    double worst = 0;
    bool fin = false;
    for (int k = 0; k < 400; k++) {
      std::array<double, 6> q{};
      if (!p.step(sim.state().t + k * dt, q, fin)) break;
      // 与 Scheduler::tick 相同次序：playTraj → sim.setJointPositionTarget → monitor.preDispatch
      sim.setJointPositionTarget(q);
      mon.preDispatch(sim, dt);
      const auto& tgt = sim.pendingQTarget();
      for (int i = 0; i < 6; i++) {
        double over = std::max(m.qmin[i] - tgt[i], tgt[i] - m.qmax[i]);
        if (over > 1e-12) { leaked++; worst = std::max(worst, over); }
      }
      sim.step();
      mon.postStep(sim, dt);
    }
    CHECK(leaked == 0, "越限指令泄漏到下发层 %d 次（最差 %.3f rad）", leaked, worst);
    CHECK(mon.counters().posClamps > 0, "应记录到拦截（计数=%d）",
          mon.counters().posClamps);
    CHECK(sim.state().q[3] <= m.qmax[3] + 1e-9, "仿真状态不得越限，实际 %.4f", sim.state().q[3]);
    std::printf("[B1] 旧规划器越限轨迹（末点 4.0 rad ≫ qmax=%.3f）：400 拍**零泄漏**到底层，拦截 %d 次，仿真末态 q3=%.3f\n",
                m.qmax[3], mon.counters().posClamps, sim.state().q[3]);
  }

  if (g_fail == 0) std::printf("test_safety PASS\n");
  else std::printf("test_safety FAIL (%d)\n", g_fail);
  return g_fail ? 1 : 0;
}
