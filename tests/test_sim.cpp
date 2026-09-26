// tests/test_sim.cpp — 仿真积分器 + RL 环境单元测试
#include "arm/sim.hpp"
#include <cstdio>
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

int main() {
  RobotConf conf = RobotConf::desktop6();

  // ---- 1. 位置伺服收敛 ----
  {
    ArmSim sim(conf, 0.002);
    std::array<double, 6> target{0.5, -0.6, 0.7, 0.3, 0.4, -0.5};
    sim.setJointPositionTarget(target);
    for (int i = 0; i < 4000; i++) sim.step();
    for (int k = 0; k < 6; k++)
      CHECK(std::abs(sim.state().q[k] - target[k]) < 0.01, "轴 %d 未收敛 %.4f", k, sim.state().q[k]);
  }

  // ---- 2. 软件限位 ----
  {
    ArmSim sim(conf, 0.002);
    std::array<double, 6> target{};
    for (int k = 0; k < 6; k++) target[k] = conf.arm.qmax[k] + 1.5;   // 越界目标
    sim.setJointPositionTarget(target);
    for (int i = 0; i < 4000; i++) sim.step();
    for (int k = 0; k < 6; k++)
      CHECK(sim.state().q[k] <= conf.arm.qmax[k] + 1e-9, "轴 %d 顶穿限位 %.4f", k, sim.state().q[k]);
  }

  // ---- 3. 速度指令限幅 + 急停减速 ----
  {
    ArmSim sim(conf, 0.002);
    std::array<double, 6> v{};
    v[0] = 100.0;   // 远超 vmax
    sim.setJointVelocityTarget(v);
    for (int i = 0; i < 200; i++) sim.step();
    CHECK(std::abs(sim.state().qd[0]) <= conf.arm.vmax[0] * 1.01, "速度未限幅 %.3f", sim.state().qd[0]);
    sim.setEStop(true);
    for (int i = 0; i < 2000; i++) sim.step();
    CHECK(std::abs(sim.state().qd[0]) < 1e-3, "急停未停稳 %.4f", sim.state().qd[0]);
  }

  // ---- 3b. 关节摩擦（P0-补-3）：静摩擦死区 + 稳态跌落 ----
  {
    ArmSim sim(conf, 0.002);
    // 静摩擦死区：|vs|·Kv ≤ τs 的速度指令不产生运动（粘滞锁定）
    std::array<double, 6> v{};
    v[0] = 0.005;   // τd = 0.005·5 = 0.025 < τs = 0.16
    sim.setJointVelocityTarget(v);
    for (int i = 0; i < 1000; i++) sim.step();
    CHECK(std::abs(sim.state().qd[0]) < 1e-4, "静摩擦死区内应粘滞 qd=%.6f", sim.state().qd[0]);
    CHECK(std::abs(sim.state().q[0] - conf.home[0]) < 1e-4, "静摩擦死区内不应爬行");
    // 库仑+粘性摩擦 → 速度跟踪稳态跌落（sim2real 真实效应）
    v[0] = 2.0;
    sim.setJointVelocityTarget(v);
    for (int i = 0; i < 600; i++) sim.step();   // 600 步内不顶限位墙（qmax=2.967）
    double qd = std::abs(sim.state().qd[0]);
    CHECK(qd > 0.5 && qd < 2.0 - 0.02, "应有摩擦稳态跌落 qd=%.4f（指令 2.0）", qd);
  }

  // ---- 3c. RL 基线去摩擦（黄金回归前提）----
  {
    auto rlc = RLEnv::rlBaseline(conf);
    CHECK(rlc.fric.coul[0] == 0 && rlc.fric.visc[0] == 0 && rlc.fric.stic[0] == 0, "RL 基线应无摩擦");
  }

  // ---- 4. 重力补偿钩子 ----
  {
    RobotConf c2 = conf;
    c2.payload.mass = 0.5;
    std::array<double, 6> q{0.3, -0.5, 0.6, 0, 0.4, 0};
    auto tau = c2.gravityCompTorque(q);
    double n2 = 0;
    for (double x : tau) n2 += x * x;
    CHECK(n2 > 1e-6, "负载力矩应非零");
    c2.payload.mass = 0;
    auto tau0 = c2.gravityCompTorque(q);
    double n0 = 0;
    for (double x : tau0) n0 += x * x;
    CHECK(n0 < 1e-12, "零负载力矩应为 0");
  }

  // ---- 5. RLEnv：维度 / 终止 / 确定性 ----
  {
    RLEnv env(conf, 0.002, 5);
    env.reset(42);
    RLObs o = env.observe();
    bool finite = true;
    for (double x : o.x) if (!std::isfinite(x)) finite = false;
    CHECK(finite, "观测含非有限值");

    double reward = 0;
    bool done = false;
    int steps = 0;
    std::array<double, 6> act{};
    while (!done && steps < 5000) {
      o = env.step(act, reward, done);
      steps++;
    }
    CHECK(done, "episode 未终止");
    CHECK(steps * 5 * 0.002 <= RLEnv::kMaxT + 1e-6, "episode 超时 %.3f", steps * 5 * 0.002);
    CHECK(std::isfinite(reward), "reward 非有限");

    // 确定性：同种子轨迹一致
    RLEnv e1(conf, 0.002, 5), e2(conf, 0.002, 5);
    e1.reset(1234);
    e2.reset(1234);
    std::array<double, 6> a1{0.1, -0.2, 0.3, 0, 0, 0};
    double r1, r2;
    bool d1, d2;
    for (int i = 0; i < 50; i++) {
      RLObs o1 = e1.step(a1, r1, d1);
      RLObs o2 = e2.step(a1, r2, d2);
      for (int k = 0; k < 18; k++)
        CHECK(std::abs(o1.x[k] - o2.x[k]) < 1e-12, "确定性破坏 [%d] step %d", k, i);
      CHECK(std::abs(r1 - r2) < 1e-12, "reward 确定性破坏");
    }
  }

  if (g_fail == 0) std::printf("test_sim PASS\n");
  else std::printf("test_sim FAIL (%d)\n", g_fail);
  return g_fail ? 1 : 0;
}
