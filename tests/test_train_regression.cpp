// tests/test_train_regression.cpp — 训练可复现性验收基准（A3）
// 目的：把「我跑过、它是对的」变成「任何人任何时候都能自动验证它是对的」。
// 断言：
//   ① BC loss 单调下降且最终 < 初始值的 50%（短程 40 epoch，固定种子）
//   ② **同种子两次运行参数逐位一致**（确定性承诺的直接检验）
//   ③ 如实断言当前能力边界：短程 REINFORCE 的回合回报改善**不显著**（方差主导）。
//      若将来真正出现统计显著改善，本测试会 FAIL 并提示更新能力边界文档——避免
//      文档长期停在旧结论，也不假装有提升。
// 复现：cmake -B build-learn -DARM_ENABLE_CTORCH=ON && cmake --build build-learn -j
//       ./build-learn/tests/test_train_regression
#include "learning/policy.hpp"
#include "learning/reinforce.hpp"
#include "arm/sim.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>

using namespace learn;
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

namespace {

// 短程训练管线（与 arm_train 同构的缩微版）：合成示教 → BC → 短程 REINFORCE
struct ShortRun {
  std::vector<double> bcLosses;
  std::vector<double> returns;
  std::vector<float> params;    // 最终参数（逐位可比）
  double finalLoss = 0;
};

std::array<double, kAct> jointPExpert(RLEnv& env) {
  std::array<double, kAct> a{};
  const auto& qg = env.goalConfig();
  const auto& q = env.sim().state().q;
  for (int i = 0; i < kAct; i++) a[i] = std::clamp((qg[i] - q[i]) * 2.5, -1.0, 1.0);
  return a;
}

ShortRun runShort(uint64_t seed, int hidden, int demoEpisodes, int bcEpochs, int rlEpisodes,
                  int updateEvery = 4) {
  RobotConf conf = RobotConf::desktop6();
  RLEnv env(conf, 0.002, 5, 4.0);
  MLPPolicy policy(size_t(hidden), seed);
  REINFORCETrainer trainer(policy, 5e-4, 0.995, 0.02);
  const int maxSteps = int(RLEnv::kMaxT / (5 * 0.002)) + 2;
  ShortRun out;

  // ---- 合成示教 ----
  std::vector<std::array<double, kObs>> obss;
  std::vector<std::array<double, kAct>> acts;
  for (int d = 0; d < demoEpisodes; d++) {
    env.reset(seed * 1000 + uint64_t(d));
    for (int t = 0; t < maxSteps; t++) {
      RLObs o = env.observe();
      auto a = jointPExpert(env);
      double r;
      bool done;
      env.step(a, r, done);
      obss.push_back(o.x);
      acts.push_back(a);
      if (done) break;
    }
  }

  // ---- BC ----
  for (int e = 0; e < bcEpochs; e++) {
    double l = trainer.bcStep(obss, acts);
    out.bcLosses.push_back(l);
  }
  out.finalLoss = out.bcLosses.empty() ? 0.0 : out.bcLosses.back();

  // ---- 短程 REINFORCE（与 arm_train 同批大小风格，缩短到秒级）----
  for (int ep = 0; ep < rlEpisodes; ep++) {
    env.reset(seed * 7919 + uint64_t(ep));
    Episode e;
    for (int t = 0; t < maxSteps; t++) {
      RLObs o = env.observe();
      auto a = policy.sample(o.x);
      double r;
      bool done;
      env.step(a, r, done);
      e.steps.push_back({o.x, a, r});
      e.totalReward += r;
      if (done) { e.success = env.success(); break; }
    }
    out.returns.push_back(e.totalReward);
    trainer.addEpisode(std::move(e));
    if ((ep + 1) % updateEvery == 0) {
      double l = trainer.update();
      CHECK(std::isfinite(l), "REINFORCE update loss 非有限: %g", l);
    }
  }
  out.params = policy.dumpParams();
  return out;
}

double meanOf(const std::vector<double>& v, size_t a, size_t b) {
  double s = 0;
  for (size_t i = a; i < b; i++) s += v[i];
  return b > a ? s / double(b - a) : 0.0;
}
double stdOf(const std::vector<double>& v, size_t a, size_t b) {
  double m = meanOf(v, a, b), s = 0;
  for (size_t i = a; i < b; i++) s += (v[i] - m) * (v[i] - m);
  return b > a ? std::sqrt(s / double(b - a)) : 0.0;
}

}  // namespace

int main() {
  const uint64_t seed = 7;
  const int hidden = 24, demos = 4, bcEpochs = 100, rlEps = 12;
  std::printf("[cfg] seed=%llu hidden=%d demos=%d bcEpochs=%d rlEpisodes=%d（短程基准）\n",
              (unsigned long long)seed, hidden, demos, bcEpochs, rlEps);

  ShortRun A = runShort(seed, hidden, demos, bcEpochs, rlEps);

  // ---- ① BC loss：下降 + 低于初始值 50% ----
  CHECK(A.bcLosses.size() == size_t(bcEpochs), "BC 记录数 %zu", A.bcLosses.size());
  CHECK(std::isfinite(A.bcLosses[0]) && A.bcLosses[0] > 0, "初始 BC loss 应为正有限值: %.5f",
        A.bcLosses[0]);
  std::printf("[bc] loss %.5f → %.5f（首/末）\n", A.bcLosses[0], A.bcLosses.back());
  CHECK(A.bcLosses.back() < 0.5 * A.bcLosses[0],
        "BC loss 未降至初始 50%% 以下: %.5f → %.5f", A.bcLosses[0], A.bcLosses.back());
  size_t third = A.bcLosses.size() / 3;
  double bcFirst = meanOf(A.bcLosses, 0, third), bcLast = meanOf(A.bcLosses, A.bcLosses.size() - third, A.bcLosses.size());
  std::printf("[bc] 分段均值 首段 %.5f → 末段 %.5f\n", bcFirst, bcLast);
  CHECK(bcLast < 0.7 * bcFirst, "BC 末段均值未显著低于首段: %.5f → %.5f", bcFirst, bcLast);

  // ---- ② 同种子确定性：两次运行逐位一致 ----
  ShortRun B = runShort(seed, hidden, demos, bcEpochs, rlEps);
  CHECK(A.params.size() == B.params.size() && !A.params.empty(), "参数规模不一致");
  bool bitEqual =
      A.params.size() == B.params.size() &&
      std::memcmp(A.params.data(), B.params.data(), A.params.size() * sizeof(float)) == 0;
  CHECK(bitEqual, "同种子两次运行参数不逐位一致（确定性承诺被破坏）");
  bool lossEqual = A.bcLosses.size() == B.bcLosses.size() &&
                   std::memcmp(A.bcLosses.data(), B.bcLosses.data(),
                               A.bcLosses.size() * sizeof(double)) == 0;
  CHECK(lossEqual, "同种子两次运行 BC loss 序列不一致");
  bool retEqual = A.returns.size() == B.returns.size() &&
                  std::memcmp(A.returns.data(), B.returns.data(),
                              A.returns.size() * sizeof(double)) == 0;
  CHECK(retEqual, "同种子两次运行回报序列不一致");
  // 参数指纹（人工复核用）
  {
    unsigned long long h = 1469598103934665603ull;
    for (float f : A.params) {
      unsigned char b[4];
      std::memcpy(b, &f, 4);
      for (int i = 0; i < 4; i++) { h ^= b[i]; h *= 1099511628211ull; }
    }
    std::printf("[det] 参数 FNV-1a 指纹 = %016llx（两次运行一致=%s）\n", h, bitEqual ? "是" : "否");
  }

  // ---- ③ 如实断言能力边界：短程 REINFORCE 改善不显著 ----
  {
    size_t n = A.returns.size();
    CHECK(n >= 6, "回合数不足以判边界");
    size_t k = n / 3;
    double m1 = meanOf(A.returns, 0, k), s1 = stdOf(A.returns, 0, k);
    double m2 = meanOf(A.returns, n - k, n), s2 = stdOf(A.returns, n - k, n);
    double sd = std::sqrt(0.5 * (s1 * s1 + s2 * s2));
    double diff = m2 - m1;
    bool significant = sd > 1e-9 && std::abs(diff) > 2.0 * sd;
    std::printf("[boundary] 回报 首段 %.2f±%.2f → 末段 %.2f±%.2f（Δ=%.2f, 合并 σ=%.2f）: %s\n",
                m1, s1, m2, s2, diff, sd,
                significant ? "**显著改善**（请更新 README/PR 的能力边界与基准数字）"
                            : "不显著（方差主导）——与 README/PR 的如实说明一致");
    for (double r : A.returns) CHECK(std::isfinite(r), "回报非有限: %g", r);
    // 边界断言：当前配置下不宣称改善。若此断言失败，说明能力边界真的变了 → 更新文档与基准。
    CHECK(!significant,
          "短程 REINFORCE 出现统计显著改善——请更新 README/PR 的能力边界描述与基准数字");
  }

  if (g_fail == 0) std::printf("test_train_regression PASS\n");
  else std::printf("test_train_regression FAIL (%d)\n", g_fail);
  return g_fail ? 1 : 0;
}
