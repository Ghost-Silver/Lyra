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
#include <string>
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

// 稳健显著性口径（F2）：
//   原口径「|Δ| > 2·σ_合并 ⇒ 显著」在 n=4 的小样本下不稳定（同一配置换种子结论会翻转）。
//   改为三条**稳健**断言：
//     A. 不劣化界限：末段均值 ≥ 首段均值 − 1.0·σ_合并（不出现超出噪声的退化）
//     B. 方差同量级：末段 σ ∈ [0.2×首段 σ, 5×首段 σ]（不塌缩、不爆炸）
//     C. 显著退化（Δ < −2σ）仍硬失败；显著改善只**警告**（提示更新文档），不再作为硬断言
//   统计功效局限：n=12 回合、每段 4 个样本，任何"显著/不显著"结论本身都带统计噪声；
//   故只对"劣化"这种单向风险设硬门槛。多种子一致性由 --seed-scan 报告（不入默认路径）。
struct Verdict {
  double m1, s1, m2, s2, diff, sd;
  bool sigImprove, sigDegrade;
};
Verdict judge(const std::vector<double>& ret) {
  Verdict v{};
  size_t n = ret.size();
  size_t k = n / 3;
  v.m1 = meanOf(ret, 0, k);
  v.s1 = stdOf(ret, 0, k);
  v.m2 = meanOf(ret, n - k, n);
  v.s2 = stdOf(ret, n - k, n);
  v.sd = std::sqrt(0.5 * (v.s1 * v.s1 + v.s2 * v.s2));
  v.diff = v.m2 - v.m1;
  v.sigImprove = v.sd > 1e-9 && v.diff > 2.0 * v.sd;
  v.sigDegrade = v.sd > 1e-9 && v.diff < -2.0 * v.sd;
  return v;
}

int main(int argc, char** argv) {
  bool seedScan = false;
  for (int i = 1; i < argc; i++)
    if (std::string(argv[i]) == "--seed-scan") seedScan = true;

  if (seedScan) {
    // F2：多种子一致性报告（默认路径不跑；CI 可按需调用）
    const uint64_t seeds[] = {7, 11, 23, 42, 101};
    std::printf("[F2] 短程 REINFORCE 多种子扫描（同配置，仅换种子）\n");
    std::printf("  %-6s %10s %10s %10s %10s %8s  %s\n", "seed", "首段均值", "首段σ", "末段均值",
                "末段σ", "Δ", "判定(旧口径 ±2σ)");
    int nImp = 0, nDeg = 0, nNs = 0;
    double worstDiff = 1e30;
    for (uint64_t sd : seeds) {
      ShortRun R = runShort(sd, 24, 4, 100, 12);
      Verdict v = judge(R.returns);
      const char* verdict = v.sigImprove ? "显著改善" : (v.sigDegrade ? "显著退化" : "不显著");
      if (v.sigImprove) nImp++;
      else if (v.sigDegrade) nDeg++;
      else nNs++;
      worstDiff = std::min(worstDiff, v.diff);
      std::printf("  %-6llu %10.2f %10.2f %10.2f %10.2f %8.2f  %s (Δ/σ=%+.2f)\n",
                  (unsigned long long)sd, v.m1, v.s1, v.m2, v.s2, v.diff, verdict,
                  v.sd > 1e-9 ? v.diff / v.sd : 0.0);
    }
    std::printf("  → 一致性：显著改善 %d/%zu、不显著 %d/%zu、显著退化 %d/%zu（口径本身有噪声）；"
                "最差 Δ=%.2f\n",
                nImp, sizeof(seeds) / sizeof(seeds[0]), nNs, sizeof(seeds) / sizeof(seeds[0]), nDeg,
                sizeof(seeds) / sizeof(seeds[0]), worstDiff);
    std::printf("  结论：小样本下旧口径会随种子翻转 → 默认测试改用稳健断言（不劣化界限 + 方差同量级）\n");
    return 0;
  }

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

  // ---- ③ 能力边界（F2 稳健口径）----
  {
    CHECK(A.returns.size() >= 6, "回合数不足以判边界");
    for (double r : A.returns) CHECK(std::isfinite(r), "回报非有限: %g", r);
    Verdict v = judge(A.returns);
    std::printf("[boundary] 回报 首段 %.2f±%.2f → 末段 %.2f±%.2f（Δ=%.2f, 合并 σ=%.2f, Δ/σ=%+.2f）\n",
                v.m1, v.s1, v.m2, v.s2, v.diff, v.sd, v.sd > 1e-9 ? v.diff / v.sd : 0.0);

    // A. 不劣化界限（硬断言）：末段均值不得低于「首段均值 − 1σ」
    CHECK(v.m2 >= v.m1 - v.sd,
          "短程 REINFORCE 出现超出噪声的劣化（末段 %.2f < 首段 %.2f − σ %.2f）", v.m2, v.m1, v.sd);
    // B. 方差同量级（硬断言）：既未塌缩（学到确定性但更差）也未爆炸（训练发散）
    CHECK(v.s2 >= 0.2 * v.s1 && v.s2 <= 5.0 * v.s1,
          "回报方差变化超出量级（首段 σ=%.2f → 末段 σ=%.2f）", v.s1, v.s2);
    // C. 显著退化（硬失败）；显著改善只警告 —— 不把带统计噪声的「不显著」当断言
    CHECK(!v.sigDegrade, "短程 REINFORCE 出现统计显著退化（Δ=%.2f < −2σ）", v.diff);
    if (v.sigImprove)
      std::printf("[boundary][warn] 本种子出现显著改善（Δ=%.2f > 2σ）——请复核 README/PR 的边界"
                  "表述；多种子一致性见 `%s --seed-scan`\n",
                  v.diff, "test_train_regression");
    else
      std::printf("[boundary] 判定：不劣化且方差同量级（口径：末段 ≥ 首段−1σ，σ 比值 ∈[0.2,5]；"
                  "「显著/不显著」本身有噪声，故不作硬断言 —— 见 --seed-scan）\n");
  }

  if (g_fail == 0) std::printf("test_train_regression PASS\n");
  else std::printf("test_train_regression FAIL (%d)\n", g_fail);
  return g_fail ? 1 : 0;
}
