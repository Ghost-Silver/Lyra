// tests/test_policy_mechanics.cpp — 策略/训练机制单测（F1）
//
// 覆盖此前无测试的四项「静默失效」高风险点：
//   1) 可学习 logStd：确实进入优化器、被更新、熵项下方向正确（σ 增大）
//   2) 熵正则项：loss 值 = nll − β·Σlogσ 的定量关系（不是「看起来没炸」）
//   3) 全局梯度裁剪 clipGradNorm：人为大梯度 → 裁剪后范数**等于**阈值，且方向不变
//   4) advantage 白化：白化后均值≈0/标准差≈1；全同号退化分支不除零（走 1.0 兜底）
#include "learning/policy.hpp"
#include "learning/reinforce.hpp"

#include <cmath>
#include <cstdio>
#include <vector>

using namespace learn;

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

static Tensor makeObs(int B, uint64_t seed) {
  Tensor t(ShapeTag{}, {size_t(B), size_t(kObs)}, DType::kFloat, DeviceType::kCPU);
  float* d = t.data<float>();
  for (int i = 0; i < B * kObs; i++)
    d[i] = float(std::sin(0.37 * double(i) + 0.11 * double(seed)));
  return t;
}
static Tensor makeAct(int B, uint64_t seed) {
  Tensor t(ShapeTag{}, {size_t(B), size_t(kAct)}, DType::kFloat, DeviceType::kCPU);
  float* d = t.data<float>();
  for (int i = 0; i < B * kAct; i++)
    d[i] = float(0.4 * std::cos(0.21 * double(i) + 0.07 * double(seed)));
  return t;
}
static Tensor makeW(const std::vector<float>& vals) {
  Tensor t(ShapeTag{}, {vals.size(), 1}, DType::kFloat, DeviceType::kCPU);
  float* d = t.data<float>();
  for (size_t i = 0; i < vals.size(); i++) d[i] = vals[i];
  return t;
}

int main() {
  // ================= 1. logStd：优化器成员 + 熵项梯度方向 + 实际更新 =================
  {
    MLPPolicy pol(16, 11);
    Tensor obs = makeObs(5, 1), act = makeAct(5, 1);
    Tensor w0 = makeW(std::vector<float>(5, 0.0f));   // 权重全零：只剩熵项起作用
    const double beta = 0.05;
    Tensor loss = pol.loss(obs, act, w0, beta);
    AutoGrad::backward(loss.getRelatedNode(), false);

    // 解析梯度：loss = 0 − β·Σ_{b,j} logσ_j ⇒ ∂/∂logσ_j = −β·B（与动作、观测无关）
    const float* gs = pol.logStd().grad_ptr();
    const int B = 5;
    bool normCorrect = (gs != nullptr);
    double maxDev = 0;
    for (int j = 0; j < kAct && gs; j++)
      maxDev = std::max(maxDev, std::abs(double(gs[j]) - (-beta * B)));
    CHECK(normCorrect && maxDev < 1e-4,
          "熵项对 logStd 的梯度应为 −β·B=%.3f（实测最大偏差 %.2e）", -beta * B, maxDev);

    // logStd 必须在参数列表里（否则优化器永远不会更新它）
    bool inParams = false;
    for (Tensor* p : pol.parameters())
      if (p == &pol.logStd()) inParams = true;
    CHECK(inParams, "logStd 未进入 parameters()（会被优化器忽略）");

    // 走一次真实优化：σ 必须增大（熵奖励的方向），且位移量级 ~lr
    std::vector<float> before(kAct);
    for (int j = 0; j < kAct; j++) before[size_t(j)] = pol.logStd().data_read<float>()[j];
    AutoGrad::backward(pol.loss(obs, act, w0, beta).getRelatedNode(), false);  // 重新建图（上面已反传过）
    ctorch_ext::Adam opt(pol.parameters(), 1e-2);
    opt.clipGradNorm(5.0);
    opt.step();
    const float* ls = pol.logStd().data_read<float>();
    double dmin = 1e9, dmax = -1e9;
    for (int j = 0; j < kAct; j++) {
      double d = double(ls[j]) - double(before[size_t(j)]);
      dmin = std::min(dmin, d);
      dmax = std::max(dmax, d);
    }
    CHECK(dmin > 0, "logStd 未被更新或方向错误（Δ∈[%.2e, %.2e]，应全为正：熵奖励增大 σ）", dmin, dmax);
    CHECK(dmax < 1e-1, "logStd 位移失控（Δmax=%.3e，量级应 ~lr=1e-2）", dmax);
    std::printf("[F1-1] logStd：梯度=−β·B=%.3f（偏差 %.1e）；一次 Adam(lr=1e-2) 后 ΔlogStd∈[%.2e,%.2e]"
                "（σ=%.3f→%.3f，方向正确）\n",
                -beta * B, maxDev, dmin, dmax, std::exp(before[0]), std::exp(double(ls[0])));
  }

  // ================= 2. 熵正则项：loss 值定量关系 =================
  {
    MLPPolicy pol(16, 21);
    Tensor obs = makeObs(7, 2), act = makeAct(7, 2);
    Tensor w = makeW({1.f, -1.f, 0.5f, -0.5f, 0.25f, -0.25f, 0.f});
    Tensor l0 = pol.loss(obs, act, w, 0.0);
    Tensor l1 = pol.loss(obs, act, w, 0.10);
    const float* ls = pol.logStd().data_read<float>();
    double sumLogSigma = 0;
    for (int j = 0; j < kAct; j++) sumLogSigma += ls[j];
    const int B = 7;
    double lhs = double(l0.item<float>()) - double(l1.item<float>());
    // 语义（policy.hpp 注释）：entropy = Σ_i Σ_j logσ_j 为**批内总和**，
    //   故 L = nll − β·Σ_i H_i ⇒ 差值 = β·B·Σ_j logσ_j（有效熵权重随批量线性放大；
    //   若需 per-sample 平均熵，应改为 β/B）。
    double rhs = 0.10 * double(B) * sumLogSigma;
    CHECK(std::abs(lhs - rhs) < 1e-3 * (1.0 + std::abs(rhs)),
          "熵项定量关系不符：loss(β=0)−loss(β=0.1)=%.5f，应为 β·Σlogσ=%.5f", lhs, rhs);
    std::printf("[F1-2] 熵正则：loss(β=0)−loss(β=0.10) = %.5f，β·B·Σlogσ = %.5f（B=%d，差 %.2e）"
                " —— 语义=批内总熵（有效权重随批量线性放大）\n",
                lhs, rhs, B, std::abs(lhs - rhs));
  }

  // ================= 3. clipGradNorm：裁到阈值且方向不变 =================
  {
    MLPPolicy pol(16, 31);
    Tensor obs = makeObs(4, 3), act = makeAct(4, 3);
    // 巨大 advantage → 巨大梯度（确保裁剪被触发，而不是「看起来没炸」）
    Tensor w = makeW({1e4f, -1e4f, 8e3f, -8e3f});
    Tensor loss = pol.loss(obs, act, w, 0.0);
    AutoGrad::backward(loss.getRelatedNode(), false);
    ctorch_ext::Adam opt(pol.parameters(), 3e-3);
    auto flatten = [&]() {
      std::vector<double> v;
      for (Tensor* p : pol.parameters()) {
        const float* g = p->grad_ptr();
        if (!g) continue;
        for (size_t i = 0; i < p->numel(); i++) v.push_back(double(g[i]));
      }
      return v;
    };
    std::vector<double> pre = flatten();
    double preNorm = opt.gradNorm();
    const double thr = 1.0;
    opt.clipGradNorm(thr);
    std::vector<double> post = flatten();
    double postNorm = opt.gradNorm();

    CHECK(preNorm > 10.0 * thr, "构造的大梯度不足（裁剪前范数 %.3f，需 ≫ %.1f）", preNorm, thr);
    CHECK(std::abs(postNorm - thr) < 1e-6, "裁剪后范数应精确等于阈值：%.9f（实测 %.9f）", thr, postNorm);
    // 方向不变：逐元素应为同一缩放因子
    double ratioDev = 0, maxAbs = 1e-30;
    for (size_t i = 0; i < pre.size(); i++) {
      double want = pre[i] * (thr / preNorm);
      ratioDev = std::max(ratioDev, std::abs(post[i] - want));
      maxAbs = std::max(maxAbs, std::abs(want));
    }
    // 梯度是 float32：绝对偏差需按量级相对判定（裁剪前范数 ~1e5）
    CHECK(ratioDev < 1e-5 * maxAbs, "裁剪应等比缩放（绝对偏差 %.2e，量级 %.2e）", ratioDev, maxAbs);
    std::printf("[F1-3] 梯度裁剪：裁剪前范数 %.1f → 裁剪(阈值 %.0f) 后 %.9f（=阈值），"
                "等比缩放逐元素偏差 %.1e\n",
                preNorm, thr, postNorm, ratioDev);
  }

  // ================= 4. advantage 白化：均值≈0/方差≈1 + 退化分支不除零 =================
  {
    MLPPolicy pol(16, 41);
    REINFORCETrainer tr(pol, 1e-3, 0.98, 0.01, 0.95, 5.0);
    auto mkEp = [](double base, double step, int T, uint64_t sd) {
      Episode ep;
      for (int t = 0; t < T; t++) {
        EpisodeStep st;
        for (int i = 0; i < kObs; i++) st.obs[size_t(i)] = std::sin(0.3 * i + 0.1 * t + double(sd));
        for (int i = 0; i < kAct; i++) st.act[size_t(i)] = 0.2 * std::cos(0.2 * i + 0.05 * t);
        st.reward = base + step * t;
        ep.steps.push_back(st);
      }
      return ep;
    };
    tr.addEpisode(mkEp(1.0, 0.3, 6, 1));
    tr.addEpisode(mkEp(-1.0, 0.1, 6, 2));
    double l = tr.update();
    const auto& w1 = tr.lastWhiten();
    CHECK(w1.n == 12, "白化样本数 %zu（应为 12）", w1.n);
    CHECK(!w1.stdFloorUsed && w1.rawStd > 1e-3, "本用例不应触发退化分支（rawStd=%.4f）", w1.rawStd);
    CHECK(std::abs(w1.postMean) < 1e-6, "白化后均值应≈0（实测 %.3e）", w1.postMean);
    CHECK(std::abs(w1.postStd - 1.0) < 2e-3, "白化后标准差应≈1（实测 %.6f）", w1.postStd);
    CHECK(std::isfinite(l), "update 返回非有限 loss: %g", l);

    // 退化分支：各步回报完全相同（rawStd≈0）→ 必须走 1.0 兜底而非除零
    REINFORCETrainer tr2(pol, 1e-3, 0.98, 0.01, 0.95, 5.0);
    // 注意：常数奖励经折扣累积后 R_t 并非常数（γ^t 加权），故用**全零奖励**构造
    // 真正零方差的 advantage 批（R≡0 ⇒ wMean=0, wStd=0 ⇒ 必然走兜底分支）
    auto flat = mkEp(0.0, 0.0, 5, 3);
    tr2.addEpisode(flat);
    tr2.addEpisode(flat);
    double l2 = tr2.update();
    const auto& w2 = tr2.lastWhiten();
    CHECK(w2.rawStd < 1e-6 && w2.stdFloorUsed,
          "零方差批应触发退化分支（rawStd=%.2e, floor=%d）", w2.rawStd, int(w2.stdFloorUsed));
    CHECK(std::isfinite(w2.postMean) && std::isfinite(w2.postStd) && std::abs(w2.postMean) < 1e-9,
          "退化分支不应产生 NaN/Inf（postMean=%.3e postStd=%.3e）", w2.postMean, w2.postStd);
    CHECK(std::isfinite(l2), "退化分支 update 返回非有限 loss: %g", l2);
    std::printf("[F1-4] 白化：正常批 postMean=%.2e postStd=%.4f（rawStd=%.3f）；"
                "全同回报批 rawStd=%.2e → 兜底 floor=%d，postMean=%.1e（无除零）\n",
                w1.postMean, w1.postStd, w1.rawStd, w2.rawStd, int(w2.stdFloorUsed), w2.postMean);
  }

  if (g_fail == 0) std::printf("test_policy_mechanics PASS\n");
  else std::printf("test_policy_mechanics FAIL (%d)\n", g_fail);
  return g_fail ? 1 : 0;
}
