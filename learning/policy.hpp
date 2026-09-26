// learning/policy.hpp — MLP 高斯策略 (obs 18 → act 6)
// mean = MLP(obs)（Linear 栈 + tanh），log_std 为 6 维可学习叶子；
// a ~ N(mean, diag(σ²))，log π(a|s) 用 CTorch 算子建图，梯度端到端。
#pragma once
#include "Tensor.h"
#include "AutoGrad.h"
#include "arm/sim.hpp"
#include "ctorch_ext/linear.hpp"
#include <array>
#include <cmath>
#include <random>
#include <vector>

namespace learn {

using ctorch_ext::Activation;
using ctorch_ext::Linear;

constexpr int kObs = arm::RLEnv::kObsDim;   // 18
constexpr int kAct = arm::RLEnv::kActDim;   // 6
constexpr double kLog2Pi = 1.8378770664093453;

// std::vector → 1D Tensor 辅助（CTorch 无 vector 构造函数）
inline Tensor fromVector(const std::vector<float>& v) {
  Tensor t(ShapeTag{}, {v.size()}, DType::kFloat, DeviceType::kCPU);
  float* d = t.data<float>();
  std::copy(v.begin(), v.end(), d);
  return t;
}

class MLPPolicy {
 public:
  explicit MLPPolicy(int hidden = 64, uint64_t seed = 1)
      : l1_(kObs, hidden, Activation::Tanh, seed + 1),
        l2_(hidden, hidden, Activation::Tanh, seed + 2),
        l3_(hidden, kAct, Activation::None, seed + 3),
        logStd_(ShapeTag{}, {kAct}, DType::kFloat, DeviceType::kCPU),
        rng_(seed) {
    float* s = logStd_.data<float>();
    for (int i = 0; i < kAct; i++) s[i] = -0.7f;   // σ ≈ 0.50 初始（足够探索）
    logStd_.requires_grad(true);
  }

  // mean 前向：obs [B,18] → [B,6]
  Tensor mean(const Tensor& obs) {
    return l3_.forward(l2_.forward(l1_.forward(obs)));
  }

  // 批量对数概率：返回 [B,1]（图内可微，叶子 = W/b/logStd）
  Tensor logProb(const Tensor& obs, const Tensor& act) {
    size_t B = obs.size(0);
    Tensor mu = mean(obs);
    // Lσ = ones(B,1)·logStdᵀ → (B,6)（用 MatMul 展开，保证可微汇总）
    Tensor onesB(ShapeTag{}, {B, 1}, DType::kFloat, DeviceType::kCPU);
    onesB.ones();
    Tensor lsRow = logStd_.reshape({1, kAct});
    Tensor Lsig = onesB.matmul(lsRow);          // (B,6) = log σ
    Tensor sigma = Lsig.exp();                  // σ
    Tensor diff = act - mu;                     // (B,6)
    Tensor z = diff / sigma;
    Tensor z2 = z * z;
    // 按步加权前要把权重铺成 (B,6)，这里先给出「等权」逐行 logπ：sum 维 1
    Tensor row = z2.sum(1, true) * (-0.5) - Lsig.sum(1, true) - 0.5 * kLog2Pi * double(kAct);
    return row;                                  // (B,1)
  }

  // 加权策略梯度损失 + 熵正则：
  //   L = −Σ_t w_t·logπ_t − β·B·H ,  H = Σ_j logσ_j + C
  // w: [B,1] 常数张量（advantage，无梯度）
  Tensor loss(const Tensor& obs, const Tensor& act, const Tensor& w, double entropyCoef) {
    size_t B = obs.size(0);
    Tensor mu = mean(obs);
    Tensor onesB(ShapeTag{}, {B, 1}, DType::kFloat, DeviceType::kCPU);
    onesB.ones();
    Tensor Lsig = onesB.matmul(logStd_.reshape({1, kAct}));   // (B,6)
    Tensor sigma = Lsig.exp();
    Tensor z = (act - mu) / sigma;
    Tensor z2 = z * z;
    // W6 = w · ones(1,6) → (B,6) 权重广播（MatMul 展开保持可微）
    Tensor ones6(ShapeTag{}, {1, kAct}, DType::kFloat, DeviceType::kCPU);
    ones6.ones();
    Tensor W6 = w.matmul(ones6);                 // (B,6)
    Tensor weightedNll = (W6 * z2) * 0.5 + W6 * Lsig;
    Tensor nll = weightedNll.sum();
    Tensor entropy = Lsig.sum();                  // H 的可学习部分（Σ logσ）
    // 熵奖励：L 含 −β·Σ_i H_i（entropy=Σ_iΣ_j logσ_j 已含批求和，勿再乘 B）
    return nll - entropyCoef * entropy;
  }

  // σ 访问（诊断/日志用）
  Tensor& logStd() { return logStd_; }
  const Tensor& logStd() const { return logStd_; }

  // 确定性采样（给定 RNG）：单步动作（重参数不必要，REINFORCE 用 score func）
  std::array<double, kAct> sample(const std::array<double, kObs>& obs) {
    Tensor o = fromVector(std::vector<float>(obs.begin(), obs.end()));
    o = o.reshape({1, size_t(kObs)});
    Tensor mu = mean(o);
    const float* m = mu.data_read<float>();
    const float* ls = logStd_.data_read<float>();
    std::array<double, kAct> a{};
    std::normal_distribution<double> N(0.0, 1.0);
    for (int i = 0; i < kAct; i++)
      a[i] = double(m[i]) + std::exp(double(ls[i])) * N(rng_);
    return a;
  }

  std::array<double, kAct> deterministic(const std::array<double, kObs>& obs) {
    Tensor o = fromVector(std::vector<float>(obs.begin(), obs.end()));
    o = o.reshape({1, size_t(kObs)});
    Tensor mu = mean(o);
    const float* m = mu.data_read<float>();
    std::array<double, kAct> a{};
    for (int i = 0; i < kAct; i++) a[i] = double(m[i]);
    return a;
  }

  std::vector<Tensor*> parameters() {
    auto p = l1_.parameters();
    auto q = l2_.parameters();
    auto r = l3_.parameters();
    p.insert(p.end(), q.begin(), q.end());
    p.insert(p.end(), r.begin(), r.end());
    p.push_back(&logStd_);
    return p;
  }

  // 序列化：全部参数拍平导出/导入（checkpoint）
  std::vector<float> dumpParams() const {
    std::vector<float> out;
    auto dump = [&](const Tensor& t) {
      const float* d = t.data_read<float>();
      out.insert(out.end(), d, d + t.numel());
    };
    dump(l1_.W()); dump(l1_.b());
    dump(l2_.W()); dump(l2_.b());
    dump(l3_.W()); dump(l3_.b());
    dump(logStd_);
    return out;
  }
  bool loadParams(const std::vector<float>& in) {
    if (in.size() != dumpParams().size()) return false;
    size_t off = 0;
    auto load = [&](Tensor& t) {
      float* d = t.data<float>();
      std::memcpy(d, in.data() + off, t.numel() * sizeof(float));
      off += t.numel();
    };
    load(l1_.W()); load(l1_.b());
    load(l2_.W()); load(l2_.b());
    load(l3_.W()); load(l3_.b());
    load(logStd_);
    return true;
  }

  std::mt19937_64& rng() { return rng_; }

 private:
  Linear l1_, l2_, l3_;
  Tensor logStd_;
  std::mt19937_64 rng_;
};

}  // namespace learn
