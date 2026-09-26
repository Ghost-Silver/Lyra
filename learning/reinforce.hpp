// learning/reinforce.hpp — REINFORCE + baseline（滑动平均 return）
// 策略梯度：∇θ J ≈ Σ_t ∇θ log π(a_t|s_t) · (R_t − b)，baseline b 为回报滑动平均；
// 熵正则 β·H 防策略方差塌缩；Adam 更新（ctorch_ext），全局梯度范数裁剪。
// 附示教轨迹行为克隆（BC）预热——示教-回放轨迹可喂给 RL 预训练。
#pragma once
#include "policy.hpp"
#include "ctorch_ext/adam.hpp"
#include "arm/control/json.hpp"
#include <cstdio>
#include <cmath>
#include <deque>
#include <vector>

namespace learn {

struct EpisodeStep {
  std::array<double, kObs> obs{};
  std::array<double, kAct> act{};
  double reward = 0;
};
struct Episode {
  std::vector<EpisodeStep> steps;
  double totalReward = 0;
  bool success = false;
};

class REINFORCETrainer {
 public:
  static constexpr int kStepsPerBatch = 2;
 private:
 public:
  REINFORCETrainer(MLPPolicy& policy, double lr = 3e-3, double gamma = 0.98,
                   double entropyCoef = 0.01, double baselineDecay = 0.95,
                   double maxGradNorm = 5.0)
      : policy_(policy), opt_(policy.parameters(), lr), gamma_(gamma),
        entropy_(entropyCoef), bDecay_(baselineDecay), maxGrad_(maxGradNorm) {}

  ctorch_ext::Adam& optimizer() { return opt_; }

  void addEpisode(Episode ep) {
    // baseline 由 update() 按折扣回报维护（与 advantage 同量纲）
    batch_.push_back(std::move(ep));
  }

  // 设置 BC 锚定数据（防止 RL 更新把策略拖离示教流形）；λ=0 关闭
  void setAnchor(std::vector<std::array<double, kObs>> obss,
                 std::vector<std::array<double, kAct>> acts, double lambda = 10.0) {
    anchorObs_ = std::move(obss);
    anchorAct_ = std::move(acts);
    anchorL_ = lambda;
  }

  // 一次更新（累积的全部 episode 打成一批），返回标量 loss
  double update() {
    if (batch_.empty()) return 0;
    std::vector<float> obsv, actv, wv;
    size_t N = 0;
    for (auto& ep : batch_) N += ep.steps.size();
    obsv.reserve(N * kObs);
    actv.reserve(N * kAct);
    wv.reserve(N);
    // 折扣回报 R[t]——advantage 与 baseline 同量纲（均按折扣回报计）
    std::vector<std::vector<double>> Rs(batch_.size());
    double sumR = 0; size_t nR = 0;
    for (size_t ei = 0; ei < batch_.size(); ei++) {
      auto& ep = batch_[ei];
      size_t T = ep.steps.size();
      Rs[ei].assign(T, 0.0);
      double acc = 0;
      for (size_t t = T; t-- > 0;) {
        acc = gamma_ * acc + ep.steps[t].reward;
        Rs[ei][t] = acc;
        sumR += acc; nR++;
      }
    }
    // baseline = 折扣回报的滑动平均（首用批均值预热）
    double batchMeanR = nR ? sumR / double(nR) : 0.0;
    if (!hasBaseline_) { totalR_ = batchMeanR; hasBaseline_ = true; }
    double b = totalR_;
    for (size_t ei = 0; ei < batch_.size(); ei++) {
      auto& ep = batch_[ei];
      for (size_t t = 0; t < ep.steps.size(); t++) {
        for (double x : ep.steps[t].obs) obsv.push_back(float(x));
        for (double x : ep.steps[t].act) actv.push_back(float(x));
        wv.push_back(float(Rs[ei][t] - b));        // advantage：折扣回报 − 滑动平均 baseline
      }
    }
    totalR_ = bDecay_ * totalR_ + (1.0 - bDecay_) * batchMeanR;
    // 批内中心化 + 标准差白化（保留 EMA baseline 语义，消掉整批同号的退化梯度/量纲）
    double wMean = 0, wVar = 0, wMin = 1e30, wMax = -1e30;
    for (float v : wv) { wMean += v; wMin = std::min(wMin, double(v)); wMax = std::max(wMax, double(v)); }
    wMean /= double(wv.size());
    for (float v : wv) wVar += (v - wMean) * (v - wMean);
    double wStd = std::sqrt(wVar / double(wv.size()));
    if (wStd < 1e-6) wStd = 1.0;
    for (float& v : wv) v = float((v - wMean) / wStd);
    std::printf("[w-stat] N=%zu raw[%.2f..%.2f] mean=%.2f std=%.2f\n", wv.size(), wMin, wMax, wMean, wStd);

    Tensor obs = fromVector(obsv);
    obs = obs.reshape({N, size_t(kObs)});
    Tensor act = fromVector(actv);
    act = act.reshape({N, size_t(kAct)});
    Tensor w = fromVector(wv);
    w = w.reshape({N, 1});

    // 同批多步梯度（Adam 单步位移 ~lr，多步才能让策略移动可见量级）
    double l = 0;
    for (int rep = 0; rep < kStepsPerBatch; rep++) {
      Tensor loss = policy_.loss(obs, act, w, entropy_);
      // BC 锚定项：λ·MSE(mean(sE), aE)（同一张图一并反传）
      if (anchorL_ > 0 && !anchorObs_.empty()) {
        std::vector<float> ao, aa;
        for (auto& s : anchorObs_) for (double x : s) ao.push_back(float(x));
        for (auto& u : anchorAct_) for (double x : u) aa.push_back(float(x));
        Tensor obsE = fromVector(ao); obsE = obsE.reshape({anchorObs_.size(), size_t(kObs)});
        Tensor actE = fromVector(aa); actE = actE.reshape({anchorAct_.size(), size_t(kAct)});
        Tensor diff = policy_.mean(obsE) - actE;
        Tensor bc = (diff * diff).mean();
        loss = loss + anchorL_ * bc;
      }
      AutoGrad::backward(loss.getRelatedNode(), false);
      opt_.clipGradNorm(maxGrad_);
      opt_.step();
      opt_.zeroGrad();
      l = double(loss.item<float>());
    }
    batch_.clear();
    return l;
  }

  // 行为克隆（示教预训练）：(obs, act) 对的 MSE
  double bcStep(const std::vector<std::array<double, kObs>>& obss,
                const std::vector<std::array<double, kAct>>& acts) {
    size_t N = obss.size();
    if (!N) return 0;
    std::vector<float> o, a;
    for (auto& s : obss) for (double x : s) o.push_back(float(x));
    for (auto& u : acts) for (double x : u) a.push_back(float(x));
    Tensor obs = fromVector(o); obs = obs.reshape({N, size_t(kObs)});
    Tensor act = fromVector(a); act = act.reshape({N, size_t(kAct)});
    Tensor mu = policy_.mean(obs);
    Tensor diff = mu - act;
    Tensor loss = (diff * diff).mean();
    AutoGrad::backward(loss.getRelatedNode(), false);
    opt_.clipGradNorm(maxGrad_);
    opt_.step();
    opt_.zeroGrad();
    return double(loss.item<float>());
  }

  double baseline() const { return totalR_; }
  size_t pendingEpisodes() const { return batch_.size(); }

 private:
  MLPPolicy& policy_;
  ctorch_ext::Adam opt_;
  double gamma_, entropy_, bDecay_, maxGrad_;
  double totalR_ = 0;
  bool hasBaseline_ = false;
  std::vector<Episode> batch_;
  std::vector<std::array<double, kObs>> anchorObs_;
  std::vector<std::array<double, kAct>> anchorAct_;
  double anchorL_ = 0;
};

// ---- 示教轨迹（web 导出的 JSON）→ BC (obs, act) 对 ----
// 轨迹点为关节路标；段内线性插值，动作 = 归一化的关节速度方向，
// obs 由 RLEnv 语义生成（目标位姿 = 终点构型的 FK）。
inline void teachJsonToPairs(arm::RLEnv& env, const arm::json::Value& demo,
                             std::vector<std::array<double, kObs>>& obss,
                             std::vector<std::array<double, kAct>>& acts,
                             int substeps = 4) {
  const auto& pts = demo.get("points");
  if (!pts.isArray() || pts.size() < 2) return;
  size_t n = pts.size();
  std::vector<std::array<double, 6>> qs(n);
  for (size_t i = 0; i < n; i++)
    for (int j = 0; j < 6; j++) qs[i][j] = pts[i].numAt(j, 0.0);
  const auto& goal = qs.back();
  for (size_t i = 0; i + 1 < n; i++) {
    for (int s = 0; s < substeps; s++) {
      double a = double(s) / substeps;
      std::array<double, 6> q{};
      std::array<double, kAct> act{};
      for (int j = 0; j < 6; j++) {
        q[j] = qs[i][j] + a * (qs[i + 1][j] - qs[i][j]);
        act[j] = std::clamp((qs[i + 1][j] - qs[i][j]) * 2.0, -1.0, 1.0);
      }
      env.reset(0, &q, &goal);
      arm::RLObs o = env.observe();
      obss.push_back(o.x);
      acts.push_back(act);
    }
  }
}

}  // namespace learn
