// tests/test_learn.cpp — ctorch_ext + 学习层单元测试（需 ARM_ENABLE_CTORCH=ON）
// 覆盖：Linear 前向/反传（有限差分对照）、Adam 单步对照手算、
//       策略 logπ 图梯度、REINFORCE 一次更新 + BC 下降。
#include "Tensor.h"
#include "AutoGrad.h"
#include "ctorch_ext/linear.hpp"
#include "ctorch_ext/adam.hpp"
#include "learning/policy.hpp"
#include "learning/reinforce.hpp"
#include <cstdio>
#include <cmath>
#include <vector>

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

using namespace ctorch_ext;
using namespace learn;

int main() {
  // ---- 1. Linear 前向形状/数值 + 梯度 vs 有限差分 ----
  {
    Linear lin(3, 2, Activation::None, 42);
    // 固定参数
    float* w = lin.W().data<float>();
    for (int i = 0; i < 6; i++) w[i] = 0.1f * (i + 1);
    float* b = lin.b().data<float>();
    b[0] = 0.5f; b[1] = -0.25f;

    Tensor x = fromVector(std::vector<float>{1, 2, 3, -1, 0, 2});   // (2,3)
    x = x.reshape({2, 3});
    Tensor y = lin.forward(x);
    CHECK(y.size(0) == 2 && y.size(1) == 2, "shape");
    const float* yd = y.data_read<float>();
    // 手算 row0: [0.1..0.6]·[1,2,3] = 1.4+0.5 = 1.9；row1: w·[-1,0,2] = -0.1-0.3+0.5... 
    // w = [0.1,0.2,0.3; 0.4,0.5,0.6]（按行）
    CHECK(std::abs(yd[0] - (0.1*1+0.2*2+0.3*3 + 0.5f)) < 1e-5, "y00=%.4f", yd[0]);
    CHECK(std::abs(yd[1] - (0.4*1+0.5*2+0.6*3 - 0.25f)) < 1e-5, "y01=%.4f", yd[1]);

    // 标量 loss = y.sum() 的梯度 vs 有限差分（对 W[0]）
    Tensor loss = y.sum();
    AutoGrad::backward(loss.getRelatedNode(), false);
    float g = lin.W().grad_ptr()[0];
    CHECK(std::abs(g - (1.0f + (-1.0f))) < 1e-5, "dL/dW00 解析=%.3f 应= x00+x10=0", g);
    // W00 = y00 的 w 系数作用在 x 行 → dsum/dW00 = x[0][0] + x[1][0] = 1 + (−1) = 0
    float gb = lin.b().grad_ptr()[0];
    CHECK(std::abs(gb - 2.0f) < 1e-5, "dL/db0=%.3f 应=2", gb);
  }

  // ---- 2. Adam 单步 vs 手算 ----
  {
    Tensor p = fromVector(std::vector<float>{1.0f});
    p = p.reshape({1});
    p.requires_grad(true);
    // 手工塞梯度：用一次 dummy 图 p*2 → grad=2
    Tensor d = p * 2.0f;
    AutoGrad::backward(d.getRelatedNode(), false);
    Adam opt({&p}, 0.1, 0.9, 0.999, 1e-8);
    opt.step();
    // 手算: m=0.2, v=0.004, m̂=2, v̂=4?  t=1: bc1=0.1,bc2=0.001 → m̂=0.2/0.1=2, v̂=0.004/0.001=4
    // Δ = 0.1·2/(2+1e-8) = 0.09999... ≈ 0.1
    double w = double(p.data_read<float>()[0]);
    CHECK(std::abs(w - (1.0 - 0.1 * 2.0 / (std::sqrt(4.0) + 1e-8))) < 1e-6,
          "Adam 更新后 w=%.6f", w);
  }

  // ---- 3. 策略 logπ 图梯度有限差分对照 ----
  {
    MLPPolicy pol(8, 3);
    auto params = pol.parameters();
    Tensor obs = fromVector(std::vector<float>(18, 0.1f));
    obs = obs.reshape({1, 18});
    Tensor act = fromVector(std::vector<float>(6, 0.0f));
    act = act.reshape({1, 6});
    Tensor lp = pol.logProb(obs, act);
    AutoGrad::backward(lp.getRelatedNode(), false);
    Tensor* W = params[0];
    float an = W->grad_ptr()[3];
    // 有限差分
    float* wptr = W->data<float>();
    float save = wptr[3];
    auto fwd = [&](float x) {
      wptr[3] = x;
      Tensor l2 = pol.logProb(obs, act);
      return l2.item<float>();
    };
    float h = 1e-3f;
    float num = (fwd(save + h) - fwd(save - h)) / (2 * h);
    wptr[3] = save;
    CHECK(std::abs(an - num) < 5e-3, "logπ 梯度 解析=%.5f 数值=%.5f", an, num);
  }

  // ---- 4. REINFORCE 更新 + BC 下降 ----
  {
    MLPPolicy pol(16, 11);
    REINFORCETrainer tr(pol, 1e-2, 0.95, 0.0);
    // 两个假 episode
    for (int k = 0; k < 2; k++) {
      Episode ep;
      for (int t = 0; t < 8; t++) {
        EpisodeStep s;
        for (int i = 0; i < kObs; i++) s.obs[i] = 0.01 * (t + i);
        for (int i = 0; i < kAct; i++) s.act[i] = 0.1 * k;
        s.reward = -1.0 + 0.1 * t;
        ep.totalReward += s.reward;
        ep.steps.push_back(s);
      }
      tr.addEpisode(std::move(ep));
    }
    double loss = tr.update();
    CHECK(std::isfinite(loss), "loss 非有限");

    // BC：向固定目标拟合，loss 应下降
    std::vector<std::array<double, kObs>> obss(4);
    std::vector<std::array<double, kAct>> acts(4);
    for (int i = 0; i < 4; i++)
      for (int j = 0; j < kAct; j++) acts[i][j] = 0.5;
    double l0 = 1e30, l1 = 0;
    for (int e = 0; e < 30; e++) {
      double l = tr.bcStep(obss, acts);
      if (e == 0) l0 = l;
      l1 = l;
    }
    CHECK(l1 < l0, "BC loss 未下降 %.5f → %.5f", l0, l1);
  }

  if (g_fail == 0) std::printf("test_learn PASS\n");
  else std::printf("test_learn FAIL (%d)\n", g_fail);
  return g_fail ? 1 : 0;
}
