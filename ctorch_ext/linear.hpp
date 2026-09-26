// ctorch_ext/linear.hpp — 给 CTorch 补的前置功①：Linear 全连接层
// y = act(x·Wᵀ + b)。W(out,in)、b(out) 为可学习叶子张量（requires_grad），
// 前向完全由 CTorch 可微算子组合（MatMul/Add/Tanh/ReLU/Sigmoid），
// 梯度经 AutoGrad 反传到 W/b。偏置用 ones·b 的外积展开，保证图内可微。
#pragma once
#include "Tensor.h"
#include "AutoGrad.h"
#include <cmath>
#include <random>
#include <vector>

namespace ctorch_ext {

enum class Activation { None, ReLU, Tanh, Sigmoid };

class Linear {
 public:
  Linear(size_t in, size_t out, Activation act = Activation::None, uint64_t seed = 0)
      : in_(in), out_(out), act_(act),
        W_(ShapeTag{}, {out, in}, DType::kFloat, DeviceType::kCPU),
        b_(ShapeTag{}, {out}, DType::kFloat, DeviceType::kCPU) {
    // Xavier 均匀初始化（tanh）/ He 均匀（ReLU），确定性种子
    std::mt19937_64 rng(seed ? seed : uint64_t(in * 131 + out));
    double g = (act == Activation::ReLU) ? std::sqrt(6.0 / double(in))
                                         : std::sqrt(6.0 / double(in + out));
    std::uniform_real_distribution<double> U(-g, g);
    float* w = W_.data<float>();
    for (size_t i = 0; i < in * out; i++) w[i] = float(U(rng));
    b_.zero();
    W_.requires_grad(true);
    b_.requires_grad(true);
  }

  // x: [B, in] → [B, out]
  Tensor forward(const Tensor& x) {
    size_t B = x.size(0);
    // y = x · Wᵀ
    Tensor y = x.matmul(W_.t());
    // 偏置展开：ones(B,1) · b(1,out) → (B,out)（全程可微，梯度汇总到 b）
    Tensor onesB(ShapeTag{}, {B, 1}, DType::kFloat, DeviceType::kCPU);
    onesB.ones();
    Tensor brow = b_.reshape({1, out_});
    y = y + onesB.matmul(brow);
    switch (act_) {
      case Activation::ReLU: y = y.relu(); break;
      case Activation::Tanh: y = y.tanh(); break;
      case Activation::Sigmoid: y = y.sigmoid(); break;
      case Activation::None: break;
    }
    return y;
  }

  std::vector<Tensor*> parameters() { return {&W_, &b_}; }
  Tensor& W() { return W_; }
  Tensor& b() { return b_; }
  const Tensor& W() const { return W_; }
  const Tensor& b() const { return b_; }
  size_t in() const { return in_; }
  size_t out() const { return out_; }

 private:
  size_t in_, out_;
  Activation act_;
  Tensor W_, b_;
};

}  // namespace ctorch_ext
