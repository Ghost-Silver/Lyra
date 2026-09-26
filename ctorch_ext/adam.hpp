// ctorch_ext/adam.hpp — 给 CTorch 补的前置功②：Adam 优化器
// 标准 Adam（Kingma & Ba, 2015）：m = β1 m + (1−β1)g；v = β2 v + (1−β2)g²；
// 偏置修正 m̂ = m/(1−β1^t)、v̂ = v/(1−β2^t)；θ −= α·m̂/(√v̂+ε)。
// 附带全局梯度范数裁剪 utility。参数/梯度经 CTorch 原始指针读写（就地更新）。
#pragma once
#include "Tensor.h"
#include <cmath>
#include <cstring>
#include <vector>

namespace ctorch_ext {

class Adam {
 public:
  Adam(std::vector<Tensor*> params, double lr = 3e-4,
       double beta1 = 0.9, double beta2 = 0.999, double eps = 1e-8,
       double weightDecay = 0.0)
      : params_(std::move(params)), lr_(lr), b1_(beta1), b2_(beta2),
        eps_(eps), wd_(weightDecay) {
    for (Tensor* p : params_) {
      m_.emplace_back(ShapeTag{}, p->shape(), DType::kFloat, p->device());
      v_.emplace_back(ShapeTag{}, p->shape(), DType::kFloat, p->device());
      m_.back().zero();
      v_.back().zero();
      steps_.push_back(0);
    }
  }

  void setLr(double lr) { lr_ = lr; }
  double lr() const { return lr_; }

  // 单步更新（无梯度的参数自动跳过）
  void step() {
    for (size_t k = 0; k < params_.size(); k++) {
      Tensor* p = params_[k];
      float* gptr = p->grad_ptr();
      if (!gptr) continue;
      float* w = p->data<float>();
      float* m = m_[k].data<float>();
      float* v = v_[k].data<float>();
      size_t n = p->numel();
      long t = ++steps_[k];
      double bc1 = 1.0 - std::pow(b1_, double(t));
      double bc2 = 1.0 - std::pow(b2_, double(t));
      for (size_t i = 0; i < n; i++) {
        double g = gptr[i] + wd_ * w[i];
        m[i] = float(b1_ * m[i] + (1.0 - b1_) * g);
        v[i] = float(b2_ * v[i] + (1.0 - b2_) * g * g);
        double mh = m[i] / bc1;
        double vh = v[i] / bc2;
        w[i] -= float(lr_ * mh / (std::sqrt(vh) + eps_));
      }
    }
  }

  void zeroGrad() {
    for (Tensor* p : params_) p->zero_grad();
  }

  // 全局梯度 L2 范数（裁剪前用）
  double gradNorm() const {
    double s = 0;
    for (Tensor* p : params_) {
      float* gptr = p->grad_ptr();
      if (!gptr) continue;
      size_t n = p->numel();
      for (size_t i = 0; i < n; i++) s += double(gptr[i]) * gptr[i];
    }
    return std::sqrt(s);
  }

  // 超过 maxNorm 时等比缩放全部梯度
  void clipGradNorm(double maxNorm) {
    double n = gradNorm();
    if (n <= maxNorm || n < 1e-12) return;
    double s = maxNorm / n;
    for (Tensor* p : params_) {
      float* gptr = p->grad_ptr();
      if (!gptr) continue;
      size_t cnt = p->numel();
      for (size_t i = 0; i < cnt; i++) gptr[i] = float(gptr[i] * s);
    }
  }

 private:
  std::vector<Tensor*> params_;
  std::vector<Tensor> m_, v_;
  std::vector<long> steps_;
  double lr_, b1_, b2_, eps_, wd_;
};

}  // namespace ctorch_ext
