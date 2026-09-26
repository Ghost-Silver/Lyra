// tests/test_ctorch_transpose_grad.cpp — CTorch 转置参数梯度回归（A1，PATCHES.md 第 7 条）
// 背景：`transposeNoGrad` 只改元数据（内存平铺序不变），旧实现按线性下标把 (m,n) 梯度写进
// (n,m) 叶子缓冲区 → `y = x.matmul(W.t())` 类**转置参数**的梯度整体错位且不报错。
// 本测试是补丁有效性的唯一硬证据，必须同时具备：
//   ① 正向断言：解析梯度 vs 中央差分逐元素吻合（1e-5）
//   ② 负向断言：把历史错误值喂给同一个比较器必须**判不通过**（证明测试真能抓到回归，
//      而不是「任何值都通过」的假通过）
// 回退该补丁后本测试必须 FAIL（见 PR 回复中的实测输出）。
#include "Tensor.h"
#include "AutoGrad.h"

#include <cmath>
#include <cstdio>
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

// 比较器：解析梯度（float*）vs 参考梯度（double 向量）的最大绝对偏差
static double maxAbsDiff(const float* g, const std::vector<double>& ref) {
  double m = 0;
  for (size_t i = 0; i < ref.size(); i++) m = std::max(m, std::abs(double(g[i]) - ref[i]));
  return m;
}
static bool gradMatches(const float* g, const std::vector<double>& ref, double tol) {
  return maxAbsDiff(g, ref) <= tol;
}

static void printVec(const char* tag, const float* g, size_t n) {
  std::printf("  %s = [", tag);
  for (size_t i = 0; i < n; i++) std::printf("%s%.4f", i ? " " : "", double(g[i]));
  std::printf("]\n");
}

int main() {
  // ================= 用例 1：y = x·Wᵀ，L = sum(y)（历史 bug 的原始复现用例）=================
  // x = [[1,2,3],[0,0,0]]，W 形状 (2,3)：
  //   L = Σ_i Σ_j Σ_k x_ik W_jk  ⇒  dL/dW_jk = Σ_i x_ik = [1,2,3]（两行相同）
  //   正确值（行优先展平）：[1 2 3 1 2 3]
  //   历史错误值（转置布局原样落盘）：[1 1 2 2 3 3]
  //   （与 third_party/PATCHES.md 第 7 条记录的实测数字同源）
  {
    const int B = 2, IN = 3, OUT = 2;
    Tensor x(ShapeTag{}, {B, IN}, DType::kFloat, DeviceType::kCPU);
    {
      float* d = x.data<float>();
      const float xv[B * IN] = {1, 2, 3, 0, 0, 0};
      for (int i = 0; i < B * IN; i++) d[i] = xv[i];
    }
    Tensor W(ShapeTag{}, {OUT, IN}, DType::kFloat, DeviceType::kCPU);
    {
      float* d = W.data<float>();
      for (int i = 0; i < OUT * IN; i++) d[i] = 0.1f * float(i + 1);   // 值任意（L 对 W 线性）
    }
    W.requires_grad(true);

    Tensor y = x.matmul(W.t());          // (2,3)·(3,2) → (2,2)；经 .t() 的转置参数
    Tensor loss = y.sum();
    AutoGrad::backward(loss.getRelatedNode(), false);

    const float* g = W.grad_ptr();
    CHECK(g != nullptr, "转置参数应收到梯度");
    printVec("dL/dW 解析", g, OUT * IN);

    // ① 正向断言：与解析期望值一致
    std::vector<double> expect = {1, 2, 3, 1, 2, 3};
    CHECK(gradMatches(g, expect, 1e-5), "解析梯度与期望值不符，最大偏差 %.3e",
          maxAbsDiff(g, expect));

    // ① 正向断言：与中央差分一致（独立于手算期望，抓任何系统性错位）
    std::vector<double> numeric;
    {
      float* w = W.data<float>();
      std::vector<float> base(OUT * IN);
      for (int i = 0; i < OUT * IN; i++) base[i] = w[i];
      const double h = 1e-2;   // float32 张量：h 需足够大以避开抵消误差（1e-3 实测偏差 1.7e-4）
      auto lossAt = [&]() {
        Tensor yy = x.matmul(W.t());
        Tensor s = yy.sum();
        return double(s.data_read<float>()[0]);
      };
      for (int i = 0; i < OUT * IN; i++) {
        w[i] = base[i] + float(h);
        double lp = lossAt();
        w[i] = base[i] - float(h);
        double lm = lossAt();
        w[i] = base[i];
        numeric.push_back((lp - lm) / (2 * h));
      }
    }
    CHECK(gradMatches(g, numeric, 1e-4), "解析梯度 vs 中央差分不符，最大偏差 %.3e",
          maxAbsDiff(g, numeric));

    // ② 负向断言：历史错误值必须被判不通过（证明判别力）
    std::vector<double> buggy = {1, 1, 2, 2, 3, 3};
    double dnum = 0, dbug = 0;
    for (int i = 0; i < OUT * IN; i++) {
      dnum += std::abs(numeric[i] - expect[i]);
      dbug += std::abs(numeric[i] - buggy[i]);
    }
    std::printf("  |numeric-correct|_1=%.2e   |numeric-buggy|_1=%.2e\n", dnum, dbug);
    CHECK(dnum < 1e-3, "中央差分本身应贴合正确值（_1 偏差 %.2e，float32 差分精度）", dnum);
    CHECK(!gradMatches(g, buggy, 1e-4), "负向断言失败：错误值也通过了比较器（测试无判别力）");
    CHECK(dbug > 1e-3, "错误值与真值应显著可分（_1 差 %.2e）", dbug);
  }

  // ================= 用例 2：非线性 loss + 反向转置（xᵀ·W） =================
  // L = Σ(y²)，y = xᵀ·W；梯度同样经转置视图汇总 —— 覆盖 GradAccumulator 路径。
  {
    const int B = 4, IN = 3, OUT = 2;
    Tensor x(ShapeTag{}, {B, IN}, DType::kFloat, DeviceType::kCPU);
    {
      float* d = x.data<float>();
      for (int i = 0; i < B * IN; i++) d[i] = 0.3f * float(i % 5) - 0.6f;
    }
    Tensor W(ShapeTag{}, {OUT, B}, DType::kFloat, DeviceType::kCPU);   // (2,4)
    {
      float* d = W.data<float>();
      for (int i = 0; i < OUT * B; i++) d[i] = 0.05f * float(i + 1);
    }
    W.requires_grad(true);
    // 参数与输入**两侧都经转置视图**：xᵀ (3,4) · Wᵀ (4,2) → (3,2)
    Tensor y = x.t().matmul(W.t());
    Tensor loss = y.matmul(y.t()).sum();   // 非线性（二次）loss
    AutoGrad::backward(loss.getRelatedNode(), false);
    const float* g = W.grad_ptr();
    CHECK(g != nullptr, "xᵀ·Wᵀ 的参数应收到梯度");

    std::vector<double> numeric;
    {
      float* w = W.data<float>();
      std::vector<float> base(OUT * B);
      for (int i = 0; i < OUT * B; i++) base[i] = w[i];
      const double h = 1e-2;   // float32 张量：h 需足够大以避开抵消误差（1e-3 实测偏差 1.7e-4）
      auto lossAt = [&]() {
        Tensor yy = x.t().matmul(W.t());
        Tensor s = yy.matmul(yy.t()).sum();
        return double(s.data_read<float>()[0]);
      };
      for (int i = 0; i < OUT * B; i++) {
        w[i] = base[i] + float(h);
        double lp = lossAt();
        w[i] = base[i] - float(h);
        double lm = lossAt();
        w[i] = base[i];
        numeric.push_back((lp - lm) / (2 * h));
      }
    }
    CHECK(gradMatches(g, numeric, 1e-4), "非线性用例解析 vs 中央差分不符，最大偏差 %.3e",
          maxAbsDiff(g, numeric));
    // 该用例的负向对照：全零梯度（旧实现错位后常见形态之一）必须判不通过
    std::vector<double> zeros(numeric.size(), 0.0);
    CHECK(!gradMatches(g, zeros, 1e-4), "负向断言失败：零梯度也通过了比较器");
  }

  if (g_fail == 0) std::printf("test_ctorch_transpose_grad PASS\n");
  else std::printf("test_ctorch_transpose_grad FAIL (%d)\n", g_fail);
  return g_fail ? 1 : 0;
}
