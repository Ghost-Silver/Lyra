# third_party 补丁记录（PATCHES.md）

对 vendored 依赖的最小必要修改，逐条记录以保证可追溯/可复现。

**补丁本体**：`third_party/patches/ctorch-lyra.patch`（CTorch 树）+
`third_party/patches/c3-lyra.patch`（嵌套 C3 树），
`bash third_party/patches/apply.sh` 一键幂等应用。

## 依赖获取（勿用 SSH / 递归 submodule）

外层 `third_party/CTorch` 与 CTorch 内嵌套 `c3` 的上游 `.gitmodules` 均为 SSH URL，
沙盒/无钥环境会失败。手动 HTTPS 获取并检出外层 gitlink 固定的 SHA：

```bash
git clone https://github.com/ShengFlow/CTorch.git third_party/CTorch
git -C third_party/CTorch checkout 439a458df50a69e7589e363cb3e776e39f77bc69
git clone https://github.com/ShengFlow/C3.git third_party/CTorch/c3
git -C third_party/CTorch/c3 checkout e96f6c577ae0ec7bd09b6c7dd9dd0adaad2d7a71
bash third_party/patches/apply.sh
```

（补丁 P8 把 CTorch 的 c3 submodule URL 改为 HTTPS；外层 `.gitmodules` 同样已改 HTTPS。）

## CTorch（ShengFlow/CTorch @ 439a458df50a69e7589e363cb3e776e39f77bc69，MIT）
## 嵌套 C3（ShengFlow/C3 @ e96f6c577ae0ec7bd09b6c7dd9dd0adaad2d7a71）

1. **`c3/src/C3/C3Engine.cpp`**：补 `#include "C3/GeneratedKernel.h"`。
   原因：`GeneratedKernel` 原仅经 `MLIRKernelGen.h`（`#ifdef CT_ENABLE_MLIR` 块内）间接引入；
   在 `-DCT_ENABLE_MLIR=OFF`（Lyra 的默认集成方式，免 LLVM 依赖）下缺失该声明导致编译失败。
   影响：仅新增一条 include，无逻辑改动。

2. **`CMakeLists.txt` + `c3/CMakeLists.txt`（两份平行源表）**：把 `MLIRKernelGen.cpp`、
   `JITCache.cpp` 从无条件源表移入 `if(CT_ENABLE_MLIR)` 块。
   原因：两文件无条件包含 MLIR/LLVM 头（`C3Dialect.h`、`llvm/IR/LLVMContext.h`），
   `CT_ENABLE_MLIR=OFF` 构建下必然失败（上游打包缺陷，两份源表都需同步）。
   影响：仅源表归类；`MLIRToLLVMIR.cpp` 自带 `#ifdef CT_ENABLE_MLIR` 守卫，保留原位。

3. **`include/ops/SiLU.h`**：补 `#include <cmath>`。
   原因：`std::exp` 依赖缺失（此前靠传递包含碰巧编过）；`SwiGLU.h` 经包含本头文件一并修复。

4. **`src/AutoGrad/Nodes/GradAccumulator.cpp`、`src/CtorchScheduler.cpp`、
   `src/kernels/CPU-SIMD/{CrossEntropy_SIMD_kernel,SIMDMath,SIMDWrapper,Softmax_SIMD_kernel}.cpp`**：
   补 `#include <immintrin.h>`。原因：x86 SIMD intrinsics 缺头文件（此前靠传递包含碰巧编过）。

5. **`CMakeLists.txt`**：`src/Distributed/MPSBackend.mm` 从无条件 `CT-Distributed` 源表移入
   `if(APPLE)` 块。原因：Objective-C++ 文件在 Linux/无 ObjC 前端的 GCC 下无法编译
   （v0.5.2 曾修过其它 .mm，漏了这处）。

6. **`src/kernels/MPS/MPS_kernel_stubs.cpp`**：`Softmax_MPS_kernel` 桩从
   `extern "C" void` 改为与 `kernels.h` 一致的 C++ `Tensor Softmax_MPS_kernel(const Tensor&, int)`。
   原因：签名不一致导致 Linux 链接缺符号（MPS 在 Linux 本就是桩）。

7. **⚠️ 梯度正确性修复（关键）**：
   - `src/AutoGrad/Nodes/TransposeNode.cpp`：`backward` 的转置梯度补 `.contiguous()` 物化。
   - `src/AutoGrad/Nodes/GradAccumulator.cpp`：入射梯度累加前统一 `.contiguous()` 物化。
   原因：`transposeNoGrad` 只是元数据视图（内存平铺序不变），下游按线性下标累加梯度时
   会把 (m,n) 布局梯度原样写进 (n,m) 叶子缓冲区——`y = x.matmul(W.t())` 这类**转置参数**
   的梯度整体错位且不报错（实测 dL/dW=[1 1 2 | 2 3 3]，正确值 [1 2 3 | 1 2 3]；
   修复后逐元素吻合有限差分）。凡经 `.t()/transpose()` 进图的可学习参数都会中招。
   回归测试：`tests/test_ctorch_transpose_grad.cpp`（正向：解析 vs 中央差分；负向：
   历史错误值 [1 1 2 2 3 3] 必须判不通过 → 保证测试有判别力）。运行：
   `WITH_LEARN=1 bash tests/run_tests.sh`；回退本补丁后该测试**必须 FAIL**（见 PR #1 回复实测）。

## 构建期兼容垫片（非依赖树内改动）

- `cmake/compat_bf16.h`：GCC 12 无 `__bf16` 内建类型（CTorch `CoreDefs.h` 需要），
  由顶层 CMakeLists 在 `GNU && GCC < 13` 时 `-include` 强制包含。占位类型带 float
  双向转换；Lyra 学习层只用 float32，bfloat16 数值语义不会被触发。

- CTorch CMake 开关（force cache）：`CT_ENABLE_MLIR=OFF`（免 LLVM）、
  `CT_ENABLE_LTO=OFF`（其 ThinLTO 选项 GCC 不识别）、`FETCH_GOOGLETEST=OFF`。
