// cmake/compat_bf16.h — CTorch 编译兼容垫片（GCC 12 无 __bf16 内建类型）
// CTorch (CoreDefs.h) 使用 `__bf16`（GCC 13+ / Clang 内建）。本垫片在更老的
// GCC 上提供同名占位类型：仅保证编译期类型完备（含与 float 的双向转换）；
// bfloat16 数值路径不会被 Lyra 学习层（float32）触发，语义按 float 简化。
// 由顶层 CMakeLists 在 GCC < 13 时经 -include 强制包含，勿在业务代码里直接引用。
#pragma once
#if defined(__GNUC__) && !defined(__clang__) && (__GNUC__ < 13)
#ifndef CT_COMPAT_BF16_DEFINED
#define CT_COMPAT_BF16_DEFINED
struct __bf16 {
  float v = 0.0f;
  __bf16() = default;
  __bf16(float f) : v(f) {}
  __bf16(double f) : v(float(f)) {}
  __bf16(int i) : v(float(i)) {}
  operator float() const { return v; }
};
#endif
#endif
