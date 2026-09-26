#!/usr/bin/env bash
# tests/run_tests.sh — 直接 g++ 构建并运行测试。
#
# 学习层（CTorch）三条规则（A2：不再「绿色但没测」）：
#   1) 未构建且未请求         → 如实报告 Learn: SKIPPED(not built)，退出码 0，
#                              且汇总行**明确区分** Layer 0 结论与学习层结论；
#   2) 已配置/已请求但缺二进制 → **FAIL**（退出码 1）——「绿色但没测」的经典陷阱；
#   3) WITH_LEARN=1           → 先构建（configure + build），任何构建失败即 FAIL。
#
# Runtime 层（可选）：ws_probe.py —— C2 资源上限三场景实测（第 17 连接 503 / 半开回收 /
#   超长请求头丢弃）+ 静态文件 413 / 缓存命中 / symlink 403。需 arm_sim 与 python3。
#
# 用法：
#   bash tests/run_tests.sh              # Layer 0 全跑 + 学习层（若已构建）
#   WITH_LEARN=1 bash tests/run_tests.sh # 强制构建并运行学习层（CI 推荐）
set -uo pipefail
cd "$(dirname "$0")/.."
mkdir -p build-tests
CXX=${CXX:-g++}
FLAGS="-std=c++17 -O2 -Wall -Wextra -Ilib -I."
WITH_LEARN=${WITH_LEARN:-0}

layer0_tests=(test_kinematics test_planning test_sim test_json_ws test_serial test_safety)
learn_tests=(test_learn test_ctorch_transpose_grad test_train_regression)

l0_pass=0
l0_fail=0
for t in "${layer0_tests[@]}"; do
  echo "== build $t =="
  if ! $CXX $FLAGS "tests/$t.cpp" -o "build-tests/$t"; then
    echo "BUILD FAIL: $t"
    l0_fail=$((l0_fail + 1))
    continue
  fi
  echo "== run $t =="
  if "./build-tests/$t"; then l0_pass=$((l0_pass + 1)); else l0_fail=$((l0_fail + 1)); fi
done

# ---------- 学习层 ----------
learn_dir=""
for c in build-learn build; do
  if [ -f "$c/CMakeCache.txt" ] && grep -q "ARM_ENABLE_CTORCH:BOOL=ON" "$c/CMakeCache.txt"; then
    learn_dir="$c"
    break
  fi
done

# WITH_LEARN=1：构建（若尚未构建则先 configure）；构建失败 → FAIL
if [ "$WITH_LEARN" = "1" ]; then
  if ! command -v cmake >/dev/null 2>&1; then
    echo "FAIL: WITH_LEARN=1 但找不到 cmake"
    l0_fail=$((l0_fail + 1))
    learn_dir="build-learn"
  else
    echo "== configure build-learn (ARM_ENABLE_CTORCH=ON) =="
    if ! cmake -B build-learn -DARM_ENABLE_CTORCH=ON >/tmp/lyra_cmake_cfg.log 2>&1; then
      echo "FAIL: cmake configure 失败（详见 /tmp/lyra_cmake_cfg.log）"
      l0_fail=$((l0_fail + 1))
    else
      echo "== build build-learn =="
      if ! cmake --build build-learn -j >/tmp/lyra_cmake_build.log 2>&1; then
        echo "FAIL: 学习层构建失败（详见 /tmp/lyra_cmake_build.log）"
        l0_fail=$((l0_fail + 1))
      fi
    fi
    learn_dir="build-learn"
  fi
fi

learn_pass=0
learn_fail=0
learn_skipped=0
found_any=0
for t in "${learn_tests[@]}"; do
  bin=""
  for d in "$learn_dir" build-learn build; do
    [ -n "$d" ] && [ -x "$d/tests/$t" ] && bin="$d/tests/$t" && break
  done
  if [ -z "$bin" ]; then
    # 预期跳过（既没构建也没请求）vs 意外缺失（已配置/已请求）
    if [ "$WITH_LEARN" = "1" ] || [ -n "$learn_dir" ]; then
      echo "FAIL: 学习层测试缺失 $t（已配置/已请求，但无二进制）"
      learn_fail=$((learn_fail + 1))
    else
      learn_skipped=$((learn_skipped + 1))
    fi
    continue
  fi
  found_any=$((found_any + 1))
  echo "== run $t =="
  if "$bin"; then learn_pass=$((learn_pass + 1)); else learn_fail=$((learn_fail + 1)); fi
done

# ---------- Runtime 层：端口/资源探针（C2，可选）----------
rt_status="SKIPPED(arm_sim not built)"
rt_fail=0
armsim=""
for d in build-learn build; do
  [ -x "$d/arm_sim" ] && armsim="$d/arm_sim" && break
done
if [ -n "$armsim" ] && command -v python3 >/dev/null 2>&1; then
  echo "== run ws_probe (C2 资源上限三场景) =="
  if python3 tests/ws_probe.py --spawn "$armsim" --port $((18000 + $$ % 2000)) --web web; then
    rt_status="PASS"
  else
    rt_status="FAIL"
    rt_fail=1
  fi
fi

# ---------- 汇总（如实反映实际执行数）----------
echo "----------------------------------------------------------------"
echo "Layer0: $l0_pass passed, $l0_fail failed (of ${#layer0_tests[@]})"
if [ "$found_any" -gt 0 ]; then
  echo "Learn : $learn_pass passed, $learn_fail failed (of ${#learn_tests[@]})"
elif [ "$learn_fail" -gt 0 ]; then
  echo "Learn : FAIL (已配置/已请求但缺少二进制)"
elif [ "$learn_skipped" -eq "${#learn_tests[@]}" ]; then
  echo "Learn : SKIPPED(not built)（需 cmake -B build-learn -DARM_ENABLE_CTORCH=ON 构建）"
else
  echo "Learn : SKIPPED($learn_skipped not built)"
fi

echo "Runtime: $rt_status"
if [ "$l0_fail" -eq 0 ] && [ "$learn_fail" -eq 0 ] && [ "$rt_fail" -eq 0 ]; then
  if [ "$found_any" -eq 0 ] && [ "$learn_skipped" -eq "${#learn_tests[@]}" ]; then
    echo "RESULT: PASS (Layer 0 only; 学习层/Runtime 跳过项见上，不并入 Layer 0 结论)"
  else
    echo "RESULT: PASS"
  fi
  exit 0
else
  echo "RESULT: FAIL"
  exit 1
fi
