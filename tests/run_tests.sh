#!/usr/bin/env bash
# tests/run_tests.sh — 无 CMake 直接 g++ 构建并运行全部 Layer 0 测试；
# 若已用 ARM_ENABLE_CTORCH=ON 构建过（build-learn），追加跑 test_learn（学习层）。
set -euo pipefail
cd "$(dirname "$0")/.."
mkdir -p build-tests
CXX=${CXX:-g++}
FLAGS="-std=c++17 -O2 -Wall -Wextra -Ilib -I."
fail=0
for t in test_kinematics test_planning test_sim test_json_ws test_serial; do
  echo "== build $t =="
  $CXX $FLAGS "tests/$t.cpp" -o "build-tests/$t"
  echo "== run $t =="
  "./build-tests/$t" || fail=1
done

# 学习层测试（需 CTorch）：优先用 CMake 产物
if [ -x "build-learn/tests/test_learn" ]; then
  echo "== run test_learn (ARM_ENABLE_CTORCH) =="
  "./build-learn/tests/test_learn" || fail=1
elif [ -x "build/tests/test_learn" ]; then
  echo "== run test_learn (ARM_ENABLE_CTORCH) =="
  "./build/tests/test_learn" || fail=1
else
  echo "== skip test_learn（需 cmake -B build-learn -DARM_ENABLE_CTORCH=ON 构建） =="
fi

if [ "$fail" = 0 ]; then echo "ALL TESTS PASS"; else echo "TESTS FAILED"; exit 1; fi
