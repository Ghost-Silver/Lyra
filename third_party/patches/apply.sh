#!/usr/bin/env bash
# third_party/patches/apply.sh — 一键应用 Lyra 对 vendored CTorch / C3 的集成补丁。
# 用法（在拉好依赖后执行一次）：
#   git submodule update --init third_party/CTorch   # 或手动 HTTPS clone（见 ../PATCHES.md）
#   bash third_party/patches/apply.sh
# 幂等：已应用则跳过。补丁清单与原因见 ../PATCHES.md。
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
CT="$HERE/../CTorch"
[ -d "$CT/src" ] || { echo "错误：$CT 不存在，请先取得 CTorch 源码（见 ../PATCHES.md）"; exit 1; }

if git -C "$CT" apply --reverse --check "$HERE/ctorch-lyra.patch" >/dev/null 2>&1; then
  echo "[skip] ctorch-lyra.patch 已应用"
else
  git -C "$CT" apply "$HERE/ctorch-lyra.patch"
  echo "[ok]   ctorch-lyra.patch"
fi

if [ -d "$CT/c3" ]; then
  if git -C "$CT/c3" apply --reverse --check "$HERE/c3-lyra.patch" >/dev/null 2>&1; then
    echo "[skip] c3-lyra.patch 已应用"
  else
    git -C "$CT/c3" apply "$HERE/c3-lyra.patch"
    echo "[ok]   c3-lyra.patch"
  fi
fi
echo "完成。"
