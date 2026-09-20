#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
BUILD_DIR="$ROOT_DIR/build"

echo "=== 正在编译机械臂维护工具 (arm_maint) ==="
mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"

cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo ..
cmake --build . -j"$(nproc)"

echo ""
echo "=== 编译成功! 产物位于: $BUILD_DIR/arm_maint ==="
echo "如需免 sudo 执行，请运行: ./scripts/grant_cap.sh"
