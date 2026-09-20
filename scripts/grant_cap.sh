#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
TARGET_BIN="${1:-$ROOT_DIR/build/arm_maint}"

if [[ ! -f "$TARGET_BIN" ]]; then
    echo "[错误] 未找到二进制文件: $TARGET_BIN，请先执行 ./scripts/build.sh 编译！" >&2
    exit 1
fi

echo "正在为 $TARGET_BIN 赋予原始套接字网络能力 (CAP_NET_RAW, CAP_NET_ADMIN)..."
sudo setcap cap_net_raw,cap_net_admin=eip "$TARGET_BIN"
echo "[成功] 授权完成！现场维护人员现在无需加 sudo，可直接执行: $TARGET_BIN status"
