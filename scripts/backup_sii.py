#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""只读备份从站完整 SII EEPROM 镜像 (不覆盖已有文件)。"""

import argparse
import os
import sys

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from sii_utils import (
    ECT_SII_ID,
    ECT_SII_MANUF,
    MAX_SII_BYTES,
    SII_HEADER_WORDS,
    SII_SIZE_WORD,
    sii_crc,
    words_to_bytes,
    words_u32,
    write_bytes_exclusive,
)
from soem_rpc import RpcError, SoemRpc


def parse_positive_int(text: str, maximum: int = 65535) -> int:
    val = int(text, 0)
    if val < 1 or val > maximum:
        raise argparse.ArgumentTypeError(f"必须在 1..{maximum} 范围内: {text}")
    return val


def main() -> int:
    parser = argparse.ArgumentParser(description="只读备份从站完整 SII EEPROM 镜像")
    parser.add_argument("-i", "--iface", default="eno0", help="EtherCAT 网卡名称 (默认: eno0)")
    parser.add_argument("--slave", type=parse_positive_int, required=True, help="物理从站序号 (1..N)")
    parser.add_argument("--backup", required=True, help="输出备份文件路径 (不得已存在)")
    args = parser.parse_args()

    if os.path.exists(args.backup):
        print(f"[失败] 备份文件已存在 (拒绝覆盖): {args.backup}", file=sys.stderr)
        return 1

    try:
        with SoemRpc(args.iface) as rpc:
            slaves = rpc.slaves()
            if args.slave > len(slaves):
                print(f"[失败] --slave {args.slave} 超出总线从站数量 ({len(slaves)})", file=sys.stderr)
                return 1
            slave_info = slaves[args.slave - 1]

            rpc.eeprom_acquire(args.slave)
            try:
                words = rpc.eeprom_read_words(args.slave, 0, SII_HEADER_WORDS)
                if words_u32(words, ECT_SII_MANUF) != slave_info["vendor"] or \
                   words_u32(words, ECT_SII_ID) != slave_info["product"]:
                    print("[失败] SII 身份与在线扫描结果不一致，未保存备份", file=sys.stderr)
                    return 1

                total_bytes = (words[SII_SIZE_WORD] + 1) * 128
                if total_bytes < SII_HEADER_WORDS * 2 or total_bytes > MAX_SII_BYTES:
                    print(f"[失败] SII 声明容量无效 ({total_bytes} bytes)，未保存备份", file=sys.stderr)
                    return 1

                total_words = total_bytes // 2
                if total_words > SII_HEADER_WORDS:
                    # 分块读取避免单次 RPC 行过长
                    pos = SII_HEADER_WORDS
                    while pos < total_words:
                        chunk = min(256, total_words - pos)
                        words.extend(rpc.eeprom_read_words(args.slave, pos, chunk))
                        pos += chunk

                write_bytes_exclusive(args.backup, words_to_bytes(words))
                crc_ok = sii_crc(words) == (words[7] & 0xFF)
                print(
                    f"[成功] EEPROM 只读备份完成，{total_bytes} bytes；文件={args.backup}; "
                    f"配置 CRC={'OK' if crc_ok else '不匹配'}"
                )
                return 0
            finally:
                rpc.eeprom_restore(args.slave)
    except (RpcError, OSError) as exc:
        print(f"[失败] {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
