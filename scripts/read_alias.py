#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""读取从站 ESC Station Alias、EEPROM Alias 与配置 CRC。"""

import argparse
import os
import sys

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from sii_utils import ECT_REG_ALIAS, SII_ALIAS_WORD, sii_crc
from soem_rpc import RpcError, SoemRpc


def parse_positive_int(text: str, maximum: int = 65535) -> int:
    val = int(text, 0)
    if val < 1 or val > maximum:
        raise argparse.ArgumentTypeError(f"必须在 1..{maximum} 范围内: {text}")
    return val


def main() -> int:
    parser = argparse.ArgumentParser(description="读取从站 ESC / EEPROM Alias 与 CRC")
    parser.add_argument("-i", "--iface", default="eno0", help="EtherCAT 网卡名称 (默认: eno0)")
    parser.add_argument("--slave", type=parse_positive_int, required=True, help="物理从站序号 (1..N)")
    args = parser.parse_args()

    try:
        with SoemRpc(args.iface) as rpc:
            slaves = rpc.slaves()
            if args.slave > len(slaves):
                print(f"[失败] --slave {args.slave} 超出总线从站数量 ({len(slaves)})", file=sys.stderr)
                return 1

            esc_alias = int.from_bytes(rpc.read_esc(args.slave, ECT_REG_ALIAS, 2), "little")
            rpc.eeprom_acquire(args.slave)
            try:
                words = rpc.eeprom_read_words(args.slave, 0, 8)
            finally:
                rpc.eeprom_restore(args.slave)

            stored_crc = words[7] & 0xFF
            calc_crc = sii_crc(words)
            crc_match = stored_crc == calc_crc
            print(
                f"[{'成功' if crc_match else '失败'}] ESC Alias={esc_alias}; "
                f"EEPROM Alias={words[SII_ALIAS_WORD]}; "
                f"CRC stored={stored_crc}, calculated={calc_crc} "
                f"({'OK' if crc_match else '不匹配'})"
            )
            return 0 if crc_match else 1
    except RpcError as exc:
        print(f"[失败] {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
