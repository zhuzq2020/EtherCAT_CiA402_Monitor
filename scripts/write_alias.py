#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""安全备份后写入从站 Station Alias (支持 同川 / 意优 / 标准 CiA-402)。"""

import argparse
import os
import sys

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from sii_utils import (
    ECT_REG_ALIAS,
    ECT_REG_ALSTAT,
    ECT_SII_ID,
    ECT_SII_MANUF,
    MAX_SII_BYTES,
    SII_ALIAS_WORD,
    SII_HEADER_WORDS,
    SII_SIZE_WORD,
    sii_crc,
    verify_alias_image,
    words_to_bytes,
    words_u32,
    write_bytes_exclusive,
)
from soem_rpc import RpcError, SoemRpc

TONGCHUAN_VENDOR_ID = 0x00075500
EYOU_VENDOR_ID = 0x00001097
SDO_TC_ECAT_ALARM = 0x302D
SDO_EYOU_EEPROM_UNLOCK = 0x2679
SDO_STATUS_WORD = 0x6041
EC_STATE_INIT = 0x01
EC_STATE_PRE_OP = 0x02


def parse_positive_int(text: str, maximum: int = 65535) -> int:
    if text.startswith(("+", "-")):
        raise argparse.ArgumentTypeError(f"不允许符号前缀: {text}")
    val = int(text, 0)
    if val < 1 or val > maximum:
        raise argparse.ArgumentTypeError(f"必须在 1..{maximum} 范围内: {text}")
    return val


def al_state_name(state: int) -> str:
    return {1: "INIT", 2: "PRE-OP", 3: "BOOT", 4: "SAFE-OP", 8: "OP"}.get(state & 0x0F, f"0x{state:x}")


def main() -> int:
    parser = argparse.ArgumentParser(description="备份并安全写入从站 SII EEPROM Station Alias")
    parser.add_argument("-i", "--iface", default="eno0", help="EtherCAT 网卡名称 (默认: eno0)")
    parser.add_argument("--slave", type=parse_positive_int, required=True, help="物理扫描从站序号 (1..N)")
    parser.add_argument("--alias", type=parse_positive_int, required=True, help="目标 Station Alias (1..65535)")
    parser.add_argument("--backup", required=True, help="写前备份路径 (不得已存在)")
    parser.add_argument("--confirm", default="", help="精确确认短语: WRITE-EEPROM-ALIAS")
    args = parser.parse_args()

    if args.confirm != "WRITE-EEPROM-ALIAS":
        print(
            "\n[安全拦截] 写 Alias 会永久修改电机 SII EEPROM。\n"
            "           请核对物理拓扑并追加: --confirm WRITE-EEPROM-ALIAS\n",
            file=sys.stderr,
        )
        return 1

    after_path = args.backup + ".after.bin"
    if os.path.exists(args.backup) or os.path.exists(after_path):
        print(f"[失败] 备份文件或写后镜像已存在，拒绝覆盖: {args.backup}", file=sys.stderr)
        return 1

    try:
        with SoemRpc(args.iface) as rpc:
            slaves = rpc.slaves()
            if args.slave > len(slaves):
                print(f"[失败] --slave {args.slave} 超出本次扫描范围 (1..{len(slaves)})", file=sys.stderr)
                return 1
            s_info = slaves[args.slave - 1]
            vendor = s_info["vendor"]
            product = s_info["product"]
            if vendor == 0 or product == 0:
                print("[失败] 在线从站身份无效，拒绝写入", file=sys.stderr)
                return 1

            al_stat = int.from_bytes(rpc.read_esc(args.slave, ECT_REG_ALSTAT, 2), "little")
            esc_alias = int.from_bytes(rpc.read_esc(args.slave, ECT_REG_ALIAS, 2), "little")
            if (al_stat & 0x10) or ((al_stat & 0x0F) not in (EC_STATE_INIT, EC_STATE_PRE_OP)):
                print(f"[失败] 写 Alias 要求无 AL 错误且处于 INIT/PRE-OP，实际={al_state_name(al_stat)}", file=sys.stderr)
                return 1

            if s_info["mbx_proto"] & 0x04:
                sw = rpc.read_sdo_int(args.slave, SDO_STATUS_WORD, 0)
                if sw & 0x0004:
                    print("[失败] 驱动器处于 Operation Enabled 状态，禁止写 EEPROM", file=sys.stderr)
                    return 1

            print(
                f"[EEPROM] slave={args.slave}, vendor=0x{vendor:x}, product=0x{product:x}, "
                f"AL={al_state_name(al_stat)}, ESC Alias={esc_alias}"
            )

            if vendor == TONGCHUAN_VENDOR_ID:
                alarm = rpc.read_sdo_int(args.slave, SDO_TC_ECAT_ALARM, 0)
                print(f"[EEPROM] 同川 Pr0.45 ECAT 报警码={alarm}；走标准 SII 流程")
                if alarm != 0:
                    print("[失败] 同川 ECAT 报警未清除，未执行 EEPROM 写入", file=sys.stderr)
                    return 1

            eyou_unlocked = False
            eeprom_acquired = False
            try:
                if vendor == EYOU_VENDOR_ID:
                    if not (s_info["mbx_proto"] & 0x04):
                        print("[失败] 意优从站不支持 CoE，无法开启 EEPROM 写权限", file=sys.stderr)
                        return 1
                    eyou_unlocked = True
                    rpc.write_sdo(args.slave, SDO_EYOU_EEPROM_UNLOCK, 0, bytes([1]))

                rpc.eeprom_acquire(args.slave)
                eeprom_acquired = True

                words = rpc.eeprom_read_words(args.slave, 0, SII_HEADER_WORDS)
                if words_u32(words, ECT_SII_MANUF) != vendor or words_u32(words, ECT_SII_ID) != product:
                    print("[失败] SII EEPROM 与在线从站身份不一致", file=sys.stderr)
                    return 1
                if sii_crc(words) != (words[7] & 0xFF):
                    print("[失败] 原始 SII 配置 CRC 不匹配，拒绝写入", file=sys.stderr)
                    return 1
                if words[SII_ALIAS_WORD] == args.alias:
                    print(
                        f"[成功] EEPROM Alias 已为目标值 ({args.alias}) 且 CRC 正确，未执行写入；"
                        f"当前 ESC Alias={esc_alias}"
                    )
                    return 0

                print(f"[EEPROM] Alias 修改: EEPROM={words[SII_ALIAS_WORD]} -> {args.alias}，ESC Alias={esc_alias}")
                total_bytes = (words[SII_SIZE_WORD] + 1) * 128
                if total_bytes < SII_HEADER_WORDS * 2 or total_bytes > MAX_SII_BYTES:
                    print("[失败] SII EEPROM 声明容量无效，拒绝写入", file=sys.stderr)
                    return 1
                total_words = total_bytes // 2
                pos = SII_HEADER_WORDS
                while pos < total_words:
                    chunk = min(256, total_words - pos)
                    words.extend(rpc.eeprom_read_words(args.slave, pos, chunk))
                    pos += chunk

                write_bytes_exclusive(args.backup, words_to_bytes(words))
                print(f"[EEPROM] 完整写前备份已保存: {args.backup} ({total_bytes} bytes)")

                # Re-probe configuration words 0..7 before first write
                if rpc.eeprom_read_words(args.slave, 0, 8) != words[:8]:
                    print("[失败] 备份后 SII 配置区发生变化，未执行写入", file=sys.stderr)
                    return 1

                before = list(words)
                header = list(words[:8])
                header[SII_ALIAS_WORD] = args.alias
                header[7] = (header[7] & 0xFF00) | sii_crc(header)

                print(f"[WARN] 即将为物理从站 {args.slave} 写入 EEPROM Alias {args.alias}，写入期间请勿断电。")
                rpc.eeprom_write_header(args.slave, header)

                after: list = []
                pos = 0
                while pos < total_words:
                    chunk = min(256, total_words - pos)
                    after.extend(rpc.eeprom_read_words(args.slave, pos, chunk))
                    pos += chunk

                write_bytes_exclusive(after_path, words_to_bytes(after))
                print(f"[EEPROM] 完整写后镜像已保存: {after_path} ({total_bytes} bytes)")

                ok, verify_err = verify_alias_image(before, after, args.alias)
                if not ok:
                    print(f"[失败] {verify_err}；原备份={args.backup}，写后镜像={after_path}", file=sys.stderr)
                    return 1

                print(
                    f"[成功] EEPROM Alias 和 CRC 写入成功，完整镜像校验通过！\n"
                    f"       备份={args.backup}；写后镜像={after_path}。\n"
                    f"       请安全重新上电后运行 scripts/read_alias.py 和 arm_maint status 验证 ESC Alias。"
                )
                return 0
            finally:
                if eeprom_acquired:
                    try:
                        rpc.eeprom_restore(args.slave)
                    except Exception as exc:
                        print(f"[错误] 恢复 EEPROM 控制权失败: {exc}", file=sys.stderr)
                if eyou_unlocked:
                    try:
                        rpc.write_sdo(args.slave, SDO_EYOU_EEPROM_UNLOCK, 0, bytes([0]))
                    except Exception as exc:
                        print(f"[错误] 关闭意优 EEPROM 写权限失败: {exc}", file=sys.stderr)
    except (RpcError, OSError) as exc:
        print(f"[失败] {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
