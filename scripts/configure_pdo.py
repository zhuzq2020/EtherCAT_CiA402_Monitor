#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""同川 (Tongchuan) PDO 映射只读核验、备份与对齐脚本 (支持单从站或批量 --slaves)。"""

import argparse
import os
import sys
from typing import List, Tuple

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from sii_utils import write_bytes_exclusive
from soem_rpc import RpcError, SoemRpc

TONGCHUAN_VENDOR_ID = 0x00075500
EC_STATE_PRE_OP = 0x02

DESIRED_PDO = [
    [0x60400010, 0x607A0020, 0x60FF0020, 0x60710010, 0x60B10020, 0x60600008],
    [0x60410010, 0x60610008, 0x60640020, 0x606C0020, 0x60770010,
     0x60F40020, 0x603F0010, 0x60790020, 0x300F0010],
]
ASSIGNMENTS = [0x1C12, 0x1C13]
MAPS = [0x1601, 0x1A01]


def parse_slave_list(text: str) -> List[int]:
    if not text or text.endswith(","):
        raise ValueError("空的从站列表项")
    seen = set()
    result = []
    for token in text.split(","):
        if not token:
            raise ValueError("空的从站列表项")
        if "-" in token:
            parts = token.split("-")
            if len(parts) != 2:
                raise ValueError(f"无效范围: {token}")
            first, last = int(parts[0], 0), int(parts[1], 0)
            if first < 1 or last < first or last > 65535:
                raise ValueError(f"无效范围: {token}")
            for n in range(first, last + 1):
                if n in seen:
                    raise ValueError(f"重复从站: {n}")
                seen.add(n)
                result.append(n)
        else:
            n = int(token, 0)
            if n < 1 or n > 65535 or n in seen:
                raise ValueError(f"无效或重复从站: {token}")
            seen.add(n)
            result.append(n)
    return result


def inspect_slave_pdo(rpc: SoemRpc, slave: int) -> Tuple[bool, List[bool], str, str]:
    """Read and compare current PDO mapping against DESIRED_PDO. Returns (ok, matches, snapshot, detail)."""
    vendor = rpc.read_sdo_int(slave, 0x1018, 1)
    if vendor != TONGCHUAN_VENDOR_ID:
        return False, [False, False], "", f"仅支持同川 Vendor=0x75500；实际 Vendor=0x{vendor:x}"
    product = rpc.read_sdo_int(slave, 0x1018, 2)

    slaves = rpc.slaves()
    status = rpc.read_sdo_int(slave, 0x6041, 0)
    if slaves[slave - 1]["state"] != EC_STATE_PRE_OP or (status & 4):
        return False, [False, False], "", "要求 PREOP 且驱动未处于 Operation Enabled"

    lines = [f"slave {slave} vendor {vendor} product {product}"]
    matches = [True, True]

    for side in range(2):
        count = rpc.read_sdo_int(slave, ASSIGNMENTS[side], 0)
        matches[side] = (count == 1)
        lines.append(f"{ASSIGNMENTS[side]:x}:0 {count}")
        indices = []
        for n in range(1, count + 1):
            idx = rpc.read_sdo_int(slave, ASSIGNMENTS[side], n)
            indices.append(idx)
            lines.append(f"{ASSIGNMENTS[side]:x}:{n} {idx:x}")
            matches[side] = matches[side] and (idx == MAPS[side])
        if MAPS[side] not in indices:
            indices.append(MAPS[side])
        for idx in indices:
            entries = rpc.read_sdo_int(slave, idx, 0)
            lines.append(f"{idx:x}:0 {entries}")
            if idx == MAPS[side]:
                matches[side] = matches[side] and (entries == len(DESIRED_PDO[side]))
            for n in range(1, entries + 1):
                val = rpc.read_sdo_int(slave, idx, n)
                lines.append(f"{idx:x}:{n} {val:x}")
                if idx == MAPS[side]:
                    matches[side] = (
                        matches[side]
                        and n <= len(DESIRED_PDO[side])
                        and val == DESIRED_PDO[side][n - 1]
                    )

    snapshot = "\n".join(lines) + "\n"
    detail = f"RxPDO(17B): {'一致' if matches[0] else '需重写'}; TxPDO(25B): {'一致' if matches[1] else '需重写'}"
    return True, matches, snapshot, detail


def configure_slave_pdo(rpc: SoemRpc, slave: int, backup_path: str, save: bool) -> Tuple[bool, str]:
    ok, matches, snapshot, detail = inspect_slave_pdo(rpc, slave)
    if not ok:
        return False, detail
    print(snapshot, end="")
    if matches[0] and matches[1] and not save:
        return True, detail

    write_bytes_exclusive(backup_path, snapshot.encode("utf-8"))

    for side in range(2):
        if matches[side]:
            continue
        assign_idx = ASSIGNMENTS[side]
        map_idx = MAPS[side]

        def assign(enable: bool) -> None:
            payload = bytes([
                1 if enable else 0,
                0,
                map_idx & 0xFF,
                (map_idx >> 8) & 0xFF,
            ])
            rpc.write_sdo(slave, assign_idx, 0, payload if enable else payload[:2], ca=True)
            cnt = rpc.read_sdo_int(slave, assign_idx, 0)
            if cnt != (1 if enable else 0):
                raise RpcError(f"CA 分配数量回读不一致: 0x{assign_idx:04x}:00 expected={int(enable)} actual={cnt}")
            if enable:
                idx = rpc.read_sdo_int(slave, assign_idx, 1)
                if idx != map_idx:
                    raise RpcError(f"CA 分配索引回读不一致: expected=0x{map_idx:04x} actual=0x{idx:04x}")

        assign(False)
        rpc.write_sdo(slave, map_idx, 0, bytes([0]))
        if rpc.read_sdo_int(slave, map_idx, 0) != 0:
            return False, f"0x{map_idx:04x}:00 清零回读失败"

        for n, entry in enumerate(DESIRED_PDO[side], start=1):
            rpc.write_sdo(slave, map_idx, n, entry.to_bytes(4, "little"))

        rpc.write_sdo(slave, map_idx, 0, bytes([len(DESIRED_PDO[side])]))
        if rpc.read_sdo_int(slave, map_idx, 0) != len(DESIRED_PDO[side]):
            return False, f"0x{map_idx:04x}:00 激活数量回读失败"

        for n, expected in enumerate(DESIRED_PDO[side], start=1):
            actual = rpc.read_sdo_int(slave, map_idx, n)
            if actual != expected:
                return False, f"激活后映射回读不一致 0x{map_idx:04x}:{n} expected=0x{expected:x} actual=0x{actual:x}"

        assign(True)

    ok2, matches2, _, detail2 = inspect_slave_pdo(rpc, slave)
    if not ok2 or not (matches2[0] and matches2[1]):
        return False, f"最终 PDO 回读不一致 ({detail2})；原映射备份: {backup_path}"

    if save:
        slaves = rpc.slaves()
        sw_now = rpc.read_sdo_int(slave, 0x6041, 0)
        if slaves[slave - 1]["state"] != EC_STATE_PRE_OP or (sw_now & 4):
            return False, "映射已校验，但保存前状态检查失败，未发送保存命令"
        rpc.write_sdo(slave, 0x313D, 0, (33).to_bytes(2, "little"), timeout_us=5000000)
        import time
        time.sleep(0.5)
        ok3, matches3, _, detail3 = inspect_slave_pdo(rpc, slave)
        if not ok3 or not (matches3[0] and matches3[1]):
            return False, f"保存命令已应答，但保存后映射检查失败: {detail3}"
        return True, "PDO SAVE ACK: Rx=17B Tx=25B；0x313D:00=33 已应答，保存后映射一致"

    return True, "PDO WRITE PASS: Rx=17B Tx=25B；未保存 Flash"


def main() -> int:
    parser = argparse.ArgumentParser(description="同川 PDO 映射只读比较与批量写入脚本")
    parser.add_argument("-i", "--iface", default="eno0", help="EtherCAT 网卡名称 (默认: eno0)")
    parser.add_argument("--slave", type=int, default=0, help="单个物理从站序号 (1..N)")
    parser.add_argument("--slaves", default="", help="批量物理从站列表或范围，例如 2-8,10-16")
    parser.add_argument("--confirm", action="store_true", help="确认执行写入 (默认仅只读比较)")
    parser.add_argument("--backup", default="", help="单从站写入前的备份文件路径 (不得已存在)")
    parser.add_argument("--backup-dir", default="", help="批量写入前的备份目录 (不得已存在)")
    parser.add_argument("--save", action="store_true", help="映射核验后写 0x313D:00=33 保存")
    args = parser.parse_args()

    if args.save and not args.confirm:
        print("--save 必须同时指定 --confirm 和备份路径", file=sys.stderr)
        return 1

    if args.slaves:
        if args.slave > 0 or args.backup or (args.confirm and not args.backup_dir):
            print("批量模式请使用 --slaves <list> [--confirm --backup-dir DIR]，不要混用 --slave/--backup", file=sys.stderr)
            return 1
        try:
            targets = parse_slave_list(args.slaves)
        except ValueError as exc:
            print(f"[错误] --slaves 无效: {exc}", file=sys.stderr)
            return 1
    else:
        if args.slave <= 0 or args.backup_dir or (args.confirm and not args.backup):
            print("单从站模式必须指定 --slave N；写入需同时指定 --confirm --backup FILE", file=sys.stderr)
            return 1
        targets = [args.slave]

    try:
        with SoemRpc(args.iface) as rpc:
            for s in targets:
                ok, _, snapshot, detail = inspect_slave_pdo(rpc, s)
                if not ok:
                    print(f"[FAIL] slave {s}: {detail}", file=sys.stderr)
                    return 1
                if not args.confirm:
                    print(snapshot, end="")
                print(f"[CHECK] slave {s}: {detail}")

            if not args.confirm:
                return 0

            if args.backup_dir:
                try:
                    os.mkdir(args.backup_dir)
                except OSError as exc:
                    print(f"[失败] 无法新建批次备份目录 (父目录须存在且不能复用): {args.backup_dir}: {exc}", file=sys.stderr)
                    return 1

            completed = 0
            for s in targets:
                bpath = (
                    os.path.join(args.backup_dir, f"slave{s}-pdo-before.txt")
                    if args.backup_dir
                    else args.backup
                )
                ok, detail = configure_slave_pdo(rpc, s, bpath, args.save)
                print(f"{'[PASS]' if ok else '[FAIL]'} slave {s}: {detail}")
                if not ok:
                    print(
                        f"批次停止，已完成 {completed}/{len(targets)}；"
                        "此前成功的电机不自动回滚，后续电机未写入。",
                        file=sys.stderr,
                    )
                    return 1
                completed += 1

            if args.save:
                print("保存命令均已应答；断电保留仍需安全重新上电后只读验证。")
            print(f"PDO batch PASS: {completed} slaves")
            return 0
    except (RpcError, OSError) as exc:
        print(f"[失败] {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
