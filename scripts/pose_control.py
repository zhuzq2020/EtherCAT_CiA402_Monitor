#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
========================================================================================
       通用机械臂多姿态平滑同步控制脚本 (支持 法奥 Fairino / 同川 Tongchuan / 意优 Eyou)
========================================================================================

【使用方法】:
  1. 直接修改下方 POSES 数组中的各关节绝对角度 (单位: 度 °)，或使用 --config 指定 JSON 文件。
  2. 通过底层 `arm_maint rpc` 实时 PDO 会话驱动，全程保持使能、无启停间隙，支持法奥/同川/意优。
  3. 运行示例:
     - 单次运行一组姿态 (自动识别总线伺服):
         python3 scripts/pose_control.py -i eno0 --mode once --speed 20.0 --interval 2.0
     - 指定部分轴运行 (例如双臂总线中仅控制右臂 S2..S8 或 D18..D24):
         python3 scripts/pose_control.py -i eno0 --axes S2,S3,S4,S5,S6,S7,S8 --mode once
     - 重复循环运行 (无限循环，Ctrl+C 安全失能退出):
         python3 scripts/pose_control.py -i eno0 --mode repeat --speed 20.0 --interval 2.0
     - 仅预览规划 (不连接硬件):
         python3 scripts/pose_control.py --dry-run
========================================================================================
"""

import argparse
import json
import os
import subprocess
import sys

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(SCRIPT_DIR)
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from soem_rpc import DEFAULT_BIN, RpcError, SoemRpc

# ==============================================================================
#                     【用户姿态配置区域 / User Configuration】
# ==============================================================================
POSES = [
    #   A1      A2      A3      A4      A5      A6      A7
    [  0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0  ],  # 姿态 1: 零位/基准姿态
    [ 15.0,   40.0,  -15.0,   30.0,    0.0,   20.0,   10.0  ],  # 姿态 2: 工作就绪姿态
    [ 30.0,  -10.0,   15.0,   45.0,  -20.0,   10.0,   25.0  ],  # 姿态 3: 抓取预备姿态
    [  0.0,   30.0,  -25.0,   60.0,   15.0,  -15.0,    0.0  ],  # 姿态 4: 避障过渡姿态
    [-20.0,   70.0,   20.0,   50.0,    0.0,    0.0,  -20.0  ],  # 姿态 5: 放置终点姿态
]

SPEED = 20.0       # 默认关节规划速度 (°/s)
INTERVAL = 2.0     # 各姿态到达后停留保持时间 (秒)
MODE = "once"      # "once" | "repeat"
REPEAT_COUNT = 0   # repeat 模式下的轮次 (0 = 无限循环)
IFACE = "eno0"     # EtherCAT 网卡名称
# ==============================================================================

BUILD_SH = os.path.join(SCRIPT_DIR, "build.sh")


def validate_poses(poses):
    if not poses:
        print("[错误] 姿态列表为空。", file=sys.stderr)
        sys.exit(1)
    dof = len(poses[0])
    if dof == 0:
        print("[错误] 姿态关节维度不能为 0。", file=sys.stderr)
        sys.exit(1)
    for idx, p in enumerate(poses):
        if len(p) != dof:
            print(f"[错误] 姿态 {idx + 1} 关节数不一致: 期望 {dof} 轴，实测 {len(p)} 轴: {p}", file=sys.stderr)
            sys.exit(1)
        for val in p:
            if not isinstance(val, (int, float)):
                print(f"[错误] 姿态 {idx + 1} 包含非数值项: {val}", file=sys.stderr)
                sys.exit(1)
    return dof


def print_preview(poses, dof, speed, interval, mode, repeat, arm_type, axes_arg):
    print("\n" + "=" * 92)
    print(f"          通用机械臂 {dof} 轴多姿态同步轨迹控制 (2ms 实时 PDO 驱动 | 类型: {arm_type})")
    print("=" * 92)
    mode_desc = "【单次运行一组姿态】" if mode == "once" else (
        "【重复循环运行】(无限循环，Ctrl+C 退出)" if repeat <= 0 else f"【重复循环运行】(共 {repeat} 轮)"
    )
    print(f"  受控轴系 : {axes_arg if axes_arg else '自动识别总线上全部受支持伺服轴 (法奥/同川/意优)'}")
    print(f"  运行模式 : {mode_desc}")
    print(f"  规划速度 : {speed:.1f} °/s (五次多项式平滑加减速)")
    print(f"  姿态间隔 : {interval:.2f} s ({int(interval * 1000)} ms)")
    print("-" * 92)

    headers = " ".join(f"{'A' + str(j + 1) + '(°)':>7}" for j in range(dof))
    print(f"{'序号':<6} | {headers} | {'最大转角':<8} | {'预计耗时':<8}")
    print("-" * 92)

    prev = poses[0]
    total_cycle_ms = 0
    for i, p in enumerate(poses):
        if i == 0:
            max_d = 0.0
            move_ms = 0
        else:
            max_d = max(abs(p[j] - prev[j]) for j in range(dof))
            move_ms = int(round((max_d / speed) * 1000.0))
            if move_ms < 2000 and max_d > 0.05:
                move_ms = 2000
            elif move_ms == 0:
                move_ms = 500
        dwell_ms = int(interval * 1000)
        total_cycle_ms += move_ms + dwell_ms
        vals = " ".join(f"{p[j]:>7.1f}" for j in range(dof))
        print(f"姿态 {i + 1:<2} | {vals} | {max_d:>7.1f}° | {(move_ms / 1000.0):>6.2f}s")
        prev = p

    print("-" * 92)
    print(f"  单轮预计总耗时: 约 {total_cycle_ms / 1000.0:.1f} 秒")
    print("=" * 92 + "\n")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="通用机械臂多姿态平滑同步控制脚本 (支持 法奥/同川/意优)",
        formatter_class=argparse.RawTextHelpFormatter,
    )
    parser.add_argument("-m", "--mode", choices=["once", "repeat"], default=MODE,
                        help="运行模式: once (单轮) 或 repeat (循环)")
    parser.add_argument("-s", "--speed", type=float, default=SPEED,
                        help=f"规划速度 (°/s, 默认: {SPEED})")
    parser.add_argument("-t", "--interval", type=float, default=INTERVAL,
                        help=f"到位停留时间 (s, 默认: {INTERVAL})")
    parser.add_argument("-r", "--repeat", type=int, default=REPEAT_COUNT,
                        help="repeat 轮次 (0=无限循环, 默认: 0)")
    parser.add_argument("-i", "--iface", type=str, default=IFACE,
                        help=f"EtherCAT 网卡名称 (默认: {IFACE})")
    parser.add_argument("--arm-type", choices=["auto", "fairino", "tongchuan", "eyou"], default="auto",
                        help="机械臂类型标识 (默认 auto 自动探测)")
    parser.add_argument("--axes", type=str, default="",
                        help="可选目标轴列表，逗号分隔，如 J1,J2..J7 或 S2..S8 或 D18..D24")
    parser.add_argument("--config", type=str, default="",
                        help="可选自定义 poses.json 路径")
    parser.add_argument("--dry-run", action="store_true",
                        help="仅预览轨迹规划，不连接硬件")
    args = parser.parse_args()

    speed = max(0.5, min(args.speed, 90.0))
    interval = max(0.0, args.interval)
    interval_ms = int(interval * 1000)
    repeat_count = 1 if args.mode == "once" else (args.repeat if args.repeat > 0 else 0)

    if args.config:
        with open(args.config, "r", encoding="utf-8") as f:
            cfg = json.load(f)
        poses = cfg["poses"]
    else:
        poses = POSES

    dof = validate_poses(poses)
    print_preview(poses, dof, speed, interval, args.mode, repeat_count, args.arm_type, args.axes)

    if args.dry_run:
        print("[Dry-run] 规划分析完成，未启动底层硬件。\n")
        return 0

    if not os.path.exists(DEFAULT_BIN):
        print("[构建] 正在编译底层 SOEM 主站程序 ...")
        if subprocess.call([BUILD_SH], cwd=ROOT_DIR) != 0:
            print("[错误] 底层程序编译失败！", file=sys.stderr)
            return 1

    try:
        with SoemRpc(args.iface) as rpc:
            target_slaves = rpc.motion_open(args.axes)
            if len(target_slaves) != dof:
                print(
                    f"[错误] 姿态维度不匹配: 检测到 {len(target_slaves)} 个目标伺服轴，但姿态维度为 {dof}。",
                    file=sys.stderr,
                )
                rpc.motion_close()
                return 1

            try:
                round_idx = 0
                while True:
                    round_idx += 1
                    round_desc = f"共 {repeat_count} 轮" if repeat_count > 0 else "无限循环"
                    print(f"\n>>> 开始第 {round_idx} 轮姿态轨迹 ({round_desc}) <<<")

                    for pose_idx, target_degs in enumerate(poses):
                        status = rpc.motion_status()
                        cur_degs = [ax["deg"] for ax in status["axes"]]
                        max_delta = max(abs(t - c) for t, c in zip(target_degs, cur_degs))

                        move_ms = int(round((max_delta / speed) * 1000.0))
                        if move_ms < 2000 and max_delta > 0.05:
                            move_ms = 2000
                        elif move_ms == 0:
                            move_ms = 500

                        plan_str = " ".join(
                            f"{ax['name']}:{c:.1f}°->{t:.1f}°"
                            for ax, c, t in zip(status["axes"], cur_degs, target_degs)
                        )
                        print(f"[姿态 {pose_idx + 1}/{len(poses)}] 最大转角={max_delta:.2f}° | 耗时={move_ms}ms | [{plan_str}]")

                        rpc.motion_move(move_ms, target_degs)
                        rpc.motion_hold(interval_ms if interval_ms > 0 else 100)

                        after = rpc.motion_status()
                        print(f"  到位检验 (WKC={after['wkc']}/{after['expected_wkc']} | 重试={after['retries']}):")
                        for ax, tgt in zip(after["axes"], target_degs):
                            err = ax["deg"] - tgt
                            print(
                                f"    {ax['name']:<4} | 设定={tgt:>7.2f}° | 实测={ax['deg']:>7.2f}° "
                                f"| 误差={err:>+6.2f}° | 扭矩={ax['torque'] * 0.1:>5.1f}% | SW=0x{ax['sw']:04x}"
                            )

                    if repeat_count > 0 and round_idx >= repeat_count:
                        final_st = rpc.motion_status()
                        print(
                            f"\n[完成] 已完成全部 {repeat_count} 轮姿态运行 | "
                            f"总周期={final_st['cycles']} | 重试={final_st['retries']} | "
                            f"最大交换={final_st['max_cycle_us']}us | 最大抖动={final_st['max_late_us']}us"
                        )
                        break
            finally:
                rpc.motion_close()
        return 0
    except KeyboardInterrupt:
        print("\n[系统] 捕获中断信号 (Ctrl+C)，已平稳失能退出。")
        return 0
    except RpcError as exc:
        print(f"[错误] {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
