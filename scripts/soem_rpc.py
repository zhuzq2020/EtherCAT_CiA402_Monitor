#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Shared Python client for the single-session SOEM RPC interface (`arm_maint rpc`)."""

import json
import os
import shutil
import subprocess
from typing import Dict, List, Optional


DEFAULT_BIN = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "build", "arm_maint"
)


def needs_sudo(binary_path: str) -> bool:
    if os.geteuid() == 0:
        return False
    getcap = shutil.which("getcap")
    if getcap:
        res = subprocess.run([getcap, binary_path], capture_output=True, text=True)
        if "cap_net_raw" in res.stdout:
            return False
    return True


class RpcError(RuntimeError):
    pass


class SoemRpc:
    """Context manager maintaining one open SOEM bus session via `arm_maint rpc`."""

    def __init__(self, iface: str, binary: str = DEFAULT_BIN):
        self.iface = iface
        self.binary = binary
        self.proc: Optional[subprocess.Popen] = None

    def __enter__(self) -> "SoemRpc":
        if not os.path.exists(self.binary):
            raise RpcError(f"未找到底层二进制程序: {self.binary}，请先运行 ./scripts/build.sh")
        cmd = []
        if needs_sudo(self.binary):
            cmd.append("sudo")
        cmd.extend([self.binary, "-i", self.iface, "rpc"])
        self.proc = subprocess.Popen(
            cmd,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=None,
            text=True,
            bufsize=1,
        )
        assert self.proc.stdout is not None
        ready = self.proc.stdout.readline().strip()
        if not ready.startswith("READY "):
            raise RpcError(f"RPC 会话启动失败: {ready or '无响应'}")
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        if self.proc is None:
            return
        try:
            if self.proc.poll() is None and self.proc.stdin is not None:
                self.proc.stdin.write("quit\n")
                self.proc.stdin.flush()
                self.proc.wait(timeout=5)
        except Exception:
            self.proc.terminate()
            try:
                self.proc.wait(timeout=3)
            except Exception:
                self.proc.kill()
        finally:
            self.proc = None

    def _call(self, command: str) -> str:
        if self.proc is None or self.proc.stdin is None or self.proc.stdout is None:
            raise RpcError("RPC 会话未启动")
        self.proc.stdin.write(command + "\n")
        self.proc.stdin.flush()
        reply = self.proc.stdout.readline()
        if not reply:
            raise RpcError("RPC 子进程意外退出")
        reply = reply.rstrip("\r\n")
        if reply == "OK":
            return ""
        if reply.startswith("OK "):
            return reply[3:]
        if reply.startswith("ERR "):
            raise RpcError(reply[4:])
        raise RpcError(f"协议响应异常: {reply}")

    def slaves(self) -> List[Dict]:
        return json.loads(self._call("slaves"))

    def read_sdo(self, slave: int, index: int, subindex: int = 0, timeout_us: Optional[int] = None) -> bytes:
        cmd = f"read-sdo {slave} 0x{index:04x} 0x{subindex:02x}"
        if timeout_us is not None:
            cmd += f" {timeout_us}"
        return bytes.fromhex(self._call(cmd))

    def write_sdo(
        self,
        slave: int,
        index: int,
        subindex: int,
        data: bytes,
        ca: bool = False,
        timeout_us: Optional[int] = None,
    ) -> None:
        cmd = f"write-sdo {slave} 0x{index:04x} 0x{subindex:02x} {1 if ca else 0} {data.hex()}"
        if timeout_us is not None:
            cmd += f" {timeout_us}"
        self._call(cmd)

    def read_sdo_int(self, slave: int, index: int, subindex: int = 0, signed: bool = False) -> int:
        raw = self.read_sdo(slave, index, subindex)
        return int.from_bytes(raw, "little", signed=signed)

    def read_esc(self, slave: int, reg: int, size: int) -> bytes:
        return bytes.fromhex(self._call(f"read-esc {slave} 0x{reg:04x} {size}"))

    def write_esc(self, slave: int, reg: int, data: bytes) -> None:
        self._call(f"write-esc {slave} 0x{reg:04x} {data.hex()}")

    def eeprom_acquire(self, slave: int) -> Dict:
        return json.loads(self._call(f"eeprom-acquire {slave}"))

    def eeprom_restore(self, slave: int) -> None:
        self._call(f"eeprom-restore {slave}")

    def eeprom_read_words(self, slave: int, start_word: int, count: int) -> List[int]:
        raw = bytes.fromhex(self._call(f"eeprom-read {slave} {start_word} {count}"))
        return [int.from_bytes(raw[i : i + 2], "little") for i in range(0, len(raw), 2)]

    def eeprom_write_header(self, slave: int, words_8: List[int]) -> None:
        if len(words_8) != 8:
            raise ValueError("配置区必须正好包含 8 个 16-bit words")
        payload = b"".join(int(w).to_bytes(2, "little") for w in words_8)
        self._call(f"eeprom-write-header {slave} {payload.hex()}")

    # Real-time PDO motion session primitives
    def motion_open(self, axes_csv: str = "") -> List[int]:
        cmd = f"motion-open {axes_csv}".strip()
        return json.loads(self._call(cmd))

    def motion_move(self, move_ms: int, target_degs: List[float]) -> None:
        deg_csv = ",".join(f"{d:.6f}" for d in target_degs)
        self._call(f"motion-move {int(move_ms)} {deg_csv}")

    def motion_hold(self, hold_ms: int) -> None:
        self._call(f"motion-hold {int(hold_ms)}")

    def motion_status(self) -> Dict:
        return json.loads(self._call("motion-status"))

    def motion_close(self) -> None:
        self._call("motion-close")
