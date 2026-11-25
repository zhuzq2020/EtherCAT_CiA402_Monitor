#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""SII EEPROM CRC-8, full-image integrity check, and exclusive binary persistence."""

import os
from typing import List, Tuple

SII_ALIAS_WORD = 4
SII_SIZE_WORD = 0x003E
SII_HEADER_WORDS = 64
MAX_SII_BYTES = 128 * 1024

ECT_SII_MANUF = 0x0008
ECT_SII_ID = 0x000A

ECT_REG_ALSTAT = 0x0130
ECT_REG_ALIAS = 0x0012


def sii_crc(words: List[int]) -> int:
    """Compute EtherCAT SII configuration CRC-8 over words 0..6 (low byte first)."""
    crc = 0xFF
    for i in range(14):
        byte = (words[i // 2] >> ((i % 2) * 8)) & 0xFF
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ (0x07 if (crc & 0x80) else 0)) & 0xFF
    return crc


def words_u32(words: List[int], offset: int) -> int:
    return (words[offset] & 0xFFFF) | ((words[offset + 1] & 0xFFFF) << 16)


def words_to_bytes(words: List[int]) -> bytes:
    return b"".join((w & 0xFFFF).to_bytes(2, "little") for w in words)


def write_bytes_exclusive(path: str, data: bytes) -> None:
    """Atomically create a new file (O_CREAT | O_EXCL) and fsync; refuses to overwrite."""
    parent = os.path.dirname(os.path.abspath(path))
    if parent:
        os.makedirs(parent, exist_ok=True)
    flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL
    if hasattr(os, "O_NOFOLLOW"):
        flags |= os.O_NOFOLLOW
    fd = os.open(path, flags, 0o644)
    try:
        view = memoryview(data)
        offset = 0
        while offset < len(view):
            written = os.write(fd, view[offset:])
            if written <= 0:
                raise OSError("写入文件失败")
            offset += written
        os.fsync(fd)
    finally:
        os.close(fd)


def verify_alias_image(before: List[int], after: List[int], alias: int) -> Tuple[bool, str]:
    if len(before) < SII_HEADER_WORDS or len(before) != len(after):
        return False, "完整 SII 镜像长度不一致"
    if after[SII_ALIAS_WORD] != alias:
        return False, f"Alias 回读不一致，目标={alias}，实际={after[SII_ALIAS_WORD]}"
    header = list(before[:8])
    header[SII_ALIAS_WORD] = alias
    expected_crc = (before[7] & 0xFF00) | sii_crc(header)
    for idx, b_val in enumerate(before):
        expected = alias if idx == SII_ALIAS_WORD else (expected_crc if idx == 7 else b_val)
        if after[idx] != expected:
            return False, f"完整 SII 校验失败，word={idx}，期望={expected}，实际={after[idx]}"
    return True, "OK"
