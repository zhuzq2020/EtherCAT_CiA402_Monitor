#!/usr/bin/env python3
"""Pre-bus CLI validation tests for Python workflow scripts and core C++ CLI."""

from pathlib import Path
import subprocess
import sys

binary = sys.argv[1]
root_dir = Path(__file__).resolve().parent.parent
scripts_dir = root_dir / "scripts"
sys.path.insert(0, str(scripts_dir))

from sii_utils import sii_crc, verify_alias_image

# 1. Verify Python SII CRC & image verification match C++ reference vectors
words = [0x0C80, 0x6681, 0, 0, 0, 0, 0, 0x0062]
assert sii_crc(words) == 0x62
before = [0] * 1024
before[0] = 0x0C80
before[1] = 0x6681
before[7] = 98
after = list(before)
after[4] = 18
after[7] = sii_crc(after)
assert after[7] == 174
assert verify_alias_image(before, after, 18)[0]
assert not verify_alias_image(before, before, 18)[0]

# 2. write_alias.py rejects malformed targets and missing confirmation before bus access
write_alias_py = str(scripts_dir / "write_alias.py")
for flag in ("--slave", "--alias"):
    for value in ("0", "-1", "1oops", "1.5", "65536", "D18", "+18"):
        res = subprocess.run([sys.executable, write_alias_py, flag, value, "--help"], capture_output=True, text=True)
        assert res.returncode != 0, (flag, value, res)
res = subprocess.run(
    [sys.executable, write_alias_py, "--slave", "1", "--alias", "18", "--backup", "/tmp/unused-alias-cli.bin"],
    capture_output=True, text=True,
)
assert res.returncode == 1 and "WRITE-EEPROM-ALIAS" in res.stderr

# 3. configure_pdo.py rejects invalid targets/combinations before bus access
cfg_pdo_py = str(scripts_dir / "configure_pdo.py")
for args in (
    [],
    ["--confirm"],
    ["--slave", "1", "--confirm"],
    ["--slaves", "2-8", "--confirm"],
    ["--slaves", "2-8", "--slave", "2"],
    ["--slaves", "2-8", "--backup", "unused"],
    ["--backup-dir", "unused"],
    ["--slave", "2", "--save"],
    ["--slaves", "2-8", "--save"],
    ["--slave", "2", "--save", "--confirm"],
):
    res = subprocess.run([sys.executable, cfg_pdo_py, *args], capture_output=True, text=True)
    assert res.returncode == 1, (args, res)

for bad in ("", "0", "8-2", "2,2", "2-4,3", "2,", ",2", "2,,3", "S2", "2-65536", "2-3-4"):
    res = subprocess.run([sys.executable, cfg_pdo_py, "--slaves", bad], capture_output=True, text=True)
    assert res.returncode == 1, (bad, res)

# 4. backup_sii.py rejects missing/invalid args
res = subprocess.run([sys.executable, str(scripts_dir / "backup_sii.py"), "--slave", "0"], capture_output=True, text=True)
assert res.returncode != 0

# 5. scripts/pose_control.py --dry-run succeeds for Fairino/Tongchuan/Eyou modes
pose_py = str(scripts_dir / "pose_control.py")
for arm_type in ("auto", "fairino", "tongchuan", "eyou"):
    res = subprocess.run([sys.executable, pose_py, "--arm-type", arm_type, "--dry-run"], capture_output=True, text=True)
    assert res.returncode == 0 and "Dry-run" in res.stdout, (arm_type, res)

# 6. Core C++ CLI rejects removed commands and malformed jog/sdo arguments before bus init
for args in (
    ["clear-fault"],
    ["start-pdo"],
    ["jog-enabled"],
    ["zero-torque", "--all", "--confirm"],
    ["monitor"],
    ["test-rotate"],
    ["run-poses"],
    ["rotate", "J7", "1.0"],
    ["rotate-abs", "J7", "0.0"],
    ["jog"],
    ["jog", "J7", "bad_deg"],
    ["jog", "J7", "1.0", "-5"],
    ["jog", "--axis", "J7", "--deg", "abc"],
    ["write-sdo", "--axis", "J1", "--index", "0x6060", "--type", "i8", "--value", "8"],
    ["set-zero", "--axis", "J1"],
):
    res = subprocess.run([binary, *args], capture_output=True, text=True)
    assert res.returncode == 1 and "[INFO]" not in res.stdout, (args, res)

print("Python workflows, SII utils, pose_control dry-run, and streamlined C++ CLI checks passed")
