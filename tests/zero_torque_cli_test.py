"""Reject ambiguous or unconfirmed sensor writes before opening the bus."""
import subprocess
import sys

binary = sys.argv[1]
for args, message in (
    ([], "--confirm"),
    (["--axis", "D18"], "--confirm"),
    (["--all"], "--confirm"),
    (["--confirm"], "必须且只能指定"),
    (["--axis", "D18", "--all", "--confirm"], "必须且只能指定"),
):
    result = subprocess.run([binary, "zero-torque", *args], capture_output=True, text=True)
    assert result.returncode == 1, (args, result)
    assert message in result.stderr, (args, result)
    assert "[INFO]" not in result.stdout, (args, result)
result = subprocess.run([binary, "--help"], capture_output=True, text=True)
assert result.returncode == 0 and "zero-torque" in result.stdout
print("Torque zero CLI rejects missing confirmation and ambiguous targets before bus access")
