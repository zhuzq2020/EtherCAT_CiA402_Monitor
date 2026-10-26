import subprocess
import sys

binary = sys.argv[1]
# --help ensures an incorrectly accepted number cannot initialize the bus.
for flag in ('--slave', '--alias'):
    for value in ('0', '-1', '1oops', '1.5', '65536', 'D18', '+18'):
        result = subprocess.run([binary, flag, value, '--help'], capture_output=True, text=True)
        assert result.returncode == 1, (flag, value, result.returncode, result.stdout, result.stderr)
        assert '[INFO]' not in result.stdout
    result = subprocess.run([binary, flag], capture_output=True, text=True)
    assert result.returncode == 1
result = subprocess.run([binary, 'write-alias', '--slave', '1', '--alias', '18',
                         '--backup', '/tmp/unused-alias-cli-test.bin'], capture_output=True, text=True)
assert result.returncode == 1 and '[INFO]' not in result.stdout
print('Alias CLI rejects malformed targets and missing confirmation before bus initialization')
