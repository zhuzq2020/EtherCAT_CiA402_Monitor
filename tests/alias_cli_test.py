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
# PDO writes require an explicit physical target and a backup before opening a NIC.
for args in (['configure-pdo'], ['configure-pdo', '--confirm'],
             ['configure-pdo', '--slave', '1', '--confirm']):
    result = subprocess.run([binary, *args], capture_output=True, text=True)
    assert result.returncode == 1 and '[INFO]' not in result.stdout, result
for value in ('', '0', '8-2', '2,2', '2-4,3', '2,', ',2', '2,,3', 'S2', '2-65536', '2-3-4'):
    result = subprocess.run([binary, 'configure-pdo', '--slaves', value, '--help'], capture_output=True, text=True)
    assert result.returncode == 1 and '[INFO]' not in result.stdout, (value, result)
for value in ('2-8', '2,3,5', '2-8,10-16'):
    result = subprocess.run([binary, 'configure-pdo', '--slaves', value, '--help'], capture_output=True, text=True)
    assert result.returncode == 0, (value, result)
for args in (['--slaves', '2-8', '--confirm'],
             ['--slaves', '2-8', '--slave', '2'],
             ['--slaves', '2-8', '--backup', 'unused'],
             ['--backup-dir', 'unused']):
    result = subprocess.run([binary, 'configure-pdo', *args], capture_output=True, text=True)
    assert result.returncode == 1 and '[INFO]' not in result.stdout, (args, result)
# Save cannot be requested from a read-only invocation or another command.
for args in (['configure-pdo', '--slave', '2', '--save'],
             ['configure-pdo', '--slaves', '2-8', '--save'],
             ['configure-pdo', '--slave', '2', '--save', '--confirm'],
             ['status', '--save', '--confirm']):
    result = subprocess.run([binary, *args], capture_output=True, text=True)
    assert result.returncode == 1 and '[INFO]' not in result.stdout, (args, result)
