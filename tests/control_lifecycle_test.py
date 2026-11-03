import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time

# Fixture links a test-only bus: no raw socket or real drive access.
binary = sys.argv[1]
with tempfile.TemporaryDirectory(prefix='arm-maint-lifecycle-') as directory:
    audit = Path(directory) / 'audit.log'
    env = dict(os.environ, ARM_MAINT_TEST_AUDIT=str(audit))
    def run(*args):
        result = subprocess.run([binary, '-i', 'lo', *args], env=env, capture_output=True,
                                text=True, stdin=subprocess.DEVNULL, timeout=10)
        assert result.returncode == 0, (args, result.stdout, result.stderr)
        return result.stdout
    def events():
        return audit.read_text().splitlines() if audit.exists() else []
    def until_empty():
        for _ in range(100):
            if not audit.exists() or events()[-1:] == ['SHUTDOWN']:
                return
            time.sleep(.01)
        raise AssertionError(events())
    # Early child failures must retain their cause instead of reporting timeout.
    for failure, expected in [('signal', '信号 11'), ('exit', '退出码=23'),
                              ('error', 'mock PDO mapping rejected')]:
        result = subprocess.run([binary, '-i', 'lo', 'start-pdo'],
                                env=dict(env, ARM_MAINT_TEST_FAIL=failure), capture_output=True,
                                text=True, timeout=10)
        assert result.returncode == 1 and expected in result.stderr, (failure, result.stderr)
        assert '启动超过 120 秒' not in result.stderr
    started = False
    try:
        # A running instance on loopback would interfere; never stop it.
        runtime = Path('/tmp') / f'arm_maint-control-{os.geteuid()}'
        assert not (runtime / 'lo.sock').exists(), 'loopback control service already present'
        result = run('start-pdo'); started = True
        assert '可控制伺服=14' in result
        assert events() == ['PREPARE 14'], events()
        time.sleep(.1)
        result = run('start-pdo')
        assert '可控制伺服=14' in result
        assert events() == ['PREPARE 14'], 'repeated start must neither reinitialize nor enable'
        assert '使能成功' in run('enable', '--axis', 'S2,S3')
        assert events() == ['PREPARE 14', 'ENABLED S2', 'ENABLED S3'], events()
        run('start-pdo')
        assert events() == ['PREPARE 14', 'ENABLED S2', 'ENABLED S3'], 'start preserves existing enable'
        assert 'PDO 通信已停止' in run('stop-pdo')
        until_empty(); started = False
        assert events() == ['PREPARE 14', 'ENABLED S2', 'ENABLED S3',
                            'DISABLED S2', 'DISABLED S3', 'SHUTDOWN'], events()
        assert '无需停止' in run('stop-pdo')
        # The socket and interface lock must be released before stop returns.
        assert '可控制伺服=14' in run('start-pdo'); started = True
        assert events().count('PREPARE 14') == 2
        run('stop-pdo'); started = False
        print('Real CLI/IPC lifecycle with mock bus: discovery, no implicit enable, reuse, disable, stop and restart passed')
    finally:
        if started:
            run('stop-pdo')
