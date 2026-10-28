import os
from pathlib import Path
import socket
import subprocess
import sys
import threading

binary = sys.argv[1]
iface = 'testpdo' + str(os.getpid())
assert len(iface) <= 15
runtime = Path('/tmp') / ('arm_maint-control-' + str(os.geteuid()))
created = not runtime.exists()
runtime.mkdir(mode=0o700, exist_ok=True)
path = runtime / (iface + '.sock')

def run(args):
    return subprocess.run([binary, '-i', iface] + args, capture_output=True, text=True, timeout=5)

def exchange(args, response, expected):
    server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    server.bind(str(path)); server.listen(1); server.settimeout(4)
    requests = []
    def handle():
        connection, _ = server.accept()
        with connection:
            data = b''
            while not data.endswith(b'\n'):
                data += connection.recv(2048)
            requests.append(data.decode().strip())
            connection.sendall(response.encode())
    thread = threading.Thread(target=handle)
    thread.start()
    try:
        result = run(args)
        thread.join(4)
        assert not thread.is_alive()
        assert requests and expected in requests[0], requests
        assert '[INFO]' not in result.stdout, result.stdout  # No raw-socket initialization/rescan.
        return result
    finally:
        server.close(); path.unlink(missing_ok=True)

try:
    result = run(['jog-enabled', '--axis', 'D18,D19', '--delta_deg', '1'])
    assert result.returncode == 1
    assert 'D18 未使能' in result.stdout and 'D19 未使能' in result.stdout
    assert '[INFO]' not in result.stdout
    result = exchange(['enable', '--axis', 'D18', '--axis', 'D19'],
                      'OK\nD18；D19 [使能成功]\n', 'enable --axis D18 --axis D19')
    assert result.returncode == 0 and '[使能成功]' in result.stdout
    result = exchange(['jog-enabled', '--axis', 'D18,D19', '--delta_deg', '1', '--speed_rpm', '.3'],
                      'ERR\nD19 未使能（SwitchOnDisabled）；整条命令拒绝\n',
                      'jog-enabled --axis D18 --axis D19')
    assert result.returncode == 1 and 'D19 未使能' in result.stdout
    result = exchange(['jog-enabled', '--axis', 'D18,D19', '--delta_deg', '1.0,-0.5', '--speed_rpm', '.3'],
                      'OK\nD18；D19 [点动完成]\n',
                      'jog-enabled --axis D18 --axis D19 --delta_deg 1,-0.5')
    assert result.returncode == 0
    result = exchange(['jog-enabled', '--axis', 'D19', '--axis', 'D18', '--delta_count', '-2000,5000'],
                      'OK\nD19；D18 [点动完成]\n',
                      'jog-enabled --axis D19 --axis D18 --delta_count -2000,5000')
    assert result.returncode == 0
    result = exchange(['jog-enabled', '--axis', 'D18,D19', '--delta_deg', '0,-0.5'],
                      'OK\nD18；D19 [点动完成]\n', '--delta_deg 0,-0.5')
    assert result.returncode == 0
    result = exchange(['disable', '--all'], 'OK\nD18；D19 [失能成功]\n', 'disable --all')
    assert result.returncode == 0
    result = exchange(['status'], 'OK\nD18 OperationEnabled position=100\n', 'status')
    assert result.returncode == 0 and 'position=100' in result.stdout
    result = exchange(['read-pos', '--axis', 'D18'], 'OK\nD18 position=100\n', 'read-pos D18')
    assert result.returncode == 0 and 'position=100' in result.stdout
    result = exchange(['write-alias', '--slave', '1', '--alias', '11', '--backup', 'unused.bin',
                       '--confirm', 'WRITE-EEPROM-ALIAS'],
                      'OK\nD18 OperationEnabled\n', 'status')
    assert result.returncode == 1 and 'disable --all' in result.stderr
    for args in [
        ['enable', '--axis', 'D18oops'],
        ['enable', '--axis', '0'],
        ['enable', '--axis', 'D18,'],
        ['enable', '--axis', 'D18', '--axis', ''],
        ['enable', '--all'],
        ['jog-enabled', '--axis', 'D18', '--delta_deg', 'nan'],
        ['jog-enabled', '--axis', 'D18', '--delta_deg', '1', '--delta_count', '1'],
        ['jog-enabled', '--axis', 'D18', '--delta_count', '1.5'],
        ['jog-enabled', '--axis', 'D18'],
        ['jog-enabled', '--axis', 'D18,D19', '--delta_deg', '1,2,3'],
        ['jog-enabled', '--axis', 'D18', '--delta_deg', '1,-.5'],
        ['jog-enabled', '--axis', 'D18,D19', '--delta_deg', '1,'],
        ['jog-enabled', '--axis', 'D18,D19', '--delta_deg', ',1'],
        ['jog-enabled', '--axis', 'D18,D19', '--delta_deg', '1,,2'],
        ['jog-enabled', '--axis', 'D18,D19', '--delta_deg', '1,nan'],
        ['jog-enabled', '--axis', 'D18,D19', '--delta_deg', '1,inf'],
        ['jog-enabled', '--axis', 'D18,D19', '--delta_deg', '0,0'],
        ['jog-enabled', '--axis', 'D18,D19', '--delta_count', '5000,-2.5'],
        ['jog-enabled', '--axis', 'D18,D19', '--delta_count', '5000,2147483648'],
        ['disable', '--axis', 'D18', '--all'],
    ]:
        result = run(args)
        assert result.returncode == 1 and '[INFO]' not in result.stdout, (args, result.stdout, result.stderr)
    print('Standalone multi-axis CLI, named errors, IPC routing and no-rescan checks passed')
finally:
    path.unlink(missing_ok=True)
    if created:
        try: runtime.rmdir()
        except OSError: pass
