#!/usr/bin/env bash
# Run under sudo. Stop at a crash or just after PDO preparation, before enable.
set -euo pipefail
ROOT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
if [[ $# -ne 1 ]]; then
    echo "Usage: sudo $0 <iface>" >&2
    exit 2
fi
if [[ ${EUID} -ne 0 ]]; then
    echo "Run this diagnostic with sudo." >&2
    exit 2
fi
command -v gdb >/dev/null
# The preparation breakpoint is mandatory. GDB aborts this command file before
# running if it cannot be resolved, preventing an unguarded diagnostic run.
GDB_COMMANDS=$(mktemp)
trap 'rm -f "$GDB_COMMANDS"' EXIT
cat > "$GDB_COMMANDS" <<'GDB'
set pagination off
set confirm off
set breakpoint pending off
set print elements 16
set follow-fork-mode child
set detach-on-fork on
break robot_maint::PersistentPdoBus::prepare
handle SIGSEGV stop print nopass
handle SIGABRT stop print nopass
handle SIGBUS stop print nopass
handle SIGILL stop print nopass
run
finish
thread apply all bt
info registers
quit
GDB
exec_status=0
gdb -q -batch -x "$GDB_COMMANDS" --args "$ROOT_DIR/build/arm_maint" start-pdo -i "$1" || exec_status=$?
exit "$exec_status"
