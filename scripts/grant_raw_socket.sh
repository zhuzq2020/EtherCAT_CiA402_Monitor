#!/usr/bin/env bash
set -euo pipefail

project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
binary="${1:-${project_dir}/build/ethercat_cia402_monitor}"

if [[ ! -x "${binary}" ]]; then
    echo "Executable not found: ${binary}" >&2
    echo "Build the project first, or pass the executable path as argument 1." >&2
    exit 1
fi

sudo setcap cap_net_raw,cap_net_admin+eip "${binary}"
getcap "${binary}"
echo "Raw EtherCAT socket capability granted. Re-run this script after replacing/rebuilding the executable."

