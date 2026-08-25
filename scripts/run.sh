#!/usr/bin/env bash
set -euo pipefail

project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
binary="${project_dir}/build/ethercat_cia402_monitor"
local_plugins="${project_dir}/.deps/qt6/usr/lib/x86_64-linux-gnu/qt6/plugins"
local_qt_lib="${project_dir}/.deps/qt6/usr/lib/x86_64-linux-gnu"
bundled_soem_lib="${project_dir}/third_party/soem/lib"

if [[ ! -x "${binary}" ]]; then
  echo "Executable not found: ${binary}" >&2
  exit 1
fi

if [[ -d "${local_plugins}" ]]; then
  export QT_PLUGIN_PATH="${local_plugins}${QT_PLUGIN_PATH:+:${QT_PLUGIN_PATH}}"
fi

export LD_LIBRARY_PATH="${bundled_soem_lib}:${local_qt_lib}${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"

exec "${binary}" "$@"
