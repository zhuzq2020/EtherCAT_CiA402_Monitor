#!/usr/bin/env bash
set -euo pipefail

project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
package_dir="${project_dir}/.deps/qt6-packages"
prefix="${project_dir}/.deps/qt6"

packages=(
  libb2-1
  qt6-base-dev qt6-base-dev-tools qmake6 qmake6-bin qt6-qpa-plugins
  libqt6concurrent6 libqt6core6 libqt6dbus6 libqt6gui6 libqt6network6
  libqt6opengl6 libqt6openglwidgets6 libqt6printsupport6 libqt6sql6
  libqt6test6 libqt6widgets6 libqt6xml6
)

mkdir -p "${package_dir}" "${prefix}"
(
  cd "${package_dir}"
  apt-get download "${packages[@]}"
)

for package in "${package_dir}"/*.deb; do
  dpkg-deb -x "${package}" "${prefix}"
done

echo "Qt 6 development files extracted without sudo."
echo "Configure with:"
echo "  cmake -S '${project_dir}' -B '${project_dir}/build' -DCMAKE_PREFIX_PATH='${prefix}/usr'"
