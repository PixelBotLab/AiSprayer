#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
APP_BUILD="$(cd "${SCRIPT_DIR}/../../../scripts" && pwd)/build.sh"
if [[ -x "${APP_BUILD}" ]]; then
  exec "${APP_BUILD}" --only motion
fi
MOTION="$(cd "${SCRIPT_DIR}/.." && pwd)"
BUILD="${MOTION}/build"
JOBS=$(nproc 2>/dev/null || sysctl -n hw.ncpu 2>/dev/null || echo 4)
cmake -S "${MOTION}" -B "${BUILD}" -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build "${BUILD}" --parallel "${JOBS}"
echo "built: ${BUILD}/motion_cli ${BUILD}/libmotion_core.a ${BUILD}/libmotion_c.*"
