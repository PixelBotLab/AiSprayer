#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
# app/src/core/motion/scripts → 上溯 4 级才是 app/scripts（原来只上溯 3 级，cd 失败后
# set -e 直接退出，导致本脚本从未走到统一构建入口）。
APP_SCRIPTS="${SCRIPT_DIR}/../../../../scripts"
if [[ -x "${APP_SCRIPTS}/build.sh" ]]; then
  exec "${APP_SCRIPTS}/build.sh" --only motion
fi
MOTION="$(cd "${SCRIPT_DIR}/.." && pwd)"
BUILD="${MOTION}/build"
JOBS=$(nproc 2>/dev/null || sysctl -n hw.ncpu 2>/dev/null || echo 4)
cmake -S "${MOTION}" -B "${BUILD}" -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build "${BUILD}" --parallel "${JOBS}"
echo "built: ${BUILD}/motion_cli ${BUILD}/libmotion_core.a ${BUILD}/libmotion_c.*"
