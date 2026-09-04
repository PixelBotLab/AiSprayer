#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
MOTION="$(cd "${SCRIPT_DIR}/.." && pwd)"
BUILD="${MOTION}/build"
cmake -S "${MOTION}" -B "${BUILD}" -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build "${BUILD}" -j"$(nproc)"
echo "built: ${BUILD}/motion_cli ${BUILD}/libmotion_core.a ${BUILD}/libmotion_c.so"
