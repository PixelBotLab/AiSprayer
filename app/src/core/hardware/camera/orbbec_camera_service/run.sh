#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(git -C "${SCRIPT_DIR}" rev-parse --show-toplevel 2>/dev/null || (cd "${SCRIPT_DIR}/../../../../../.." && pwd))"

export LD_LIBRARY_PATH="${PROJECT_ROOT}/third_party/install/lib:${LD_LIBRARY_PATH}"

BIN="${SCRIPT_DIR}/bin/orbbec_camera_service"
CONFIG="${PROJECT_ROOT}/configs/aisprayer_config.yaml"

if [ ! -f "${BIN}" ]; then
    echo "[!] Binary not found at ${BIN}. Building now..."
    mkdir -p "${SCRIPT_DIR}/build"
    cd "${SCRIPT_DIR}/build"
    cmake ..
    make -j$(nproc)
fi

echo "[*] Starting AiSprayer Orbbec C++ Camera Service..."
echo "[*] Unified Config file: ${CONFIG}"
echo "[*] Working directory: ${PROJECT_ROOT}"

cd "${PROJECT_ROOT}"
exec "${BIN}" --config "${CONFIG}" "$@"
