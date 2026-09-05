#!/usr/bin/env bash
# =============================================================================
# AiSprayer Docker Container Entrypoint
# =============================================================================
set -e

APP_ROOT="/app"
cd "${APP_ROOT}"

# Set up environment
export PATH="/opt/venv/bin:${APP_ROOT}/bin:${PATH}"
export PYTHONPATH="${APP_ROOT}/app/src:${APP_ROOT}/tools:${PYTHONPATH:-}"
export LD_LIBRARY_PATH="${APP_ROOT}/lib:${APP_ROOT}/third_party/install/lib:/usr/local/lib:/usr/lib:${LD_LIBRARY_PATH:-}"
export PYTHONUNBUFFERED=1

# Hardware & Platform Diagnostics
ARCH=$(uname -m)
echo "============================================================"
echo " AiSprayer Container Starting..."
echo " Architecture: ${ARCH}"
echo " Local Time:   $(date)"
echo "============================================================"

if [[ "${ARCH}" == "aarch64" || "${ARCH}" == "arm64" ]]; then
    echo "━━━ RK3588 Hardware Node Status ━━━"
    [[ -c /dev/mpp_service ]] && echo "  ✔ MPP Video Hardware Service: /dev/mpp_service (Ready)" || echo "  ✘ /dev/mpp_service missing (Host MPP hardware encoder unavailable)"
    [[ -c /dev/rga ]]         && echo "  ✔ RGA 2D Hardware Accelerator: /dev/rga (Ready)"         || echo "  ✘ /dev/rga missing"
    [[ -d /dev/dri ]]         && echo "  ✔ DRM / GPU Render Nodes: /dev/dri (Ready)"             || echo "  ✘ /dev/dri missing"
    [[ -d /dev/bus/usb ]]     && echo "  ✔ USB Subsystem: /dev/bus/usb (Ready for Orbbec Camera)" || echo "  ✘ /dev/bus/usb missing"
fi

# Clean up stale locks & sockets
echo "Cleaning up any stale locks..."
rm -f "${APP_ROOT}/.orbbec.lock" 2>/dev/null || true

# Ensure persistent directories exist
mkdir -p "${APP_ROOT}/data/calib" "${APP_ROOT}/data/template_group" "${APP_ROOT}/app/logs" "${APP_ROOT}/app/data"

# If custom command was passed, execute it
if [[ $# -gt 0 && "$1" != "all" ]]; then
    exec "$@"
fi

# Default mode: run full system
echo "Starting AiSprayer Service Stack..."
echo "  * Backend API:  http://0.0.0.0:8000"
echo "  * Web Stream:   http://0.0.0.0:8008"
echo "  * RTSP Server:  rtsp://0.0.0.0:8554"
echo "  * HTTP Control: http://0.0.0.0:18080"
echo "============================================================"

# Execute Python FastAPI backend (which internally starts C++ Camera Service and MobileSAM)
exec python3 "${APP_ROOT}/app/src/main.py"
