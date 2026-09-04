#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
MOTION="$(cd "${SCRIPT_DIR}/.." && pwd)"
REPO="$(cd "${MOTION}/../../../.." && pwd)"
BUILD="${MOTION}/build"
CLI="${BUILD}/motion_cli"

CONFIG="${REPO}/configs/aisprayer_config.yaml"
INPUT="${REPO}/data/template_group/2026-09-03_225937/scan.auto.path.yaml"
OUTPUT="${BUILD}/out/scan.auto.cpp.poi.path.yaml"
SPEED=120
STEP=1.5

if [[ ! -x "${CLI}" ]]; then
  echo "motion_cli 不存在，先构建..."
  "${SCRIPT_DIR}/build.sh"
fi

mkdir -p "$(dirname "${OUTPUT}")"

if [[ $# -gt 0 && ( $1 == optimize || $1 == verify || $1 == fk || $1 == ik ) ]]; then
  exec "${CLI}" --config "${CONFIG}" "$@"
fi

exec "${CLI}" --config "${CONFIG}" optimize \
  --input "${INPUT}" \
  --output "${OUTPUT}" \
  --speed "${SPEED}" \
  --step "${STEP}" \
  "$@"
