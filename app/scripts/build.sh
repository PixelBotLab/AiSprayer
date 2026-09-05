#!/usr/bin/env bash
# 按当前主机编译 C++ 组件：third_party → motion → orbbec_camera_service（含 follow）。
# RK3588 (Linux aarch64) 保持原有 MPP/RGA 优化；macOS / x86 Linux 走软编码 + 真机 Orbbec SDK。
#
# 默认会编：
#   deps    third_party → install/（RK: MPP/RGA/RKNN/Orbbec/ZLM；generic: Orbbec/ZLM/OpenH264）
#   motion  libmotion_core / libmotion_io / libmotion_c / motion_cli / 单测
#   follow  standalone：libfollow、follow_node/pose（有 SDK 时）、验收工具与单测
#   camera  orbbec_camera_service（内嵌 follow + follow_config 库）+ 离线单测
# 不编：C++ planner、默认不编 Faiss / RKNN 前端。
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
APP_DIR="$(dirname "$SCRIPT_DIR")"
PROJECT_ROOT="$(dirname "$APP_DIR")"

jobs_n() { nproc 2>/dev/null || sysctl -n hw.ncpu 2>/dev/null || echo 4; }
JOBS="$(jobs_n)"
ONLY=""
FORCE=false
CLEAN=false

while [[ $# -gt 0 ]]; do
    case "$1" in
        --jobs) JOBS="$2"; shift 2 ;;
        --only) ONLY="$2"; shift 2 ;;
        --force) FORCE=true; shift ;;
        -c|--clean) CLEAN=true; shift ;;
        -h|--help)
            cat <<'EOF'
Usage: app/scripts/build.sh [-c|--clean] [--only deps|motion|follow|camera] [--jobs N] [--force]

  (default)  third_party + motion + follow + camera service
  --only deps    只编 third_party
  --only motion  只编 motion（libmotion_c / motion_cli）
  --only follow  只编 follow standalone（follow_node / 单测；无 Orbbec SDK 时跳过设备目标）
  --only camera  只编 orbbec_camera_service（内嵌 follow 库，不含 follow_node）
  --force        转发给 third_party/build.sh --force
  -c, --clean    删除编译产物后退出（可与 --only 组合）
                 默认：motion/build、follow/build、camera/build+bin
                 --only deps：只删 third_party/install（保留 src/ 源码树）
EOF
            exit 0
            ;;
        *) echo "未知参数: $1" >&2; exit 1 ;;
    esac
done

OS=$(uname -s)
ARCH=$(uname -m)
if [[ "$OS" == "Linux" && "$ARCH" == "aarch64" ]]; then
    PROFILE=rk3588
    ENABLE_RK3588=ON
else
    PROFILE=generic
    ENABLE_RK3588=OFF
fi

rm_tree() {
    local d="$1"
    if [[ -e "$d" ]]; then
        echo "rm -rf ${d}"
        rm -rf "$d"
    fi
}

clean_deps() {
    rm_tree "${PROJECT_ROOT}/third_party/install"
}

clean_motion() {
    rm_tree "${PROJECT_ROOT}/app/src/core/motion/build"
}

clean_follow() {
    rm_tree "${PROJECT_ROOT}/app/src/core/follow/build"
    rm_tree "${PROJECT_ROOT}/app/src/core/follow/build_test"
}

clean_camera() {
    local cam="${PROJECT_ROOT}/app/src/core/hardware/camera/orbbec_camera_service"
    rm_tree "${cam}/build"
    rm_tree "${cam}/build_mock"
    rm_tree "${cam}/build_generic"
    rm_tree "${cam}/bin"
}

if $CLEAN; then
    case "${ONLY}" in
        "")
            clean_motion
            clean_follow
            clean_camera
            echo "clean done.（未删 third_party/install；若要清依赖: $0 -c --only deps）"
            ;;
        deps)   clean_deps; echo "clean done." ;;
        motion) clean_motion; echo "clean done." ;;
        follow) clean_follow; echo "clean done." ;;
        camera) clean_camera; echo "clean done." ;;
        *)
            echo "未知 --only: ${ONLY}（deps|motion|follow|camera）" >&2
            exit 1
            ;;
    esac
    exit 0
fi

echo "=========================================="
echo " AiSprayer C++ build"
echo " host=${OS} ${ARCH}  profile=${PROFILE}  jobs=${JOBS}"
echo " ENABLE_RK3588=${ENABLE_RK3588}"
echo "=========================================="

if [[ "$OS" == "Darwin" ]]; then
    if [[ -z "${OpenMP_ROOT:-}" ]] && command -v brew >/dev/null 2>&1; then
        _omp="$(brew --prefix libomp 2>/dev/null || true)"
        if [[ -n "$_omp" && -d "$_omp" ]]; then
            export OpenMP_ROOT="$_omp"
            echo "OpenMP_ROOT=${OpenMP_ROOT}"
        fi
    fi
fi

build_deps() {
    local extra=()
    $FORCE && extra+=(--force)
    bash "${PROJECT_ROOT}/third_party/build.sh" --profile "${PROFILE}" --jobs "${JOBS}" "${extra[@]}"
}

build_motion() {
    local src="${PROJECT_ROOT}/app/src/core/motion"
    cmake -S "${src}" -B "${src}/build" -DCMAKE_BUILD_TYPE=RelWithDebInfo
    cmake --build "${src}/build" --parallel "${JOBS}"
    echo "motion: ${src}/build/motion_cli  ${src}/build/libmotion_c.*"
}

build_follow() {
    local src="${PROJECT_ROOT}/app/src/core/follow"
    cmake -S "${src}" -B "${src}/build" \
          -DCMAKE_BUILD_TYPE=RelWithDebInfo \
          -DFOLLOW_STANDALONE=ON \
          -DFOLLOW_ENABLE_RKNN=OFF
    cmake --build "${src}/build" --parallel "${JOBS}"
    echo "follow: ${src}/build/follow_replay ${src}/build/follow_node ${src}/build/follow_pose"
}

build_camera() {
    local src="${PROJECT_ROOT}/app/src/core/hardware/camera/orbbec_camera_service"
    cmake -S "${src}" -B "${src}/build" \
          -DCMAKE_BUILD_TYPE=Release \
          -DENABLE_RK3588="${ENABLE_RK3588}" \
          -DFOLLOW_ENABLE_RKNN=OFF
    cmake --build "${src}/build" --parallel "${JOBS}"
    echo "camera: ${src}/bin/orbbec_camera_service"
}

case "${ONLY}" in
    "")
        build_deps
        build_motion
        build_follow
        build_camera
        ;;
    deps)   build_deps ;;
    motion) build_motion ;;
    follow) build_follow ;;
    camera) build_camera ;;
    *)
        echo "未知 --only: ${ONLY}（deps|motion|follow|camera）" >&2
        exit 1
        ;;
esac

echo "build done."
