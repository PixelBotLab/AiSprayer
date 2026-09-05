#!/usr/bin/env bash
# =============================================================================
# docker/build.sh — AiSprayer 跨平台 Docker 镜像构建脚本
#
# 支持平台:
#   - rk3588  (Linux aarch64, 包含板载 NPU/MPP 优化, 默认)
#   - x86_64  (Linux x86_64, 通用服务器/PC 模拟运行)
# =============================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
cd "$PROJECT_ROOT"

# Color helpers
RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'; CYAN='\033[0;36m'; BOLD='\033[1m'; NC='\033[0m'
info()    { echo -e "${GREEN}[INFO]${NC}  $*"; }
warn()    { echo -e "${YELLOW}[WARN]${NC}  $*"; }
error()   { echo -e "${RED}[ERROR]${NC} $*"; exit 1; }
section() { echo -e "\n${BOLD}${CYAN}━━━ $* ━━━${NC}"; }

# Default parameters
HOST_ARCH=$(uname -m)
case "$HOST_ARCH" in
    aarch64|arm64) DEFAULT_PLATFORM="rk3588" ;;
    x86_64)        DEFAULT_PLATFORM="x86_64" ;;
    *)             DEFAULT_PLATFORM="generic" ;;
esac

TARGET_PLATFORM="${DEFAULT_PLATFORM}"
CUSTOM_TAG=""
NO_CACHE=""
PUSH=false

show_help() {
    cat <<EOF
Usage: docker/build.sh [OPTIONS]

选项:
  --platform <rk3588|x86_64>  目标平台（默认自动检测: ${DEFAULT_PLATFORM}）
  -t, --tag <tag>             自定义镜像标签（默认根据平台生成）
  --no-cache                  构建时禁用 Docker 缓存
  --push                      构建完成后推送到远程镜像仓库
  -h, --help                  显示此帮助信息

示例:
  bash docker/build.sh                         # 按当前机器架构构建
  bash docker/build.sh --platform rk3588       # 构建 RK3588 目标镜像
  bash docker/build.sh -t aisprayer:v1.0       # 自定义镜像 tag
EOF
    exit 0
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --platform)
            TARGET_PLATFORM="$2"; shift 2 ;;
        -t|--tag)
            CUSTOM_TAG="$2"; shift 2 ;;
        --no-cache)
            NO_CACHE="--no-cache"; shift ;;
        --push)
            PUSH=true; shift ;;
        -h|--help)
            show_help ;;
        *)
            error "未知参数: $1（使用 -h/--help 查看帮助）" ;;
    esac
done

section "AiSprayer Docker 镜像构建"
info "当前主机架构: ${HOST_ARCH}"
info "构建目标平台: ${TARGET_PLATFORM}"

# Check docker daemon
if ! docker info &>/dev/null; then
    error "Docker 守护进程未启动或当前用户无权访问 Docker。请确保 Docker 已安装并运行（sudo systemctl start docker）。"
fi

# Determine Docker build platform and default tag
case "$TARGET_PLATFORM" in
    rk3588|arm64|aarch64)
        DOCKER_PLATFORM="linux/arm64"
        DEFAULT_TAG="aisprayer:rk3588-latest"
        ;;
    x86_64|amd64)
        DOCKER_PLATFORM="linux/amd64"
        DEFAULT_TAG="aisprayer:x86-latest"
        ;;
    *)
        DOCKER_PLATFORM="linux/${HOST_ARCH}"
        DEFAULT_TAG="aisprayer:${TARGET_PLATFORM}-latest"
        ;;
esac

IMAGE_TAG="${CUSTOM_TAG:-$DEFAULT_TAG}"
info "目标 Docker 平台: ${DOCKER_PLATFORM}"
info "最终镜像标签:     ${IMAGE_TAG}"

# Check Git LFS models
section "检查模型大文件完整性"
LFS_WARNING=false
for m in models/*.rknn models/*.onnx; do
    if [[ -f "$m" && ! -L "$m" ]]; then
        fsize=$(wc -c < "$m" 2>/dev/null || echo 0)
        if [[ "$fsize" -lt 1000 ]]; then
            warn "发现疑似未拉取的 Git LFS 指针文件: $m (${fsize} bytes)"
            LFS_WARNING=true
        fi
    fi
done

if $LFS_WARNING; then
    warn "建议先执行 'git lfs pull' 拉取完整权重，否则打包进镜像的模型可能无法使用！"
else
    info "模型权重文件完整性检查通过。"
fi

section "准备独立分发包 (install/)"
if [[ ! -d "install" || ! -f "install/bin/orbbec_camera_service" || ! -f "install/lib/libmotion_c.so" ]]; then
    info "检测到 install/ 目录未就绪，正在自动调用 ./install.sh 生成分发包..."
    bash ./install.sh
else
    info "独立分发包 install/ 已就绪 ($(du -sh install | cut -f1))。"
fi

section "执行 Docker 构建"
BUILD_CMD=(
    docker build
    --platform "${DOCKER_PLATFORM}"
    -f docker/Dockerfile
    -t "${IMAGE_TAG}"
)

[[ -n "$NO_CACHE" ]] && BUILD_CMD+=("$NO_CACHE")
BUILD_CMD+=(".")

info "运行命令: ${BUILD_CMD[*]}"
"${BUILD_CMD[@]}"

section "构建完成"
info "镜像 ${IMAGE_TAG} 构建成功！"
docker images "${IMAGE_TAG}"

if $PUSH; then
    section "推送到镜像仓库"
    info "正在推送: ${IMAGE_TAG} ..."
    docker push "${IMAGE_TAG}"
    info "推送完成。"
fi

echo ""
echo "============================================================"
echo " 在 RK3588 板子上启动容器推荐命令:"
echo "   bash docker/run.sh -t ${IMAGE_TAG}"
echo "============================================================"
