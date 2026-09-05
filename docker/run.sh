#!/usr/bin/env bash
# =============================================================================
# docker/run.sh — AiSprayer 容器运行启动脚本（支持板载 RK3588 硬件加速透传）
#
# 在 RK3588 开发板上运行时，会自动透传:
#   - NPU 驱动与 DRM 渲染节点 (/dev/dri)
#   - 硬件视频编解码器 (/dev/mpp_service)
#   - 2D 图形硬件加速器 (/dev/rga)
#   - 奥比中光 3D 深度相机 USB 总线 (/dev/bus/usb)
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

# Default configuration
HOST_ARCH=$(uname -m)
CONTAINER_NAME="aisprayer"
case "$HOST_ARCH" in
    aarch64|arm64) DEFAULT_IMAGE="aisprayer:rk3588-latest" ;;
    *)             DEFAULT_IMAGE="aisprayer:x86-latest" ;;
esac

IMAGE_NAME="${DEFAULT_IMAGE}"
RUN_MODE="daemon"       # daemon | interactive
NETWORK_MODE="host"     # host | bridge
ACTION="run"            # run | stop | restart
EXTRA_ARGS=()

show_help() {
    cat <<EOF
Usage: docker/run.sh [OPTIONS] [-- <COMMAND>]

选项:
  -t, --tag, --image <name>   指定运行的镜像名称 (默认: ${DEFAULT_IMAGE})
  -n, --name <name>           指定容器名称 (默认: ${CONTAINER_NAME})
  -i, --interactive           以交互模式启动 bash 终端 (默认后台运行)
  --bridge                    使用 Docker 网桥模式端口映射 (默认使用 host 主机网络)
  --stop                      停止并删除正在运行的容器
  --restart                   重启容器
  -h, --help                  显示此帮助信息

示例:
  bash docker/run.sh                                    # 在后台启动 AiSprayer 全套服务
  bash docker/run.sh -i                                 # 进入容器进行交互调试
  bash docker/run.sh --stop                             # 停止容器
  bash docker/run.sh -t aisprayer:rk3588-v1.0           # 运行指定镜像
EOF
    exit 0
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        -t|--tag|--image)
            IMAGE_NAME="$2"; shift 2 ;;
        -n|--name)
            CONTAINER_NAME="$2"; shift 2 ;;
        -i|--interactive)
            RUN_MODE="interactive"; shift ;;
        --bridge)
            NETWORK_MODE="bridge"; shift ;;
        --stop)
            ACTION="stop"; shift ;;
        --restart)
            ACTION="restart"; shift ;;
        -h|--help)
            show_help ;;
        --)
            shift
            EXTRA_ARGS=("$@")
            break
            ;;
        *)
            EXTRA_ARGS+=("$1")
            shift
            ;;
    esac
done

if [[ "$ACTION" == "stop" ]]; then
    section "停止容器"
    if docker ps -a --format '{{.Names}}' | grep -Eq "^${CONTAINER_NAME}\$"; then
        info "正在停止并移除容器 ${CONTAINER_NAME}..."
        docker rm -f "${CONTAINER_NAME}" || true
        info "容器 ${CONTAINER_NAME} 已停止并移除。"
    else
        info "容器 ${CONTAINER_NAME} 未运行。"
    fi
    exit 0
fi

if [[ "$ACTION" == "restart" ]]; then
    section "重启容器"
    docker rm -f "${CONTAINER_NAME}" 2>/dev/null || true
fi

# Check docker daemon
if ! docker info &>/dev/null; then
    error "Docker 守护进程未启动。请先执行 sudo systemctl start docker"
fi

# Ensure image exists
if ! docker image inspect "${IMAGE_NAME}" &>/dev/null; then
    warn "本地未找到镜像: ${IMAGE_NAME}"
    echo "是否尝试自动构建该镜像? [y/N]"
    read -r ans
    if [[ "$ans" =~ ^[Yy]$ ]]; then
        bash docker/build.sh -t "${IMAGE_NAME}"
    else
        error "镜像不存在，已退出。"
    fi
fi

# Ensure host data directories exist
mkdir -p "${PROJECT_ROOT}/data/calib" \
         "${PROJECT_ROOT}/data/template_group" \
         "${PROJECT_ROOT}/configs" \
         "${PROJECT_ROOT}/app/logs" \
         "${PROJECT_ROOT}/app/data"

section "配置硬件透传与参数"

# Base docker run arguments
DOCKER_RUN_ARGS=(
    run
    --name "${CONTAINER_NAME}"
)

# Privileged mode for direct hardware access (USB, MPP, RGA, NPU)
DOCKER_RUN_ARGS+=(--privileged)

# Device mappings on Linux
if [[ "$HOST_ARCH" == "aarch64" || "$HOST_ARCH" == "arm64" ]]; then
    info "检测到 ARM64 架构，配置 RK3588 硬件加速设备透传:"
    if [[ -c /dev/mpp_service ]]; then
        DOCKER_RUN_ARGS+=(--device /dev/mpp_service:/dev/mpp_service)
        echo "  ✔ 透传 MPP 硬件编解码: /dev/mpp_service"
    fi
    if [[ -c /dev/rga ]]; then
        DOCKER_RUN_ARGS+=(--device /dev/rga:/dev/rga)
        echo "  ✔ 透传 RGA 2D 加速器:  /dev/rga"
    fi
    if [[ -d /dev/dri ]]; then
        DOCKER_RUN_ARGS+=(--device /dev/dri:/dev/dri)
        echo "  ✔ 透传 DRM 渲染节点:   /dev/dri"
    fi
    if [[ -d /dev/bus/usb ]]; then
        DOCKER_RUN_ARGS+=(-v /dev/bus/usb:/dev/bus/usb)
        echo "  ✔ 挂载 USB 总线设备:   /dev/bus/usb"
    fi
    DOCKER_RUN_ARGS+=(-v /dev:/dev -v /sys:/sys)
fi

# Volume persistence (data, configs, logs)
DOCKER_RUN_ARGS+=(
    -v "${PROJECT_ROOT}/data:/app/data"
    -v "${PROJECT_ROOT}/configs:/app/configs"
    -v "${PROJECT_ROOT}/app/logs:/app/app/logs"
)

# Network configuration
if [[ "$NETWORK_MODE" == "host" ]]; then
    info "网络模式: Host 模式（超低流媒体延迟，直通宿主机端口）"
    DOCKER_RUN_ARGS+=(--net=host)
else
    info "网络模式: Bridge 端口映射模式"
    DOCKER_RUN_ARGS+=(
        -p 8000:8000
        -p 5173:5173
        -p 18080:18080
        -p 8554:8554
        -p 8008:8008
        -p 1935:1935
    )
fi

# Clean up any existing container with same name
if docker ps -a --format '{{.Names}}' | grep -Eq "^${CONTAINER_NAME}\$"; then
    warn "容器 ${CONTAINER_NAME} 已存在，正在自动清理..."
    docker rm -f "${CONTAINER_NAME}" || true
fi

# Launch mode
if [[ "$RUN_MODE" == "interactive" ]]; then
    section "以交互模式启动容器"
    DOCKER_RUN_ARGS+=(-it --rm)
    if [[ ${#EXTRA_ARGS[@]} -eq 0 ]]; then
        EXTRA_ARGS=(/bin/bash)
    fi
    info "运行命令: docker ${DOCKER_RUN_ARGS[*]} ${IMAGE_NAME} ${EXTRA_ARGS[*]}"
    exec docker "${DOCKER_RUN_ARGS[@]}" "${IMAGE_NAME}" "${EXTRA_ARGS[@]}"
else
    section "以后台守护进程启动容器"
    DOCKER_RUN_ARGS+=(-d --restart unless-stopped)
    info "运行命令: docker ${DOCKER_RUN_ARGS[*]} ${IMAGE_NAME} ${EXTRA_ARGS[*]}"
    CONTAINER_ID=$(docker "${DOCKER_RUN_ARGS[@]}" "${IMAGE_NAME}" "${EXTRA_ARGS[@]}")
    info "容器启动成功！ID: ${CONTAINER_ID:0:12}"
    
    echo ""
    echo "============================================================"
    echo " AiSprayer 容器正在运行中 (${CONTAINER_NAME})"
    echo "   * 查看容器状态: docker ps -f name=${CONTAINER_NAME}"
    echo "   * 实时查看日志: docker logs -f ${CONTAINER_NAME}"
    echo "   * 停止服务容器: bash docker/run.sh --stop"
    echo "============================================================"
fi
