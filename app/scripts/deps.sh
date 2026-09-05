#!/usr/bin/env bash
# =============================================================================
# app/scripts/deps.sh — AiSprayer 全平台系统、C++ 与应用依赖管理脚本
#
# 支持平台：
#   - rk3588    (Linux aarch64): apt 包 (MPP/RGA/DRM/OpenCV/Eigen/etc.) + small_gicp + env.sh
#   - linux_x86 (Linux x86_64) : apt 包 (nasm/OpenCV/Eigen/etc.) + small_gicp + env.sh
#   - macos     (Darwin arm64/x86_64): brew 包 (OpenCV/libomp/Eigen/etc.) + small_gicp + env.sh
#
# 用法：
#   bash app/scripts/deps.sh                      # 自动检测当前平台并安装全部依赖
#   bash app/scripts/deps.sh --platform rk3588    # 显式指定平台为 rk3588
#   bash app/scripts/deps.sh --platform linux_x86 # 显式指定平台为 linux_x86
#   bash app/scripts/deps.sh --platform macos     # 显式指定平台为 macos
#   bash app/scripts/deps.sh --system-only        # 仅安装系统 C++ 库与 vendored 源码
#   bash app/scripts/deps.sh --python-only        # 仅安装 Python 后端依赖
#   bash app/scripts/deps.sh --frontend-only      # 仅安装前端 Node.js 依赖
#   bash app/scripts/deps.sh --check-only         # 仅检测缺失项，不执行安装
# =============================================================================

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
APP_DIR="$(dirname "$SCRIPT_DIR")"
PROJECT_ROOT="$(dirname "$APP_DIR")"

# ── 输出辅助 ─────────────────────────────────────────────────────────────────
GREEN='\033[0;32m'; YELLOW='\033[1;33m'; RED='\033[0;31m'
CYAN='\033[0;36m'; BOLD='\033[1m'; NC='\033[0m'
info()    { echo -e "${GREEN}[INFO]${NC}  $*"; }
warn()    { echo -e "${YELLOW}[WARN]${NC}  $*"; }
error()   { echo -e "${RED}[ERROR]${NC} $*"; exit 1; }
section() { echo -e "\n${BOLD}${CYAN}━━━ $* ━━━${NC}"; }

# ── 参数解析 ─────────────────────────────────────────────────────────────────
PLATFORM=""
MODE="all" # all, system-only, python-only, frontend-only, check-only

while [[ $# -gt 0 ]]; do
    case "$1" in
        --platform)
            PLATFORM="$2"; shift 2 ;;
        --system-only)
            MODE="system-only"; shift ;;
        --python-only)
            MODE="python-only"; shift ;;
        --frontend-only)
            MODE="frontend-only"; shift ;;
        --check-only)
            MODE="check-only"; shift ;;
        -h|--help)
            cat <<'EOF'
Usage: app/scripts/deps.sh [OPTIONS]

选项:
  --platform <rk3588|linux_x86|macos>  显式覆盖目标平台（默认自动检测）
  --system-only                       仅检测并安装系统 C++ 库与第三方 vendored 源码
  --python-only                       仅安装 Python 虚拟环境与后端依赖
  --frontend-only                     仅安装前端 Node.js 依赖
  --check-only                        仅检测缺失依赖，不执行安装
  -h, --help                          显示本帮助信息
EOF
            exit 0
            ;;
        *)
            error "未知参数: $1（使用 --help 查看帮助）"
            ;;
    esac
done

# ── 平台自动检测 ─────────────────────────────────────────────────────────────
detect_platform() {
    local os arch
    os=$(uname -s)
    arch=$(uname -m)

    if [[ "$os" == "Darwin" ]]; then
        echo "macos"
    elif [[ "$os" == "Linux" ]]; then
        if [[ "$arch" == "aarch64" || "$arch" == "arm64" ]]; then
            echo "rk3588"
        else
            echo "linux_x86"
        fi
    else
        echo "unknown"
    fi
}

if [[ -z "$PLATFORM" ]]; then
    PLATFORM="$(detect_platform)"
fi

# 统一标准化 platform 命名
case "$PLATFORM" in
    rk3588|RK3588)
        PLATFORM="rk3588" ;;
    linux_x86|linux-x86|x86_64|x86)
        PLATFORM="linux_x86" ;;
    macos|Darwin|darwin|mac)
        PLATFORM="macos" ;;
    *)
        error "不支持或未知的平台: '$PLATFORM'（有效值: rk3588 | linux_x86 | macos）"
        ;;
esac

echo "=========================================="
echo " AiSprayer 跨平台依赖管理 (deps.sh)"
echo " 目标平台: ${PLATFORM} ($(uname -s) $(uname -m))"
echo " 执行模式: ${MODE}"
echo " 项目根目录: ${PROJECT_ROOT}"
echo "=========================================="

# ── 1. 系统 C++ 依赖安装 ─────────────────────────────────────────────────────
install_macos_deps() {
    section "macOS 系统 & C++ 依赖检查 (Homebrew)"

    if ! command -v brew &>/dev/null; then
        error "未检测到 Homebrew，请先安装: https://brew.sh/"
    fi

    # 1. 命令行基础工具检测
    local CLI_TOOLS=(git git-lfs cmake pkg-config)
    if ! command -v node &>/dev/null; then
        CLI_TOOLS+=(node)
    fi

    # 2. C++ 库 Formula 列表
    local BREW_FORMULAS=(
        git-lfs
        eigen
        yaml-cpp
        tinyxml2
        cli11
        opencv
        libomp
        googletest
        cpp-httplib
        openssl@3
    )

    if [[ "$(uname -m)" == "x86_64" ]]; then
        BREW_FORMULAS+=(nasm)
    fi

    local INSTALLED_FORMULAS
    INSTALLED_FORMULAS="$(brew list --formula 2>/dev/null || true)"

    local MISSING=()

    # 检查命令行工具
    for tool in ${CLI_TOOLS[@]+"${CLI_TOOLS[@]}"}; do
        if command -v "$tool" &>/dev/null; then
            echo "  ✔ [cli] $tool 已安装 ($(command -v "$tool"))"
        else
            echo "  ✘ [cli] $tool 未找到，将通过 brew 安装"
            MISSING+=("$tool")
        fi
    done

    # 检查 Homebrew C++ 库
    for formula in ${BREW_FORMULAS[@]+"${BREW_FORMULAS[@]}"}; do
        if echo "$INSTALLED_FORMULAS" | grep -qxF "$formula"; then
            echo "  ✔ [brew] $formula 已就绪"
        else
            echo "  ✘ [brew] $formula 未安装"
            MISSING+=("$formula")
        fi
    done

    if [[ ${#MISSING[@]} -gt 0 ]]; then
        if [[ "$MODE" == "check-only" ]]; then
            warn "发现缺失 Homebrew 依赖: ${MISSING[*]}"
            return 0
        fi
        info "正在安装缺失的 Homebrew 依赖: ${MISSING[*]} ..."
        brew install ${MISSING[@]+"${MISSING[@]}"}
        info "Homebrew 依赖安装完成。"
    else
        info "所有 macOS 系统 C++ 依赖均已满足。"
    fi
}

install_apt_deps() {
    local target_profile="$1"
    section "Linux 系统 & C++ 依赖检查 (APT / ${target_profile})"

    if ! command -v apt-get &>/dev/null; then
        warn "非 Debian/Ubuntu 系统，未找到 apt-get，跳过 apt 依赖自动安装。"
        return 0
    fi

    local SUDO=""
    if [[ $EUID -ne 0 ]]; then
        if command -v sudo &>/dev/null; then
            SUDO="sudo"
        else
            warn "当前非 root 用户且无 sudo 命令，安装系统包可能需要 root 权限。"
        fi
    fi

    local COMMON_PACKAGES=(
        build-essential
        cmake
        git
        git-lfs
        wget
        g++
        pkg-config
        python3-pip
        python3-dev
        python3-venv
        libssl-dev
        libsqlite3-dev
        libeigen3-dev
        libyaml-cpp-dev
        libtinyxml2-dev
        libcli11-dev
        libopencv-dev
        libomp-dev
        libgtest-dev
        libboost-all-dev
        nodejs
        npm
    )

    local PLATFORM_PACKAGES=()
    if [[ "$target_profile" == "rk3588" ]]; then
        # RK3588 专有硬件加速支持 (MPP / RGA / DRM)
        PLATFORM_PACKAGES=(
            libdrm-dev
            libopenblas-dev
        )
    elif [[ "$target_profile" == "linux_x86" ]]; then
        # Linux x86: OpenH264 SIMD 需要 nasm
        PLATFORM_PACKAGES=(
            nasm
        )
    fi

    local ALL_PACKAGES=(${COMMON_PACKAGES[@]+"${COMMON_PACKAGES[@]}"} ${PLATFORM_PACKAGES[@]+"${PLATFORM_PACKAGES[@]}"})
    local MISSING=()

    for pkg in ${ALL_PACKAGES[@]+"${ALL_PACKAGES[@]}"}; do
        if dpkg -s "$pkg" &>/dev/null; then
            echo "  ✔ [apt] $pkg 已安装"
        else
            echo "  ✘ [apt] $pkg 未安装"
            MISSING+=("$pkg")
        fi
    done

    # 针对 libhttplib-dev 特殊探测（部分旧版 Ubuntu apt 源无此包）
    if ! dpkg -s libhttplib-dev &>/dev/null; then
        if apt-cache show libhttplib-dev &>/dev/null; then
            MISSING+=(libhttplib-dev)
        fi
    fi

    if [[ ${#MISSING[@]} -gt 0 ]]; then
        if [[ "$MODE" == "check-only" ]]; then
            warn "发现缺失 APT 依赖: ${MISSING[*]}"
            return 0
        fi
        info "正在更新 apt 源并安装缺失系统依赖: ${MISSING[*]} ..."
        $SUDO apt-get update
        $SUDO apt-get install -y ${MISSING[@]+"${MISSING[@]}"}
        info "APT 系统依赖安装完成。"
    else
        info "所有 Linux (${target_profile}) 系统 C++ 依赖均已满足。"
    fi
}

# ── 2. Vendored C++ 代码仓库依赖 (全平台通用) ─────────────────────────────────
ensure_vendored_deps() {
    section "Vendored C++ 源码库检查 (small_gicp & httplib)"

    # 1. small_gicp (follow 算法核心依赖)
    local SMALL_GICP_DIR="${PROJECT_ROOT}/third_party/src/small_gicp"
    if [[ -f "${SMALL_GICP_DIR}/CMakeLists.txt" ]]; then
        echo "  ✔ small_gicp 源码已就绪: ${SMALL_GICP_DIR}"
    else
        echo "  ✘ small_gicp 缺失"
        if [[ "$MODE" != "check-only" ]]; then
            info "正在克隆 small_gicp (master 分支)..."
            mkdir -p "${PROJECT_ROOT}/third_party/src"
            git clone --depth 1 -b master https://github.com/koide3/small_gicp "${SMALL_GICP_DIR}"
            info "small_gicp 克隆完成。"
        fi
    fi

    # 2. httplib.h 单头文件保底（当系统包管理器未提供时）
    local HTTPLIB_FOUND=false
    for p in /usr/include/httplib.h /usr/local/include/httplib.h /opt/homebrew/include/httplib.h "${PROJECT_ROOT}/third_party/install/include/httplib.h"; do
        if [[ -f "$p" ]]; then
            HTTPLIB_FOUND=true
            echo "  ✔ httplib.h 头文件已就绪: $p"
            break
        fi
    done

    if ! $HTTPLIB_FOUND; then
        echo "  ✘ httplib.h 未在系统路径中找到"
        if [[ "$MODE" != "check-only" ]]; then
            info "正在下载单头文件 httplib.h 至 third_party/install/include/ ..."
            mkdir -p "${PROJECT_ROOT}/third_party/install/include"
            if command -v curl &>/dev/null; then
                curl -fsSL https://raw.githubusercontent.com/yhirose/cpp-httplib/v0.18.0/httplib.h -o "${PROJECT_ROOT}/third_party/install/include/httplib.h" || true
            elif command -v wget &>/dev/null; then
                wget -q https://raw.githubusercontent.com/yhirose/cpp-httplib/v0.18.0/httplib.h -O "${PROJECT_ROOT}/third_party/install/include/httplib.h" || true
            fi
            if [[ -f "${PROJECT_ROOT}/third_party/install/include/httplib.h" ]]; then
                info "httplib.h 保底下载成功。"
            else
                warn "httplib.h 下载失败，如后续编译 follow_health 报错请确认网络或安装 cpp-httplib。"
            fi
        fi
    fi
}

# ── 执行主流程 ───────────────────────────────────────────────────────────────
case "$MODE" in
    python-only)
        info "仅安装 Python 后端依赖..."
        bash "${SCRIPT_DIR}/env.sh"
        exit 0
        ;;
    frontend-only)
        info "仅安装前端依赖..."
        cd "${APP_DIR}/frontend"
        npm install --registry=https://registry.npmmirror.com
        exit 0
        ;;
esac

# 1. 安装系统包
case "$PLATFORM" in
    macos)
        install_macos_deps
        ;;
    rk3588)
        install_apt_deps "rk3588"
        ;;
    linux_x86)
        install_apt_deps "linux_x86"
        ;;
esac

# 2. 检查并补齐 vendored 源码
ensure_vendored_deps

# 3. 检查并拉取 Git LFS 模型权重 (RKNN / ONNX)
ensure_git_lfs_models() {
    section "Git LFS 与模型权重检查 (RKNN/ONNX)"

    if ! command -v git-lfs &>/dev/null; then
        warn "系统未找到 git-lfs，跳过 Git LFS 模型自动拉取。"
        return 0
    fi

    local LFS_PENDING=false
    for model_file in "${PROJECT_ROOT}/models"/*.rknn "${PROJECT_ROOT}/models"/*.onnx; do
        if [[ -f "$model_file" && ! -L "$model_file" ]]; then
            local fsize
            fsize=$(wc -c < "$model_file" 2>/dev/null || echo 0)
            if [[ "$fsize" -lt 1000 ]]; then
                if grep -q "version https://git-lfs" "$model_file" 2>/dev/null; then
                    LFS_PENDING=true
                    echo "  ✘ 发现未拉取的 LFS 指针文件: $(basename "$model_file") (${fsize} bytes)"
                fi
            fi
        fi
    done

    if $LFS_PENDING; then
        if [[ "$MODE" == "check-only" ]]; then
            warn "存在未拉取的 Git LFS 大文件模型，请执行 'git lfs pull'"
            return 0
        fi
        info "正在拉取 Git LFS 模型文件..."
        (cd "${PROJECT_ROOT}" && git lfs install --skip-repo 2>/dev/null || true; git lfs pull)
        info "Git LFS 模型文件拉取完成。"
    else
        echo "  ✔ Git LFS 模型权重已就绪"
    fi
}
ensure_git_lfs_models

# 4. 若为 all 模式，继续串联执行 env.sh 搭建 Python 与前端环境
if [[ "$MODE" == "all" ]]; then
    section "Python 后端与前端依赖环境管理"
    bash "${SCRIPT_DIR}/env.sh"
fi

section "完成"
info "AiSprayer 依赖部署检查全部完成！"
info "下一步编译可运行: ./app/scripts/build.sh"
info "启动全系统可运行: ./app/scripts/run.sh"
