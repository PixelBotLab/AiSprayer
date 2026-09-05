#!/usr/bin/env bash
# =============================================================================
# install.sh — AiSprayer 独立分发安装包构建脚本
#
# 功能:
#   在项目根目录下生成可独立运行的 install/ 目录。
#   将 Python 代码、模型权重 (RKNN/ONNX)、脚本、配置文件、第三方依赖动态库、
#   C++ 编译产物 (orbbec_camera_service, libmotion_c) 统一分门别类归档，
#   既可直接在宿主机独立运行，也可作为整体直接打包进 Docker 镜像。
# =============================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="${SCRIPT_DIR}"
INSTALL_DIR="${PROJECT_ROOT}/install"

RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'; CYAN='\033[0;36m'; BOLD='\033[1m'; NC='\033[0m'
info()    { echo -e "${GREEN}[INFO]${NC}  $*"; }
warn()    { echo -e "${YELLOW}[WARN]${NC}  $*"; }
error()   { echo -e "${RED}[ERROR]${NC} $*"; exit 1; }
section() { echo -e "\n${BOLD}${CYAN}━━━ $* ━━━${NC}"; }

CLEAN=false
BUILD_MISSING=false

show_help() {
    cat <<EOF
Usage: ./install.sh [OPTIONS]

选项:
  -c, --clean           安装前清空现有的 install/ 目录
  -b, --build-missing   若发现 C++ 产物或前端缺失，自动调用编译脚本构建
  -h, --help            显示此帮助信息

示例:
  ./install.sh                # 快速打包生成 install/ 独立运行目录
  ./install.sh --clean        # 全新干净打包
EOF
    exit 0
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        -c|--clean) CLEAN=true; shift ;;
        -b|--build-missing) BUILD_MISSING=true; shift ;;
        -h|--help) show_help ;;
        *) error "未知参数: $1（使用 -h/--help 查看帮助）" ;;
    esac
done

section "1. 检查构建依赖与预编译产物"

# 1.1 检查 C++ 相机微服务
CAMERA_BIN="${PROJECT_ROOT}/app/src/core/hardware/camera/orbbec_camera_service/bin/orbbec_camera_service"
if [[ ! -f "${CAMERA_BIN}" ]]; then
    if $BUILD_MISSING; then
        info "未找到 C++ 相机服务，正在自动编译..."
        bash "${PROJECT_ROOT}/app/scripts/build.sh" --only camera
    else
        error "未找到 C++ 相机微服务: ${CAMERA_BIN}\n请先运行: bash app/scripts/build.sh --only camera 或使用 ./install.sh -b"
    fi
fi
info "✔ C++ 相机服务: ${CAMERA_BIN}"

# 1.2 检查 C++ 机械臂运动学动态库
MOTION_LIB=""
for candidate in \
    "${PROJECT_ROOT}/app/src/core/motion/bin/libmotion_c.so" \
    "${PROJECT_ROOT}/app/src/core/motion/build/libmotion_c.so"; do
    if [[ -f "${candidate}" ]]; then
        MOTION_LIB="${candidate}"
        break
    fi
done
if [[ -z "${MOTION_LIB}" ]]; then
    if $BUILD_MISSING; then
        info "未找到 libmotion_c.so，正在自动编译..."
        bash "${PROJECT_ROOT}/app/scripts/build.sh" --only motion
        MOTION_LIB="${PROJECT_ROOT}/app/src/core/motion/build/libmotion_c.so"
    else
        error "未找到 libmotion_c.so。请先运行: bash app/scripts/build.sh --only motion 或使用 ./install.sh -b"
    fi
fi
info "✔ C++ 运动学库: ${MOTION_LIB}"

# 1.3 检查 motion_cli
MOTION_CLI=""
for candidate in \
    "${PROJECT_ROOT}/app/src/core/motion/bin/motion_cli" \
    "${PROJECT_ROOT}/app/src/core/motion/build/motion_cli"; do
    if [[ -f "${candidate}" ]]; then
        MOTION_CLI="${candidate}"
        break
    fi
done

# 1.4 检查前端静态站点
FRONTEND_DIST="${PROJECT_ROOT}/app/frontend/dist"
if [[ ! -d "${FRONTEND_DIST}" || ! -f "${FRONTEND_DIST}/index.html" ]]; then
    warn "前端构建目录 ${FRONTEND_DIST} 不存在或不完整！"
else
    info "✔ Web 前端产物: ${FRONTEND_DIST}"
fi

# 1.5 检查第三方硬件库
RKNN_LIB="${PROJECT_ROOT}/third_party/install/lib/librknnrt.so"
if [[ ! -f "${RKNN_LIB}" ]]; then
    warn "未在 third_party/install/lib 中找到 librknnrt.so！"
fi

section "2. 准备 install 目录结构"
if $CLEAN && [[ -d "${INSTALL_DIR}" ]]; then
    info "正在清理旧的 install/ 目录..."
    rm -rf "${INSTALL_DIR}"
fi

mkdir -p "${INSTALL_DIR}"/{bin,lib,configs,models,tools,third_party,data/calib,data/template_group,logs,app/logs}
mkdir -p "${INSTALL_DIR}/app"/{src,urdf,scripts,frontend}

section "3. 分类同步归档文件"

# 3.1 C++ 可执行文件 (bin/)
info "归档可执行程序至 bin/ ..."
cp -p "${CAMERA_BIN}" "${INSTALL_DIR}/bin/orbbec_camera_service"
chmod +x "${INSTALL_DIR}/bin/orbbec_camera_service"

if [[ -n "${MOTION_CLI}" && -f "${MOTION_CLI}" ]]; then
    cp -p "${MOTION_CLI}" "${INSTALL_DIR}/bin/motion_cli"
    chmod +x "${INSTALL_DIR}/bin/motion_cli"
fi

# 3.2 C++ 动态链接库 (lib/) — 仅运行时 .so，严禁打包 .a 静态库与源码头文件
info "归档编译完成的动态共享库 (.so) 至 lib/ ..."
cp -p "${MOTION_LIB}" "${INSTALL_DIR}/lib/libmotion_c.so"

if [[ -d "${PROJECT_ROOT}/third_party/install/lib" ]]; then
    cp -d "${PROJECT_ROOT}/third_party/install/lib"/*.so* "${INSTALL_DIR}/lib/" 2>/dev/null || true
    # 同时在 third_party/install/lib 仅放置 .so 动态库供历史兼容，不包含任何头文件
    mkdir -p "${INSTALL_DIR}/third_party/install/lib"
    cp -d "${PROJECT_ROOT}/third_party/install/lib"/*.so* "${INSTALL_DIR}/third_party/install/lib/" 2>/dev/null || true
fi

# 3.3 第三方 Python Wheels (third_party/) — 仅打包 Python 安装包，不包含 C++ 源码和 include 头文件
if compgen -G "${PROJECT_ROOT}/third_party/*.whl" > /dev/null; then
    info "归档第三方 Python Wheels 至 third_party/ ..."
    cp -p "${PROJECT_ROOT}"/third_party/*.whl "${INSTALL_DIR}/third_party/"
fi

# 3.4 纯 Python 业务代码 (app/src/) — 严格剔除 C++ 源码、生成物、头文件、src 目录与 tests
info "归档核心 Python 代码至 app/src/ (剔除 C++ 源码树、中间物、头文件与单测) ..."
rsync -a --delete \
    --exclude="__pycache__" \
    --exclude="*.pyc" \
    --exclude="*.pyo" \
    --exclude="*.pyd" \
    --exclude=".pytest_cache" \
    --exclude="*.cpp" \
    --exclude="*.hpp" \
    --exclude="*.c" \
    --exclude="*.h" \
    --exclude="*.cc" \
    --exclude="*.o" \
    --exclude="*.a" \
    --exclude="CMakeLists.txt" \
    --exclude="*.cmake" \
    --exclude="Makefile" \
    --exclude="build" \
    --exclude="build_*" \
    --exclude="CMakeFiles" \
    --exclude="CMakeCache.txt" \
    --exclude="cmake_install.cmake" \
    --exclude="docs" \
    --exclude="doc" \
    --exclude="*.md" \
    --exclude="tests" \
    --exclude="test" \
    --exclude="test_*.py" \
    --exclude="*.test" \
    --exclude="Log" \
    --exclude="logs" \
    --exclude="*.log" \
    --exclude="out" \
    --exclude="core/hardware/camera/orbbec_camera_service" \
    --exclude="core/planner" \
    --exclude="core/follow" \
    --exclude="core/visioncpp" \
    "${PROJECT_ROOT}/app/src/" "${INSTALL_DIR}/app/src/"

# 清理 core/motion: 仅保留 kinematics.py, cli_client.py, __init__.py，彻底移除其 src、include、bin、scripts
rm -rf "${INSTALL_DIR}/app/src/core/motion/src" \
       "${INSTALL_DIR}/app/src/core/motion/include" \
       "${INSTALL_DIR}/app/src/core/motion/scripts" \
       "${INSTALL_DIR}/app/src/core/motion/bin" \
       "${INSTALL_DIR}/app/src/core/motion/build" \
       "${INSTALL_DIR}/app/src/core/motion/tests" \
       "${INSTALL_DIR}/app/src/core/motion/docs" 2>/dev/null || true

# 3.5 机器人资产与脚本 (app/urdf/, app/scripts/)
info "归档机器人资产与脚本至 app/ ..."
if [[ -d "${PROJECT_ROOT}/app/urdf" ]]; then
    rsync -a --delete "${PROJECT_ROOT}/app/urdf/" "${INSTALL_DIR}/app/urdf/"
fi
if [[ -d "${PROJECT_ROOT}/app/scripts" ]]; then
    rsync -a --delete \
        --exclude="build.sh" \
        --exclude="deps.sh" \
        "${PROJECT_ROOT}/app/scripts/" "${INSTALL_DIR}/app/scripts/"
fi

# 3.6 前端静态资源 (app/frontend/dist/)
if [[ -d "${FRONTEND_DIST}" ]]; then
    info "归档 Web 前端产物至 app/frontend/dist/ ..."
    rsync -a --delete "${FRONTEND_DIST}/" "${INSTALL_DIR}/app/frontend/dist/"
fi

# 3.7 配置文件 (configs/)
info "归档系统配置至 configs/ ..."
rsync -a --delete "${PROJECT_ROOT}/configs/" "${INSTALL_DIR}/configs/"

# 3.8 独立工具脚本 (tools/)
if [[ -d "${PROJECT_ROOT}/tools" ]]; then
    info "归档工具脚本至 tools/ ..."
    rsync -a --delete \
        --exclude="__pycache__" \
        --exclude="*.pyc" \
        "${PROJECT_ROOT}/tools/" "${INSTALL_DIR}/tools/"
fi

# 3.9 NPU / ONNX 模型权重 (models/，完全剔除 PyTorch .pt 权重)
info "归档 NPU (RKNN) 与 ONNX 模型权重至 models/ ..."
for model in "${PROJECT_ROOT}"/models/*.rknn "${PROJECT_ROOT}"/models/*.onnx; do
    if [[ -e "$model" ]]; then
        cp -d "$model" "${INSTALL_DIR}/models/"
    fi
done

section "4. 生成独立运行环境与启动脚本"

# 4.1 生成环境变量加载脚本 env.sh
cat > "${INSTALL_DIR}/env.sh" << 'EOF'
#!/usr/bin/env bash
# =============================================================================
# AiSprayer 独立运行环境配置脚本
# 用法: source env.sh
# =============================================================================
INSTALL_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export INSTALL_ROOT

export PATH="${INSTALL_ROOT}/bin:${PATH}"
export PYTHONPATH="${INSTALL_ROOT}/app/src:${INSTALL_ROOT}/tools:${PYTHONPATH:-}"
export LD_LIBRARY_PATH="${INSTALL_ROOT}/lib:${INSTALL_ROOT}/third_party/install/lib:/usr/local/lib:/usr/lib:${LD_LIBRARY_PATH:-}"
export PYTHONUNBUFFERED=1

# 默认绑定单线程降低多核竞争
for _name in "OMP_NUM_THREADS" "OPENBLAS_NUM_THREADS" "MKL_NUM_THREADS" "NUMEXPR_NUM_THREADS" "OPENCV_NUM_THREADS"; do
    export ${_name}=1
done

echo "[AiSprayer] 独立运行环境已配置:"
echo "  INSTALL_ROOT:    ${INSTALL_ROOT}"
echo "  PYTHONPATH:      ${PYTHONPATH}"
echo "  LD_LIBRARY_PATH: ${LD_LIBRARY_PATH}"
EOF
chmod +x "${INSTALL_DIR}/env.sh"

# 4.2 生成一键启动脚本 run.sh
cat > "${INSTALL_DIR}/run.sh" << 'EOF'
#!/usr/bin/env bash
# =============================================================================
# AiSprayer 独立运行启动脚本
# =============================================================================
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 加载独立环境
source "${SCRIPT_DIR}/env.sh"

cd "${SCRIPT_DIR}"

echo "============================================================"
echo " AiSprayer Starting (Standalone Mode)..."
echo " Time: $(date)"
echo "============================================================"

# 清理遗留锁与孤儿进程
rm -f "${SCRIPT_DIR}/.orbbec.lock" 2>/dev/null || true
pkill -9 -x orbbec_camera_service 2>/dev/null || true

# 启动主服务
exec python3 "${SCRIPT_DIR}/app/src/main.py" "$@"
EOF
chmod +x "${INSTALL_DIR}/run.sh"

section "5. 归档与校验完成"

# 严格检查确保不含任何 C/C++ 源码、头文件、构建描述文件或测试文件
RESIDUAL_FILES=$(find "${INSTALL_DIR}" -type f \( -name "*.cpp" -o -name "*.hpp" -o -name "*.c" -o -name "*.h" -o -name "CMakeLists.txt" -o -name "*.cmake" -o -name "test_*.py" -o -name "*.o" -o -name "*.a" \) | wc -l)
if [[ "${RESIDUAL_FILES}" -eq 0 ]]; then
    info "✔ 源码、头文件、中间件及测试用例过滤检查通过: 0 个残留文件"
else
    warn "发现 ${RESIDUAL_FILES} 个残留文件，正在自动清理..."
    find "${INSTALL_DIR}" -type f \( -name "*.cpp" -o -name "*.hpp" -o -name "*.c" -o -name "*.h" -o -name "CMakeLists.txt" -o -name "*.cmake" -o -name "test_*.py" -o -name "*.o" -o -name "*.a" \) -delete
    find "${INSTALL_DIR}" -type d \( -name "tests" -o -name "test" -o -name "docs" -o -name "doc" -o -name "include" -o -name "CMakeFiles" \) -exec rm -rf {} + 2>/dev/null || true
    info "✔ 清理完毕，无关文件已彻底剔除。"
fi

TOTAL_SIZE=$(du -sh "${INSTALL_DIR}" | cut -f1)
info "install 目录生成完毕！总大小: ${BOLD}${TOTAL_SIZE}${NC}"
echo ""
echo "目录结构概览:"
ls -lh "${INSTALL_DIR}"
echo ""
echo "============================================================"
echo " 宿主机独立运行测试方法:"
echo "   cd ${INSTALL_DIR}"
echo "   ./run.sh"
echo ""
echo " Docker 镜像打包方法:"
echo "   bash docker/build.sh --platform rk3588"
echo "============================================================"
