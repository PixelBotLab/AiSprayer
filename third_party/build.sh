#!/usr/bin/env bash
# =============================================================================
# third_party/build.sh — 在项目内从源码构建所有三方依赖
#
# 目录结构（均在 third_party/ 下，不污染系统路径）：
#   src/        各库源码（git clone 至此）
#   install/    统一安装前缀，供 CMakeLists.txt 使用
#     include/
#     lib/       librockchip_mpp.a (静态)
#                librga.a          (静态)
#                librknnrt.so      (动态 — Rockchip 仅提供预编译 .so)
#                libmk_api.so      (动态 — ZLMediaKit C API fat .so)
#                libOrbbecSDK.so   (动态 — Orbbec SDK v2 官方 C++ 动态库)
#
# 用法：
#   bash third_party/build.sh            # 全量构建（已存在的库跳过）
#   bash third_party/build.sh --force    # 强制全部重新构建
#   bash third_party/build.sh --jobs 8   # 指定并行数
#   bash third_party/build.sh --only mpp rga orbbec   # 只构建指定库
#
# 静态 / 动态说明：
#   MPP    → 静态（librockchip_mpp.a），链接到可执行文件内
#   RGA    → 静态（librga.a），链接到可执行文件内
#   RKNN   → 动态（librknnrt.so），Rockchip 仅提供预编译二进制，无法静态
#   ZLMK   → 动态（libmk_api.so），内嵌全部 ZLMedia 代码，自包含 fat .so
#   Orbbec → 动态（libOrbbecSDK.so），Orbbec SDK v2 官方 C++ 动态库
#
# 运行要求：
#   - 无需 root（安装到项目内，不写系统路径）
#   - 需要 git cmake make gcc g++ libssl-dev libdrm-dev
# =============================================================================

set -euo pipefail

# ── 颜色输出 ─────────────────────────────────────────────────────────────────
RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'
CYAN='\033[0;36m'; BOLD='\033[1m'; NC='\033[0m'
info()    { echo -e "${GREEN}[INFO]${NC}  $*"; }
warn()    { echo -e "${YELLOW}[WARN]${NC}  $*"; }
error()   { echo -e "${RED}[ERROR]${NC} $*"; exit 1; }
section() { echo -e "\n${BOLD}${CYAN}━━━ $* ━━━${NC}"; }

# ── 路径设置 ─────────────────────────────────────────────────────────────────
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SRC_DIR="${SCRIPT_DIR}/src"
INSTALL_DIR="${SCRIPT_DIR}/install"
JOBS=$(nproc 2>/dev/null || sysctl -n hw.ncpu 2>/dev/null || echo 4)

# ── 参数解析 ─────────────────────────────────────────────────────────────────
FORCE=false
ONLY_LIBS=()  # 若非空，只构建指定的库
PROFILE=""

while [[ $# -gt 0 ]]; do
    case "$1" in
        --force)  FORCE=true; shift ;;
        --jobs)   JOBS="$2"; shift 2 ;;
        --profile)
            PROFILE="$2"; shift 2
            ;;
        --only)
            shift
            while [[ $# -gt 0 && "$1" != --* ]]; do
                ONLY_LIBS+=("$1"); shift
            done
            ;;
        -h|--help)
            sed -n '2,30p' "$0" | grep '^#' | sed 's/^# \{0,2\}//'
            echo "  --profile rk3588|generic   默认按主机自动选择"
            exit 0
            ;;
        *) error "未知参数: $1（使用 --help 查看帮助）" ;;
    esac
done

if [[ -z "$PROFILE" ]]; then
    _os=$(uname -s | tr '[:upper:]' '[:lower:]')
    _arch=$(uname -m)
    if [[ "$_os" == "linux" && "$_arch" == "aarch64" ]]; then
        PROFILE=rk3588
    else
        PROFILE=generic
    fi
fi
info "构建配置: profile=${PROFILE}"

should_build() {
    local name="$1"
    if [[ ${#ONLY_LIBS[@]} -gt 0 ]]; then
        for lib in "${ONLY_LIBS[@]}"; do
            [[ "$lib" == "$name" ]] && return 0
        done
        return 1
    fi
    # Faiss 全工程引用计数为 0，不再默认构建；仅在 --only faiss 时构建
    if [[ "$name" == "faiss" ]]; then
        return 1
    fi
    if [[ "$PROFILE" == "generic" ]]; then
        case "$name" in
            rga|rknn|mpp) return 1 ;;
        esac
    fi
    if [[ "$PROFILE" == "rk3588" && "$name" == "openh264" ]]; then
        return 1
    fi
    return 0
}

# ── 前置检查 ─────────────────────────────────────────────────────────────────
section "环境检查"
for cmd in git cmake make; do
    command -v "$cmd" &>/dev/null || error "缺少必要工具: $cmd"
done
if ! command -v g++ &>/dev/null && ! command -v clang++ &>/dev/null; then
    error "缺少 C++ 编译器 (g++ 或 clang++)"
fi
if ! command -v pkg-config &>/dev/null; then
    warn "未找到 pkg-config，部分依赖探测会跳过"
fi

if [[ "$PROFILE" == "rk3588" ]]; then
    if ! pkg-config --exists libdrm 2>/dev/null; then
        warn "未找到 libdrm，MPP 编译可能失败"
        warn "请运行: sudo apt install libdrm-dev"
    fi
    if ! pkg-config --exists openssl 2>/dev/null; then
        warn "未找到 OpenSSL，ZLMediaKit 编译可能失败"
        warn "请运行: sudo apt install libssl-dev"
    fi
    if command -v dpkg >/dev/null 2>&1; then
        if ! dpkg -s libopenblas-dev &>/dev/null 2>&1; then
            warn "未找到 libopenblas-dev，Faiss 编译可能失败"
            warn "请运行: sudo apt install libopenblas-dev"
        fi
        if ! dpkg -s libsqlite3-dev &>/dev/null 2>&1; then
            warn "未找到 libsqlite3-dev，EventLog 编译可能失败"
            warn "请运行: sudo apt install libsqlite3-dev"
        fi
    fi

    info "正在探测底层硬件驱动节点（若当前为异构交叉编译环境，请忽略以下警告）..."
    for dev_node in "/dev/rga" "/dev/mpp_service"; do
        if [[ ! -e "$dev_node" ]]; then
            warn "未检测到硬件节点 $dev_node ！如果本机是最终运行环境，程序启动时会由于找不到硬件支持而崩溃。"
        else
            [[ -w "$dev_node" ]] || warn "当前用户没有读写 $dev_node 的权限！若以此用户运行主程序，可能会触发 Permission denied 报错。"
        fi
    done

    if [[ ! -e "/dev/rknpu" ]] && [[ ! -e "/dev/galcore" ]]; then
        if ! sudo -n ls "/sys/kernel/debug/rknpu/version" &>/dev/null && ! ls "/sys/kernel/debug/rknpu" &>/dev/null 2>&1; then
            warn "未检测到 NPU 设备节点(rknpu/galcore) 及初始化日志，神经计算驱动可能未加载或因型号不匹配而不可用。"
        fi
    fi

    if [[ ! -d "/dev/dma_heap" ]] && [[ ! -e "/dev/ion" ]]; then
        warn "未检测到 dma_heap 或 ion 内存分配器节点，MPP 与 RGA 间的高效数据流转池可能无法初始化。"
    fi
else
    if ! pkg-config --exists openssl 2>/dev/null; then
        warn "未找到 OpenSSL，ZLMediaKit 编译可能失败（macOS: brew install openssl）"
    fi
fi

mkdir -p "${SRC_DIR}" "${INSTALL_DIR}/include" "${INSTALL_DIR}/lib"

info "源码目录: ${SRC_DIR}"
info "安装目录: ${INSTALL_DIR}"
info "并行编译: -j${JOBS}"
[[ "$FORCE" == true ]] && warn "强制重新构建模式"

# ── 辅助函数 ─────────────────────────────────────────────────────────────────

# 检查构建产物是否已存在，FORCE=true 时始终返回"需要构建"
need_build() {
    local lib_file="$1"
    if [[ "$FORCE" == true ]]; then return 0; fi
    if [[ -f "${INSTALL_DIR}/lib/${lib_file}" ]]; then
        info "已存在: ${lib_file}（跳过，使用 --force 强制重新构建）"
        return 1
    fi
    if [[ "$lib_file" == *.so ]]; then
        local alt="${lib_file%.so}.dylib"
        if [[ -f "${INSTALL_DIR}/lib/${alt}" ]]; then
            info "已存在: ${alt}（跳过，使用 --force 强制重新构建）"
            return 1
        fi
    fi
    # openh264 may install as versioned .so / .dylib / .a
    if [[ "$lib_file" == libopenh264.* ]]; then
        if ls "${INSTALL_DIR}/lib"/libopenh264.* >/dev/null 2>&1; then
            info "已存在: libopenh264*（跳过，使用 --force 强制重新构建）"
            return 1
        fi
    fi
    return 0
}

clone_or_update() {
    local name="$1" url="$2" extra_args="${3:-}"
    if [[ -d "${SRC_DIR}/${name}/.git" || -d "${SRC_DIR}/${name}" ]]; then
        info "源码已存在，跳过克隆: ${name}"
    else
        info "克隆 ${name} ..."
        # shellcheck disable=SC2086
        git clone --depth=1 ${extra_args} "${url}" "${SRC_DIR}/${name}"
    fi
}

# ── 1. MPP（使用系统自带的mpp库，暂时不编译了）────────────────────────────────────────────────────────────
#section "MPP — Rockchip Media Process Platform（动态库）"
#
#if should_build mpp && need_build "librockchip_mpp.so"; then
#    clone_or_update mpp "https://github.com/rockchip-linux/mpp.git"
#
#    cmake -S "${SRC_DIR}/mpp" \
#          -B "${SRC_DIR}/mpp/build" \
#          -DRKPLATFORM=ON \
#          -DHAVE_DRM=ON \
#          -DBUILD_SHARED_LIBS=ON \
#          -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
#          -DCMAKE_BUILD_TYPE=Release \
#          -DCMAKE_INSTALL_PREFIX="${INSTALL_DIR}"
#
#    cmake --build  "${SRC_DIR}/mpp/build" -j"${JOBS}"
#    cmake --install "${SRC_DIR}/mpp/build"
#
#    # MPP 安装后头文件在 include/rockchip/，保持这个结构
#    info "MPP 动态库构建完成 → ${INSTALL_DIR}/lib/librockchip_mpp.so"
#else
#    should_build mpp || info "MPP 已跳过（--only 未指定）"
#fi

# ── 2. RGA（静态）────────────────────────────────────────────────────────────
section "RGA — Rockchip 2D Graphics Accelerator（动态库）"

if should_build rga && need_build "librga.so"; then
    RGA_TAR="${SCRIPT_DIR}/librga_1.10.5.tar.gz"
    if [[ -d "${SRC_DIR}/rga" ]]; then
        info "RGA 源码已存在，跳过解压"
    elif [[ -f "${RGA_TAR}" ]]; then
        info "正在从本地压缩包解压 RGA: $(basename "${RGA_TAR}")"
        mkdir -p "${SRC_DIR}/rga"
        tar -xzf "${RGA_TAR}" -C "${SRC_DIR}/rga" --strip-components=1
    else
        error "未找到本地 RGA 压缩包: ${RGA_TAR}，且源码不存在。请检查 ${SCRIPT_DIR} 下是否存在该文件。"
    fi

    rm -rf "${SRC_DIR}/rga/build"
    cmake -S "${SRC_DIR}/rga" \
          -B "${SRC_DIR}/rga/build" \
          -DCMAKE_BUILD_TARGET=buildroot \
          -DBUILD_SHARED_LIBS=ON \
          -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
          -DCMAKE_BUILD_TYPE=Release \
          -DCMAKE_INSTALL_PREFIX="${INSTALL_DIR}"

    cmake --build  "${SRC_DIR}/rga/build" -j"${JOBS}"
    cmake --install "${SRC_DIR}/rga/build"

    # RGA 有些版本安装头文件在 include/，这里确保它们在 include/rga/（符合项目 #include <rga/rga.h> 要求）
    mkdir -p "${INSTALL_DIR}/include/rga"
    cp -rf "${SRC_DIR}/rga/include/"*.h "${INSTALL_DIR}/include/rga/" 2>/dev/null || true
    if [[ -d "${SRC_DIR}/rga/im2d_api" ]]; then
        cp -rf "${SRC_DIR}/rga/im2d_api/"*.h "${INSTALL_DIR}/include/rga/" 2>/dev/null || true
        cp -rf "${SRC_DIR}/rga/im2d_api/"*.hpp "${INSTALL_DIR}/include/rga/" 2>/dev/null || true
    fi

    info "RGA 动态库构建完成 → ${INSTALL_DIR}/lib/librga.so"
else
    should_build rga || info "RGA 已跳过（--only 未指定）"
fi

# ── 3. RKNN Runtime（预编译 .so）─────────────────────────────────────────────
section "RKNN Runtime（预编译动态库，无法静态）"

if should_build rknn && need_build "librknnrt.so"; then
    RKNN_REPO="https://github.com/airockchip/rknn-toolkit2.git"
    RKNN_RUNTIME_SUBDIR="rknpu2/runtime/Linux/librknn_api"

    if [[ ! -d "${SRC_DIR}/rknn-toolkit2/.git" ]]; then
        info "稀疏克隆 rknn-toolkit2（只拉取 runtime 目录）..."
        git clone --depth=1 \
                  --filter=blob:none \
                  --sparse \
                  "${RKNN_REPO}" \
                  "${SRC_DIR}/rknn-toolkit2"
        git -C "${SRC_DIR}/rknn-toolkit2" \
            sparse-checkout set "${RKNN_RUNTIME_SUBDIR}"
    else
        info "rknn-toolkit2 源码已存在，跳过克隆"
    fi

    RKNN_LIB="${SRC_DIR}/rknn-toolkit2/${RKNN_RUNTIME_SUBDIR}/aarch64/librknnrt.so"
    RKNN_HDR="${SRC_DIR}/rknn-toolkit2/${RKNN_RUNTIME_SUBDIR}/include/rknn_api.h"

    [[ -f "${RKNN_LIB}" ]] || error "未找到 librknnrt.so（路径: ${RKNN_LIB}）"
    [[ -f "${RKNN_HDR}" ]] || error "未找到 rknn_api.h（路径: ${RKNN_HDR}）"

    install -Dm755 "${RKNN_LIB}" "${INSTALL_DIR}/lib/librknnrt.so"
    install -Dm644 "${RKNN_HDR}" "${INSTALL_DIR}/include/rknn_api.h"

    MATMUL_HDR="${SRC_DIR}/rknn-toolkit2/${RKNN_RUNTIME_SUBDIR}/include/rknn_matmul_api.h"
    [[ -f "${MATMUL_HDR}" ]] && \
        install -Dm644 "${MATMUL_HDR}" "${INSTALL_DIR}/include/rknn_matmul_api.h"

    info "RKNN Runtime 安装完成 → ${INSTALL_DIR}/lib/librknnrt.so"
    warn "⚠ librknnrt.so 是动态库，部署时需随可执行文件一起分发"
else
    should_build rknn || info "RKNN 已跳过（--only 未指定）"
fi

# ── 4. ZLMediaKit（fat .so，自包含全部 ZLMedia 代码）────────────────────────
section "ZLMediaKit（动态 fat .so，自包含 ZLMedia 全部代码）"

if should_build zlmk && need_build "libmk_api.so"; then
    if [[ ! -d "${SRC_DIR}/ZLMediaKit/.git" ]]; then
        info "克隆 ZLMediaKit..."
        git clone --depth=1 \
                  "https://github.com/ZLMediaKit/ZLMediaKit.git" \
                  "${SRC_DIR}/ZLMediaKit"
        git -C "${SRC_DIR}/ZLMediaKit" \
            submodule update --init --depth=1
    else
        info "ZLMediaKit 源码已存在，跳过克隆"
    fi

    rm -rf "${SRC_DIR}/ZLMediaKit/build"
    ZLM_CMAKE_EXTRA=()
    if [[ "$(uname -s)" == "Darwin" ]]; then
        for _ssl in /opt/homebrew/opt/openssl /usr/local/opt/openssl; do
            if [[ -d "$_ssl" ]]; then
                ZLM_CMAKE_EXTRA+=(-DOPENSSL_ROOT_DIR="$_ssl")
                break
            fi
        done
    fi
    cmake -S "${SRC_DIR}/ZLMediaKit" \
          -B "${SRC_DIR}/ZLMediaKit/build" \
          -DENABLE_WEBRTC=OFF \
          -DENABLE_FFMPEG=OFF \
          -DENABLE_TESTS=OFF \
          -DENABLE_PLAYER=OFF \
          -DENABLE_SERVER=ON \
          -DENABLE_API=ON \
          -DCMAKE_BUILD_TYPE=Release \
          -DCMAKE_INSTALL_PREFIX="${INSTALL_DIR}" \
          ${ZLM_CMAKE_EXTRA[@]+"${ZLM_CMAKE_EXTRA[@]}"}

    #cmake --build  "${SRC_DIR}/ZLMediaKit/build" -j"${JOBS}" --target mk_api
    cmake --build  "${SRC_DIR}/ZLMediaKit/build" -j"${JOBS}"
    cmake --install "${SRC_DIR}/ZLMediaKit/build"

    # 确保 mk_mediakit.h 已安装到 include/
    mkdir -p "${INSTALL_DIR}/include"
    ZLMK_HDR="${SRC_DIR}/ZLMediaKit/api/include/mk_mediakit.h"
    if [[ ! -f "${INSTALL_DIR}/include/mk_mediakit.h" && -f "${ZLMK_HDR}" ]]; then
        cp "${ZLMK_HDR}" "${INSTALL_DIR}/include/mk_mediakit.h"
    fi
    for hdr in "${SRC_DIR}/ZLMediaKit/api/include/"*.h; do
        dest="${INSTALL_DIR}/include/$(basename "${hdr}")"
        [[ -f "${dest}" ]] || cp "${hdr}" "${dest}"
    done

    info "ZLMediaKit 构建完成 → ${INSTALL_DIR}/lib/libmk_api.so"
    warn "⚠ libmk_api.so 是动态库，部署时需随可执行文件一起分发"
else
    should_build zlmk || info "ZLMK 已跳过（--only 未指定）"
fi

# ── 5. Faiss v1.8.0（静态，CPU/ARM64，无 GPU/Python）──────────────────────────
section "Faiss v1.8.0（静态库，ReID 特征库）"

if should_build faiss && need_build "libfaiss.a"; then
    FAISS_TAG="v1.8.0"

    if [[ ! -d "${SRC_DIR}/faiss/.git" ]]; then
        info "克隆 Faiss ${FAISS_TAG} ..."
        git clone --depth=1 --branch "${FAISS_TAG}" \
                  "https://github.com/facebookresearch/faiss.git" \
                  "${SRC_DIR}/faiss"
    else
        info "Faiss 源码已存在，跳过克隆"
    fi

    # 查找 OpenBLAS 头文件和库（系统包 libopenblas-dev 提供）
    OPENBLAS_INC=$(dpkg -L libopenblas-dev 2>/dev/null | grep "cblas.h" | head -1 | xargs dirname || echo "/usr/include")
    OPENBLAS_LIB=$(find /usr/lib /usr/lib/aarch64-linux-gnu -name "libopenblas.so*" 2>/dev/null | head -1 | xargs dirname || echo "/usr/lib/aarch64-linux-gnu")

    rm -rf "${SRC_DIR}/faiss/build"
    cmake -S "${SRC_DIR}/faiss" \
          -B "${SRC_DIR}/faiss/build" \
          -DFAISS_ENABLE_GPU=OFF \
          -DFAISS_ENABLE_PYTHON=OFF \
          -DBUILD_TESTING=OFF \
          -DBUILD_SHARED_LIBS=OFF \
          -DFAISS_OPT_LEVEL=generic \
          -DBLA_VENDOR=OpenBLAS \
          -DCMAKE_BUILD_TYPE=Release \
          -DCMAKE_INSTALL_PREFIX="${INSTALL_DIR}" \
          -DCMAKE_CXX_FLAGS="-I${OPENBLAS_INC}" \
          -DCMAKE_EXE_LINKER_FLAGS="-L${OPENBLAS_LIB}"

    cmake --build  "${SRC_DIR}/faiss/build" -j"${JOBS}" --target faiss
    cmake --install "${SRC_DIR}/faiss/build" --component faiss_headers
    # 手动复制静态库（install target 有时包含额外组件，只需要 libfaiss.a）
    install -Dm644 "${SRC_DIR}/faiss/build/faiss/libfaiss.a" "${INSTALL_DIR}/lib/libfaiss.a"
    # 头文件
    mkdir -p "${INSTALL_DIR}/include/faiss"
    cp -r "${SRC_DIR}/faiss/faiss/"*.h "${INSTALL_DIR}/include/faiss/"
    [[ -d "${SRC_DIR}/faiss/faiss/impl" ]] && \
        cp -r "${SRC_DIR}/faiss/faiss/impl" "${INSTALL_DIR}/include/faiss/"
    [[ -d "${SRC_DIR}/faiss/faiss/utils" ]] && \
        cp -r "${SRC_DIR}/faiss/faiss/utils" "${INSTALL_DIR}/include/faiss/"

    info "Faiss 静态库构建完成 → ${INSTALL_DIR}/lib/libfaiss.a"
    info "运行时依赖（需系统已安装）：libopenblas.so"
else
    should_build faiss || info "Faiss 已跳过（--only 未指定）"
fi

# ── 6. OrbbecSDK v2（动态库）──────────────────────────────────────────────────
section "OrbbecSDK v2（动态库，深度相机 SDK）"

if { should_build orbbec || should_build orbbecsdk; } && need_build "libOrbbecSDK.so"; then
    clone_or_update OrbbecSDK_v2 "https://github.com/orbbec/OrbbecSDK_v2.git"

    rm -rf "${SRC_DIR}/OrbbecSDK_v2/build"
    ORBBEC_CFLAGS="-O3"
    if [[ "$PROFILE" == "rk3588" ]]; then
        ORBBEC_CFLAGS="-O3 -mcpu=cortex-a76.cortex-a55 -mtune=cortex-a76 -ftree-vectorize -fomit-frame-pointer"
    fi
    cmake -S "${SRC_DIR}/OrbbecSDK_v2" \
          -B "${SRC_DIR}/OrbbecSDK_v2/build" \
          -DOB_BUILD_EXAMPLES=OFF \
          -DOB_BUILD_TESTS=OFF \
          -DOB_BUILD_TOOLS=OFF \
          -DOB_BUILD_DOCS=OFF \
          -DOB_INSTALL_EXAMPLES_SOURCE=OFF \
          -DCMAKE_BUILD_TYPE=Release \
          -DCMAKE_C_FLAGS="${ORBBEC_CFLAGS}" \
          -DCMAKE_CXX_FLAGS="${ORBBEC_CFLAGS}" \
          -DCMAKE_INSTALL_PREFIX="${INSTALL_DIR}"

    cmake --build  "${SRC_DIR}/OrbbecSDK_v2/build" -j"${JOBS}"
    cmake --install "${SRC_DIR}/OrbbecSDK_v2/build"

    info "OrbbecSDK v2 动态库构建完成 → ${INSTALL_DIR}/lib/libOrbbecSDK.so"
    warn "⚠ libOrbbecSDK.so 是动态库，部署时需随可执行文件一起分发"
else
    { should_build orbbec || should_build orbbecsdk; } || info "OrbbecSDK 已跳过（--only 未指定）"
fi

# ── 7. OpenH264（仅 generic：非 RK 软编码）──────────────────────────────────
section "OpenH264（软件 H.264，非 RK3588）"

if should_build openh264 && need_build "libopenh264.so"; then
    OPENH264_TAG="v2.4.1"
    if [[ ! -d "${SRC_DIR}/openh264/.git" ]]; then
        info "克隆 OpenH264 ${OPENH264_TAG} ..."
        git clone --depth=1 --branch "${OPENH264_TAG}" \
                  "https://github.com/cisco/openh264.git" \
                  "${SRC_DIR}/openh264"
    else
        info "OpenH264 源码已存在，跳过克隆"
    fi
    _os=$(uname -s | tr '[:upper:]' '[:lower:]')
    _arch=$(uname -m)
    OH264_OS=linux
    OH264_ARCH=x86_64
    if [[ "$_os" == "darwin" ]]; then
        OH264_OS=darwin
        if [[ "$_arch" == "arm64" ]]; then
            OH264_ARCH=arm64
        else
            OH264_ARCH=x86_64
        fi
    else
        if [[ "$_arch" == "aarch64" || "$_arch" == "arm64" ]]; then
            OH264_ARCH=aarch64
        else
            OH264_ARCH=x86_64
        fi
    fi
    OPENH264_EXTRA_FLAGS=()
    if [[ "${OH264_ARCH}" == "x86_64" ]] && ! command -v nasm &>/dev/null; then
        warn "未找到 nasm 汇编器，OpenH264 将使用纯 C 实现（ASM=No）编译"
        OPENH264_EXTRA_FLAGS+=("ASM=No")
    fi
    make -C "${SRC_DIR}/openh264" -j"${JOBS}" OS="${OH264_OS}" ARCH="${OH264_ARCH}" ${OPENH264_EXTRA_FLAGS[@]+"${OPENH264_EXTRA_FLAGS[@]}"}
    make -C "${SRC_DIR}/openh264" PREFIX="${INSTALL_DIR}" ${OPENH264_EXTRA_FLAGS[@]+"${OPENH264_EXTRA_FLAGS[@]}"} install
    info "OpenH264 安装完成 → ${INSTALL_DIR}/lib"
else
    should_build openh264 || info "OpenH264 已跳过（rk3588 不需要，或 --only 未指定）"
fi

# ── 完成摘要 ─────────────────────────────────────────────────────────────────
section "构建完成"

echo ""
echo -e "${BOLD}安装目录: ${INSTALL_DIR}${NC}"
echo ""
echo "库文件一览："
for f in librockchip_mpp.so librga.so librknnrt.so libmk_api.so libmk_api.dylib libfaiss.a libOrbbecSDK.so libOrbbecSDK.dylib libopenh264.so libopenh264.dylib libopenh264.a; do
    path="${INSTALL_DIR}/lib/${f}"
    if [[ -f "${path}" ]]; then
        size=$(du -sh -L "${path}" 2>/dev/null | cut -f1)
        if [[ "${f}" == *.a ]]; then
            echo -e "  ${GREEN}✓ 静态${NC}  ${f}  (${size})"
        else
            echo -e "  ${YELLOW}✓ 动态${NC}  ${f}  (${size})"
        fi
    else
        echo -e "  ${RED}✗ 缺失${NC}  ${f}"
    fi
done

echo ""
echo -e "${BOLD}编译项目：${NC}"
echo "  cmake -B build"
echo "  cmake --build build -j$(nproc)"
echo ""
echo -e "${BOLD}部署时需要携带的动态库：${NC}"
echo "  ${INSTALL_DIR}/lib/librknnrt.so"
echo "  ${INSTALL_DIR}/lib/libmk_api.so"
echo "  ${INSTALL_DIR}/lib/libOrbbecSDK.so"
echo "  libopenblas.so（系统包，随 OS 分发或单独打包）"
echo ""
echo -e "${BOLD}静态链接到可执行文件（无需部署）：${NC}"
echo "  librockchip_mpp.a"
echo "  librga.a"
echo "  libfaiss.a"
