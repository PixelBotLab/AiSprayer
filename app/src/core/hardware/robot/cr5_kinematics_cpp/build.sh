#!/usr/bin/env bash
set -e

# Resolve the directory of this script
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${SCRIPT_DIR}/build"

echo "=================================================="
echo "🔨 Building cr5_kinematics_cpp"
echo "📂 Source Directory: ${SCRIPT_DIR}"
echo "📂 Build Directory:  ${BUILD_DIR}"
echo "=================================================="

# Optional clean build
if [[ "$1" == "--clean" || "$1" == "-c" ]]; then
    echo "🧹 Cleaning previous build artifacts..."
    rm -rf "${BUILD_DIR}" "${SCRIPT_DIR}/libur_kin.so" "${SCRIPT_DIR}/test_cr5_kinematics"
fi

mkdir -p "${BUILD_DIR}"
cd "${BUILD_DIR}"

# Detect number of CPU cores for parallel compilation
NUM_CORES=$(getconf _NPROCESSORS_ONLN 2>/dev/null || sysctl -n hw.ncpu 2>/dev/null || echo 4)

echo "⚙️  Configuring CMake..."
cmake "${SCRIPT_DIR}" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_SHARED_LIBRARY_SUFFIX=".so"

echo "🚀 Compiling targets with ${NUM_CORES} threads..."
cmake --build . --config Release -j "${NUM_CORES}"

# Ensure libur_kin.so is available in SCRIPT_DIR for ctypes / python
if [ -f "${BUILD_DIR}/libur_kin.so" ]; then
    cp -f "${BUILD_DIR}/libur_kin.so" "${SCRIPT_DIR}/libur_kin.so"
elif [ -f "${BUILD_DIR}/libur_kin.dylib" ]; then
    cp -f "${BUILD_DIR}/libur_kin.dylib" "${SCRIPT_DIR}/libur_kin.so"
fi

if [ -f "${BUILD_DIR}/test_cr5_kinematics" ]; then
    cp -f "${BUILD_DIR}/test_cr5_kinematics" "${SCRIPT_DIR}/test_cr5_kinematics"
fi

echo "=================================================="
echo "✅ Build completed successfully!"
echo "📦 Output library:    ${SCRIPT_DIR}/libur_kin.so"
if [ -f "${SCRIPT_DIR}/test_cr5_kinematics" ]; then
    echo "🎯 Test executable:   ${SCRIPT_DIR}/test_cr5_kinematics"
fi
echo "=================================================="

# Run test if requested or if flag is passed
if [[ "$1" == "--test" || "$1" == "-t" || "$2" == "--test" || "$2" == "-t" ]]; then
    echo "🧪 Running C++ Kinematics Tests..."
    cd "${SCRIPT_DIR}"
    ./test_cr5_kinematics
fi
