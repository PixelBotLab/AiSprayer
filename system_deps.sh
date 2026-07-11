#!/bin/bash

# 严格模式：一旦发生任何错误，立刻停止执行
set -e

# ------------------------------------------------------------------------------
# 1. 安装重型系统工具及 C++ 矩阵/解析基础库
# ------------------------------------------------------------------------------
echo "📦 正在安装 Linux 系统基础依赖工具..."
PACKAGES=(build-essential cmake git wget g++ python3-pip python3-dev libeigen3-dev libcli11-dev libboost-all-dev liboctomap-dev libfcl-dev libassimp-dev liborocos-kdl-dev libyaml-cpp-dev libtinyxml2-dev libconsole-bridge-dev libbullet-dev libpcl-dev libbullet-extras-dev libomp-dev libcxxopts-dev nodejs)
MISSING_PACKAGES=()
for pkg in "${PACKAGES[@]}"; do
    if ! dpkg -s "$pkg" &> /dev/null; then
        MISSING_PACKAGES+=("$pkg")
    fi
done

if [ ${#MISSING_PACKAGES[@]} -gt 0 ]; then
    echo "  ↳ 发现缺失系统依赖包: ${MISSING_PACKAGES[*]}，开始安装..."
    sudo apt-get update
    sudo apt-get install -y "${MISSING_PACKAGES[@]}"
else
    echo "  ↳ 所有系统依赖包已安装，跳过系统包部署。"
fi

# 如果系统中未通过 nodejs 安装包自带 npm，则独立安装 npm
if ! command -v npm &> /dev/null; then
    if ! dpkg -s npm &> /dev/null; then
        echo "  ↳ 正在安装 npm..."
        sudo apt-get install -y npm
    fi
fi