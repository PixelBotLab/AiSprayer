#!/bin/bash
# -*- coding: utf-8 -*-

# AiSprayer - Robotic Calibration GUI Standalone Packager (Shell Version)
# Please make sure you are running this inside the 'inexbot' conda environment.

# Exit immediately if a command exits with a non-zero status
set -e

# 1. Ensure we are in the correct workspace directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
cd "$SCRIPT_DIR"
echo "[*] Workspace root: $SCRIPT_DIR"

# Parse command line options
CLEAN_BUILD=false
LITE_BUILD=false
while getopts "cl" opt; do
    case $opt in
        c)
            CLEAN_BUILD=true
            ;;
        l)
            LITE_BUILD=true
            ;;
        \?)
            echo "Usage: $0 [-c] [-l]"
            exit 1
            ;;
    esac
done

if [ "$CLEAN_BUILD" = true ]; then
    echo "[*] Cleaning up previous build artifacts..."
    rm -rf build/ dist/ calib_ui.spec build_venv/
    echo "[+] Cleaned build/, dist/, calib_ui.spec, and build_venv/"
fi

# 2. Check if conda environment is inexbot
CURRENT_ENV=$(basename "$CONDA_DEFAULT_ENV" 2>/dev/null || echo "")
if [ "$CURRENT_ENV" != "inexbot" ]; then
    echo "[!] Warning: You are not in the 'inexbot' conda environment (current: '$CURRENT_ENV')."
    echo "[*] Please run 'conda activate inexbot' first if you see import/library errors."
fi

# Resolve the exact Python executable from active Conda environment
PYTHON_EXE="python3"
if [ -n "$CONDA_PREFIX" ] && [ -f "$CONDA_PREFIX/bin/python" ]; then
    PYTHON_EXE="$CONDA_PREFIX/bin/python"
fi

# 3. Locate site-packages directory to gather camera SDK binaries
SITE_PACKAGES=$($PYTHON_EXE -c "import site; print(site.getsitepackages()[0])")
echo "[*] site-packages path: $SITE_PACKAGES"

PYINSTALLER_BIN="pyinstaller"

if [ "$LITE_BUILD" = true ]; then
    echo "[*] Lite build mode enabled. Setting up a temporary virtual environment (without MKL)..."
    
    # Create build_venv if not exists
    if [ ! -d "build_venv" ]; then
        echo "[*] Creating build_venv using $PYTHON_EXE..."
        $PYTHON_EXE -m venv build_venv
        echo "[*] Installing dependencies from PyPI (OpenBLAS versions)..."
        build_venv/bin/pip install --upgrade pip
        build_venv/bin/pip install pyyaml numpy scipy opencv-python PyQt5 pyrealsense2 pyinstaller
        
        # Copy Orbbec SDK files to build_venv site-packages
        VENV_SP=$(find build_venv/lib -maxdepth 2 -name "site-packages" | head -n 1)
        if [ -n "$VENV_SP" ]; then
            if [ -f "$SITE_PACKAGES/pyorbbecsdk.cpython-38-x86_64-linux-gnu.so" ]; then
                cp "$SITE_PACKAGES/pyorbbecsdk.cpython-38-x86_64-linux-gnu.so" "$VENV_SP/"
            fi
            # Copy all versions of libOrbbecSDK.so (like libOrbbecSDK.so.2)
            cp "$SITE_PACKAGES"/libOrbbecSDK.so* "$VENV_SP/"
            if [ -d "$SITE_PACKAGES/extensions" ]; then
                cp -r "$SITE_PACKAGES/extensions" "$VENV_SP/"
            fi
            echo "[+] Dependencies installed and SDK libraries copied to build_venv."
        else
            echo "[!] Error: Could not locate site-packages in build_venv."
            exit 1
        fi
    else
        echo "[*] Using existing build_venv."
    fi
    
    PYINSTALLER_BIN="build_venv/bin/pyinstaller"
    # Update site packages path to use the one inside build_venv for the camera assets copy
    SITE_PACKAGES=$(find build_venv/lib -maxdepth 2 -name "site-packages" | head -n 1)
else
    # 4. Check if pyinstaller is installed in host env
    if ! command -v pyinstaller &> /dev/null; then
        echo "[!] PyInstaller is not installed in the current environment."
        echo "[*] Attempting to install pyinstaller via pip..."
        pip install pyinstaller
        echo "[*] PyInstaller installed successfully."
    fi
fi

# 5. Formulate PyInstaller arguments
ENTRY_POINT="src/aisprayer/tools/calib/5.calib_ui.py"
NRC_SO_SRC="src/aisprayer/core/hardware/robot/inexbot_v24_03_py38/_nrc_host.so"
NRC_SO_DEST="aisprayer/core/hardware/robot/inexbot_v24_03_py38"

ADD_LIBS=""

# Check if Orbbec SDK files exist
found_orbbec=false
for lib_path in "$SITE_PACKAGES"/libOrbbecSDK.so*; do
    if [ -f "$lib_path" ]; then
        echo "[+] Found Orbbec SDK: $lib_path"
        ADD_LIBS="$ADD_LIBS --add-binary=$lib_path:."
        found_orbbec=true
    fi
done
if [ "$found_orbbec" = false ]; then
    echo "[!] Warning: libOrbbecSDK.so* not found."
fi

if [ -d "$SITE_PACKAGES/extensions" ]; then
    echo "[+] Found Orbbec extensions: $SITE_PACKAGES/extensions"
    ADD_LIBS="$ADD_LIBS --add-data=$SITE_PACKAGES/extensions:extensions"
else
    echo "[!] Warning: extensions folder not found."
fi

# 6. Run PyInstaller
echo "[*] Packaging calib_ui..."
$PYINSTALLER_BIN --onefile \
    --name=calib_ui \
    --paths=src \
    --add-binary="${NRC_SO_SRC}:${NRC_SO_DEST}" \
    --hidden-import=aisprayer.core.hardware.camera.orbbec_driver \
    --hidden-import=aisprayer.core.hardware.camera.realsense_driver \
    --hidden-import=pyorbbecsdk \
    --hidden-import=pyorbbecsdk2 \
    --hidden-import=pyrealsense2 \
    --exclude-module=PyQt5.QtWebEngine \
    --exclude-module=PyQt5.QtWebEngineCore \
    --exclude-module=PyQt5.QtWebKit \
    --exclude-module=PyQt5.QtMultimedia \
    --exclude-module=PyQt5.QtQuick \
    --exclude-module=PyQt5.QtQml \
    --exclude-module=PyQt5.QtSql \
    --exclude-module=PyQt5.QtBluetooth \
    --exclude-module=PyQt5.QtXmlPatterns \
    --exclude-module=matplotlib \
    --exclude-module=scipy.spatial.transform.tests \
    $ADD_LIBS \
    "$ENTRY_POINT"

# 7. Copy configuration files to dist/ next to executable
echo "[*] Copying configurations to dist/ for standalone runtime..."
rm -rf dist/configs
if [ -d "configs" ]; then
    cp -r configs dist/
    echo "[+] Copied configs to dist/configs"
fi

echo -e "\n[✔] Packaging completed successfully!"
echo "    Executable path: $SCRIPT_DIR/dist/calib_ui"
echo "    To run: cd dist && ./calib_ui"
