#!/bin/bash

# AiSprayer App Environment Setup & Dependency Manager Script
# This script creates or updates the Python .venv environment inside app/ and installs all dependencies.

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
APP_DIR="$(dirname "$SCRIPT_DIR")"
PROJECT_ROOT="$(dirname "$APP_DIR")"
VENV_DIR="$APP_DIR/.venv"

# PyPI mirror setting for reliable & fast installation
PIP_INDEX_URL="${PIP_INDEX_URL:-https://pypi.tuna.tsinghua.edu.cn/simple}"

echo "=========================================="
echo "  AiSprayer App Environment Setup & Update  "
echo "=========================================="
echo "App Directory:  $APP_DIR"
echo "Venv Location:  $VENV_DIR"
echo "=========================================="

# 1. Check Python 3
if ! command -v python3 &> /dev/null; then
    echo "❌ Error: python3 is not installed or not in PATH."
    exit 1
fi

PYTHON_VER=$(python3 --version)
echo "🔍 Using system python: $PYTHON_VER"

# 2. Create or Update Virtual Environment
echo ""
if [ -d "$VENV_DIR" ]; then
    echo "✔️ Virtual environment already exists at $VENV_DIR"
    echo "⏳ Updating existing virtual environment..."
else
    echo "⏳ Creating virtual environment at $VENV_DIR ..."
    python3 -m venv "$VENV_DIR"
    echo "✅ Virtual environment created."
fi

VENV_PYTHON="$VENV_DIR/bin/python3"
VENV_PIP="$VENV_DIR/bin/pip"

# 3. Upgrade Pip
echo ""
echo "[1/3] Upgrading pip inside .venv ..."
"$VENV_PYTHON" -m pip install --upgrade pip -i "$PIP_INDEX_URL" --retries 5

# 4. Install / Update Python Backend Dependencies
echo ""
echo "[2/3] Installing/Updating Python Backend Dependencies..."
if [ -f "$APP_DIR/requirements.txt" ]; then
    echo "  ↳ Installing from $APP_DIR/requirements.txt using mirror ($PIP_INDEX_URL)..."
    "$VENV_PIP" install -r "$APP_DIR/requirements.txt" -i "$PIP_INDEX_URL" --retries 5
else
    echo "⚠️ app/requirements.txt not found! Installing core packages directly..."
    "$VENV_PIP" install fastapi "uvicorn[standard]" sqlalchemy pydantic numpy scipy opencv-python pyyaml python-multipart requests trimesh open3d -i "$PIP_INDEX_URL" --retries 5
fi

# Link/Install workspace root package if setup.py or pyproject.toml exists
if [ -f "$PROJECT_ROOT/setup.py" ] || [ -f "$PROJECT_ROOT/pyproject.toml" ]; then
    echo "  ↳ Installing project root package in editable mode..."
    "$VENV_PIP" install -e "$PROJECT_ROOT"
fi

# Install local hardware SDK wheels from third_party directory (e.g. pyorbbecsdk)
if [ -d "$PROJECT_ROOT/third_party" ]; then
    echo "  ↳ Installing local hardware SDK wheels from third_party directory..."
    for whl in "$PROJECT_ROOT/third_party"/*.whl; do
        if [ -f "$whl" ]; then
            echo "    ⏳ Installing wheel: $(basename "$whl") ..."
            "$VENV_PIP" install --no-deps "$whl" 2>/dev/null || true
        fi
    done
fi

echo "✅ Backend dependencies up to date."

# 5. Check and Install Frontend Node.js Dependencies
echo ""
echo "[3/3] Checking Node.js Frontend Dependencies..."
cd "$APP_DIR/frontend" || exit 1

if ! command -v npm &> /dev/null; then
    echo "⚠️ Warning: npm command not found. Skipping frontend dependency check."
else
    if [ -d "node_modules" ]; then
        echo "  ✔️ Frontend node_modules exists. Running npm install to verify/update..."
    else
        echo "  ⏳ Installing Node.js frontend dependencies..."
    fi
    npm install --registry=https://registry.npmmirror.com
fi

echo "✅ Frontend dependencies check complete."

echo ""
echo "=========================================="
echo "🎉 Environment setup & update complete!"
echo "To activate this environment manually, run:"
echo "  source app/.venv/bin/activate"
echo ""
echo "To start the application, run:"
echo "  ./app/scripts/run.sh"
echo "=========================================="
