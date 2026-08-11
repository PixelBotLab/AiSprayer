#!/bin/bash

# AiSprayer App Dependencies Installation Script
# This script installs all required Python backend and Node.js frontend dependencies.
# It checks if packages are already installed to avoid redundant work.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
APP_DIR="$(dirname "$SCRIPT_DIR")"

echo "=========================================="
echo "Checking AiSprayer Dependencies..."
echo "=========================================="

echo ""
echo "[1/2] Checking Backend Dependencies (Python) ..."

check_python_pkg() {
    local pkg_name=$1
    local import_name=${2:-$1}
    
    if python3 -c "import $import_name" &> /dev/null; then
        echo "  ✔️  $pkg_name is already installed."
    else
        echo "  ⏳ Installing $pkg_name ..."
        pip install "$pkg_name" --break-system-packages
    fi
}

# Check and install python packages
check_python_pkg "fastapi"
check_python_pkg "uvicorn[standard]" "uvicorn"
check_python_pkg "sqlalchemy"
check_python_pkg "opencv-python" "cv2"
check_python_pkg "pydantic"
check_python_pkg "numpy"

echo "✅ Backend dependencies check complete."

echo ""
echo "[2/2] Checking Frontend Dependencies (Node/npm) ..."
cd "$APP_DIR/frontend" || exit 1

if [ -d "node_modules" ]; then
    echo "  ✔️  Frontend node_modules already exists. Skipping npm install."
    echo "      (If you need to force update, run 'npm install' manually in app/frontend)"
else
    echo "  ⏳ Installing Node.js dependencies (this may take a minute) ..."
    npm install --registry=https://registry.npmmirror.com
    npm install tailwindcss @tailwindcss/postcss postcss autoprefixer react-router-dom lucide-react recharts --registry=https://registry.npmmirror.com
    if [ $? -ne 0 ]; then
        echo "❌ Failed to install Node.js dependencies."
        exit 1
    fi
fi

echo "✅ Frontend dependencies check complete."

echo ""
echo "=========================================="
echo "🎉 All dependencies are ready!"
echo "You can now run: ./app/scripts/run.sh"
echo "=========================================="
