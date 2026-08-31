#!/bin/bash

# AiSprayer App Startup Script
# This script starts both the FastAPI backend and the Vite frontend.
# Logs are written to app/logs/

# Ensure we are in the app directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
APP_DIR="$(dirname "$SCRIPT_DIR")"
PROJECT_ROOT="$(dirname "$APP_DIR")"
cd "$APP_DIR" || exit 1

LOG_DIR="$APP_DIR/logs"
DATA_DIR="$APP_DIR/data"
ROOT_DATA_DIR="$PROJECT_ROOT/data"
BACKEND_LOG="$LOG_DIR/backend.log"
FRONTEND_LOG="$LOG_DIR/frontend.log"

# Determine target non-root user (for permission restoration)
TARGET_USER="${SUDO_USER:-$(whoami)}"
TARGET_GROUP="$(id -gn "$TARGET_USER" 2>/dev/null || echo "staff")"

echo "=========================================="
echo "Starting AiSprayer Application..."
echo "Logs will be written to: $LOG_DIR"
if [ -n "$SUDO_USER" ]; then
    echo "🔒 Running in elevated mode (sudo) for full Orbbec Depth Camera access."
    echo "🛡️ File ownership in data/ and logs/ will automatically restore to: $TARGET_USER"
fi
echo "=========================================="

# Create directories if they don't exist
mkdir -p "$LOG_DIR" "$DATA_DIR" "$ROOT_DATA_DIR"

# Restore permissions at startup as well
if [ -n "$SUDO_USER" ]; then
    chown -R "$TARGET_USER:$TARGET_GROUP" "$ROOT_DATA_DIR" "$DATA_DIR" "$LOG_DIR" 2>/dev/null || true
    chmod -R u+rwX,g+rwX,a+rwX "$ROOT_DATA_DIR" "$DATA_DIR" "$LOG_DIR" 2>/dev/null || true
fi

echo "[0/2] Cleaning up any existing AiSprayer processes..."
pkill -9 -f "main.py" 2>/dev/null
pkill -9 -f "vite" 2>/dev/null
pkill -9 -f "orbbec_camera_service" 2>/dev/null

echo "Checking and freeing required ports..."
for port in 8000 5173 18080 8554 8008 1935; do
    lsof -t -i :$port | xargs -r kill -9 2>/dev/null
done

# Give OS a moment to reap processes
sleep 1

echo "[1/2] Starting FastAPI Backend..."
# Use .venv python if available, fallback to system python3
if [ -f "$APP_DIR/.venv/bin/python3" ]; then
    PYTHON_BIN="$APP_DIR/.venv/bin/python3"
    echo "Using virtual environment Python: $PYTHON_BIN"
else
    PYTHON_BIN="python3"
    echo "Using system python: $PYTHON_BIN"
fi

cd "$APP_DIR/src" || exit 1
# stdout/stderr 输出到终端控制台并通过 tee 同步落盘到文件：
# 方便控制台实时交互排查，同时日志文件保留完整记录。
# C++ 相机服务与 Python 抽排层已具备非阻塞与有界缓冲保护，不受终端消费速度波及。
PYTHONUNBUFFERED=1 PYTHONPATH="$PROJECT_ROOT/src:$APP_DIR/src:$PYTHONPATH" "$PYTHON_BIN" main.py 2>&1 | tee -a "$LOG_DIR/backend.console.log" &
BACKEND_PID=$!
echo "Backend started with PID: $BACKEND_PID"

echo "[2/2] Starting Vite Frontend..."
# Start frontend in background, use tee to print to console AND write to log
cd "$APP_DIR/frontend" || exit 1
npm run dev 2>&1 | tee -a "$FRONTEND_LOG" &
FRONTEND_PID=$!
echo "Frontend started with PID: $FRONTEND_PID"

LAN_IP=$(hostname -I 2>/dev/null | awk '{print $1}')
[ -z "$LAN_IP" ] && LAN_IP="127.0.0.1"

echo "=========================================="
echo "AiSprayer is now running!"
echo "Backend:  http://$LAN_IP:8000  (http://localhost:8000)"
echo "Frontend: http://$LAN_IP:5173  (http://localhost:5173)"
echo "Press Ctrl+C to stop both processes."
echo "=========================================="

# Trap SIGINT, SIGTERM, and EXIT to kill background processes
cleanup() {
    trap - SIGINT SIGTERM EXIT
    echo ""
    echo "Stopping AiSprayer..."
    
    # 1. Graceful termination
    [ -n "$BACKEND_PID" ] && kill -TERM "$BACKEND_PID" 2>/dev/null
    [ -n "$FRONTEND_PID" ] && kill -TERM "$FRONTEND_PID" 2>/dev/null
    
    # 2. Forceful termination after brief pause
    sleep 0.5
    [ -n "$BACKEND_PID" ] && kill -9 "$BACKEND_PID" 2>/dev/null
    [ -n "$FRONTEND_PID" ] && kill -9 "$FRONTEND_PID" 2>/dev/null
    pkill -9 -f "main.py" 2>/dev/null
    pkill -9 -f "vite" 2>/dev/null
    pkill -9 -f "orbbec_camera_service" 2>/dev/null
    
    # 3. Forcefully free ports
    echo "Forcefully freeing required ports..."
    for port in 8000 5173 18080 8554 8008 1935; do
        lsof -t -i :$port | xargs -r kill -9 2>/dev/null
    done
    
    # 4. Restoring file ownership to target user so root files are never left behind
    if [ -n "$SUDO_USER" ]; then
        echo "🛡️ Restoring file permissions in data/ and logs/ to user '$TARGET_USER'..."
        chown -R "$TARGET_USER:$TARGET_GROUP" "$ROOT_DATA_DIR" "$DATA_DIR" "$LOG_DIR" 2>/dev/null || true
        chmod -R u+rwX,g+rwX,a+rwX "$ROOT_DATA_DIR" "$DATA_DIR" "$LOG_DIR" 2>/dev/null || true
    fi

    # Double check and verify ports are free
    if lsof -t -i :8000 >/dev/null || lsof -t -i :5173 >/dev/null || lsof -t -i :18080 >/dev/null; then
        echo "Warning: Failed to free some ports."
    else
        echo "Required ports are verified free."
    fi
    
    echo "AiSprayer stopped successfully."
    exit 0
}

trap cleanup SIGINT SIGTERM EXIT

# Wait for background processes to finish
wait $BACKEND_PID
wait $FRONTEND_PID
