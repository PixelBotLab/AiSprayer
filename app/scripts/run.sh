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

echo "Checking and freeing ports 8000 and 5173..."
lsof -t -i :8000 | xargs -r kill -9 2>/dev/null
lsof -t -i :5173 | xargs -r kill -9 2>/dev/null

# Give OS a moment to reap processes
sleep 1

# 关掉 Orbbec 相机的 USB 自动休眠（power/control=auto 时内核会主动把设备挂起）。
# 实测 Gemini 336L 工作电流 896mA（USB3 上限 900mA 边缘）+ auto 策略，运行中会间歇性
# 从总线掉线（dmesg: USB disconnect）—— 表现为取流突然停滞、看门狗反复重连。
# 设备每次重新枚举后节点路径不变但实例会重建，所以每次启动都要重新设一遍。
echo "[Prep] Disabling USB autosuspend for Orbbec cameras..."
for vid in /sys/bus/usb/devices/*/idVendor; do
    if grep -q "^2bc5$" "$vid" 2>/dev/null; then
        dev_dir="$(dirname "$vid")"
        if echo on > "$dev_dir/power/control" 2>/dev/null; then
            echo "  - $dev_dir: autosuspend -> on"
        else
            echo "  - $dev_dir: 无权设置（需要 sudo 运行本脚本才能生效）"
        fi
    fi
done

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
# stdout/stderr 必须落到文件，不能接终端：后端日志的 console handler 写 stdout，
# 一旦终端那头停止消费（SSH 掉线/终端挂起），write() 会阻塞住 logging，进而把
# C++ 相机服务的日志管道抽排也卡死 —— 整个服务（含 HTTP）被日志背压冻住，
# 页面上就是"使能 follow 失败，后端服务无响应"。落文件后这条故障链不存在。
PYTHONUNBUFFERED=1 PYTHONPATH="$PROJECT_ROOT/src:$APP_DIR/src:$PYTHONPATH" "$PYTHON_BIN" main.py >> "$LOG_DIR/backend.console.log" 2>&1 &
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
    
    # 3. Forcefully free ports
    echo "Forcefully freeing ports 8000 and 5173..."
    lsof -t -i :8000 | xargs -r kill -9 2>/dev/null
    lsof -t -i :5173 | xargs -r kill -9 2>/dev/null
    
    # 4. Restoring file ownership to target user so root files are never left behind
    if [ -n "$SUDO_USER" ]; then
        echo "🛡️ Restoring file permissions in data/ and logs/ to user '$TARGET_USER'..."
        chown -R "$TARGET_USER:$TARGET_GROUP" "$ROOT_DATA_DIR" "$DATA_DIR" "$LOG_DIR" 2>/dev/null || true
        chmod -R u+rwX,g+rwX,a+rwX "$ROOT_DATA_DIR" "$DATA_DIR" "$LOG_DIR" 2>/dev/null || true
    fi

    # Double check and verify ports are free
    if lsof -t -i :8000 >/dev/null || lsof -t -i :5173 >/dev/null; then
        echo "Warning: Failed to free some ports."
    else
        echo "Ports 8000 and 5173 are verified free."
    fi
    
    echo "AiSprayer stopped successfully."
    exit 0
}

trap cleanup SIGINT SIGTERM EXIT

# Wait for background processes to finish
wait $BACKEND_PID
wait $FRONTEND_PID
