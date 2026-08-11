#!/bin/bash

# AiSprayer App Startup Script
# This script starts both the FastAPI backend and the Vite frontend.
# Logs are written to app/logs/

# Ensure we are in the app directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
APP_DIR="$(dirname "$SCRIPT_DIR")"
cd "$APP_DIR" || exit 1

LOG_DIR="$APP_DIR/logs"
BACKEND_LOG="$LOG_DIR/backend.log"
FRONTEND_LOG="$LOG_DIR/frontend.log"

echo "=========================================="
echo "Starting AiSprayer Application..."
echo "Logs will be written to: $LOG_DIR"
echo "=========================================="

# Create logs directory if it doesn't exist
mkdir -p "$LOG_DIR"

# Cleanup previous run logs if desired (optional)
# rm -f "$BACKEND_LOG" "$FRONTEND_LOG"

echo "[0/2] Cleaning up any existing AiSprayer processes..."
pkill -f "python3 main.py" 2>/dev/null
pkill -f "vite" 2>/dev/null

echo "Checking and freeing ports 8000 and 5173..."
lsof -t -i :8000 | xargs -r kill -9 2>/dev/null
lsof -t -i :5173 | xargs -r kill -9 2>/dev/null

# Give OS a moment to reap processes
sleep 1

echo "[1/2] Starting FastAPI Backend..."
# Start backend in background. Python's internal logging handles both console and file output.
cd "$APP_DIR/src" || exit 1
PYTHONUNBUFFERED=1 python3 main.py &
BACKEND_PID=$!
echo "Backend started with PID: $BACKEND_PID"

echo "[2/2] Starting Vite Frontend..."
# Start frontend in background, use tee to print to console AND write to log
cd "$APP_DIR/frontend" || exit 1
npm run dev 2>&1 | tee -a "$FRONTEND_LOG" &
FRONTEND_PID=$!
echo "Frontend started with PID: $FRONTEND_PID"

echo "=========================================="
echo "AiSprayer is now running!"
echo "Backend: http://localhost:8000"
echo "Frontend: http://localhost:5173"
echo "Press Ctrl+C to stop both processes."
echo "=========================================="

# Trap SIGINT and SIGTERM to kill background processes gracefully
cleanup() {
    echo ""
    echo "Stopping AiSprayer..."
    echo "Killing Backend (PID: $BACKEND_PID)..."
    kill $BACKEND_PID 2>/dev/null
    pkill -f "python3 main.py" 2>/dev/null
    
    echo "Killing Frontend (PID: $FRONTEND_PID)..."
    kill $FRONTEND_PID 2>/dev/null
    pkill -f "vite" 2>/dev/null
    
    echo "Forcefully freeing ports 8000 and 5173..."
    lsof -t -i :8000 | xargs -r kill -9 2>/dev/null
    lsof -t -i :5173 | xargs -r kill -9 2>/dev/null
    
    # Double check and verify ports are free
    if lsof -t -i :8000 >/dev/null || lsof -t -i :5173 >/dev/null; then
        echo "Warning: Failed to free some ports."
    else
        echo "Ports 8000 and 5173 are verified free."
    fi
    
    echo "AiSprayer stopped successfully."
    exit 0
}

trap cleanup SIGINT SIGTERM

# Wait for background processes to finish
wait $BACKEND_PID
wait $FRONTEND_PID
