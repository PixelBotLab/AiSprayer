import logging
import asyncio
from datetime import datetime
from typing import List, Dict, Any
from fastapi import WebSocket

class WebSocketLogHandler(logging.Handler):
    """
    A logging handler that forwards log records to the global LogService.
    Strips terminal color codes from the levelname if any exist, as the 
    frontend will handle its own color rendering.
    """
    def __init__(self):
        super().__init__()
        # Use a simple formatter that just grabs the message
        self.setFormatter(logging.Formatter("%(message)s"))

    def emit(self, record):
        try:
            # Filter out noisy uvicorn logs from frontend display
            if record.name.startswith("uvicorn"):
                return

            msg = self.format(record)
            
            # Clean up color codes if they got into the record
            levelname = record.levelname
            if "\033" in levelname:
                # Strip standard terminal colors
                import re
                levelname = re.sub(r'\033\[[0-9;]*m', '', levelname)

            log_entry = {
                "time": datetime.fromtimestamp(record.created).strftime("%H:%M:%S"),
                "level": levelname,
                "logger": record.name,
                "message": msg
            }
            log_service.add_log(log_entry)
        except Exception:
            self.handleError(record)


class LogService:
    def __init__(self):
        self.active_connections: List[WebSocket] = []
        self.loop: asyncio.AbstractEventLoop = None
        self.queue: asyncio.Queue = None

    def initialize(self):
        """Called during FastAPI startup to get the running loop."""
        self.loop = asyncio.get_running_loop()
        self.queue = asyncio.Queue()
        # Start the broadcast background task
        self.loop.create_task(self.broadcast_loop())

    def add_log(self, log_entry: Dict[str, Any]):
        """Called by the logging handler from any thread."""
        if self.loop and self.queue and not self.loop.is_closed():
            self.loop.call_soon_threadsafe(self.queue.put_nowait, log_entry)

    async def connect(self, websocket: WebSocket):
        await websocket.accept()
        self.active_connections.append(websocket)

    def disconnect(self, websocket: WebSocket):
        if websocket in self.active_connections:
            self.active_connections.remove(websocket)

    async def broadcast_loop(self):
        while True:
            try:
                log_entry = await self.queue.get()
                dead_connections = []
                for connection in self.active_connections:
                    try:
                        await connection.send_json(log_entry)
                    except Exception:
                        dead_connections.append(connection)
                
                for connection in dead_connections:
                    self.disconnect(connection)
            except asyncio.CancelledError:
                break
            except Exception as e:
                print(f"Error in log broadcast loop: {e}")

# Global singletons
log_service = LogService()
ws_log_handler = WebSocketLogHandler()
