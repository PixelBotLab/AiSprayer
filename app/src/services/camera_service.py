"""
Backward-compatible proxy shim for camera_service.
All implementation has moved to `apps.camera.services.camera_service`.
"""
from apps.camera.services.camera_service import camera_service, CameraService

__all__ = ["camera_service", "CameraService"]
