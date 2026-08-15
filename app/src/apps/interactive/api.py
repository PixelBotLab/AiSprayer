from fastapi import APIRouter, HTTPException, BackgroundTasks
from pydantic import BaseModel
import os
import time
import shutil
import logging
import cv2
import numpy as np
import yaml
import open3d as o3d
from services.camera_service import camera_service
from core.vision.point_cloud_processor import depth_to_pcd
from apps.interactive.sam_service import sam_service
from apps.interactive.reconstruction_service import reconstruction_service
from apps.interactive.path_service import manual_path_service

router = APIRouter(prefix="/api/interactive", tags=["Interactive"])
logger = logging.getLogger(__name__)

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../.."))
TEMPLATE_GROUP_DIR = os.path.join(PROJECT_ROOT, "data", "template_group")

os.makedirs(TEMPLATE_GROUP_DIR, exist_ok=True)

class CreateTemplateRequest(BaseModel):
    name: str | None = None

@router.get("/templates")
def list_templates():
    if not os.path.exists(TEMPLATE_GROUP_DIR):
        return {"templates": []}
    
    templates = []
    for item in os.listdir(TEMPLATE_GROUP_DIR):
        item_path = os.path.join(TEMPLATE_GROUP_DIR, item)
        if os.path.isdir(item_path):
            templates.append(item)
    
    # Sort by descending order (newest first usually)
    templates.sort(reverse=True)
    return {"templates": templates}

@router.post("/templates")
def create_template(req: CreateTemplateRequest):
    name = req.name
    if not name:
        name = time.strftime("%Y%m%d_%H%M%S")
    
    template_path = os.path.join(TEMPLATE_GROUP_DIR, name)
    if os.path.exists(template_path):
        raise HTTPException(status_code=400, detail="Template name already exists")
        
    try:
        os.makedirs(template_path, exist_ok=True)
        logger.info(f"Created new template directory: {template_path}")
        return {"message": "Template created successfully", "name": name}
    except Exception as e:
        logger.error(f"Failed to create template '{name}': {e}")
        raise HTTPException(status_code=500, detail=str(e))

@router.get("/templates/{name}/files")
def list_template_files(name: str):
    template_path = os.path.join(TEMPLATE_GROUP_DIR, name)
    if not os.path.exists(template_path):
        raise HTTPException(status_code=404, detail="Template not found")
        
    files = []
    for item in os.listdir(template_path):
        item_path = os.path.join(template_path, item)
        if os.path.isfile(item_path):
            size = os.path.getsize(item_path)
            ctime = os.path.getctime(item_path)
            files.append({"name": item, "size": size, "ctime": ctime})
            
    files.sort(key=lambda x: x["ctime"], reverse=True)
    return {"files": files}

@router.post("/templates/{name}/capture")
def capture_template_data(name: str):
    template_path = os.path.join(TEMPLATE_GROUP_DIR, name)
    if not os.path.exists(template_path):
        raise HTTPException(status_code=404, detail="Template not found")
        
    color_frame = camera_service.get_latest_frame()
    depth_frame = camera_service.get_latest_depth()
    if color_frame is None or depth_frame is None:
        logger.error(f"Capture failed for template '{name}': Camera frames not available.")
        raise HTTPException(status_code=503, detail="Camera frames not available")
        
    intr_k, intr_d = camera_service.get_intrinsics()
    if intr_k is None:
        logger.error(f"Capture failed for template '{name}': Camera intrinsics not available.")
        raise HTTPException(status_code=503, detail="Camera intrinsics not available")
        
    try:
        logger.info(f"Starting data capture for template '{name}'...")
        # Save color image
        color_path = os.path.join(template_path, "scan.jpg")
        cv2.imwrite(color_path, color_frame)
        logger.info(f"Saved color image: {color_path} ({color_frame.shape[1]}x{color_frame.shape[0]})")
        
        # Save depth data
        depth_path = os.path.join(template_path, "scan.depth.npy")
        np.save(depth_path, depth_frame)
        logger.info(f"Saved depth matrix: {depth_path}")
        
        # Convert to point cloud and save
        intr_list = intr_k.tolist()
        intr_params = [intr_list[0][0], intr_list[1][1], intr_list[0][2], intr_list[1][2]]
        points = depth_to_pcd(depth_frame, intr_params)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        pcd_path = os.path.join(template_path, "scan.pcd")
        o3d.io.write_point_cloud(pcd_path, pcd)
        logger.info(f"Saved 3D point cloud: {pcd_path} ({len(points)} points)")
        
        # Save metadata / params
        meta = {
            "version": "1.0",
            "template_name": name,
            "timestamp": time.time(),
            "camera_params": {
                "intrinsic_matrix": intr_list,
                "distortion_coeffs": intr_d.tolist() if intr_d is not None else [],
                "width": color_frame.shape[1],
                "height": color_frame.shape[0]
            }
        }
        params_path = os.path.join(template_path, "scan.params.yaml")
        with open(params_path, 'w') as f:
            yaml.dump(meta, f, default_flow_style=False)
        logger.info(f"Saved camera metadata: {params_path}")
            
        logger.info(f"Successfully completed all captures for template '{name}'.")
        return {"message": "Data captured successfully", "template": name}
    except Exception as e:
        logger.error(f"Capture error for template '{name}': {e}", exc_info=True)
        raise HTTPException(status_code=500, detail=str(e))

class PredictRequest(BaseModel):
    points: list[list[int]]
    labels: list[int]

class SaveMasksRequest(BaseModel):
    committed_masks: list[dict]

@router.get("/templates/{name}/masks")
def get_template_masks(name: str):
    template_path = os.path.join(TEMPLATE_GROUP_DIR, name)
    if not os.path.exists(template_path):
        raise HTTPException(status_code=404, detail="Template not found")
        
    data = sam_service.get_template_masks(template_path)
    if data is None:
        return {"masks": []}
    return data

@router.post("/templates/{name}/sam/init")
def init_sam(name: str):
    template_path = os.path.join(TEMPLATE_GROUP_DIR, name)
    if not os.path.exists(template_path):
        raise HTTPException(status_code=404, detail="Template not found")
        
    success = sam_service.init_template(template_path, name)
    if not success:
        logger.error(f"Failed to initialize SAM for template '{name}'.")
        raise HTTPException(status_code=500, detail="Failed to initialize SAM for this template")
    return {"message": "SAM initialized successfully"}

@router.post("/templates/{name}/sam/predict")
def predict_sam(name: str, req: PredictRequest):
    try:
        result = sam_service.predict_action(name, req.points, req.labels)
        return result
    except Exception as e:
        logger.error(f"Prediction failed for template '{name}': {e}", exc_info=True)
        raise HTTPException(status_code=500, detail=str(e))

@router.post("/templates/{name}/sam/save")
def save_sam(name: str, req: SaveMasksRequest):
    template_path = os.path.join(TEMPLATE_GROUP_DIR, name)
    if not os.path.exists(template_path):
        raise HTTPException(status_code=404, detail="Template not found")
        
    try:
        success = sam_service.save_masks(template_path, name, req.committed_masks)
        if not success:
            raise HTTPException(status_code=500, detail="Failed to save masks")
        return {"message": "Masks saved successfully"}
    except PermissionError as e:
        logger.warning(f"Save masks permission error for template '{name}': {e}")
        raise HTTPException(status_code=403, detail=f"Permission denied: {str(e)}")
    except Exception as e:
        logger.warning(f"Save masks failed for template '{name}': {e}")
        raise HTTPException(status_code=500, detail=str(e))

@router.post("/templates/{name}/reconstruct")
def reconstruct_surface(name: str):
    template_path = os.path.join(TEMPLATE_GROUP_DIR, name)
    if not os.path.exists(template_path):
        raise HTTPException(status_code=404, detail="Template not found")
        
    try:
        result = reconstruction_service.reconstruct_surface(template_path, name)
        return result
    except FileNotFoundError as e:
        logger.warning(f"Reconstruction file missing for '{name}': {e}")
        raise HTTPException(status_code=400, detail=str(e))
    except ValueError as e:
        logger.warning(f"Reconstruction validation error for '{name}': {e}")
        raise HTTPException(status_code=400, detail=str(e))
    except Exception as e:
        logger.warning(f"Reconstruction failed for template '{name}': {e}")
        raise HTTPException(status_code=500, detail=f"Reconstruction error: {str(e)}")


class SamplePointRequest(BaseModel):
    u: int
    v: int
    standoff_dist_mm: float = 150.0

@router.post("/templates/{name}/sample_point")
def sample_point(name: str, req: SamplePointRequest):
    template_path = os.path.join(TEMPLATE_GROUP_DIR, name)
    if not os.path.exists(template_path):
        raise HTTPException(status_code=404, detail="Template not found")

    try:
        result = manual_path_service.sample_point_pose(name, req.u, req.v, req.standoff_dist_mm)
        return result
    except Exception as e:
        logger.warning(f"Sample point calculation failed for '{name}' at ({req.u},{req.v}): {e}")
        raise HTTPException(status_code=500, detail=f"Point sampling failed: {str(e)}")

@router.get("/templates/{name}/manual_paths")
def get_manual_paths(name: str):
    try:
        return manual_path_service.load_manual_paths(name)
    except Exception as e:
        logger.warning(f"Get manual paths failed for template '{name}': {e}")
        raise HTTPException(status_code=500, detail=f"Failed to load manual paths: {str(e)}")

class SaveManualPathsRequest(BaseModel):
    paths: list
    standoff_distance_mm: float = 150.0

@router.post("/templates/{name}/manual_paths")
def save_manual_paths(name: str, req: SaveManualPathsRequest):
    try:
        data = {
            "paths": req.paths,
            "standoff_distance_mm": req.standoff_distance_mm
        }
        manual_path_service.save_manual_paths(name, data)
        return {"message": "Manual paths saved successfully"}
    except PermissionError as e:
        logger.warning(f"Save manual paths permission error for '{name}': {e}")
        raise HTTPException(status_code=403, detail=f"Permission denied: {str(e)}")
    except Exception as e:
        logger.warning(f"Save manual paths error for '{name}': {e}")
        raise HTTPException(status_code=500, detail=f"Failed to save manual paths: {str(e)}")


@router.get("/templates/{name}/session_data")
def get_session_data(name: str):
    """
    One-shot endpoint: returns all data needed for fully client-side normal computation.
    - depth_flat: base64-encoded float32 array (row-major, h*w values in mm)
    - width, height: depth image dimensions
    - intrinsics: {fx, fy, cx, cy}
    - T_base_camera: 4x4 row-major float list (translation in mm)
    - calib_source: description string
    """
    import base64
    from apps.interactive.reconstruction_service import reconstruction_service

    template_path = os.path.join(TEMPLATE_GROUP_DIR, name)
    if not os.path.exists(template_path):
        raise HTTPException(status_code=404, detail="Template not found")

    # Load depth
    depth_npy = os.path.join(template_path, "scan.depth.npy")
    depth_png = os.path.join(template_path, "scan.depth.png")
    if os.path.exists(depth_npy):
        depth_map = np.load(depth_npy).astype(np.float32)
    elif os.path.exists(depth_png):
        depth_map = cv2.imread(depth_png, cv2.IMREAD_UNCHANGED).astype(np.float32)
    else:
        raise HTTPException(status_code=400, detail="Depth map not found for this template")

    if depth_map is None:
        raise HTTPException(status_code=400, detail="Failed to load depth map")

    h, w = depth_map.shape[:2]

    # Load calibration
    try:
        T_cam_to_base_m, k_matrix, calib_desc = reconstruction_service.get_latest_calibration()
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to load calibration: {str(e)}")

    # Intrinsics
    if k_matrix is not None:
        fx, fy = float(k_matrix[0, 0]), float(k_matrix[1, 1])
        cx, cy = float(k_matrix[0, 2]), float(k_matrix[1, 2])
    else:
        fx, fy, cx, cy = 900.0, 900.0, 640.0, 400.0

    # Convert transform: translate meters -> mm
    T_base_camera = T_cam_to_base_m.copy()
    T_base_camera[0:3, 3] *= 1000.0

    # Encode depth map as base64 float32
    depth_bytes = depth_map.flatten().astype(np.float32).tobytes()
    depth_b64 = base64.b64encode(depth_bytes).decode('ascii')

    return {
        "width": w,
        "height": h,
        "depth_flat_b64": depth_b64,
        "intrinsics": {"fx": fx, "fy": fy, "cx": cx, "cy": cy},
        "T_base_camera": T_base_camera.flatten().tolist(),
        "calib_source": calib_desc
    }

