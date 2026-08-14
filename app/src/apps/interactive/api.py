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
    except Exception as e:
        logger.error(f"Save masks failed for template '{name}': {e}", exc_info=True)
        raise HTTPException(status_code=500, detail=str(e))
