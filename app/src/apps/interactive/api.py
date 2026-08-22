from fastapi import APIRouter, HTTPException, BackgroundTasks
from pydantic import BaseModel, Field
from typing import Optional, List
import os
import math
import time
import shutil
import logging
import cv2
import numpy as np
import yaml
import open3d as o3d
from services.camera_service import camera_service
from services.robot_service import robot_service
from core.vision.point_cloud_processor import depth_to_pcd
from apps.interactive.sam_service import sam_service
from apps.interactive.reconstruction_service import reconstruction_service
from apps.interactive.manual_path_service import manual_path_service
from apps.interactive.path_verification_service import path_verification_service, DEFAULT_POI_TOLERANCE_RPY_DEG


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

@router.delete("/templates/{name}")
def delete_template(name: str):
    template_path = os.path.join(TEMPLATE_GROUP_DIR, name)
    if not os.path.exists(template_path):
        raise HTTPException(status_code=404, detail="Template not found")
    try:
        shutil.rmtree(template_path)
        logger.info(f"Deleted template directory: {template_path}")
        return {"message": f"Template '{name}' deleted successfully"}
    except Exception as e:
        logger.error(f"Failed to delete template '{name}': {e}")
        raise HTTPException(status_code=500, detail=str(e))

@router.delete("/templates/{name}/files/{filename}")
def delete_template_file(name: str, filename: str):
    template_path = os.path.join(TEMPLATE_GROUP_DIR, name)
    if not os.path.exists(template_path):
        raise HTTPException(status_code=404, detail="Template not found")
    
    file_path = os.path.join(template_path, filename)
    # Prevent path traversal
    if not os.path.abspath(file_path).startswith(os.path.abspath(template_path)):
        raise HTTPException(status_code=400, detail="Invalid filename")
        
    if not os.path.exists(file_path):
        raise HTTPException(status_code=404, detail=f"File '{filename}' not found")
        
    try:
        if os.path.isfile(file_path):
            os.remove(file_path)
            logger.info(f"Deleted template file: {file_path}")
            return {"message": f"File '{filename}' deleted successfully"}
        elif os.path.isdir(file_path):
            shutil.rmtree(file_path)
            logger.info(f"Deleted template directory: {file_path}")
            return {"message": f"Directory '{filename}' deleted successfully"}
    except Exception as e:
        logger.error(f"Failed to delete file '{filename}' in '{name}': {e}")
        raise HTTPException(status_code=500, detail=str(e))

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
def get_manual_paths(name: str, state_type: str = "raw", use_opt: bool = False):
    try:
        actual_state = "opt" if use_opt else state_type
        return manual_path_service.load_manual_paths(name, state_type=actual_state, use_opt=use_opt)
    except Exception as e:
        logger.warning(f"Get manual paths failed for template '{name}': {e}")
        raise HTTPException(status_code=500, detail=f"Failed to load manual paths: {str(e)}")


class SaveManualPathsRequest(BaseModel):
    paths: list
    standoff_distance_mm: float = 150.0

@router.post("/templates/{name}/manual_paths")
def save_manual_paths(name: str, req: SaveManualPathsRequest, state_type: str = "raw"):
    try:
        data = {
            "paths": req.paths,
            "standoff_distance_mm": req.standoff_distance_mm
        }
        manual_path_service.save_manual_paths(name, data, state_type=state_type)
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


class PoiConstraintConfig(BaseModel):
    ref_rpy_deg: list[float] | None = None  # e.g. [0.0, 0.0, 0.0]
    tolerance_rpy_deg: list[float] = Field(default_factory=lambda: list(DEFAULT_POI_TOLERANCE_RPY_DEG))  # [tol_rx, tol_ry, tol_rz]
    anchor_source: str | None = None  # 'home' | 'live' | 'manual'


class KinematicsOptions(BaseModel):
    step_size_mm: float = 1.5
    linear_velocity_mm_s: float = 120.0
    tcp_offset_xyz_mm: list[float] | None = None  # e.g. [50.0, 0.0, 0.0]
    tcp_offset_rpy_deg: list[float] | None = None  # e.g. [0.0, 90.0, 0.0]
    max_joint_vel_deg_s: list[float] | None = None  # e.g. [150.0, 150.0, 150.0, 180.0, 180.0, 300.0]


class VerifyPathRequest(BaseModel):
    state_type: str = "raw"  # 'raw' | 'opt' | 'poi'
    use_opt: bool = False
    options: KinematicsOptions = KinematicsOptions()


class OptimizePathRequest(BaseModel):
    mode: str = "opt"  # 'opt' | 'poi'
    poi_config: PoiConstraintConfig | None = None
    options: KinematicsOptions = KinematicsOptions()


@router.post("/templates/{name}/verify_paths")
def verify_paths(name: str, req: VerifyPathRequest = VerifyPathRequest()):
    try:
        actual_state = "opt" if req.use_opt else req.state_type
        res = path_verification_service.verify_template_paths(
            name, 
            state_type=actual_state,
            options=req.options.model_dump()
        )
        return res
    except FileNotFoundError as e:
        raise HTTPException(status_code=404, detail=str(e))
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))
    except Exception as e:
        logger.error(f"Verification error for '{name}': {e}", exc_info=True)
        raise HTTPException(status_code=500, detail=f"Verification failed: {str(e)}")


@router.post("/templates/{name}/optimize_paths")
def optimize_paths(name: str, req: OptimizePathRequest = OptimizePathRequest()):
    try:
        poi_dict = req.poi_config.model_dump() if req.poi_config else None
        res = path_verification_service.optimize_template_paths(
            name,
            mode=req.mode,
            poi_config=poi_dict,
            options=req.options.model_dump()
        )
        return res
    except FileNotFoundError as e:
        raise HTTPException(status_code=404, detail=str(e))
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))
    except Exception as e:
        logger.error(f"Optimization error for '{name}': {e}", exc_info=True)
        raise HTTPException(status_code=500, detail=f"Optimization failed: {str(e)}")


@router.get("/templates/{name}/verification_report")
def get_verification_report(name: str, state_type: str = "raw", use_opt: bool = False):
    """
    Retrieves saved verification report from disk if exists.
    """
    actual_state = "opt" if use_opt else state_type
    report = path_verification_service.get_saved_report(name, state_type=actual_state)
    if report is not None:
        return report
    raise HTTPException(status_code=404, detail=f"No saved verification report found for '{actual_state}'.")


@router.get("/robot/urdf_tool_tcp")
def get_robot_urdf_tcp():
    """Returns TCP offset extracted directly from URDF."""
    return path_verification_service.get_urdf_tcp()


@router.get("/robot/anchor_pose")
def get_robot_anchor_pose(source: str = "home"):
    """
    Returns reference TCP poses for POI constraint configuration:
    1. Current live robot TCP pose (if connected)
    2. Robot Home pose TCP orientation
    """
    from services.robot_service import robot_service
    from core.hardware.robot.cr5_kinematics import CR5Kinematics

    solver = CR5Kinematics()
    
    # 1. Dobot home joint angles must match DobotDriver.go_home(): JointMovJ([0, 0, -90, -90, -90, 0]).
    # Use controller-frame FK here so POI Home anchor Rx/Ry/Rz matches Dobot TCP pose convention.
    home_deg = [0.0, 0.0, -90.0, -90.0, -90.0, 0.0]
    home_rad = [math.radians(v) for v in home_deg]
    home_xyz, home_rpy_raw = solver.forward_controller(home_rad)
    home_xyz = [round(float(v), 2) for v in home_xyz]
    home_rpy = [round(float(v), 2) for v in home_rpy_raw]

    # 2. Live robot TCP pose if connected
    live_pose, _ = robot_service.get_current_pose()
    live_rpy = None
    live_xyz = None
    if live_pose and len(live_pose) >= 6:
        live_xyz = [round(float(live_pose[0]), 2), round(float(live_pose[1]), 2), round(float(live_pose[2]), 2)]
        live_rpy = [round(float(live_pose[3]), 2), round(float(live_pose[4]), 2), round(float(live_pose[5]), 2)]

    if source not in {"home", "live"}:
        raise HTTPException(status_code=400, detail="source must be 'home' or 'live'")

    if source == "live":
        if not live_rpy:
            raise HTTPException(status_code=409, detail="Live robot TCP pose is unavailable. Connect the robot before capturing a live POI anchor pose.")
        selected_rpy = live_rpy
        selected_xyz = live_xyz
    else:
        selected_rpy = home_rpy
        selected_xyz = home_xyz

    return {
        "source": source,
        "is_connected": robot_service._is_connected,
        "rpy_deg": selected_rpy,
        "xyz_mm": selected_xyz,
        "default_tolerance_rpy_deg": list(DEFAULT_POI_TOLERANCE_RPY_DEG),
        "home_pose": {
            "xyz_mm": home_xyz,
            "rpy_deg": home_rpy,
            "joints_deg": home_deg
        },
        "live_pose": {
            "xyz_mm": live_xyz or home_xyz,
            "rpy_deg": live_rpy or home_rpy
        } if live_pose else None
    }


@router.get("/templates/{name}/summary")
def get_template_summary(name: str):
    """
    Returns complete atomic summary of template data in a single round-trip:
    files, masks, raw/opt/poi paths, cached raw/opt/poi reports, and URDF tool TCP info.
    """
    template_path = os.path.join(TEMPLATE_GROUP_DIR, name)
    if not os.path.exists(template_path):
        raise HTTPException(status_code=404, detail="Template not found")
        
    # 1. File items
    files = []
    for item in os.listdir(template_path):
        item_path = os.path.join(template_path, item)
        if os.path.isfile(item_path):
            size = os.path.getsize(item_path)
            ctime = os.path.getctime(item_path)
            files.append({"name": item, "size": size, "ctime": ctime})
    files.sort(key=lambda x: x["ctime"], reverse=True)

    file_set = {f["name"] for f in files}
    has_image = "scan.jpg" in file_set
    has_depth = "scan.depth.npy" in file_set or "scan.depth.png" in file_set
    has_mesh = "scan.mesh.ply" in file_set or "scan.mesh.stl" in file_set

    # 2. Masks
    masks_dict = sam_service.get_template_masks(template_path)
    masks_data = masks_dict.get("masks", []) if masks_dict else []

    # 3. Paths (Raw, Opt, POI)
    raw_paths_data = manual_path_service.load_manual_paths(name, state_type="raw")
    opt_paths_data = manual_path_service.load_manual_paths(name, state_type="opt")
    poi_paths_data = manual_path_service.load_manual_paths(name, state_type="poi")

    raw_paths = raw_paths_data.get("paths", [])
    opt_paths = opt_paths_data.get("paths", []) if opt_paths_data.get("paths") else []
    poi_paths = poi_paths_data.get("paths", []) if poi_paths_data.get("paths") else []
    standoff = raw_paths_data.get("standoff_distance_mm", 150.0)

    # 4. Diagnostic Reports (Raw, Opt, POI)
    raw_report = path_verification_service.get_saved_report(name, state_type="raw")
    opt_report = path_verification_service.get_saved_report(name, state_type="opt")
    poi_report = path_verification_service.get_saved_report(name, state_type="poi")

    # 5. URDF TCP
    urdf_tcp = path_verification_service.get_urdf_tcp()

    return {
        "template": name,
        "files": files,
        "has_image": has_image,
        "has_depth": has_depth,
        "has_mesh": has_mesh,
        "masks": masks_data,
        "raw_paths": raw_paths,
        "opt_paths": opt_paths,
        "poi_paths": poi_paths,
        "standoff_distance_mm": standoff,
        "raw_report": raw_report,
        "opt_report": opt_report,
        "poi_report": poi_report,
        "urdf_tcp": urdf_tcp
    }


# ─── Real Robot Path Waypoint Execution (move_l_queue) ───────────────────────

class ExecuteYamlPathRequest(BaseModel):
    file_name: str
    path_id: Optional[int] = None # None or 0 means all paths, 1-based index/ID
    # Linear motion parameters (MoveL queue)
    speed_l: Optional[float] = None # mm/s (笛卡尔线速度)
    acc_l: Optional[float] = None   # % (笛卡尔加速度百分比)
    # Joint motion parameters (Go Home & MovJ to start point)
    speed_j: Optional[float] = None # % (关节速度百分比)
    acc_j: Optional[float] = None   # % (关节加速度百分比)
    cp_ratio: Optional[int] = 100
    # Backward compatibility fields
    speed: Optional[float] = None
    acc: Optional[float] = None

@router.get("/templates/{name}/yaml_paths_info")
def get_yaml_paths_info(name: str, file_name: str):
    """
    读取并解析指定 YAML 轨迹文件，返回包含的全部子路径及其点数摘要，供前端交互选择。
    """
    template_path = os.path.join(TEMPLATE_GROUP_DIR, name)
    file_path = os.path.join(template_path, file_name)
    if not os.path.exists(file_path):
        raise HTTPException(status_code=404, detail=f"File '{file_name}' not found in template '{name}'")
    
    try:
        with open(file_path, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f) or {}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to parse YAML file: {e}")

    paths = data.get("paths", [])
    path_infos = []
    total_pts = 0
    for idx, p in enumerate(paths):
        pts = p.get("points", [])
        pts_count = len(pts)
        total_pts += pts_count
        path_infos.append({
            "path_id": p.get("path_id", idx + 1),
            "name": p.get("name", f"Path {idx + 1}"),
            "point_count": pts_count
        })

    return {
        "template": name,
        "file_name": file_name,
        "total_paths": len(path_infos),
        "total_points": total_pts,
        "paths": path_infos
    }

@router.post("/templates/{name}/execute_yaml_path")
def execute_yaml_path(name: str, req: ExecuteYamlPathRequest):
    """
    根据选择的路径 (单条或全部)，提取 Waypoints 位姿并通过 robot_service 发送到真实机械臂执行：
    1. 执行前先回到 Home 姿态 (使用关节参数 speed_j, acc_j)
    2. 第 1 条 path 的第 1 个 waypoint 使用 move_j 过去 (使用关节参数 speed_j, acc_j)
    3. 然后通过 robot_service.move_l_queue 连续直线执行 (使用线速度参数 speed_l, acc_l, cp_ratio)
    """
    if not robot_service.is_connected:
        raise HTTPException(status_code=400, detail="Robot is not connected. Please connect robot first in control panel.")

    template_path = os.path.join(TEMPLATE_GROUP_DIR, name)
    file_path = os.path.join(template_path, req.file_name)
    if not os.path.exists(file_path):
        raise HTTPException(status_code=404, detail=f"Path file '{req.file_name}' not found in template '{name}'")

    try:
        with open(file_path, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f) or {}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to read YAML file: {e}")

    raw_paths = data.get("paths", [])
    if not raw_paths:
        raise HTTPException(status_code=400, detail=f"No paths found in {req.file_name}")

    # 筛选待执行的路径
    target_paths = []
    if req.path_id is not None and req.path_id > 0:
        target_paths = [p for p in raw_paths if p.get("path_id") == req.path_id]
        if not target_paths and len(raw_paths) >= req.path_id:
            target_paths = [raw_paths[req.path_id - 1]]
    else:
        target_paths = raw_paths

    if not target_paths:
        raise HTTPException(status_code=400, detail=f"Selected path #{req.path_id} not found in {req.file_name}")

    # 解析 MoveL (笛卡尔线运动) 与 MoveJ (关节运动) 速度/加速度参数，优先沿用控制面板当前设定值
    curr_speed_l, curr_acc_l, curr_speed_j, curr_acc_j = robot_service.get_speed()
    speed_l = req.speed_l if req.speed_l is not None else (req.speed if req.speed is not None else curr_speed_l)
    acc_l = req.acc_l if req.acc_l is not None else (req.acc if req.acc is not None else curr_acc_l)
    speed_j = req.speed_j if req.speed_j is not None else curr_speed_j
    acc_j = req.acc_j if req.acc_j is not None else curr_acc_j

    # 1. 在执行 path 之前先回到 Home 姿态 (使用 MoveJ 关节运动参数)
    logger.info(f"execute_yaml_path: Moving robot to Home position (speed_j={speed_j}%, acc_j={acc_j}%) before trajectory execution...")
    home_ok, home_err = robot_service.go_home(speed=speed_j, acc=acc_j)
    if not home_ok:
        raise HTTPException(status_code=500, detail=f"Failed to go home before path execution: {home_err}")

    logger.info(f"execute_yaml_path: Robot has moved to Home position.")

    total_executed_pts = 0
    executed_names = []
    has_moved_j_to_start = False

    # Count total waypoints across all target paths for progress tracking
    all_poses_flat = []
    for p in target_paths:
        for pt in p.get("points", []):
            if pt.get("tcp_pose_base"):
                all_poses_flat.append(pt)
    total_waypoints_all = len(all_poses_flat)
    waypoints_done = 0

    for p_idx, path in enumerate(target_paths):
        pts = path.get("points", [])
        poses = []
        for pt in pts:
            tcp = pt.get("tcp_pose_base")
            if tcp:
                poses.append({
                    "x": float(tcp.get("x", 0.0)),
                    "y": float(tcp.get("y", 0.0)),
                    "z": float(tcp.get("z", 0.0)),
                    "rx": float(tcp.get("rx", 0.0)),
                    "ry": float(tcp.get("ry", 0.0)),
                    "rz": float(tcp.get("rz", 0.0)),
                    "is_radians": False, # YAML 中姿态角度为 deg
                })
        if poses:
            path_title = path.get("name", f"Path {p_idx + 1}")
            executed_names.append(path_title)

            # 2. 第 1 条有效 path 的第 1 个 waypoint，使用 move_j 快速安全过渡过去 (使用关节参数)
            if not has_moved_j_to_start:
                first_pt = poses[0]
                first_pose = [
                    first_pt["x"],
                    first_pt["y"],
                    first_pt["z"],
                    math.radians(first_pt["rx"]),
                    math.radians(first_pt["ry"]),
                    math.radians(first_pt["rz"])
                ]
                logger.info(f"execute_yaml_path: MovJ to 1st waypoint of {path_title} -> {first_pt} (speed_j={speed_j}%, acc_j={acc_j}%)")
                j_ok, j_err = robot_service.move_to_pose_j(first_pose, speed=speed_j, acc=acc_j)
                if not j_ok:
                    raise HTTPException(status_code=500, detail=f"Failed to MovJ to start waypoint of {path_title}: {j_err}")
                has_moved_j_to_start = True
                logger.info(f"execute_yaml_path: Robot has moved to 1st waypoint of {path_title}.")

            # 3. 批量发送 waypoints：第 1 条 path 的第 1 个 waypoint 已经通过 MovJ 到达，跳过
            #    后续 path 从第 1 个 waypoint 开始（没有做 MovJ 过渡）
            exec_poses = poses[1:] if p_idx == 0 else poses
            logger.info(f"execute_yaml_path: Executing {len(exec_poses)}/{len(poses)} waypoints on {path_title} "
                        f"(skip 1st: {p_idx == 0}, speed_l={speed_l} mm/s, acc_l={acc_l}%, cp={req.cp_ratio})")
            
            batch_ok, batch_err = True, ""
            if exec_poses:
                batch_ok, batch_err = robot_service.move_l_queue(
                    exec_poses,
                    speed=speed_l,
                    acc=acc_l,
                    cp_ratio=req.cp_ratio if req.cp_ratio is not None else 100,
                    wait=True
                )
            if not batch_ok:
                raise HTTPException(status_code=500, detail=f"Execution error on {path_title}: {batch_err}")
            waypoints_done += len(poses)
            total_executed_pts += len(poses)
            # 批量执行完成后广播一次路径进度
            robot_service.broadcast_exec_progress(
                current_waypoint=waypoints_done,
                total_waypoints=total_waypoints_all,
                path_idx=p_idx,
                total_paths=len(target_paths)
            )

    logger.info(f"execute_yaml_path: Returning robot to Home position after trajectory (speed_j={speed_j}%, acc_j={acc_j}%)...")
    home_ok, home_err = robot_service.go_home(speed=speed_j, acc=acc_j)
    if not home_ok:
        raise HTTPException(status_code=500, detail=f"Failed to go home after path execution: {home_err}")


    return {
        "status": "success",
        "message": f"Successfully executed {len(target_paths)} path(s) ({total_executed_pts} points): {', '.join(executed_names)}",
        "executed_paths": len(target_paths),
        "total_points": total_executed_pts
    }



