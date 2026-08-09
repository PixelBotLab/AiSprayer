import os
import cv2
import numpy as np

# VisionProcessor 导入被移至用到它的函数内，避免 2D 模块不必要地加载 open3d

def split_jeans_mask(mask_2d, depth_threshold_ratio=0.1, overlap_px=12):
    """
    Split the YOLO mask into two (left and right) if a deep crotch is detected.
    Otherwise, returns the single mask.
    :param mask_2d: Boolean mask (H, W)
    :param depth_threshold_ratio: Minimum depth of the crotch defect relative to bounding box height
    :param overlap_px: Width of the shared band around the split line. Independent mesh
        reconstruction removes boundary triangles, so both legs must retain this band to cover
        the seam after they are combined.
    :return: List of masks (1 or 2 masks)
    """
    mask_uint8 = mask_2d.astype(np.uint8) * 255
    contours, _ = cv2.findContours(mask_uint8, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return [mask_2d]
        
    c = max(contours, key=cv2.contourArea)
    hull_indices = cv2.convexHull(c, returnPoints=False)
    
    # We need at least 3 points for convexity defects
    if len(hull_indices) < 3:
        return [mask_2d]
        
    defects = cv2.convexityDefects(c, hull_indices)
    if defects is None:
        return [mask_2d]
        
    x, y, w, h = cv2.boundingRect(c)
    max_depth = 0
    best_defect = None
    
    for i in range(defects.shape[0]):
        row = defects[i].flatten()
        s, e, f, d = row
        depth = d / 256.0
        
        start = tuple(c[s][0])
        end = tuple(c[e][0])
        far = tuple(c[f][0]) # crotch point
        
        # Check if depth is significant
        if depth > max_depth:
            dist_se = np.hypot(start[0] - end[0], start[1] - end[1])
            # The start-end distance should be somewhat wide (two legs)
            if dist_se > 0.2 * w and depth > depth_threshold_ratio * h:
                # The crotch point should be roughly between the two leg cuffs on X
                min_x = min(start[0], end[0])
                max_x = max(start[0], end[0])
                if min_x < far[0] < max_x:
                    max_depth = depth
                    best_defect = (start, end, far)
                    
    if best_defect is None:
        print("[*] Segmentation: No significant crotch found (likely 90/270 degrees). Returning single mask.")
        return [mask_2d]
        
    print(f"[*] Segmentation: Crotch found! Splitting into two legs. (Depth: {max_depth:.1f}px)")
    start, end, far = best_defect
    # Ensure start is left, end is right
    if start[0] > end[0]:
        start, end = end, start
        
    P_left = np.array(start)
    P_right = np.array(end)
    P_crotch = np.array(far)
    
    # Vector connecting left and right cuffs
    V = P_right - P_left
    
    H_img, W_img = mask_2d.shape
    y_idx, x_idx = np.mgrid[0:H_img, 0:W_img]
    
    dx = x_idx - P_crotch[0]
    dy = y_idx - P_crotch[1]
    
    # Signed perpendicular coordinate of each pixel relative to the split line.
    # Normalizing by |V| makes overlap_px a true pixel width regardless of the
    # distance between the two convex-hull points.
    v_norm = np.linalg.norm(V)
    if v_norm < 1e-6:
        return [mask_2d]
    signed_distance = (dx * V[0] + dy * V[1]) / v_norm

    # Preserve a shared band around the split line. Each leg is still reconstructed
    # and planned independently, but the overlap prevents the two open mesh
    # boundaries from producing an uncovered gap at the seam.
    overlap_px = max(float(overlap_px), 0.0)
    mask_left = mask_2d & (signed_distance <= overlap_px)
    mask_right = mask_2d & (signed_distance > -overlap_px)
    
    # Clean up to remove isolated noise from the split
    def keep_largest_cc(m):
        num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(m.astype(np.uint8), connectivity=8)
        if num_labels <= 1:
            return m
        largest_label = 1 + np.argmax(stats[1:, cv2.CC_STAT_AREA])
        return labels == largest_label
        
    mask_left = keep_largest_cc(mask_left)
    mask_right = keep_largest_cc(mask_right)
    
    print(f"[*] Segmentation: Keeping a {overlap_px:.1f}px overlap at the leg seam.")
    return [mask_left, mask_right]

def process_jeans_with_segmentation(raw_point_cloud, yolo_mask_2d, config, output_dir=None, base_name="jeans_smoothed"):
    """
    Splits the jeans mask (if needed) and processes them into 1 or 2 meshes.
    Returns a list of generated mesh paths.
    """
    from aisprayer.core.vision.vision_processor import VisionProcessor, DEFAULT_DATA_DIR
    if output_dir is None:
        output_dir = DEFAULT_DATA_DIR
        
    processor = VisionProcessor.from_config_dict(config)
    
    masks = split_jeans_mask(yolo_mask_2d)
    mesh_paths = []
    
    if len(masks) == 1:
        out_path = f"{base_name}.obj"
        print(f"[*] Extracting single mesh: {out_path}")
        mesh_path = processor.filter_and_smooth_jeans(raw_point_cloud, masks[0], output_dir=output_dir, output_name=out_path)
        mesh_paths.append(mesh_path)
    else:
        for i, mask in enumerate(masks):
            part_name = "left" if i == 0 else "right"
            out_path = f"{base_name}_{part_name}.obj"
            print(f"[*] Extracting part {i+1}/2 ({part_name} leg): {out_path}")
            mesh_path = processor.filter_and_smooth_jeans(raw_point_cloud, mask, output_dir=output_dir, output_name=out_path)
            mesh_paths.append(mesh_path)
            
    return mesh_paths
