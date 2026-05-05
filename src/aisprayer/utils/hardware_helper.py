import numpy as np

def verify_hardware_consistency(live=None, scan=None, calib=None):
    """
    通用硬件一致性校验网关。
    支持传入相机对象(live)或参数字典。
    :param live: 实时相机对象(BaseCamera) 或 参数字典
    :param scan: 采集元数据字典 (可选)
    :param calib: 标定结果元数据字典 (可选)
    :return: (True, "") if pass, else (False, error_msg)
    """
    def _get_params(p):
        # 兼容性处理：如果包含 camera_params 键，则取其内部字典
        if isinstance(p, dict) and "camera_params" in p:
            return p["camera_params"]
        return p

    def get_k(p): 
        p_inner = _get_params(p)
        k_list = p_inner.get("intrinsic_matrix", [])
        if not k_list or len(k_list) == 0:
            return np.array([])
        try:
            return np.array(k_list, dtype=float)
        except Exception:
            return np.array([])
        
    def get_model(p): 
        p_inner = _get_params(p)
        return p_inner.get("camera_model", "unknown")
        
    def get_res(p): 
        p_inner = _get_params(p)
        return (p_inner.get("width"), p_inner.get("height"))

    # 1. 如果 live 是相机对象，则自动提取为标准化字典
    if live is not None and not isinstance(live, dict):
        cam = live
        K, D = cam.get_intrinsics()
        live = {
            "camera_model": getattr(cam, "model_name", getattr(cam, "model", "unknown")),
            "width": getattr(cam, "width", None),
            "height": getattr(cam, "height", None),
            "intrinsic_matrix": K.tolist() if K is not None else [],
            "distortion_coeffs": D.tolist() if D is not None else []
        }

    # 2. 提取所有存在的源进行两两比对
    sources = []
    if live: sources.append(("实时硬件", live))
    if scan: sources.append(("采集数据", scan))
    if calib: sources.append(("标定文件", calib))

    if len(sources) < 2:
        return True, "样本不足，跳过校验"

    # 3. 交叉比对
    for i in range(len(sources)):
        for j in range(i + 1, len(sources)):
            name1, p1 = sources[i]
            name2, p2 = sources[j]

            if get_model(p1) != get_model(p2):
                return False, f"相机型号冲突! [{name1}:{get_model(p1)}] vs [{name2}:{get_model(p2)}]"

            if get_res(p1) != get_res(p2):
                return False, f"分辨率不匹配! [{name1}:{get_res(p1)}] vs [{name2}:{get_res(p2)}]"

            k1, k2 = get_k(p1), get_k(p2)
            if k1.size > 0 and k2.size > 0:
                if not np.allclose(k1, k2, atol=1.0):
                    diff = np.abs(k1 - k2).max()
                    return False, f"内参矩阵差异过大 (max_diff={diff:.4f})! [{name1}] vs [{name2}]。请确认是否更换了相机或调整了分辨率。"

    return True, "硬件一致性校验通过"
