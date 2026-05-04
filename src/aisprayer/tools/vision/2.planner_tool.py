import argparse
import os
import sys
import cv2
import numpy as np
import yaml

# 1. 路径锚定策略
PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../.."))
sys.path.append(os.path.join(PROJECT_ROOT, "src"))

from aisprayer.core.vision.planner import AiSprayPlanner
from aisprayer.utils.config_helper import load_config, get_abs_path
from aisprayer.utils.hardware_helper import verify_hardware_consistency

def merge_previews(scan_dirs, output_path):
    """将多个视角预览图合并为一张总图"""
    images = []
    for s_dir in scan_dirs:
        img_path = os.path.join(s_dir, "plan.png")
        if os.path.exists(img_path):
            img = cv2.imread(img_path)
            target_w = 640
            h, w = img.shape[:2]
            img_small = cv2.resize(img, (target_w, int(h * target_w / w)))
            images.append(img_small)
    
    if len(images) < 1: return
    all_img = np.vstack(images)
    cv2.imwrite(output_path, all_img)
    print(f"[+] 已生成汇总预览图: {output_path}")

def resolve_garment_id(raw_id):
    """智能转换 ID: 001 -> trouser_001"""
    if os.path.exists(raw_id): return raw_id
    if raw_id.isdigit() and not raw_id.startswith("trouser_"):
        return f"trouser_{raw_id}"
    return raw_id

def main():
    parser = argparse.ArgumentParser(description="AiSprayer 路径规划工具 (极简版)")
    parser.add_argument("--config", default=os.path.join(PROJECT_ROOT, "configs/aisprayer_config.yaml"), help="配置文件路径")
    parser.add_argument("--input", nargs='+', help="指定任务: [ID] (全视角) 或 [ID 角度] (单视角)")
    args = parser.parse_args()

    # 1. 加载配置
    full_cfg = load_config(args.config, PROJECT_ROOT)
    v_cfg = full_cfg.get("vision", {})
    p_cfg = v_cfg.get("planner", {})
    output_root = get_abs_path(v_cfg.get("output_root", "data/runs"), PROJECT_ROOT)
    
    # 2. 加载标定文件与初始化规划器
    calib_path = get_abs_path(p_cfg.get("calib_path"), PROJECT_ROOT)
    if not os.path.exists(calib_path):
        print(f"[-] 找不到标定文件: {calib_path}"); return
    with open(calib_path, 'r') as f:
        calib_res = yaml.safe_load(f)

    planner = AiSprayPlanner(calib_path=calib_path, config=p_cfg)

    # 3. 任务分发逻辑
    tasks = {} # {garment_id: [angle_dirs]}
    
    if not args.input:
        # 模式 A: 自动锁定最新裤子
        subdirs = [d for d in os.listdir(output_root) if os.path.isdir(os.path.join(output_root, d))]
        if not subdirs:
            print("[-] 错误: data/runs 下没有发现任何采集数据"); return
        target_id = max(subdirs, key=lambda d: os.path.getmtime(os.path.join(output_root, d)))
        g_dir = os.path.join(output_root, target_id)
        tasks[target_id] = [os.path.join(g_dir, d) for d in ["0", "90", "180", "270"] if os.path.exists(os.path.join(g_dir, d))]
        print(f"[*] 自动模式: 锁定最新裤子 {target_id}")
    else:
        # 模式 B: 指定 ID 或 [ID 角度]
        raw_id = args.input[0]
        target_id = resolve_garment_id(raw_id)
        g_dir = os.path.join(output_root, target_id)
        
        if not os.path.exists(g_dir):
            print(f"[-] 错误: 找不到目录 {g_dir}"); return
            
        if len(args.input) == 1:
            # 全视角规划
            tasks[target_id] = [os.path.join(g_dir, d) for d in ["0", "90", "180", "270"] if os.path.exists(os.path.join(g_dir, d))]
        else:
            # 单视角规划
            angle = args.input[1]
            angle_dir = os.path.join(g_dir, angle)
            if not os.path.exists(angle_dir):
                print(f"[-] 错误: 视角目录不存在 {angle_dir}"); return
            tasks[target_id] = [angle_dir]

    # 4. 执行批量规划
    for g_id, ang_dirs in tasks.items():
        print(f"\n" + "="*50)
        print(f"[*] 正在规划裤子: {g_id} (共 {len(ang_dirs)} 个视角)")
        print("="*50)
        
        processed_dirs = []
        for s_dir in ang_dirs:
            ang_name = os.path.basename(s_dir)
            params_path = os.path.join(s_dir, "scan.params.yaml")
            
            # 强制硬件指纹校验 (Scan vs Calib)
            if not os.path.exists(params_path):
                print(f" [!] 跳过 {ang_name}: 缺失 scan.params.yaml"); continue
            with open(params_path, 'r') as f:
                s_info = yaml.safe_load(f)
            ok, msg = verify_hardware_consistency(scan=s_info, calib=calib_res)
            if not ok:
                print(f" [!] 跳过 {ang_name}: 硬件指纹不匹配! ({msg})"); continue

            # 4. 执行规划 (包含保存与渲染)
            print(f" -> 正在规划视角: {ang_name} ...")
            traj = planner.plan_garment(s_dir, garment_id=g_id, angle=ang_name)
            if traj:
                processed_dirs.append(s_dir)

        # 生成总预览图 (如果是单视角规划则不生成)
        if len(processed_dirs) > 0:
            summary_path = os.path.join(output_root, g_id, f"summary_{g_id}.jpg")
            merge_previews(processed_dirs, summary_path)

    print("\n[OK] 规划任务全部完成。")

if __name__ == "__main__":
    main()
