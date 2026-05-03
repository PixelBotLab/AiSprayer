#!/bin/bash

# 默认工艺参数
WIDTH_MM=${1:-100}
OVERLAP=${2:-0.2}
DIST_MM=${3:-150}
V_STEP_MM=${4:-20.0}

echo "[*] 开始批量规划任务 (幅宽: ${WIDTH_MM}mm, 重叠: ${OVERLAP}, 距离: ${DIST_MM}mm, 纵向步长: ${V_STEP_MM}mm)..."

for angle in 0 90 180 270
do
    echo "------------------------------------------------"
    echo "[*] 处理视角: ${angle} 度"
    conda run -n inexbot python vision/planner.py \
        --input_dir vision/data/${angle} \
        --width_mm ${WIDTH_MM} \
        --overlap ${OVERLAP} \
        --dist_mm ${DIST_MM} \
        --v_step_mm ${V_STEP_MM}
done

echo "------------------------------------------------"
echo "[*] 正在合并对比图..."
conda run -n inexbot python vision/merge_plots.py
echo "[+] 任务全部完成。"