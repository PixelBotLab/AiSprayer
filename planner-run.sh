#!/bin/bash

# 默认工艺参数
WIDTH=${1:-80}
OVERLAP=${2:-0.2}
DIST=${3:-150}

echo "[*] 开始批量规划任务 (幅宽: ${WIDTH}mm, 重叠: ${OVERLAP}, 距离: ${DIST}mm)..."

for angle in 0 90 180 270
do
    echo "------------------------------------------------"
    echo "[*] 处理视角: ${angle} 度"
    conda run -n inexbot python vision/planner.py \
        --input_dir vision/data/${angle} \
        --width ${WIDTH} \
        --overlap ${OVERLAP} \
        --dist ${DIST}
done

echo "------------------------------------------------"
echo "[*] 正在合并对比图..."
conda run -n inexbot python vision/merge_plots.py
echo "[+] 任务全部完成。"