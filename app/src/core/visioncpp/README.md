# visioncpp

视觉批处理（泊松重建、自动航点）的 C++ CLI，与现网 `app/src/core/vision/` **并行存在**。本阶段不改 FastAPI、不改旧 `vision/`。

## 构建

```bash
app/scripts/build.sh --only visioncpp
# 清理：app/scripts/build.sh -c --only visioncpp
```

产物：`app/src/core/visioncpp/build/vision_cli`

## CLI

```bash
vision_cli recon --template-dir DIR --calib FILE [--output-dir DIR]
vision_cli auto-path --template-dir DIR --calib FILE [--output FILE]
```

缺深度/掩码/mesh、缺手眼 T、缺内参 K → 退出码 2。禁止 Identity / 默认假 K。
成功时 stdout **最后一行**为 JSON；过程日志走 stderr。

对比测试请把 `--output` / `--output-dir` 指到临时目录，不要覆盖 `data/template_group/` 里的现网 ply/yaml。

## 与旧 Python 对比

```bash
python3 app/src/core/visioncpp/python/compare_with_python.py \
  --template-dir data/template_group/2026-09-03_225937 \
  --vision-cli app/src/core/visioncpp/build/vision_cli \
  --output-dir /tmp/visioncpp_compare
```

auto-path：同一张现网 `scan.mesh.ply`，位置 ≤1mm、姿态 ≤2°。
recon：C++ 网格到深度点云的距离（不要求与旧 PLY 拓扑一致）。

## 第三方

- `third_party/PoissonRecon`：Kazhdan Adaptive Solvers / PoissonRecon（MIT + BSD 头），单线程等值面。
- KD-Tree：本目录自实现（与 nanoflann 同用途）。
