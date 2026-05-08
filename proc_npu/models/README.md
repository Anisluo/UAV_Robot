# proc_npu 模型目录

本目录用于存放 `proc_npu` 运行时加载的 RKNN 模型文件。

> 注意：真实 `.rknn` 权重**不入库**（体积大、目标板私有）。部署时通过 `scp / rsync` 推送到目标板 `@PROJECT_ROOT@` 的工作目录。`proc_npu/app/npu_application.cpp` 按 `strategy_model_name()` 以**相对路径**打开文件，因此模型放在 systemd `WorkingDirectory` 根下即可。

## 1. 当前策略与模型文件清单

| strategy_id | 枚举 | 期望文件名 | 说明 |
|-------------|------|-----------|------|
| 0 | `UAV_STRATEGY_DEFAULT`       | `default.rknn`      | 方形电池检测 |
| 1 | `UAV_STRATEGY_BATTERY_V2`    | `battery_v2.rknn`   | 电池 V2 |
| 2 | `UAV_STRATEGY_CUSTOM`        | `custom.rknn`       | 自定义占位 |
| 3 | `UAV_STRATEGY_FACE`          | —                   | 由 `face_tracker.py` 接管 |
| 4 | `UAV_STRATEGY_BATTERY_CV`    | —                   | 由 `battery_tracker.py` 接管 |
| 5 | `UAV_STRATEGY_MAVIC3_DRONE`  | **`mavic3_drone.rknn`** | DJI Mavic 3 机身识别（单类） |

`proc_npu` 的 YOLO 解码同时兼容三种输出格式（自动从 `n_outputs` 识别），输入固定 **640×640** letterbox：

- **3 输出**（airockchip 约定，推荐）：每 stride 一个合并张量 `[1, 64+nc, H, W]`
- **9 输出**（kaylorchen 约定）：每 stride 三个张量 `[1,64,H,W]` DFL + `[1,nc,H,W]` cls + `[1,1,H,W]` score-sum
- **1 输出**（ultralytics 默认 export）：`[1, 4+nc, 8400]`

只要训练/导出落在其中一种，就能直接复用现有管线。

---

## 2. `mavic3_drone.rknn` 推荐制作流程

整体路径：**公共数据集 → YOLOv8n 微调 → ONNX → rknn-toolkit2 → .rknn**。三步都有 Rockchip 官方背书，RK3588 NPU 上 INT8 量化后推理 ~10-15 ms/帧。

### 2.1 数据集（推荐三选一 + 自采合并）

三个公开 YOLO 数据集，下载即用：

- **[chuanenlin/drone-net](https://github.com/chuanenlin/drone-net)** — 2664 张 DJI 无人机图（含 Phantom/Mavic 系列），YOLO 格式标注。**最接近 Mavic 3 形态**，首选。
- **[doguilmak/Drone-Detection-YOLOv8x](https://github.com/doguilmak/Drone-Detection-YOLOv8x)** — YOLOv8 单类 `drone`，含训练脚本
- **[Ayushkumawat/Advanced-Aerial-Drone-Detection-System](https://github.com/Ayushkumawat/Advanced-Aerial-Drone-Detection-System)** — 1400 张多型号无人机

Roboflow Universe 两个现成数据集（需注册免费账号导出 YOLO 格式）：

- [yolo drone detection dataset by Ivonne](https://universe.roboflow.com/ivonne/yolo-drone-detection-dataset) — 253 张
- [drone-detection-data-set-yolov7 by pexej](https://universe.roboflow.com/drone-detection-pexej/drone-detection-data-set-yolov7) — 训练 YOLOv10 的基础集

**Mavic 3 专属数据自采（推荐补充）：**
针对你的实际机械臂视角和光照，自采 100–300 张 Mavic 3 图（桌面、不同角度、不同背景），用 [Roboflow](https://roboflow.com) 或 [labelImg](https://github.com/HumanSignal/labelImg) 标注成 `drone` 单类，拼进上面的公共数据集再微调 30–50 epoch，泛化显著提升。Google 图片搜 `DJI Mavic 3`、`DJI Mavic 3 Classic`、`Mavic 3 Pro` 能补充大量网络图，但**务必保留一部分目标场景实拍**做测试集，避免训练-部署分布偏移。

### 2.2 训练（x86 开发机，不在目标板）

```bash
pip install ultralytics==8.3.*
# data.yaml: names: ['drone']，nc: 1
yolo detect train \
    model=yolov8n.pt \
    data=drone.yaml \
    imgsz=640 epochs=100 batch=32 \
    project=runs/drone name=mavic3
```

产物：`runs/drone/mavic3/weights/best.pt`

### 2.3 导出 ONNX（Rockchip 专用导出，关键一步！）

**不要用 Ultralytics 默认 `yolo export`**。Rockchip 官方提供了专门为 RKNN 解码优化过的 YOLOv8 导出脚本，头部拆成 3 输出，能直接套进 `proc_npu/src/postprocess.cpp` 现有的 DFL 解析。

参考 **[airockchip/rknn_model_zoo](https://github.com/airockchip/rknn_model_zoo)** → `examples/yolov8/python/`：

```bash
git clone https://github.com/airockchip/ultralytics_yolov8.git
cd ultralytics_yolov8
# 改 ultralytics/cfg/default.yaml 里 model 路径为 best.pt
python ./ultralytics/engine/exporter.py
# 产出 best.onnx
```

校验导出是否正确：`netron best.onnx` 查看，顶层应有 **3 个输出**（stride 8/16/32），每个形状 `[1, 64+nc, H, W]`，与现有 `proc_npu/src/npu_infer.cpp:76-83` 的 `nc = dims[1] - 64` 计算逻辑一致。

### 2.4 ONNX → RKNN（x86 开发机，装 rknn-toolkit2）

```bash
git clone https://github.com/airockchip/rknn_model_zoo.git
cd rknn_model_zoo/examples/yolov8/python

# 准备 20-100 张真实场景图做 INT8 校准
#   dataset.txt 每行一个图片路径

python convert.py \
    /path/to/best.onnx \
    rk3588 \
    i8 \
    /path/to/mavic3_drone.rknn
```

`i8` = INT8 量化（默认，速度最快）；若精度掉得严重可改 `fp`（FP16）。

### 2.5 部署到目标板

```bash
scp mavic3_drone.rknn ubuntu@192.168.1.101:/home/ubuntu/UAV_Robot/
ssh ubuntu@192.168.1.101 'sudo systemctl restart uav-proc-npu.service'
```

HostGUI 选择「御三无人机识别 (Mavic3 drone)」下拉 → `npu.set_strategy` 触发 `load_model("mavic3_drone.rknn")`。

---

## 3. 参考资料

**开源无人机检测项目：**
- [doguilmak/Drone-Detection-YOLOv8x](https://github.com/doguilmak/Drone-Detection-YOLOv8x) — YOLOv8 单类 drone
- [doguilmak/Drone-Detection-YOLOv7](https://github.com/doguilmak/Drone-Detection-YOLOv7) — YOLOv7 版
- [hawkinglai/uav_det](https://github.com/hawkinglai/uav_det) — YOLOv8 UAV 检测
- [Ayushkumawat/Advanced-Aerial-Drone-Detection-System](https://github.com/Ayushkumawat/Advanced-Aerial-Drone-Detection-System) — YOLOv5 + OpenCV 实时
- [github.com/topics/drone-detection](https://github.com/topics/drone-detection) — GitHub 主题聚合

**RKNN 转换/部署：**
- [airockchip/rknn_model_zoo](https://github.com/airockchip/rknn_model_zoo) — Rockchip 官方，YOLOv5/v8 转换范例
- [Ultralytics RKNN 集成文档](https://docs.ultralytics.com/integrations/rockchip-rknn/)
- [Radxa YOLOv8 部署指南](https://docs.radxa.com/en/rock5/rock5c/app-development/rknn_toolkit_lite2_yolov8)
- [ZIFENG278/yolov8_rknn-toolkit2-lite](https://github.com/ZIFENG278/yolov8_rknn-toolkit2-lite) — RK3588 端推理参考

**数据集：**
- [VisDrone](https://docs.ultralytics.com/datasets/detect/visdrone/) — 注意：这是**从无人机拍摄**的地面目标数据集，**不适合**做"识别无人机"。列此处仅为避免误用。
