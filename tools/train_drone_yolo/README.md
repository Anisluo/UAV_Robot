# Mavic 3 drone YOLO retraining pipeline

The shipped `mavic3_drone.rknn` is a generic 80-class COCO YOLOv8;
"airplane" hits on the actual drone sit at score 0.45–0.55 with bbox
that often covers most of the frame. To get tighter, single-class
detection, retrain on a few hundred images of *your* drone in *your*
workspace.

End-to-end the loop is:

```
RK3588 capture  →  rsync to PC  →  hand-correct labels  →  YOLOv8 train
                                                                  ↓
proc_npu deploy ← scp mavic3_drone.rknn ←  rknn-toolkit2 quantize ← ONNX export
```

Steps:

## 1. Capture frames on the RK3588

```bash
# on the rk3588
python3 /home/ubuntu/UAV_Robot/tools/capture_drone_dataset.py \
    --count 300 --interval 4 --out /home/ubuntu/drone_dataset
```

While it runs, slowly rotate / move the drone through the poses the
arm camera will see in production: front, back, both side angles,
both rotor pairs visible, partial occlusion, the four propellers
folded vs. extended, etc. Aim for 300+ varied frames.

## 2. Pull to PC

```bash
rsync -avz ubuntu@192.168.1.101:/home/ubuntu/drone_dataset ./
```

## 3. Review labels

The capture script auto-bootstraps `labels/*.txt` from the live NPU
output, but COCO airplane is sloppy. Open in
[labelImg](https://github.com/HumanSignal/labelImg) /
[CVAT](https://www.cvat.ai/) / [Roboflow](https://roboflow.com/) and
fix obvious wrongs. The single class is `0 = drone`.

Tip: if the drone wasn't in frame for some captures, leave the label
file empty (no lines) — YOLO uses that as a hard negative.

## 4. Split into train / val

```bash
python tools/train_drone_yolo/split_dataset.py drone_dataset --val-ratio 0.15
```

## 5. Train

```bash
pip install ultralytics
cd tools/train_drone_yolo
python train.py --data dataset.yaml --epochs 100 --batch 16
```

Best weights end up at `runs/detect/mavic3_drone/weights/best.pt`.
Glance at `runs/detect/mavic3_drone/results.png` — mAP@0.5 should
reach 0.85+ on val with 200 hand-corrected images.

## 6. Export → ONNX → RKNN

```bash
# ONNX
python export_onnx.py --weights runs/detect/mavic3_drone/weights/best.pt

# RKNN (needs Linux x86_64 + rknn-toolkit2)
pip install rknn-toolkit2          # version that matches device's librknnrt
python onnx_to_rknn.py \
    --onnx  runs/detect/mavic3_drone/weights/best.onnx \
    --calib ../../drone_dataset/images/train \
    --output mavic3_drone.rknn \
    --target rk3588
```

`--no-quant` skips int8 quantization → larger fp16 model, no
calibration needed, but inference is ~2× slower.

## 7. Deploy

```bash
scp mavic3_drone.rknn ubuntu@192.168.1.101:/home/ubuntu/UAV_Robot/mavic3_drone.rknn
ssh ubuntu@192.168.1.101 'sudo systemctl restart uav-proc-npu'
```

After restart, send `npu.set_strategy {strategy: 5}` (HostGUI's NPU
dropdown does this) and the new model is live.

## C++ side (no edits needed)

`proc_npu/src/npu_infer.cpp` already auto-detects the model's output
shape (1, 3, or 9 outputs) and the class count (`nc_`). A
single-class YOLOv8 will arrive as `nc_ = 1` and the existing
`postprocess.cpp` per-class top-1 will return at most one drone bbox
per frame. The hold + EMA + 2-frame consensus stack from
`npu_pipeline.cpp` is unaffected. The score threshold default
(`0.45F` in `npu_application.cpp`) is fine — bumping it to 0.6+ once
the new model lands will be appropriate.
