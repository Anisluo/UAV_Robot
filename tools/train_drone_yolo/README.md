# Mavic 3 drone YOLOv8 retraining pipeline

Replace the shipped 80-class COCO `mavic3_drone.rknn` with a single-class
detector trained on your own DJI Mavic 3 captures. The COCO model
gives 0.45-0.55 score on the actual drone with sloppy bboxes; the
single-class model lands at 0.85+ with bboxes that hug the airframe.

End-to-end:

```
[RK3588] capture_drone_dataset.py
   ↓ rsync drone_dataset to GPU PC
[Linux+GPU PC] bash train_all.sh             # install deps + split + train + export ONNX
[Linux+rknn-toolkit2] python onnx_to_rknn.py # ONNX → quantized RKNN
[any] bash deploy_rknn.sh                    # scp + restart proc_npu
```

## 1. Capture frames on the RK3588

A 500-image bootstrap dataset is already produced by the existing
session. Refresh / extend it with:

```bash
ssh ubuntu@192.168.1.101 \
  'python3 /home/ubuntu/UAV_Robot/tools/capture_drone_dataset.py \
       --count 300 --interval 5 --out /home/ubuntu/drone_dataset \
       --prefix drone_v3'
```

While the script runs, slowly **rotate / tilt / move** the drone
through the poses the arm camera will see in production:

- 4 sides (front, back, left, right)
- propellers folded vs extended
- camera gimbal pointed every direction
- partial occlusion by a hand
- different lighting if practical

Aim for 300+ frames per session, 2-3 sessions = 1000+ frames total.

The script auto-bootstraps a YOLO `<cls> <cx> <cy> <w> <h>` label by
querying the gateway for the current class=4 detection and clipping
it to the white-plate ROI. The labels are sloppy (COCO airplane bbox
clipped to plate) but cut > 80 % of the manual click work.

## 2. Pull dataset to your training PC

```bash
rsync -avz ubuntu@192.168.1.101:/home/ubuntu/drone_dataset ./
```

## 3. Hand-correct labels

```bash
# Quick scan — render every label onto every image:
python3 tools/train_drone_yolo/review_labels.py drone_dataset
# open drone_dataset/preview/ in any image viewer; spot-check 50-100
# images for obviously wrong / missing boxes.

# Detailed touch-up:
pip install labelImg
labelImg drone_dataset/images drone_dataset/classes.txt
# arrow keys: prev/next   |   w: new bbox   |   d: delete
```

If the drone wasn't in frame for some captures, leave the
corresponding `labels/<file>.txt` empty (no lines) — YOLO trains
those as hard negatives.

## 4. Train (one shot)

```bash
cd tools/train_drone_yolo
bash train_all.sh                       # 100 epochs, yolov8n, batch 16
# or with overrides:
EPOCHS=200 BATCH=32 bash train_all.sh
```

This installs ultralytics if missing, splits 85/15 train/val, trains,
and exports best.pt → best.onnx.

Glance at `runs/detect/mavic3_drone/results.png` — mAP@0.5 should
reach 0.85+ on val once the labels are clean.

## 5. ONNX → RKNN

Has to run on a Linux x86_64 box with `rknn-toolkit2` installed (no
official Windows / macOS support). Same machine as training works
fine.

```bash
pip install rknn-toolkit2          # version that matches device's librknnrt
python3 onnx_to_rknn.py \
    --onnx  runs/detect/mavic3_drone/weights/best.onnx \
    --calib ../../drone_dataset/images/train \
    --output mavic3_drone.rknn \
    --target rk3588
```

Pass `--no-quant` to skip int8 quantization → larger fp16 model, no
calibration concerns, but ~2× slower inference.

## 6. Deploy

```bash
bash deploy_rknn.sh mavic3_drone.rknn ubuntu@192.168.1.101
```

`deploy_rknn.sh`:

- backs up the existing `mavic3_drone.rknn` with a timestamped suffix
- scp's the new model into place
- restarts `uav-proc-npu` and re-arms strategy id 5
- prints the journal tail so you can confirm `[npu_infer] loaded …
  outputs=N nc=1` (note `nc=1` — the single-class header).

After this lands, on HostGUI you should see UAV detection score
0.85+, bbox tight around the drone, no phantom hits when the drone
is removed. Bump the runtime threshold to 0.65 once confirmed:

```bash
ssh ubuntu@192.168.1.101 'sudo python3 -c "
import socket
s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
s.connect(\"/tmp/uav_proc_npu.ctrl.sock\")
s.sendall(b\"{\\\"id\\\":1,\\\"method\\\":\\\"npu.set_threshold\\\",\\\"params\\\":{\\\"threshold\\\":0.65}}\\n\")
print(s.recv(512).decode())
"'
```

## C++ side — no edits needed

`proc_npu/src/npu_infer.cpp` auto-detects the model's output shape
(1, 3, or 9 outputs) and the class count (`nc_`). A single-class
YOLOv8 lands as `nc_ = 1`. The existing `postprocess.cpp`
anchor-based union still applies (with one class it's a no-op
shortcut). The 600 ms hold + EMA + 2-frame consensus stack from
`npu_pipeline.cpp` is unaffected. The plate-anchored logic in
`tools/battery_detect.py` is unrelated.

## Layout cheat-sheet

```
tools/train_drone_yolo/
├── README.md            # ← you are here
├── dataset.yaml         # YOLOv8 single-class config
├── split_dataset.py     # 85/15 random split
├── train.py             # ultralytics wrapper
├── train_all.sh         # one-shot: install + split + train + export
├── export_onnx.py       # best.pt → best.onnx
├── onnx_to_rknn.py      # ONNX → RKNN with int8 calibration
├── review_labels.py     # render auto-labels onto images for spot-check
└── deploy_rknn.sh       # scp + restart npu + set strategy
```
