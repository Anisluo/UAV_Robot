# DJI Mavic 3 Battery — YOLOv8 Training Pipeline

The current live detector is `battery_detect.py` (classical CV, HSV +
contour, 26/26 on `battery_data/`). This folder contains the scripts to
**upgrade** that to a proper YOLOv8 model that runs on the RK3588 NPU —
same output format, so `battery_tracker.py` keeps working unchanged.

## Why a second pipeline exists

Classical CV is fast to iterate and needs zero training data, but it
breaks outside the distribution it was tuned on (different lighting,
different backgrounds, batteries partly occluded by a gripper finger,
etc.). A small YOLOv8n fine-tune on labelled battery images generalises
to those cases without manual re-tuning.

## Prerequisites

You need a **Linux x86_64** machine for the last step — `rknn-toolkit2`
(the ONNX → `.rknn` converter) only ships x86_64 Linux wheels. The RK3588
itself runs the resulting `.rknn` fine but cannot convert.

The rest of the pipeline (label, train, ONNX export) runs on any OS with
Python 3.9+.

## Steps

### 1. Auto-label battery_data/

The classical detector is already 100% on `battery_data/`, so we use it
to bootstrap labels instead of labelling by hand.

```sh
python auto_label.py --images ../../../battery_data --out labels/
```

Each image gets a YOLO-format `.txt` with one line
`class x_center y_center w h` (all normalised to [0,1], class=0
= battery). Review the annotated PNGs in `labels/visu/` and delete any
wrong ones.

### 2. Build the dataset directory

```sh
python split_dataset.py --images ../../../battery_data \
                        --labels labels \
                        --out dataset \
                        --val-split 0.2
```

Produces `dataset/{images,labels}/{train,val}/` plus `dataset/data.yaml`.

### 3. Train YOLOv8n

```sh
pip install ultralytics
yolo train model=yolov8n.pt data=dataset/data.yaml \
           epochs=80 imgsz=640 batch=8 name=battery_v1
```

Training checkpoint ends up at `runs/detect/battery_v1/weights/best.pt`.

### 4. Export to ONNX

```sh
yolo export model=runs/detect/battery_v1/weights/best.pt format=onnx \
            imgsz=640 opset=13
```

→ `runs/detect/battery_v1/weights/best.onnx`.

### 5. ONNX → RKNN  (Linux x86_64 host)

```sh
pip install rknn-toolkit2        # 2.x
python onnx_to_rknn.py \
    --onnx runs/detect/battery_v1/weights/best.onnx \
    --out  battery_v2.rknn \
    --quantize \
    --calib-dir ../../../battery_data
```

### 6. Deploy to RK3588

```sh
scp battery_v2.rknn ubuntu@192.168.10.2:/home/ubuntu/UAV_Robot/
sudo systemctl restart uav-proc-npu
```

Then in HostGUI select **"电池 V2 (battery_v2)"** (strategy id=1) and
click **应用**. Unlike strategy 4 (classical CV via Python sidecar),
strategy 1 runs on the NPU directly through `proc_npu` — no Python
process involved.

## Files in this folder

- `auto_label.py` — runs `battery_detect.detect()` on every image in a
  folder, writes YOLO-format label files + overlays.
- `split_dataset.py` — 80/20 random split into train/val with
  symlink-or-copy semantics; emits a matching `data.yaml`.
- `onnx_to_rknn.py` — thin wrapper around `rknn-toolkit2` tuned for the
  640×640 YOLOv8 export with int8 quantisation calibrated on
  `battery_data/`.
