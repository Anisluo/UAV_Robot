#!/usr/bin/env python3
"""Train YOLOv8n on the Mavic 3 drone dataset.

Run on a Linux machine with PyTorch + CUDA + ultralytics installed:

    pip install ultralytics
    python train.py --data dataset.yaml --epochs 100

Outputs go to ./runs/detect/<run-name>/. The best model is
weights/best.pt — this is what export_onnx.py picks up.

Tested with ultralytics 8.0.x against a 200-image / single-class
training set captured via capture_drone_dataset.py.
"""
import argparse
import os
import sys


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--data", default="dataset.yaml",
                   help="path to dataset YAML (default: dataset.yaml in cwd)")
    p.add_argument("--model", default="yolov8n.pt",
                   help="base checkpoint to fine-tune (default yolov8n.pt — "
                        "small enough for RKNN, fits the RK3588 NPU)")
    p.add_argument("--epochs", type=int, default=100)
    p.add_argument("--imgsz", type=int, default=640,
                   help="must stay 640: matches what proc_npu's preprocess "
                        "letterboxes to; changing this breaks bbox decoding")
    p.add_argument("--batch", type=int, default=16)
    p.add_argument("--name", default="mavic3_drone",
                   help="run folder name under runs/detect/")
    args = p.parse_args()

    try:
        from ultralytics import YOLO
    except ImportError:
        print("ERROR: pip install ultralytics", file=sys.stderr)
        sys.exit(1)

    model = YOLO(args.model)
    model.train(
        data=args.data,
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=args.batch,
        name=args.name,
        device=0 if os.environ.get("CUDA_VISIBLE_DEVICES", "") != "" else "auto",
        # Aggressive augmentation helps when the dataset is small and the
        # capture session was done from one camera angle. Drop these
        # toward 0 if the trained model overfits on the reflection of
        # the white plate.
        degrees=15.0,
        translate=0.1,
        scale=0.5,
        fliplr=0.5,
        mosaic=1.0,
    )


if __name__ == "__main__":
    main()
