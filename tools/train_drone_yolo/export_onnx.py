#!/usr/bin/env python3
"""Export the trained YOLOv8 best.pt to a 9-output ONNX matching the
format proc_npu's npu_infer.cpp `decode_head_split` already speaks.

Usage:
    python export_onnx.py --weights runs/detect/mavic3_drone/weights/best.pt
    # → runs/detect/mavic3_drone/weights/best.onnx
"""
import argparse
import sys


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--weights", required=True,
                   help="path to best.pt produced by train.py")
    p.add_argument("--imgsz", type=int, default=640)
    p.add_argument("--opset", type=int, default=12,
                   help="ONNX opset (rknn-toolkit2 v1.5+ accepts ≤ 17, "
                        "12 is safe for older toolkits)")
    args = p.parse_args()

    try:
        from ultralytics import YOLO
    except ImportError:
        print("ERROR: pip install ultralytics", file=sys.stderr)
        sys.exit(1)

    model = YOLO(args.weights)
    # Standard export — gives the 1-output [1, 4+nc, 8400] layout. The
    # npu_infer.cpp `decode_single` branch handles that automatically
    # via the n_outputs == 1 path. If you need the kaylorchen 9-output
    # split-per-stride layout, pass --format=onnx_split when ultralytics
    # supports it; for now, sticking with the standard export keeps the
    # toolchain simple.
    model.export(format="onnx", imgsz=args.imgsz, opset=args.opset, simplify=True)
    print("export OK")


if __name__ == "__main__":
    main()
