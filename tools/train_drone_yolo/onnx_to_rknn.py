#!/usr/bin/env python3
"""Convert YOLOv8 ONNX → RKNN for the RK3588 NPU.

Run on a Linux x86_64 machine with rknn-toolkit2 installed
(https://github.com/airockchip/rknn-toolkit2). Works against best.onnx
exported by export_onnx.py.

    pip install rknn-toolkit2  # exact version per your kernel + librknnrt
    python onnx_to_rknn.py \\
        --onnx runs/detect/mavic3_drone/weights/best.onnx \\
        --calib  ../../drone_dataset/images/train \\
        --output mavic3_drone.rknn

The calib directory is 50–200 representative training JPGs — RKNN's
quantization needs to see real frames to set int8 scales.
"""
import argparse
import os
import sys
from pathlib import Path


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--onnx", required=True, help="path to best.onnx")
    p.add_argument("--calib", required=True,
                   help="directory of representative JPGs for quantization "
                        "(use the train split from the captured dataset)")
    p.add_argument("--output", default="mavic3_drone.rknn",
                   help="output RKNN path (default: mavic3_drone.rknn)")
    p.add_argument("--target", default="rk3588",
                   choices=["rk3566", "rk3568", "rk3588", "rk3576"])
    p.add_argument("--no-quant", action="store_true",
                   help="skip int8 quantization (fp16 model — slower but "
                        "no need to tune calibration)")
    args = p.parse_args()

    try:
        from rknn.api import RKNN
    except ImportError:
        print("ERROR: pip install rknn-toolkit2", file=sys.stderr)
        sys.exit(1)

    calib_dir = Path(args.calib)
    if not calib_dir.is_dir():
        sys.exit(f"calib dir not found: {calib_dir}")
    calib_files = sorted(p for p in calib_dir.iterdir()
                         if p.suffix.lower() in (".jpg", ".jpeg", ".png"))
    if not calib_files:
        sys.exit(f"no images in {calib_dir}")
    # Subsample to ~80 images for quantization — more is overkill and
    # makes the conversion glacial.
    if len(calib_files) > 80:
        calib_files = calib_files[::len(calib_files) // 80][:80]
    calib_list = "calib_list.txt"
    with open(calib_list, "w") as f:
        for img in calib_files:
            f.write(str(img.resolve()) + "\n")
    print(f"[rknn] {len(calib_files)} calibration images")

    rknn = RKNN(verbose=True)
    # YOLOv8 expects RGB normalized to [0, 1]. The C++ inference path
    # passes raw uint8 BGR via RKNN_TENSOR_UINT8 with mean/std applied
    # by the runtime, so feed the same mean/std here.
    rknn.config(
        mean_values=[[0, 0, 0]],
        std_values=[[255, 255, 255]],
        target_platform=args.target,
        # Match proc_npu's preprocess.cpp letterboxed input: NHWC BGR.
        quant_img_RGB2BGR=True,
    )
    if rknn.load_onnx(model=args.onnx) != 0:
        sys.exit("load_onnx failed")
    if rknn.build(do_quantization=not args.no_quant,
                  dataset=calib_list) != 0:
        sys.exit("build failed")
    if rknn.export_rknn(args.output) != 0:
        sys.exit("export_rknn failed")
    rknn.release()
    os.unlink(calib_list)
    print(f"[rknn] wrote {args.output}")
    print(f"[rknn] deploy with:  scp {args.output} "
          f"ubuntu@<rk3588>:/home/ubuntu/UAV_Robot/mavic3_drone.rknn")


if __name__ == "__main__":
    main()
