#!/usr/bin/env python3
"""Convert a YOLOv8 ONNX model to RKNN (RK3588 target, int8 quantised).

Runs on Linux x86_64 only — the `rknn-toolkit2` wheels are not published
for aarch64 or Windows, which is why this step cannot happen on the
RK3588 itself. The resulting .rknn file, however, runs on the RK3588 NPU
via librknnrt that's already installed there.
"""
import argparse
import pathlib
import sys


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--onnx",      required=True)
    ap.add_argument("--out",       required=True)
    ap.add_argument("--imgsz",     type=int, default=640)
    ap.add_argument("--quantize",  action="store_true")
    ap.add_argument("--calib-dir", default=None,
                    help="folder of calibration images (required for --quantize)")
    ap.add_argument("--target",    default="rk3588")
    args = ap.parse_args()

    try:
        from rknn.api import RKNN
    except ImportError:
        sys.exit("pip install rknn-toolkit2 (Linux x86_64 only)")

    calib = None
    if args.quantize:
        if not args.calib_dir:
            sys.exit("--quantize requires --calib-dir")
        # rknn-toolkit2 wants a text file listing calibration image paths.
        calib = pathlib.Path("/tmp/calib_list.txt")
        paths = sorted(str(p) for p in pathlib.Path(args.calib_dir).iterdir()
                       if p.suffix.lower() in (".jpg", ".jpeg", ".png"))
        if not paths:
            sys.exit(f"no images in {args.calib_dir}")
        calib.write_text("\n".join(paths) + "\n")

    rknn = RKNN(verbose=True)

    # Match proc_npu's runtime preprocessing: input is UINT8 NHWC;
    # proc_npu does NOT divide by 255 itself — RKNN applies mean/std
    # internally based on what we configure here.
    rknn.config(mean_values=[[0, 0, 0]],
                std_values=[[255, 255, 255]],
                target_platform=args.target,
                quantized_dtype="asymmetric_quantized-8",
                quant_img_RGB2BGR=False,
                optimization_level=3)

    assert rknn.load_onnx(model=args.onnx) == 0
    assert rknn.build(do_quantization=args.quantize,
                      dataset=str(calib) if calib else None) == 0
    assert rknn.export_rknn(args.out) == 0
    rknn.release()
    print(f"wrote {args.out}")


if __name__ == "__main__":
    main()
