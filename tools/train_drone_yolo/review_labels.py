#!/usr/bin/env python3
"""Render the YOLO auto-labels onto every image and write to a
preview directory. Run this after the dataset is rsync'd from the
rk3588 — it lets the operator scan a contact sheet for obviously
wrong / missing boxes before opening labelImg / CVAT for the
detailed touch-up pass.

    python review_labels.py /path/to/drone_dataset

Outputs <root>/preview/<image>.jpg with the bbox drawn in green
(label present) or red (empty .txt = no detection bootstrapped).
"""
import argparse
from pathlib import Path
import cv2


def yolo_to_xyxy(line, w, h):
    cls, cx, cy, bw, bh = (float(x) for x in line.split())
    return (int(cls),
            int((cx - bw / 2) * w), int((cy - bh / 2) * h),
            int((cx + bw / 2) * w), int((cy + bh / 2) * h))


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("root", type=Path)
    args = p.parse_args()

    img_dir = args.root / "images"
    lbl_dir = args.root / "labels"
    out_dir = args.root / "preview"
    out_dir.mkdir(exist_ok=True)

    n_with = n_empty = 0
    for img_path in sorted(img_dir.iterdir()):
        if img_path.suffix.lower() not in (".jpg", ".jpeg", ".png"):
            continue
        lbl_path = lbl_dir / (img_path.stem + ".txt")
        img = cv2.imread(str(img_path))
        if img is None:
            continue
        h, w = img.shape[:2]
        if lbl_path.exists() and lbl_path.stat().st_size > 0:
            for line in open(lbl_path):
                line = line.strip()
                if not line:
                    continue
                cls, x1, y1, x2, y2 = yolo_to_xyxy(line, w, h)
                cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(img, f"drone", (x1, max(y1 - 6, 12)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            n_with += 1
        else:
            cv2.putText(img, "NO LABEL", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            n_empty += 1
        cv2.imwrite(str(out_dir / img_path.name), img,
                    [cv2.IMWRITE_JPEG_QUALITY, 80])
    print(f"with-label: {n_with}, empty: {n_empty}")
    print(f"preview:    {out_dir}")


if __name__ == "__main__":
    main()
