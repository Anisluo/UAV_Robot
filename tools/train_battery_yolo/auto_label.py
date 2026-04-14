#!/usr/bin/env python3
"""Auto-label battery images using the classical detector.

Output format is YOLO:
    class  x_center  y_center  width  height    (all normalised to [0, 1])

Class is always 0 (single-class "battery" dataset).
"""
import argparse
import os
import sys
import pathlib

import cv2

# Import the classical detector that already achieves 26/26 on battery_data.
HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
import battery_detect  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--images", required=True, help="input image folder")
    ap.add_argument("--out",    required=True, help="output label folder")
    ap.add_argument("--min-score", type=float, default=0.45,
                    help="drop detector boxes below this score")
    args = ap.parse_args()

    src = pathlib.Path(args.images)
    dst = pathlib.Path(args.out)
    (dst / "visu").mkdir(parents=True, exist_ok=True)

    files = sorted(p for p in src.iterdir()
                   if p.suffix.lower() in (".jpg", ".jpeg", ".png"))
    labelled = 0
    for f in files:
        img = cv2.imread(str(f))
        if img is None:
            print(f"[skip] {f.name}: cannot read"); continue
        h, w = img.shape[:2]
        boxes = battery_detect.detect(img)
        lines = []
        for (x1, y1, x2, y2, sc) in boxes:
            if sc < args.min_score: continue
            cx = ((x1 + x2) / 2.0) / w
            cy = ((y1 + y2) / 2.0) / h
            bw = (x2 - x1) / w
            bh = (y2 - y1) / h
            lines.append(f"0 {cx:.6f} {cy:.6f} {bw:.6f} {bh:.6f}")
            cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)

        label_path = dst / (f.stem + ".txt")
        label_path.write_text("\n".join(lines))
        if lines: labelled += 1

        vis = img if max(img.shape[:2]) <= 1024 else cv2.resize(
            img, None, fx=1024/max(img.shape[:2]), fy=1024/max(img.shape[:2]))
        cv2.imwrite(str(dst / "visu" / f.name), vis,
                    [cv2.IMWRITE_JPEG_QUALITY, 80])
        print(f"[{('ok' if lines else 'empty')}] {f.name} -> "
              f"{len(lines)} box(es)")

    print(f"\n{labelled}/{len(files)} images labelled")
    print(f"Review overlays in: {dst/'visu'}")


if __name__ == "__main__":
    main()
