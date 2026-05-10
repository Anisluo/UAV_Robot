#!/usr/bin/env python3
"""Split <out>/images & <out>/labels into train / val sub-folders.

Run once after capture_drone_dataset.py + label review:

    python split_dataset.py /path/to/drone_dataset --val-ratio 0.15

Re-running is safe: it removes existing train/val sub-dirs first.
"""
import argparse
import os
import random
import shutil
from pathlib import Path


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("root", type=Path, help="dataset root produced by capture script")
    p.add_argument("--val-ratio", type=float, default=0.15)
    p.add_argument("--seed", type=int, default=42)
    args = p.parse_args()

    img_root = args.root / "images"
    lbl_root = args.root / "labels"
    if not img_root.is_dir() or not lbl_root.is_dir():
        raise SystemExit(f"expected {img_root} and {lbl_root} to exist")

    images = [p for p in img_root.iterdir()
              if p.is_file() and p.suffix.lower() in (".jpg", ".jpeg", ".png")
              and p.parent == img_root]
    if not images:
        raise SystemExit("no images found")

    random.seed(args.seed)
    random.shuffle(images)
    n_val = max(1, int(len(images) * args.val_ratio))
    val_set = set(images[:n_val])

    # Reset target folders
    for split in ("train", "val"):
        for sub in ("images", "labels"):
            target = args.root / sub / split
            if target.exists():
                shutil.rmtree(target)
            target.mkdir(parents=True)

    moved_train = moved_val = 0
    for img in images:
        split = "val" if img in val_set else "train"
        lbl = lbl_root / (img.stem + ".txt")
        if not lbl.exists():
            continue
        shutil.move(str(img), str(args.root / "images" / split / img.name))
        shutil.move(str(lbl), str(args.root / "labels" / split / lbl.name))
        if split == "val":
            moved_val += 1
        else:
            moved_train += 1

    print(f"train={moved_train}  val={moved_val}")


if __name__ == "__main__":
    main()
