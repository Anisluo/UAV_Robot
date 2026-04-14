#!/usr/bin/env python3
"""80/20 train/val split into YOLOv8's expected layout.

Emits dataset/data.yaml so `yolo train data=dataset/data.yaml` just works.
"""
import argparse
import pathlib
import random
import shutil


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--images",    required=True)
    ap.add_argument("--labels",    required=True)
    ap.add_argument("--out",       required=True)
    ap.add_argument("--val-split", type=float, default=0.2)
    ap.add_argument("--seed",      type=int,   default=0)
    args = ap.parse_args()

    src_img = pathlib.Path(args.images)
    src_lab = pathlib.Path(args.labels)
    out     = pathlib.Path(args.out)

    for split in ("train", "val"):
        (out / "images" / split).mkdir(parents=True, exist_ok=True)
        (out / "labels" / split).mkdir(parents=True, exist_ok=True)

    # Keep only images that have a non-empty label to avoid training on
    # empty samples that would push the box-regression head toward zero.
    usable = []
    for p in sorted(src_img.iterdir()):
        if p.suffix.lower() not in (".jpg", ".jpeg", ".png"): continue
        lab = src_lab / (p.stem + ".txt")
        if lab.exists() and lab.read_text().strip():
            usable.append((p, lab))
    print(f"{len(usable)} images with labels")

    random.Random(args.seed).shuffle(usable)
    n_val = max(1, int(round(len(usable) * args.val_split)))
    splits = {"val": usable[:n_val], "train": usable[n_val:]}
    for name, items in splits.items():
        for img_path, lab_path in items:
            shutil.copy(img_path, out / "images" / name / img_path.name)
            shutil.copy(lab_path, out / "labels" / name / lab_path.name)
        print(f"  {name}: {len(items)}")

    (out / "data.yaml").write_text(
        f"path: {out.resolve()}\n"
        f"train: images/train\n"
        f"val: images/val\n"
        f"names:\n"
        f"  0: battery\n"
    )
    print(f"wrote {out/'data.yaml'}")


if __name__ == "__main__":
    main()
