#!/usr/bin/env python3
"""Capture training images for the Mavic 3 drone YOLOv8 model.

Reads color frames from the proc_realsense shared-memory ring, saves
every Nth frame to a directory, and auto-bootstraps a YOLO-format
label by combining:
  • the gateway's current class=4 detection (post anchor-union, so
    the bbox already covers the whole airframe)
  • the white-plate ROI from battery_detect.py (so the bbox can't
    extend past the workspace into off-screen junk)
The operator should still open the result in labelImg / CVAT and
fix mistakes before training, but the bootstrap saves >80 % of the
click work.

Usage:
    python3 capture_drone_dataset.py                        # default 200 frames
    python3 capture_drone_dataset.py --count 500
    python3 capture_drone_dataset.py --interval 6           # every 6th source frame
    python3 capture_drone_dataset.py --out /tmp/drone_set
    python3 capture_drone_dataset.py --no-auto-label

Layout produced:
    <out>/images/<prefix>_NNNN.jpg
    <out>/labels/<prefix>_NNNN.txt           # YOLO: <cls> <cx> <cy> <w> <h>
    <out>/classes.txt                         # "drone"
"""
import argparse
import os
import socket
import sys
import time
import re

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import cv2  # noqa: E402

# Reuse the shm reader from battery_tracker so we don't redefine the
# slot layout in two places.
import battery_tracker  # noqa: E402
import battery_detect   # noqa: E402  # for _find_workspace_bbox


def _intersect_with_plate(bbox, plate, w_img, h_img):
    """Clip auto-label bbox to plate ROI ± 30 px. Returns None if the
    overlap is too thin to be useful (plate detection probably wrong)."""
    if plate is None or bbox is None:
        return bbox
    px, py, pw, ph = plate
    pad = 30
    px1 = max(0, px - pad);  py1 = max(0, py - pad)
    px2 = min(w_img, px + pw + pad);  py2 = min(h_img, py + ph + pad)
    bx1, by1, bx2, by2, sc = bbox
    cx1 = max(bx1, px1);  cy1 = max(by1, py1)
    cx2 = min(bx2, px2);  cy2 = min(by2, py2)
    if cx2 - cx1 < 30 or cy2 - cy1 < 30:
        return None  # bbox barely overlaps plate
    return (cx1, cy1, cx2, cy2, sc)


def query_npu_drone(host="127.0.0.1", port=7001, score_min=0.40):
    """Pull current detections from the gateway and return (x1,y1,x2,y2,score)
    for the highest-score class=4 (airplane) detection, or None.
    """
    try:
        s = socket.socket()
        s.settimeout(0.5)
        s.connect((host, port))
        s.sendall(b'{"id":1,"method":"npu.get_detections","params":{}}\n')
        buf = b""
        t0 = time.time()
        while time.time() - t0 < 0.5:
            try:
                chunk = s.recv(8192)
            except socket.timeout:
                break
            if not chunk:
                break
            buf += chunk
            if buf.endswith(b"\n"):
                break
        s.close()
    except OSError:
        return None
    raw = buf.split(b"\n")[0].decode("utf-8", errors="replace")
    pat = re.compile(
        r'"class_id":(\d+),"score":([\d.]+),'
        r'"x1":([\d.]+),"y1":([\d.]+),"x2":([\d.]+),"y2":([\d.]+)'
    )
    best = None
    for m in pat.finditer(raw):
        cls = int(m.group(1))
        if cls != 4:
            continue
        sc = float(m.group(2))
        if sc < score_min:
            continue
        x1, y1, x2, y2 = (float(m.group(i)) for i in (3, 4, 5, 6))
        if best is None or sc > best[4]:
            best = (x1, y1, x2, y2, sc)
    return best


def write_yolo_label(path, bbox, frame_w, frame_h, cls_id=0):
    if bbox is None:
        # Empty file = "this image has no detections" for YOLO training.
        # Useful as negatives.
        open(path, "w").close()
        return
    x1, y1, x2, y2, _ = bbox
    cx = (x1 + x2) * 0.5 / frame_w
    cy = (y1 + y2) * 0.5 / frame_h
    bw = (x2 - x1) / frame_w
    bh = (y2 - y1) / frame_h
    with open(path, "w") as f:
        f.write(f"{cls_id} {cx:.6f} {cy:.6f} {bw:.6f} {bh:.6f}\n")


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--count", type=int, default=200,
                   help="number of frames to save (default 200)")
    p.add_argument("--interval", type=int, default=4,
                   help="capture every Nth source frame (default 4 → ~7.5 fps "
                        "from a 30 fps source)")
    p.add_argument("--out", default="/home/ubuntu/drone_dataset",
                   help="output dataset root (default /home/ubuntu/drone_dataset)")
    p.add_argument("--prefix", default="drone",
                   help="filename prefix (default 'drone')")
    p.add_argument("--no-auto-label", action="store_true",
                   help="don't query gateway for auto-labels (write empty .txt)")
    p.add_argument("--score-min", type=float, default=0.40,
                   help="auto-label threshold (default 0.40 — looser than the "
                        "tracker's 0.45 because we want recall here)")
    args = p.parse_args()

    img_dir = os.path.join(args.out, "images")
    lbl_dir = os.path.join(args.out, "labels")
    os.makedirs(img_dir, exist_ok=True)
    os.makedirs(lbl_dir, exist_ok=True)
    with open(os.path.join(args.out, "classes.txt"), "w") as f:
        f.write("drone\n")

    reader = battery_tracker.ShmFrameReader()
    print(f"[capture] writing to {args.out}")
    print(f"[capture] target {args.count} frames, every {args.interval} source "
          f"frames; auto-label={'off' if args.no_auto_label else 'on'}")
    print("[capture] move / rotate the drone slowly through poses while running.")

    saved = 0
    auto_labelled = 0
    seen_fid = 0
    last_fid = -1
    while saved < args.count:
        frame = reader.read_latest()
        if frame is None:
            time.sleep(0.02)
            continue
        fid, bgr, _intr = frame
        if fid == last_fid:
            time.sleep(0.02)
            continue
        last_fid = fid
        seen_fid += 1
        if seen_fid % args.interval != 0:
            continue

        # Light auto-label: pull current NPU detection (already
        # anchor-union-merged in postprocess.cpp), then clip it to the
        # white-plate ROI so off-screen junk doesn't bleed into the
        # box. NPU runs at ~5 fps so consecutive saved frames share a
        # detection — bbox jitter is small but real, hand-review is
        # still recommended.
        bbox = None
        if not args.no_auto_label:
            bbox = query_npu_drone(score_min=args.score_min)
            if bbox is not None:
                hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
                plate = battery_detect._find_workspace_bbox(hsv)
                bbox = _intersect_with_plate(bbox, plate,
                                             bgr.shape[1], bgr.shape[0])
            if bbox is not None:
                auto_labelled += 1

        stem = f"{args.prefix}_{saved:04d}"
        cv2.imwrite(os.path.join(img_dir, f"{stem}.jpg"), bgr,
                    [cv2.IMWRITE_JPEG_QUALITY, 92])
        write_yolo_label(os.path.join(lbl_dir, f"{stem}.txt"),
                         bbox, bgr.shape[1], bgr.shape[0])
        saved += 1
        if saved % 20 == 0:
            print(f"[capture] {saved}/{args.count} "
                  f"(auto-labelled so far: {auto_labelled})")

    print(f"[capture] done. saved={saved}, auto-labelled={auto_labelled}")
    print(f"[capture] next: rsync this dir to your training PC and review "
          f"the labels with labelImg / CVAT.")


if __name__ == "__main__":
    main()
