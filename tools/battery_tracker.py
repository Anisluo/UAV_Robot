#!/usr/bin/env python3
"""
battery_tracker.py — Python sidecar that detects DJI Mavic 3 batteries
from proc_realsense's shared-memory frames and publishes UavCResult
datagrams in the same format as proc_npu, so proc_gateway /
proc_grasp / uav_robotd all see them identically (no protocol
changes).

Controlled by a trigger file:

    /tmp/uav_battery_tracker_enabled    (present = active, absent = idle)

proc_npu sets the flag when strategy = UAV_STRATEGY_BATTERY_CV (id=4)
and clears it on any other strategy. While idle, this daemon sleeps and
publishes nothing — so it does not fight with the RKNN detector.

The detection itself lives in tools/battery_detect.py; this file is
just the live plumbing (shm reader + depth sampling + socket
publisher). Mirrors face_tracker.py so bug fixes made there (e.g. the
shm slot-scan for the highest frame_id) are carried over.

Dependencies: numpy + opencv-python (already installed via pip on the
RK3588).
"""

import mmap
import os
import socket
import struct
import sys
import time

import cv2
import numpy as np

# Allow `import battery_detect` when run from anywhere.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import battery_detect  # noqa: E402


# ── IPC paths / shm layout (must match common/include/abi/*.h) ────────────
SHM_RING_NAME     = "uav_rs_ring"
NPU_RX_SOCKETS    = [
    "/tmp/uav_gw_npu_rx.sock",     # proc_gateway
    "/tmp/uav_app_npu_rx.sock",    # uav_robotd
    "/tmp/uav_grasp_npu_rx.sock",  # proc_grasp
]
TRIGGER_FLAG_PATH = "/tmp/uav_battery_tracker_enabled"

# Ring header layout (must match ShmRing in shm_ring.h)
UAV_RING_MAGIC     = 0x55565247
UAV_RING_MAX_SLOTS = 16
UAV_SLOT_WRITING   = 1
UAV_SLOT_READY     = 2

# FrameSlot — uint32 state + 4 pad + uint64 frame_id + ...  Total = 72 bytes.
FRAME_SLOT_FMT  = "=I4xQQIII5fIIII"
FRAME_SLOT_SIZE = struct.calcsize(FRAME_SLOT_FMT)

# ShmRing header — 4×u32 (16 bytes) + u64 write_index.
SHM_HEADER_FMT     = "=IIIIQ"
SHM_HEADER_SIZE    = struct.calcsize(SHM_HEADER_FMT)
SHM_SLOTS_OFFSET   = SHM_HEADER_SIZE
SHM_PAYLOAD_OFFSET = SHM_SLOTS_OFFSET + UAV_RING_MAX_SLOTS * FRAME_SLOT_SIZE

# UavDetection (52 bytes).
DET_FMT  = "=i11f4B"
DET_SIZE = struct.calcsize(DET_FMT)
MAX_DETS = 32

# UavCResult = 1680 bytes total.
CRESULT_FMT  = "=QI" + "i11f4B" * MAX_DETS + "4x"
CRESULT_SIZE = struct.calcsize(CRESULT_FMT)

BATTERY_CLASS_ID = 200   # distinct from face=100 and YOLO classes 0..79


def _pack_result(frame_id, boxes, w, h, fx, fy, cx, cy, depth_lookup):
    """boxes: list of (x1, y1, x2, y2, score). Returns packed UavCResult."""
    values = [frame_id, min(len(boxes), MAX_DETS)]
    for i in range(MAX_DETS):
        if i < len(boxes):
            x1, y1, x2, y2, sc = boxes[i]
            x1, y1, x2, y2 = float(x1), float(y1), float(x2), float(y2)
            cx_px = (x1 + x2) * 0.5
            cy_px = (y1 + y2) * 0.5
            z_mm = float(depth_lookup(int(cx_px), int(cy_px), w, h))
            has_xyz = 1 if z_mm > 0.0 else 0
            if has_xyz:
                x_mm = (cx_px - cx) / fx * z_mm
                y_mm = (cy_px - cy) / fy * z_mm
            else:
                x_mm = y_mm = 0.0
            values += [
                BATTERY_CLASS_ID,
                float(sc),
                x1, y1, x2, y2,
                x_mm, y_mm, z_mm,
                0.0, 0.0, 0.0,
                has_xyz, 0, 0, 0,
            ]
        else:
            values += [0,
                       0.0, 0.0, 0.0, 0.0, 0.0,
                       0.0, 0.0, 0.0,
                       0.0, 0.0, 0.0,
                       0, 0, 0, 0]
    return struct.pack(CRESULT_FMT, *values)


class ShmFrameReader:
    """Memory-map /dev/shm/<SHM_RING_NAME> and fetch latest ready slot."""

    def __init__(self):
        shm_path = f"/dev/shm/{SHM_RING_NAME}"
        self._fd = os.open(shm_path, os.O_RDONLY)
        size = os.fstat(self._fd).st_size
        self._mm = mmap.mmap(self._fd, size, prot=mmap.PROT_READ)
        header = struct.unpack(SHM_HEADER_FMT, self._mm[:SHM_HEADER_SIZE])
        magic, _, self._slot_count, self._slot_payload, _ = header
        if magic != UAV_RING_MAGIC:
            raise RuntimeError(f"bad magic 0x{magic:x}")

    def read_latest(self):
        """Return (frame_id, bgr_ndarray, intrinsics_dict) or None.

        The writer picks the first FREE slot (not a strictly sequential
        slot), because proc_gateway consumes slots and marks them FREE.
        So we scan every slot and pick the one with the highest frame_id
        to get the newest published frame.
        """
        best_fid = 0
        best_slot = None
        best_idx = -1
        for i in range(self._slot_count):
            off = SHM_SLOTS_OFFSET + i * FRAME_SLOT_SIZE
            s = struct.unpack(FRAME_SLOT_FMT, self._mm[off:off + FRAME_SLOT_SIZE])
            state_, fid, _, w_, h_ = s[0], s[1], s[2], s[3], s[4]
            if state_ == UAV_SLOT_WRITING:
                continue
            if fid > best_fid and w_ > 0 and h_ > 0:
                best_fid = fid
                best_slot = s
                best_idx = i
        if best_slot is None:
            return None
        (state, frame_id, ts_ns, w, h, stride, fx, fy, cx_in, cy_in,
         depth_scale, color_off, depth_off, payload_size, _) = best_slot

        payload_base = SHM_PAYLOAD_OFFSET + best_idx * self._slot_payload
        # BGR8 directly from RealSense (rs_capture sets RS2_FORMAT_BGR8).
        color_bytes = bytes(self._mm[payload_base + color_off:
                                     payload_base + color_off + stride * h])
        bgr = np.frombuffer(color_bytes, dtype=np.uint8).reshape((h, stride // 3, 3))
        bgr = bgr[:, :w, :].copy()

        depth_ptr   = payload_base + depth_off
        depth_bytes = bytes(self._mm[depth_ptr:depth_ptr + w * h * 2])
        depth = np.frombuffer(depth_bytes, dtype=np.uint16).reshape((h, w))

        intr = {"fx": fx, "fy": fy, "cx": cx_in, "cy": cy_in,
                "depth_scale": depth_scale, "depth": depth}
        return frame_id, bgr, intr

    def close(self):
        self._mm.close()
        os.close(self._fd)


def _publish(payload):
    for path in NPU_RX_SOCKETS:
        try:
            with socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM) as s:
                n = s.sendto(payload, path)
                if not hasattr(_publish, "_logged"):
                    _publish._logged = True
                    print(f"[battery_tracker] first publish: {n} bytes → {path}",
                          flush=True)
        except OSError as e:
            if not hasattr(_publish, "_err_logged"):
                _publish._err_logged = True
                print(f"[battery_tracker] publish to {path} failed: {e}",
                      flush=True)


def _sample_depth_mm(x, y, w, h, depth, depth_scale):
    if depth is None:
        return 0.0
    x = max(0, min(w - 1, x))
    y = max(0, min(h - 1, y))
    # 9x9 median around the battery centroid — battery is bigger than a
    # face so we can afford a wider sample for less noise.
    x0 = max(0, x - 4); x1 = min(w, x + 5)
    y0 = max(0, y - 4); y1 = min(h, y + 5)
    patch = depth[y0:y1, x0:x1]
    valid = patch[(patch > 0) & (patch < 10000)]
    if valid.size == 0:
        return 0.0
    z = float(np.median(valid))
    return z * depth_scale * 1000.0


# ── Reasonable battery range for the D435 on the arm. Closer than 80 mm
# means the sensor itself is reading garbage (minimum depth), and farther
# than 1200 mm is outside useful grasp distance — reject detections that
# claim depths outside this range because they're almost always looking
# through the battery at a wall or at sensor noise.
DEPTH_MIN_MM = 80.0
DEPTH_MAX_MM = 1200.0


def _iou(a, b) -> float:
    """IoU of two (x1, y1, x2, y2, score) boxes — score is ignored."""
    ax1, ay1, ax2, ay2 = a[:4]
    bx1, by1, bx2, by2 = b[:4]
    ix1 = max(ax1, bx1); iy1 = max(ay1, by1)
    ix2 = min(ax2, bx2); iy2 = min(ay2, by2)
    iw = max(0.0, ix2 - ix1); ih = max(0.0, iy2 - iy1)
    inter = iw * ih
    if inter <= 0.0:
        return 0.0
    area_a = (ax2 - ax1) * (ay2 - ay1)
    area_b = (bx2 - bx1) * (by2 - by1)
    return inter / max(1.0, area_a + area_b - inter)


def _ema_box(prev, curr, alpha=0.4):
    """Exponential smoothing of (x1, y1, x2, y2, score) toward current.

    Only applied when the current detection overlaps the previous one —
    otherwise we snap directly to the new position, because it's almost
    certainly a different battery (or a large step by the camera).
    """
    if prev is None or _iou(prev, curr) < 0.30:
        return curr
    x1 = prev[0] * (1 - alpha) + curr[0] * alpha
    y1 = prev[1] * (1 - alpha) + curr[1] * alpha
    x2 = prev[2] * (1 - alpha) + curr[2] * alpha
    y2 = prev[3] * (1 - alpha) + curr[3] * alpha
    sc = max(prev[4], curr[4])        # keep the stronger confidence
    return (int(x1), int(y1), int(x2), int(y2), sc)


def main():
    print(f"[battery_tracker] using detector: {battery_detect.__file__}",
          flush=True)

    reader = None
    last_published_fid = 0
    HOLD_MS = 600          # hold longer than face_tracker — batteries rarely
    held_boxes = []        # drop out for a frame or two under normal lighting.
    held_until_ms = 0.0
    smoothed_box = None    # EMA'd bbox across frames for stable overlay

    while True:
        if not os.path.exists(TRIGGER_FLAG_PATH):
            if reader is not None:
                reader.close()
                reader = None
            time.sleep(0.3)
            continue

        if reader is None:
            try:
                reader = ShmFrameReader()
                print("[battery_tracker] attached to shared memory", flush=True)
            except (FileNotFoundError, RuntimeError) as e:
                print(f"[battery_tracker] ring not ready: {e}", flush=True)
                time.sleep(0.5)
                continue

        try:
            r = reader.read_latest()
        except Exception as e:
            print(f"[battery_tracker] read error: {e}", flush=True)
            reader.close()
            reader = None
            continue
        if r is None:
            time.sleep(0.05)
            continue
        frame_id, bgr, intr = r
        if frame_id == last_published_fid:
            time.sleep(0.03)
            continue
        last_published_fid = frame_id

        raw_boxes = battery_detect.detect(bgr)
        h, w = bgr.shape[:2]

        # Depth-gate each raw detection: reject anything whose depth
        # reading is outside the grasp-useful range (sensor floor is
        # ~80 mm, grasp workspace ends well under 1200 mm).  Keeps the
        # tracker from publishing a "battery" box that actually lies on
        # a distant wall or reads sensor-near-field garbage.
        filtered = []
        for (x1, y1, x2, y2, sc) in raw_boxes:
            cx = int((x1 + x2) * 0.5)
            cy = int((y1 + y2) * 0.5)
            z = _sample_depth_mm(cx, cy, w, h,
                                 intr["depth"], intr["depth_scale"])
            if z > 0.0 and (z < DEPTH_MIN_MM or z > DEPTH_MAX_MM):
                continue
            filtered.append((x1, y1, x2, y2, sc))

        # Temporal EMA: if a new detection overlaps the previous one,
        # smooth positions to kill per-frame jitter. No overlap ⇒ snap
        # (probably the battery moved or a different one appeared).
        if filtered:
            smoothed_box = _ema_box(smoothed_box, filtered[0])
            boxes = [smoothed_box] + filtered[1:]
        else:
            boxes = []

        now_ms = time.time() * 1000.0
        if boxes:
            held_boxes = list(boxes)
            held_until_ms = now_ms + HOLD_MS
        elif now_ms < held_until_ms:
            boxes = list(held_boxes)
        else:
            smoothed_box = None    # stale enough to forget

        def depth_lookup(px, py, ww, hh):
            return _sample_depth_mm(px, py, ww, hh,
                                    intr["depth"], intr["depth_scale"])

        payload = _pack_result(frame_id, boxes, w, h,
                               intr["fx"], intr["fy"],
                               intr["cx"], intr["cy"],
                               depth_lookup)
        _publish(payload)

        if not hasattr(main, "_last_log_t"):
            main._last_log_t = 0.0
        now = time.time()
        if now - main._last_log_t > 1.0:
            n = len(boxes)
            best = f" best_score={boxes[0][4]:.2f}" if boxes else ""
            print(f"[battery_tracker] fid={frame_id} n={n}{best} "
                  f"size={w}x{h}", flush=True)
            main._last_log_t = now

        time.sleep(0.05)   # battery detection doesn't need 30Hz, 20Hz is plenty


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
