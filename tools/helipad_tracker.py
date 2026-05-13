#!/usr/bin/env python3
"""
helipad_tracker.py — Python sidecar that detects the workspace's
helipad marker (a black ring with a black "H" inside) from
proc_realsense's shared-memory frames and publishes UavCResult
datagrams in the same format as proc_npu.

Purpose: when the platform is empty (default scene), the GUI needs a
*positive* signal that says "platform is in view and ready for a drone",
distinct from "camera is dark / disconnected". The helipad marker
provides that — if we can see it, the camera is good and the platform
is reachable.

Controlled by a trigger file:

    /tmp/uav_helipad_tracker_enabled    (present = active, absent = idle)

proc_npu sets the flag when strategy = UAV_STRATEGY_MAVIC3_DRONE and
clears it on any other strategy. While idle, this daemon sleeps and
publishes nothing.

Mirrors battery_tracker.py for the shm-reader + publisher boilerplate;
swap in a Hough-circle + "H"-template detector.

Dependencies: numpy + opencv-python.
"""

import mmap
import os
import socket
import struct
import sys
import time

import cv2
import numpy as np


# ── IPC paths / shm layout (must match common/include/abi/*.h) ────────────
SHM_RING_NAME     = "uav_rs_ring"
NPU_RX_SOCKETS    = [
    "/tmp/uav_gw_npu_rx.sock",     # proc_gateway
    "/tmp/uav_app_npu_rx.sock",    # uav_robotd
    "/tmp/uav_grasp_npu_rx.sock",  # proc_grasp
]
TRIGGER_FLAG_PATH = "/tmp/uav_helipad_tracker_enabled"

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

# Class IDs (must match proc_npu/src/postprocess.h)
HELIPAD_CLASS_ID = 901   # distinct from drone=4, battery=200, platform=900


# ── Helipad detector ─────────────────────────────────────────────────────
def _build_h_template(size: int) -> np.ndarray:
    """Build a square binary template of an 'H' character at `size` px."""
    side = max(8, size)
    t = np.zeros((side, side), dtype=np.uint8)
    bar = max(2, side // 5)            # bar thickness ≈ 20 % of side
    t[:, :bar]       = 255             # left vertical bar
    t[:, -bar:]      = 255             # right vertical bar
    mid = side // 2
    t[mid - bar // 2:mid + bar // 2 + 1, :] = 255   # horizontal crossbar
    return t


def _match_h(binary_inside: np.ndarray) -> float:
    """Return the best template-match score for an 'H' inside `binary_inside`.

    `binary_inside` is a square inverted-binary image (255 where the marker
    is dark, 0 elsewhere). Tries a small set of template scales and picks
    the best correlation. The 'H' is symmetric under 180° rotation, so we
    only test 0° and 90° (handles the case where the marker is laid on its
    side).
    """
    if binary_inside.size == 0:
        return 0.0
    side = binary_inside.shape[0]
    best = 0.0
    for ratio in (0.85, 0.70, 0.55):
        tsize = int(side * ratio)
        if tsize < 16:
            continue
        tmpl = _build_h_template(tsize)
        for tmpl_rot in (tmpl, np.rot90(tmpl)):
            if (tmpl_rot.shape[0] > binary_inside.shape[0]
                or tmpl_rot.shape[1] > binary_inside.shape[1]):
                continue
            res = cv2.matchTemplate(binary_inside, tmpl_rot,
                                    cv2.TM_CCOEFF_NORMED)
            best = max(best, float(res.max()))
    return best


def detect_helipads(bgr: np.ndarray, debug: dict = None):
    """Detect black-circle-with-H markers. Returns list of
    (x1, y1, x2, y2, score) tuples in image coordinates.

    Strategy:
      1. Hough-find candidate circles in the blurred grayscale image.
      2. For each candidate, threshold the inside, sanity-check the
         dark-pixel ratio, then template-match an 'H'.
      3. Reject candidates whose H-match score is below kHMatchMin.
      4. NMS dedupes overlapping Hough candidates pointing at the same
         physical marker.
    """
    if bgr is None or bgr.size == 0:
        return []

    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    h_img, w_img = gray.shape
    # Median blur kills RealSense color-noise without smoothing the
    # ring's high-contrast edge.
    blurred = cv2.medianBlur(gray, 5)

    # Hough parameters tuned for a marker that occupies 60..300 px diameter
    # at typical work distances. param2 is the accumulator threshold —
    # too high and we miss the ring, too low and we get phantom circles
    # on every rounded background.
    circles = cv2.HoughCircles(
        blurred, cv2.HOUGH_GRADIENT,
        dp=1.4, minDist=80,
        param1=120, param2=38,
        minRadius=30, maxRadius=200,
    )
    if circles is None:
        if debug is not None:
            debug["circles"] = 0
        return []

    kDarkRatioMin = 0.10
    kDarkRatioMax = 0.55
    kHMatchMin    = 0.35

    raw = []
    for c in circles[0]:
        cx, cy, r = int(round(c[0])), int(round(c[1])), int(round(c[2]))
        # Crop the square that inscribes the circle (×0.78 of the bounding
        # square so we ignore the ring itself and look only at what's
        # inside it).
        inner = int(r * 0.78)
        x1 = cx - inner; y1 = cy - inner
        x2 = cx + inner; y2 = cy + inner
        if x1 < 0 or y1 < 0 or x2 >= w_img or y2 >= h_img:
            continue
        roi = gray[y1:y2, x1:x2]
        if roi.size == 0:
            continue
        # Square it off so the template scales line up.
        side = min(roi.shape[0], roi.shape[1])
        if side < 24:
            continue
        roi = roi[:side, :side]
        # Adaptive threshold so changes in ambient lighting don't kill
        # the detector. THRESH_BINARY_INV → dark pixels (the H) become
        # foreground.
        _, binary = cv2.threshold(roi, 0, 255,
                                  cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
        dark_ratio = float(np.count_nonzero(binary)) / (binary.size + 1)
        if dark_ratio < kDarkRatioMin or dark_ratio > kDarkRatioMax:
            continue

        h_score = _match_h(binary)
        if h_score < kHMatchMin:
            continue

        # Final bbox = the bounding square of the circle (includes the
        # ring so the GUI overlay shows the full marker).
        raw.append((cx - r, cy - r, cx + r, cy + r, h_score))

    # IoU-NMS to dedupe overlapping Hough candidates of the same marker.
    raw.sort(key=lambda b: b[4], reverse=True)
    kept = []
    for b in raw:
        if all(_iou(b, k) < 0.4 for k in kept):
            kept.append(b)
    if debug is not None:
        debug["circles"] = len(circles[0])
        debug["kept"]    = len(kept)
    return kept


def _iou(a, b):
    ax1, ay1, ax2, ay2 = a[:4]
    bx1, by1, bx2, by2 = b[:4]
    ix1 = max(ax1, bx1); iy1 = max(ay1, by1)
    ix2 = min(ax2, bx2); iy2 = min(ay2, by2)
    iw = max(0.0, ix2 - ix1); ih = max(0.0, iy2 - iy1)
    inter = iw * ih
    if inter <= 0.0:
        return 0.0
    area_a = max(1.0, (ax2 - ax1) * (ay2 - ay1))
    area_b = max(1.0, (bx2 - bx1) * (by2 - by1))
    return inter / (area_a + area_b - inter)


# ── shm reader + publisher (copied from battery_tracker.py) ──────────────
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


def _sample_depth_mm(x, y, w, h, depth, depth_scale):
    if depth is None:
        return 0.0
    x = max(0, min(w - 1, x))
    y = max(0, min(h - 1, y))
    x0 = max(0, x - 4); x1 = min(w, x + 5)
    y0 = max(0, y - 4); y1 = min(h, y + 5)
    patch = depth[y0:y1, x0:x1]
    valid = patch[(patch > 0) & (patch < 10000)]
    if valid.size == 0:
        return 0.0
    z = float(np.median(valid))
    return z * depth_scale * 1000.0


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
                HELIPAD_CLASS_ID,
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


def _publish(payload):
    for path in NPU_RX_SOCKETS:
        try:
            with socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM) as s:
                n = s.sendto(payload, path)
                if not hasattr(_publish, "_logged"):
                    _publish._logged = True
                    print(f"[helipad_tracker] first publish: {n} bytes → {path}",
                          flush=True)
        except OSError as e:
            if not hasattr(_publish, "_err_logged"):
                _publish._err_logged = True
                print(f"[helipad_tracker] publish to {path} failed: {e}",
                      flush=True)


def main():
    reader = None
    last_published_fid = 0
    HOLD_MS = 1500           # markers don't move — long hold reduces flicker
    held_boxes = []
    held_until_ms = 0.0
    last_log_t = 0.0
    verbose = bool(os.environ.get("HELIPAD_TRACE"))

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
                print("[helipad_tracker] attached to shared memory", flush=True)
            except (FileNotFoundError, RuntimeError) as e:
                print(f"[helipad_tracker] ring not ready: {e}", flush=True)
                time.sleep(0.5)
                continue

        try:
            r = reader.read_latest()
        except Exception as e:
            print(f"[helipad_tracker] read error: {e}", flush=True)
            reader.close()
            reader = None
            continue
        if r is None:
            time.sleep(0.05)
            continue
        frame_id, bgr, intr = r
        if frame_id == last_published_fid:
            time.sleep(0.05)
            continue
        last_published_fid = frame_id

        h, w = bgr.shape[:2]
        dbg = {}
        raw_boxes = detect_helipads(bgr, debug=dbg)

        now_ms = time.time() * 1000.0
        if raw_boxes:
            held_boxes = list(raw_boxes)
            held_until_ms = now_ms + HOLD_MS
            boxes = raw_boxes
        elif now_ms < held_until_ms:
            boxes = list(held_boxes)
        else:
            boxes = []

        def depth_lookup(px, py, ww, hh):
            return _sample_depth_mm(px, py, ww, hh,
                                    intr["depth"], intr["depth_scale"])

        payload = _pack_result(frame_id, boxes, w, h,
                               intr["fx"], intr["fy"],
                               intr["cx"], intr["cy"],
                               depth_lookup)
        _publish(payload)

        now = time.time()
        log_interval = 1.0 if verbose else 5.0
        if now - last_log_t > log_interval:
            n_kept = len(boxes)
            n_circ = dbg.get("circles", 0)
            best = f" best={boxes[0][4]:.2f}" if boxes else ""
            print(f"[helipad_tracker] fid={frame_id} hough={n_circ} kept={n_kept}{best}",
                  flush=True)
            last_log_t = now

        time.sleep(0.08)   # Hough is the costly part — 12 Hz is plenty for a static marker


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
