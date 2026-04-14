#!/usr/bin/env python3
"""
face_tracker.py - Python sidecar that detects faces from proc_realsense's
shared memory frames and publishes UavCResult datagrams in the same format
as proc_npu, so proc_gateway / proc_grasp / uav_robotd all see them
identically (no protocol changes).

Controlled by a trigger file:
    /tmp/uav_face_tracker_enabled    (present = active, absent = idle)

proc_npu sets the flag when strategy = UAV_STRATEGY_FACE (id=3) and clears
it on any other strategy. While idle, this daemon sleeps and publishes
nothing — so it does not fight with the RKNN detector.

Dependencies: numpy + opencv-python (already installed via pip on the RK3588).
"""

import ctypes
import mmap
import os
import socket
import struct
import sys
import time

import cv2
import numpy as np


# ── IPC paths / shm layout (must match common/include/abi/*.h) ────────────
SHM_RING_NAME        = "uav_rs_ring"     # /dev/shm/uav_rs_ring
FRAME_NOTIFY_SOCK    = "/tmp/uav_rs_frame_notify.sock"
NPU_RX_SOCKETS       = [
    "/tmp/uav_gw_npu_rx.sock",           # proc_gateway
    "/tmp/uav_app_npu_rx.sock",          # uav_robotd
    "/tmp/uav_grasp_npu_rx.sock",        # proc_grasp
]
TRIGGER_FLAG_PATH    = "/tmp/uav_face_tracker_enabled"

# Ring header layout (must match ShmRing in shm_ring.h)
UAV_RING_MAGIC       = 0x55565247
UAV_RING_MAX_SLOTS   = 16
UAV_SLOT_READY       = 2

# FrameSlot uses native C alignment on arm64: uint32 state + 4 pad before
# the uint64 frame_id.  Total = 72 bytes.
FRAME_SLOT_FMT       = "=I4xQQIII5fIIII"
FRAME_SLOT_SIZE      = struct.calcsize(FRAME_SLOT_FMT)   # = 72

# ShmRing header: 4×u32 (16 bytes, already 8-aligned) then u64 write_index.
SHM_HEADER_FMT       = "=IIIIQ"
SHM_HEADER_SIZE      = struct.calcsize(SHM_HEADER_FMT)   # = 24
SHM_SLOTS_OFFSET     = SHM_HEADER_SIZE
SHM_PAYLOAD_OFFSET   = SHM_SLOTS_OFFSET + UAV_RING_MAX_SLOTS * FRAME_SLOT_SIZE

# UavDetection (52 bytes): int32 class_id + 11 floats (score, bbox, xyz,
# rpy) + 3 u8 flags + 1 u8 reserved. Using "=" for fixed little-endian.
DET_FMT              = "=i11f4B"
DET_SIZE             = struct.calcsize(DET_FMT)  # = 52
MAX_DETS             = 32

# UavCResult: u64 frame_id, u32 num_detections, detections[32], trailing
# pad. Outer struct aligns to 8 bytes (uint64_t); 8+4+32*52 = 1676 → 1680.
CRESULT_FMT          = "=QI" + "i11f4B" * MAX_DETS + "4x"
CRESULT_SIZE         = struct.calcsize(CRESULT_FMT)  # = 1680


def _pack_result(frame_id, boxes, w, h, fx, fy, cx, cy, depth_lookup):
    """boxes: list of (x, y, bw, bh) in pixel coords. Returns a bytes payload."""
    values = [frame_id, min(len(boxes), MAX_DETS)]
    for i in range(MAX_DETS):
        if i < len(boxes):
            bx, by, bw, bh = boxes[i]
            x1, y1 = float(bx), float(by)
            x2, y2 = float(bx + bw), float(by + bh)
            # Reverse-project center into camera frame (mm)
            cx_px = (x1 + x2) * 0.5
            cy_px = (y1 + y2) * 0.5
            z_mm = float(depth_lookup(int(cx_px), int(cy_px), w, h))
            has_xyz = 1 if z_mm > 0.0 else 0
            if has_xyz:
                x_mm = (cx_px - cx) / fx * z_mm
                y_mm = (cy_px - cy) / fy * z_mm
            else:
                x_mm = y_mm = 0.0
            # Matches DET_FMT = i + 11 floats + 4 u8:
            # class_id, score, x1, y1, x2, y2, x_mm, y_mm, z_mm,
            # roll, pitch, yaw, has_xyz, has_rpy, grasp_mode, reserved
            values += [
                100,           # class_id: 100 = face (distinct from YOLO classes)
                0.95,          # score
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


def _find_haarcascade(name="haarcascade_frontalface_default.xml"):
    """Locate a named Haar cascade shipped with opencv-python."""
    try:
        path = os.path.join(cv2.data.haarcascades, name)
        if os.path.exists(path):
            return path
    except AttributeError:
        pass
    for root in ("/usr/share/opencv4/haarcascades",
                 "/usr/share/opencv/haarcascades"):
        c = os.path.join(root, name)
        if os.path.exists(c):
            return c
    raise RuntimeError(f"{name} not found")


def _merge_boxes(boxes, iou_thresh=0.3):
    """Collapse overlapping boxes (e.g. frontal + profile cascades agree)."""
    out = []
    for (x, y, w, h) in boxes:
        merged = False
        for i, (ox, oy, ow, oh) in enumerate(out):
            ix1 = max(x, ox); iy1 = max(y, oy)
            ix2 = min(x + w, ox + ow); iy2 = min(y + h, oy + oh)
            if ix2 > ix1 and iy2 > iy1:
                inter = (ix2 - ix1) * (iy2 - iy1)
                union = w * h + ow * oh - inter
                if inter / max(union, 1) >= iou_thresh:
                    nx = min(x, ox); ny = min(y, oy)
                    nx2 = max(x + w, ox + ow); ny2 = max(y + h, oy + oh)
                    out[i] = (nx, ny, nx2 - nx, ny2 - ny)
                    merged = True
                    break
        if not merged:
            out.append((x, y, w, h))
    return out


class ShmFrameReader:
    """Memory-map /dev/shm/<SHM_RING_NAME> and fetch latest ready slot."""

    def __init__(self):
        shm_path = f"/dev/shm/{SHM_RING_NAME}"
        self._fd = os.open(shm_path, os.O_RDONLY)
        size = os.fstat(self._fd).st_size
        self._mm = mmap.mmap(self._fd, size, prot=mmap.PROT_READ)
        header = struct.unpack(SHM_HEADER_FMT, self._mm[:SHM_HEADER_SIZE])
        magic, version, self._slot_count, self._slot_payload, self._write_index = header
        if magic != UAV_RING_MAGIC:
            raise RuntimeError(f"bad magic 0x{magic:x}")
        self._notify_fd = None

    def bind_notify(self):
        """Bind our own datagram socket path and receive frame-ready pings."""
        import tempfile
        self._notify_path = tempfile.mktemp(prefix="uav_face_notify_", suffix=".sock")
        s = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        s.bind(self._notify_path)
        s.settimeout(0.5)
        self._notify_fd = s
        # Note: proc_realsense notifies via a fixed socket path; we cannot
        # "subscribe" without protocol changes. Instead we poll the ring
        # header (write_index) — good enough for 10–30 Hz processing.

    def read_latest(self):
        """Return (frame_id, bgr_ndarray, intrinsics_dict) or None.

        The writer picks the first FREE slot (not a strictly sequential
        slot), because proc_gateway consumes slots and marks them FREE
        after reading. That means `(write_index - 1) % N` lands on a
        random stale slot — so we scan every slot and pick the one with
        the highest frame_id (ignoring state, because face_tracker is a
        read-only observer and mustn't race with the gateway's consumer).
        """
        UAV_SLOT_WRITING = 1
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
        latest_slot = best_idx
        (state, frame_id, ts_ns, w, h, stride, fx, fy, cx_in, cy_in,
         depth_scale, color_off, depth_off, payload_size, _) = best_slot

        payload_base = SHM_PAYLOAD_OFFSET + latest_slot * self._slot_payload
        # Color: BGR8 interleaved (rs_capture.cpp enables RS2_FORMAT_BGR8).
        color_bytes = bytes(self._mm[payload_base + color_off:
                                     payload_base + color_off + stride * h])
        bgr = np.frombuffer(color_bytes, dtype=np.uint8).reshape((h, stride // 3, 3))
        bgr = bgr[:, :w, :].copy()

        # Depth: uint16 raw, aligned to color size (w, h)
        depth_ptr = payload_base + depth_off
        depth_bytes = bytes(self._mm[depth_ptr:depth_ptr + w * h * 2])
        depth = np.frombuffer(depth_bytes, dtype=np.uint16).reshape((h, w))

        intr = {"fx": fx, "fy": fy, "cx": cx_in, "cy": cy_in,
                "depth_scale": depth_scale, "depth": depth}
        return frame_id, bgr, intr

    def close(self):
        self._mm.close()
        os.close(self._fd)
        if self._notify_fd is not None:
            try:
                self._notify_fd.close()
                os.unlink(self._notify_path)
            except OSError:
                pass


def _publish(payload):
    for path in NPU_RX_SOCKETS:
        try:
            with socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM) as s:
                n = s.sendto(payload, path)
                if not hasattr(_publish, "_logged"):
                    _publish._logged = True
                    print(f"[face_tracker] first publish: {n} bytes → {path} "
                          f"(payload size {len(payload)})", flush=True)
        except OSError as e:
            if not hasattr(_publish, "_err_logged"):
                _publish._err_logged = True
                print(f"[face_tracker] publish to {path} failed: {e}",
                      flush=True)


def _sample_depth_mm(x, y, w, h, depth, depth_scale):
    if depth is None:
        return 0.0
    x = max(0, min(w - 1, x))
    y = max(0, min(h - 1, y))
    # 7x7 median window for robustness
    x0 = max(0, x - 3); x1 = min(w, x + 4)
    y0 = max(0, y - 3); y1 = min(h, y + 4)
    patch = depth[y0:y1, x0:x1]
    valid = patch[(patch > 0) & (patch < 10000)]
    if valid.size == 0:
        return 0.0
    z = float(np.median(valid))
    return z * depth_scale * 1000.0  # depth_scale is meters/unit → mm


def main():
    frontal_path = _find_haarcascade("haarcascade_frontalface_default.xml")
    print(f"[face_tracker] frontal cascade: {frontal_path}", flush=True)
    frontal_cascade = cv2.CascadeClassifier(frontal_path)
    if frontal_cascade.empty():
        print("[face_tracker] failed to load frontal cascade", file=sys.stderr)
        sys.exit(1)

    profile_cascade = None
    try:
        profile_path = _find_haarcascade("haarcascade_profileface.xml")
        profile_cascade = cv2.CascadeClassifier(profile_path)
        if profile_cascade.empty():
            profile_cascade = None
        else:
            print(f"[face_tracker] profile cascade: {profile_path}", flush=True)
    except RuntimeError:
        pass

    reader = None
    last_published_fid = 0
    # Short hold window — Haar is stochastic and skips occasional frames.
    # If we had faces in the last HOLD_MS, keep re-emitting them so the
    # HostGUI overlay doesn't flicker between consecutive good frames.
    HOLD_MS = 400
    held_boxes: list = []
    held_until_ms = 0.0
    while True:
        # Gate: only run when trigger flag file exists.
        if not os.path.exists(TRIGGER_FLAG_PATH):
            if reader is not None:
                reader.close()
                reader = None
            time.sleep(0.3)
            continue

        # Lazy-open the ring (wait for proc_realsense to create it).
        if reader is None:
            try:
                reader = ShmFrameReader()
                print("[face_tracker] attached to shared memory", flush=True)
            except (FileNotFoundError, RuntimeError) as e:
                print(f"[face_tracker] ring not ready: {e}", flush=True)
                time.sleep(0.5)
                continue

        try:
            r = reader.read_latest()
        except Exception as e:
            print(f"[face_tracker] read error: {e}", flush=True)
            reader.close()
            reader = None
            continue
        if r is None:
            time.sleep(0.05)
            continue
        frame_id, bgr, intr = r
        if frame_id == last_published_fid:
            time.sleep(0.02)
            continue
        last_published_fid = frame_id

        h, w = bgr.shape[:2]
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        gray = cv2.equalizeHist(gray)
        # Tune for typical seated-at-desk framing (face ~12–80 % of frame H).
        # minNeighbors=5 prunes the speckled false positives that office
        # furniture clutter produces at finer scale factors.
        min_side = max(40, int(h * 0.12))
        max_side = int(h * 0.80)
        raw = list(frontal_cascade.detectMultiScale(
            gray, scaleFactor=1.10, minNeighbors=5,
            minSize=(min_side, min_side), maxSize=(max_side, max_side)))
        if profile_cascade is not None:
            # Right-facing profiles are detected directly; left-facing are
            # found by running the same cascade on a horizontally-flipped
            # gray image and mapping coords back.
            raw.extend(profile_cascade.detectMultiScale(
                gray, scaleFactor=1.10, minNeighbors=5,
                minSize=(min_side, min_side), maxSize=(max_side, max_side)))
            flipped = cv2.flip(gray, 1)
            for (fx_, fy_, fw_, fh_) in profile_cascade.detectMultiScale(
                    flipped, scaleFactor=1.10, minNeighbors=5,
                    minSize=(min_side, min_side), maxSize=(max_side, max_side)):
                raw.append((w - fx_ - fw_, fy_, fw_, fh_))
        boxes = _merge_boxes(
            [(int(x), int(y), int(bw), int(bh)) for (x, y, bw, bh) in raw])

        now_ms = time.time() * 1000.0
        if boxes:
            held_boxes = list(boxes)
            held_until_ms = now_ms + HOLD_MS
        elif now_ms < held_until_ms:
            # Keep last detection visible across a couple of flicker frames.
            boxes = list(held_boxes)

        def depth_lookup(px, py, ww, hh):
            return _sample_depth_mm(px, py, ww, hh, intr["depth"], intr["depth_scale"])

        payload = _pack_result(frame_id, boxes, w, h,
                               intr["fx"], intr["fy"],
                               intr["cx"], intr["cy"],
                               depth_lookup)
        _publish(payload)

        # Periodic status log (~ once/sec)
        if not hasattr(main, "_last_log_t"):
            main._last_log_t = 0.0
        now = time.time()
        if now - main._last_log_t > 1.0:
            print(f"[face_tracker] fid={frame_id} faces={len(boxes)} "
                  f"size={w}x{h}", flush=True)
            main._last_log_t = now

        # Light rate limit
        time.sleep(0.03)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
