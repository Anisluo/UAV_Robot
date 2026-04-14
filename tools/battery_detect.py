#!/usr/bin/env python3
"""Classical-CV DJI Mavic 3 battery detector.

Design notes
------------
The target is a matte black, elongated rectangular battery (≈ 2.5–3:1
aspect ratio) on a light-coloured surface (white / beige desk / lab
bench). Common distractors seen across the training set:

  * white foam packaging block  — very high brightness, easy to reject
  * brushed-aluminium bars      — metallic specular highlights with
                                  dark crevices; aspect-ratio > 4:1
  * shadows                     — soft edges, low internal texture
  * dark fabric / electronics   — can pass shape filters but have very
                                  different texture / saturation

The detector stays classical (OpenCV only, no NN) so it needs zero
training data and runs on CPU. It plugs into the same UavCResult
sidecar slot that face_tracker.py occupies in the pipeline (same
struct, same sockets, different class_id).

Pipeline (v2):
  1. HSV mask of very-dark pixels (adaptive V percentile).
  2. Morphological close + open — fills label cut-outs, kills speckle.
  3. Contour detection (external only). For each candidate compute
     geometry (area, aspect, rectangularity, fill, solidity) AND
     content features:
        * mean Saturation in the contour  — battery is neutral black
          (S≈0..30), colour-shadows and objects sit higher.
        * internal edge density (Canny)    — the battery's printed icons
          and seams create dense sub-pixel edges; smooth foam /
          uniform shadow blobs do not.
        * mean V inside the contour        — battery is genuinely dark
          even after morphology, shadows tend to be dimmer but not
          black.
  4. Two shape regimes: side-view (elongated) vs end-view (near-square).
     End-view requires strict rectangularity + solidity because
     cube-ish shadows otherwise slip through.
  5. Weighted score across all six factors; reject below SCORE_MIN.
  6. Two-pass fallback: if nothing qualifies at the primary threshold,
     retry with a stricter (darker) threshold before giving up.

The module exposes `detect(bgr) -> list[(x1,y1,x2,y2,score)]` so both
the offline sanity check (batch-run over battery_data/) and the live
sidecar share exactly the same logic.
"""
from __future__ import annotations

import cv2
import numpy as np
from typing import List, Tuple

BatteryBox = Tuple[int, int, int, int, float]   # x1, y1, x2, y2, score


# ── Geometry thresholds ──────────────────────────────────────────────────
MIN_AREA_FRAC    = 0.004    # at least 0.4 % of frame
MAX_AREA_FRAC    = 0.60     # at most 60 % of frame
ASPECT_MIN_SIDE  = 1.8      # elongated-view cut-off
ASPECT_MIN_END   = 1.0      # end-view cut-off (accept near-square)
ASPECT_MAX       = 4.5      # aluminium bars / pencils exceed this
RECT_MIN         = 0.60     # side-view rectangularity floor
RECT_MIN_END     = 0.78     # end-view rectangularity floor (stricter)
FILL_MIN         = 0.35     # axis-bbox fill ratio
SOLIDITY_MIN     = 0.80     # side-view solidity floor
SOLIDITY_MIN_END = 0.90     # end-view solidity floor (stricter)

# ── Content thresholds ───────────────────────────────────────────────────
# Saturation: battery is black, so mean S ≤ ~40 even on tinted shots.
# We reject hard above 80 (a coloured patch that merely looks dark).
SAT_MEAN_MAX     = 80
# Internal Canny-edge pixel density. Empirically, battery contours in
# battery_data sit at ~0.04–0.15 (printed icons, battery seams, terminal
# grille). Smooth shadows sit near 0. Use a very soft floor so low-res
# camera frames still pass, but penalise in score.
EDGE_DENS_FLOOR  = 0.005
EDGE_DENS_TARGET = 0.05
# V inside the candidate blob — battery stays genuinely dark even after
# morphology; dimly-lit wall/cloth regions creep higher.
V_MEAN_MAX       = 70

# ── Score gate ───────────────────────────────────────────────────────────
SCORE_MIN        = 0.42


# ── Adaptive dark threshold ──────────────────────────────────────────────
def _dark_threshold(v_channel: np.ndarray, percentile: float = 12.0,
                    hard_ceiling: int = 80) -> int:
    """Pick the V cut-off adaptively so exposure doesn't break the mask.

    Battery pixels consistently land in the darkest 8–15 % of the frame.
    Use the specified percentile (defaults to 12 th) plus a small buffer,
    capped by `hard_ceiling` so very dim desks don't drag the threshold
    into background noise.
    """
    p = int(np.percentile(v_channel, percentile))
    return max(20, min(hard_ceiling, p + 5))


def _build_mask(hsv: np.ndarray, v_thr: int, k: int) -> np.ndarray:
    v = hsv[..., 2]
    mask = (v < v_thr).astype(np.uint8) * 255
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (k, k))
    # Close fills the label cut-outs / seams; open kills background
    # speckle. One iteration of open is enough — heavier open eats
    # into the real battery silhouette at camera resolution.
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel, iterations=1)
    return mask


def _contour_features(contour: np.ndarray,
                      hsv: np.ndarray,
                      edge_map: np.ndarray) -> dict:
    """Compute all per-candidate features used by the scorer."""
    area = float(cv2.contourArea(contour))
    (_rx, _ry), (rw, rh), _rot = cv2.minAreaRect(contour)
    long_side  = max(rw, rh)
    short_side = max(1.0, min(rw, rh))
    aspect = long_side / short_side

    x, y, bw, bh = cv2.boundingRect(contour)
    hull = cv2.convexHull(contour)
    hull_area = float(cv2.contourArea(hull))

    # Mask of contour interior, clipped to the axis-aligned bbox. Used
    # both for colour statistics and edge density.
    roi_mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    cv2.drawContours(roi_mask, [contour], -1, 255, thickness=cv2.FILLED)

    sat  = hsv[..., 1][roi_mask > 0]
    vch  = hsv[..., 2][roi_mask > 0]
    edges_in = edge_map[roi_mask > 0]

    mean_sat = float(sat.mean()) if sat.size else 255.0
    mean_v   = float(vch.mean()) if vch.size else 255.0
    edge_density = float((edges_in > 0).sum()) / max(1.0, area)

    return dict(
        area=area, bbox=(x, y, bw, bh), hull_area=hull_area,
        long_side=long_side, short_side=short_side, aspect=aspect,
        mean_sat=mean_sat, mean_v=mean_v,
        edge_density=edge_density,
        rectangularity=area / max(1.0, long_side * short_side),
        fill=area / max(1.0, bw * bh),
        solidity=area / max(1.0, hull_area),
    )


def _score_candidate(f: dict, frame_area: float) -> tuple[float, bool]:
    """Return (score, is_end_view).  Score ≥ 0 only if candidate passes."""
    # Hard shape filters
    if f["area"] < MIN_AREA_FRAC * frame_area: return -1.0, False
    if f["area"] > MAX_AREA_FRAC * frame_area: return -1.0, False
    if f["aspect"] > ASPECT_MAX:               return -1.0, False
    if f["aspect"] < ASPECT_MIN_END:           return -1.0, False
    if f["fill"]   < FILL_MIN:                 return -1.0, False

    is_end_view = f["aspect"] < ASPECT_MIN_SIDE
    if is_end_view:
        if f["rectangularity"] < RECT_MIN_END:   return -1.0, False
        if f["solidity"]       < SOLIDITY_MIN_END: return -1.0, False
    else:
        if f["rectangularity"] < RECT_MIN:   return -1.0, False
        if f["solidity"]       < SOLIDITY_MIN: return -1.0, False

    # Hard content filters
    if f["mean_sat"] > SAT_MEAN_MAX: return -1.0, False
    if f["mean_v"]   > V_MEAN_MAX:   return -1.0, False

    # Score components — all normalised to [0, 1], then weighted.
    size_score   = min(1.0, f["area"] / (0.25 * frame_area))
    aspect_norm  = min(1.0, max(0.0,
                       (f["aspect"] - ASPECT_MIN_END)
                       / (3.0 - ASPECT_MIN_END)))
    sat_score    = max(0.0, 1.0 - f["mean_sat"] / SAT_MEAN_MAX)
    # Edge density shaped as a soft ramp so an image-pyramid level with
    # few captured details doesn't zero the score, but a truly smooth
    # shadow still loses out.
    edge_score = min(1.0, f["edge_density"] / EDGE_DENS_TARGET)
    edge_pass  = f["edge_density"] >= EDGE_DENS_FLOOR

    score = (0.28 * f["rectangularity"]
             + 0.18 * f["solidity"]
             + 0.18 * size_score
             + 0.12 * aspect_norm
             + 0.12 * sat_score
             + 0.12 * edge_score)
    if not edge_pass:
        score *= 0.75       # soft penalty; still allow if everything else is strong
    if is_end_view:
        score *= 0.9

    return score, is_end_view


def _find_candidates(bgr: np.ndarray, hsv: np.ndarray, edge_map: np.ndarray,
                     v_thr: int, k: int) -> list[tuple[float, BatteryBox, dict]]:
    mask = _build_mask(hsv, v_thr, k)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
    frame_area = float(bgr.shape[0] * bgr.shape[1])
    out = []
    for c in contours:
        # Cheap early-reject on bounding-box area before we do heavier work.
        xa, ya, wa, ha = cv2.boundingRect(c)
        if wa * ha < MIN_AREA_FRAC * frame_area: continue

        feats = _contour_features(c, hsv, edge_map)
        score, is_end = _score_candidate(feats, frame_area)
        if score < SCORE_MIN: continue
        x, y, bw, bh = feats["bbox"]
        out.append((score, (x, y, x + bw, y + bh, score), feats))
    return out


def detect(bgr: np.ndarray, debug: dict | None = None) -> List[BatteryBox]:
    """Return up to one bounding box per detected battery."""
    h, w = bgr.shape[:2]
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

    # Canny on a denoised grayscale so label icons and battery seams
    # light up, but desk grain does not. Low thresholds because the
    # printed graphics are low-contrast against matte black.
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (3, 3), 0)
    edge_map = cv2.Canny(blur, 40, 120)

    k = max(3, int(round(min(h, w) * 0.006)) | 1)

    # Primary pass — adaptive threshold from the 12 th percentile.
    v_thr = _dark_threshold(hsv[..., 2], percentile=12.0)
    candidates = _find_candidates(bgr, hsv, edge_map, v_thr, k)

    # Fallback — if nothing passes, try a stricter (darker) threshold.
    # Helps when the battery shares the scene with a mid-grey object:
    # the 12 th percentile is dragged up, merging blobs in the mask.
    if not candidates:
        v_thr2 = _dark_threshold(hsv[..., 2], percentile=7.0, hard_ceiling=60)
        if v_thr2 < v_thr:
            candidates = _find_candidates(bgr, hsv, edge_map, v_thr2, k)

    # Deep fallback — very lenient mask for cases where the battery sits
    # on a dark background and the top percentile is already noisy. We
    # still require SCORE_MIN so this only saves borderline misses.
    if not candidates:
        v_thr3 = _dark_threshold(hsv[..., 2], percentile=18.0, hard_ceiling=95)
        if v_thr3 > v_thr:
            candidates = _find_candidates(bgr, hsv, edge_map, v_thr3, k)

    candidates.sort(key=lambda t: -t[0])

    if debug is not None:
        debug["v_thr"] = v_thr
        debug["edge_map"] = edge_map
        debug["candidates"] = [
            {"bbox": box[:4], "score": sc, "feats": {
                kk: (round(vv, 4) if isinstance(vv, float) else vv)
                for kk, vv in f.items() if kk != "bbox"
            }}
            for sc, box, f in candidates
        ]

    return [box for _, box, _ in candidates[:1]]


if __name__ == "__main__":
    import os, sys, pathlib

    src_dir = pathlib.Path(sys.argv[1] if len(sys.argv) > 1 else "battery_data")
    dst_dir = pathlib.Path("battery_data_annotated")
    dst_dir.mkdir(exist_ok=True)

    files = sorted(p for p in src_dir.iterdir()
                   if p.suffix.lower() in (".jpg", ".jpeg", ".png"))
    hits = 0
    for f in files:
        img = cv2.imread(str(f))
        if img is None:
            print(f"[skip] {f.name}: cannot read")
            continue
        dbg = {}
        boxes = detect(img, debug=dbg)
        if boxes:
            hits += 1
            for (x1, y1, x2, y2, s) in boxes:
                cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 3)
                cv2.putText(img, f"battery {s:.2f}",
                            (x1, max(y1 - 8, 12)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            feats = dbg["candidates"][0]["feats"]
            print(f"[ok]  {f.name}  -> {boxes[0]}  "
                  f"ms={feats['mean_sat']:.1f} "
                  f"mv={feats['mean_v']:.1f} "
                  f"ed={feats['edge_density']:.3f}")
        else:
            cv2.putText(img, "NO DETECTION", (20, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 0, 255), 3)
            print(f"[MISS] {f.name} (cands={len(dbg.get('candidates', []))})")

        out = img
        if max(out.shape[:2]) > 1024:
            scale = 1024 / max(out.shape[:2])
            out = cv2.resize(out, None, fx=scale, fy=scale,
                             interpolation=cv2.INTER_AREA)
        cv2.imwrite(str(dst_dir / f.name), out,
                    [cv2.IMWRITE_JPEG_QUALITY, 80])

    print(f"\n{hits}/{len(files)} detected")
