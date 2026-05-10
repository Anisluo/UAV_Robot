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
# 2 % is the absolute minimum: anything smaller is too low-resolution
# for shape features to be meaningful. Between 2 % and SMALL_AREA_LIMIT
# (~6 %) we additionally require text/contact bright pixels inside the
# contour — that gate replaces the older blanket 4 % floor and lets
# small-but-real batteries through while still rejecting the standalone
# tripod base (3 % area, 0 bright pixels because it's just black plastic).
MIN_AREA_FRAC    = 0.020
MAX_AREA_FRAC    = 0.60     # at most 60 % of frame
ASPECT_MIN_SIDE  = 1.3      # elongated-view cut-off — the bare battery on
                            # its tripod lands at ~1.79, but when the
                            # morphology mask merges battery body + stand
                            # into one blob (top-down view) the compound
                            # shape comes through at aspect ~1.38. Both
                            # labelled end-view samples sit below 1.30, so
                            # 1.3 keeps the end-view path for those without
                            # forcing the merged-assembly into its stricter
                            # rect/solidity gates.
ASPECT_MIN_END   = 1.0      # end-view cut-off (accept near-square)
ASPECT_MAX       = 3.5      # narrow tools / wall edges exceed this; battery
                            # in any real orientation stays under ~3.
RECT_MIN         = 0.50     # side-view rectangularity floor — Mavic 3
                            # battery laid on its side / tilted toward the
                            # camera renders as a trapezoidal silhouette
                            # with rect ≈ 0.53 (clean front-view labelled
                            # samples are all ≥ 0.69, so dropping to 0.50
                            # picks up the tilted view without conflict).
RECT_MIN_END     = 0.78     # end-view rectangularity floor (stricter)
FILL_MIN         = 0.35     # axis-bbox fill ratio
SOLIDITY_MIN     = 0.72     # side-view solidity floor — same reason as
                            # above; a leaning battery produces concave
                            # extrusions (visible label / contact lip /
                            # shadow lobe) that drag solidity to ~0.76.
                            # Labelled training samples sit ≥ 0.84, so
                            # 0.72 is the largest cushion that never lets
                            # a labelled positive be rejected.
SOLIDITY_MIN_END = 0.90     # end-view solidity floor (stricter)
# Reject contours pinned against the frame border — almost always
# off-screen equipment leaking in at the edge, never the battery.
EDGE_TOUCH_PX    = 2

# ── Content thresholds ───────────────────────────────────────────────────
# Saturation: battery is black, so mean S ≤ ~40 even on tinted shots.
# We reject hard above 80 (a coloured patch that merely looks dark).
SAT_MEAN_MAX     = 80
# Internal Canny-edge pixel density. Empirically, battery contours in
# battery_data sit at ~0.04–0.15 (printed icons, battery seams, terminal
# grille). Smooth shadows sit near 0. Use a very soft floor so low-res
# camera frames still pass, but penalise in score.
EDGE_DENS_FLOOR  = 0.005
EDGE_DENS_TARGET = 0.06
# V inside the candidate blob — battery stays genuinely dark even after
# morphology; dimly-lit wall/cloth regions creep higher.
V_MEAN_MAX       = 70

# ── Inside-bright (printed text / metal contacts) ────────────────────────
# Mavic 3 batteries carry crisp white printing (model number, capacity,
# warning icons) and a metal terminal grille on one face. Both render as
# bright low-saturation pixels *inside* the otherwise-dark contour. A
# featureless black block (drone arm cap, foam edge, shadow) has none.
# We use this as
#   1. a confirmation signal — small contours are only kept when bright
#      content is present, so the standalone tripod base / random dark
#      blob can no longer pass on shape alone, and
#   2. a pose hint — PCA of the bright pixels gives the long-axis angle
#      of the printed face, useful for orienting the gripper.
TEXT_V_FLOOR     = 120     # V floor for "bright" inside the contour
TEXT_S_CEIL      = 80      # S ceiling so colour reflections don't count
# Use an absolute pixel count (not a fraction) for the small-area gate:
# at the arm camera's working distance, the contact circles read as ~10
# bright px regardless of how big the battery silhouette is. A fractional
# threshold would scale with contour area and reject distant batteries
# that have *more* fractional brightness than nearby ones — the wrong
# direction. Standalone tripod base measured 1 bright px in the same
# frame, so 5 is a clean separator.
TEXT_COUNT_FOR_SMALL = 5
SMALL_AREA_LIMIT = 0.06    # below this, text confirmation is required

# ── Smooth-block rejection ───────────────────────────────────────────────
# A smooth flat black accessory next to the battery (e.g. drone arm
# cap, stand, propeller cover) presents as: very high solidity, very
# rectangular, low edge density (no terminals or labels), partial fill,
# and slight colour cast (mean_sat above the battery's neutral black).
# These four together never appear in the labelled battery_data set —
# any one battery image that matches three of them still fails on at
# least one — so requiring all four cleanly excludes only the
# distractor. See _full_survey.py for the data this is calibrated on.
SMOOTH_BLOCK_SOLIDITY = 0.97
SMOOTH_BLOCK_EDGE     = 0.035
SMOOTH_BLOCK_FILL     = 0.70
SMOOTH_BLOCK_SAT      = 55

# ── Score gate ───────────────────────────────────────────────────────────
# Lowered together with the rect/solidity floors above. A clean battery
# on a tripod scores 0.55–0.70; a tilted/side-laid battery slips into
# the 0.40–0.45 range because rect/solidity carry less weight on it.
# The hard-reject rules (smooth-block + edge-touching + min area)
# already filter the bulk of dark-shape noise, so the score gate can
# be looser without flooding HostGUI with phantoms.
SCORE_MIN        = 0.38


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
    close_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (k, k))
    # Closing fills the battery's label cutouts and contact seams.
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, close_kernel, iterations=2)
    # Use a *larger* open kernel than the close kernel so any thin
    # bridges the close just bridged — typically the battery silhouette
    # touching off-screen equipment / cables at the top edge of the
    # frame — get split apart again. Kernel (k+2)² removes connections
    # ≤ ⌊(k+2)/2⌋ ≈ 2 px while only nibbling 2 px off the battery body
    # (insignificant against a 100×150 px silhouette). Without this the
    # battery contour would touch y=0 and the edge-touch reject would
    # drop it.
    open_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (k + 2, k + 2))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  open_kernel, iterations=1)
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

    # Inside-bright pixels (printed text + metal contacts). PCA of
    # their positions gives the printed-face long axis when present.
    V_full = hsv[..., 2]
    S_full = hsv[..., 1]
    bright_mask = (V_full >= TEXT_V_FLOOR) & (S_full <= TEXT_S_CEIL) & (roi_mask > 0)
    ys_b, xs_b = np.where(bright_mask)
    bright_count = int(xs_b.size)
    bright_frac  = bright_count / max(1.0, area)

    text_angle_deg = None
    text_dispersion = 0.0
    if bright_count >= 8:
        pts = np.column_stack([xs_b, ys_b]).astype(np.float64)
        mean = pts.mean(axis=0)
        cov  = np.cov((pts - mean).T)
        eigvals, eigvecs = np.linalg.eigh(cov)
        idx = int(np.argmax(eigvals))
        major = eigvecs[:, idx]
        text_angle_deg = float(np.degrees(np.arctan2(major[1], major[0])))
        # dispersion = sqrt(major eigenvalue) ≈ pixels span along axis;
        # high dispersion = text spread along a real long axis (good),
        # low dispersion = a single specular dot (ignore).
        text_dispersion = float(np.sqrt(max(eigvals[idx], 0.0)))

    return dict(
        area=area, bbox=(x, y, bw, bh), hull_area=hull_area,
        long_side=long_side, short_side=short_side, aspect=aspect,
        mean_sat=mean_sat, mean_v=mean_v,
        edge_density=edge_density,
        rectangularity=area / max(1.0, long_side * short_side),
        fill=area / max(1.0, bw * bh),
        solidity=area / max(1.0, hull_area),
        bright_count=bright_count,
        bright_frac=bright_frac,
        text_angle_deg=text_angle_deg,
        text_dispersion=text_dispersion,
    )


def _score_candidate(f: dict, frame_area: float) -> tuple[float, bool]:
    """Return (score, is_end_view).  Score ≥ 0 only if candidate passes."""
    # Hard shape filters
    if f["area"] < MIN_AREA_FRAC * frame_area: return -1.0, False
    if f["area"] > MAX_AREA_FRAC * frame_area: return -1.0, False
    if f["aspect"] > ASPECT_MAX:               return -1.0, False
    if f["aspect"] < ASPECT_MIN_END:           return -1.0, False
    if f["fill"]   < FILL_MIN:                 return -1.0, False

    # Small-area gate: if the contour is below SMALL_AREA_LIMIT we need
    # *some* extra signal to confirm it's a battery and not the
    # standalone tripod base, drone arm cap, or random dark blob.
    # Two ways to pass:
    #   (a) printed text / metal contacts inside (bright pixel count), or
    #   (b) very strong rectangle-and-solidity shape — this catches the
    #       Mavic 3 battery laid on its side (long thin profile, no
    #       label face visible) which has bright_count = 0 but
    #       rectangularity ≥ 0.75 and solidity ≥ 0.95. The smooth-block
    #       rule below still rejects featureless distractors with the
    #       same shape via the sat / fill / edge fingerprint.
    area_frac = f["area"] / frame_area
    if area_frac < SMALL_AREA_LIMIT:
        strong_shape = (f["rectangularity"] >= 0.75
                        and f["solidity"]    >= 0.95)
        if not strong_shape and f["bright_count"] < TEXT_COUNT_FOR_SMALL:
            return -1.0, False

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

    # Hard reject: smooth flat distractor (no internal texture, partial
    # fill, slight colour cast). All four conditions together never
    # appear on a real battery in battery_data; isolating this signature
    # keeps the smooth-block negative from competing with the real
    # battery on close scoring matches.
    if (f["solidity"]     > SMOOTH_BLOCK_SOLIDITY
            and f["edge_density"] < SMOOTH_BLOCK_EDGE
            and f["fill"]         < SMOOTH_BLOCK_FILL
            and f["mean_sat"]     > SMOOTH_BLOCK_SAT):
        return -1.0, False

    # Score components — all normalised to [0, 1], then weighted.
    # Saturate size_score at 10 % of frame: real batteries in the
    # working range fall between ~5 % and ~18 %, so 10 % is the
    # natural midpoint where the score should already be near full.
    # The old 0.25 ceiling capped most real batteries at size_score
    # ≤ 0.4, leaving the tilted/laid-over case (5 % area, low rect/
    # solidity) at total score 0.37 — just below the gate.
    size_score   = min(1.0, f["area"] / (0.10 * frame_area))
    aspect_norm  = min(1.0, max(0.0,
                       (f["aspect"] - ASPECT_MIN_END)
                       / (3.0 - ASPECT_MIN_END)))
    sat_score    = max(0.0, 1.0 - f["mean_sat"] / SAT_MEAN_MAX)
    # Edge density shaped as a soft ramp so an image-pyramid level with
    # few captured details doesn't zero the score, but a truly smooth
    # shadow still loses out. Weight bumped to 0.20 to widen the gap
    # between the textured battery and a smooth black accessory: in
    # debug capture the battery sat at edge=0.054 and the distractor
    # block at 0.029, so this is the most-discriminative signal we have.
    edge_score = min(1.0, f["edge_density"] / EDGE_DENS_TARGET)
    edge_pass  = f["edge_density"] >= EDGE_DENS_FLOOR

    # Text-content bonus: bright printing inside the contour is a
    # battery-specific positive signal — small bonus, capped at 0.10
    # so a textureless-but-clean shape can still pass on its own.
    text_bonus = min(0.10, f["bright_frac"] * 8.0)

    score = (0.25 * f["rectangularity"]
             + 0.16 * f["solidity"]
             + 0.16 * size_score
             + 0.10 * aspect_norm
             + 0.13 * sat_score
             + 0.20 * edge_score
             + text_bonus)
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
    h_img, w_img = bgr.shape[:2]
    frame_area = float(h_img * w_img)
    out = []
    for c in contours:
        # Cheap early-reject on bounding-box area before we do heavier work.
        xa, ya, wa, ha = cv2.boundingRect(c)
        if wa * ha < MIN_AREA_FRAC * frame_area: continue

        # Drop blobs pinned against the frame edge — they're equipment
        # bleeding in from off-screen, not the battery sitting on the
        # workspace. (See debug capture: a corner contour at (0,0,193,44)
        # was scoring just above the real battery centred at ~(265,84).)
        if (xa <= EDGE_TOUCH_PX
                or ya <= EDGE_TOUCH_PX
                or xa + wa >= w_img - EDGE_TOUCH_PX
                or ya + ha >= h_img - EDGE_TOUCH_PX):
            continue

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
