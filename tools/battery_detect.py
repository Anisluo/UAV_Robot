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
# Below this fraction the contour needs *some* extra signal (printed
# text or very strong shape) on top of basic area/aspect/rect filters.
# Empirical workspace data:
#   real battery silhouettes — all ≥ 4.5 % across viewing angles, with
#                              the smallest sample (back face, far)
#                              measuring 4.6 %.
#   standalone tripod base   — ~ 3 %.
# 4 % cleanly separates the two: real batteries skip the gate entirely,
# the lone base is still small-enough to be required to bring text or
# strong-shape evidence (and fails both).
SMALL_AREA_LIMIT = 0.04

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

    Implementation: np.bincount over uint8 — 10× faster than
    np.percentile (which sort-partitions a 307 k-element array) and
    way faster than np.histogram on a non-contiguous slice (which is
    what hsv[..., 2] is). np.bincount internally fast-paths uint8
    via a single O(n) pass.
    """
    cnt = np.bincount(v_channel.ravel(), minlength=256)
    cum = np.cumsum(cnt)
    target = cum[-1] * percentile / 100.0
    p = int(np.searchsorted(cum, target))
    return max(20, min(hard_ceiling, p + 5))


def _find_workspace_bbox(hsv: np.ndarray):
    """Locate the dominant white work-surface (foam plate / desk) and
    return its bounding rect (x, y, w, h), or None if no plate is
    confidently visible in the frame.

    Used to anchor the dark-mask search when the operator puts the
    battery on a clear plate against a noisy background (boxes /
    cables creep in from the top of the frame and otherwise bridge
    into the battery silhouette through morphological closing).

    Confidence checks (all must hold) keep this from firing on
    arbitrary close-up shots where there is no plate in scene:
      - largest plate-coloured component covers ≥ 25 % of the frame
      - its bbox sits inside the inner 80 % of the frame both axes
        (i.e. doesn't itself touch the frame edge — that's a sign
         we picked up background, not a plate)
    Otherwise return None and the caller falls back to whole-frame.
    """
    h_img, w_img = hsv.shape[:2]
    # Run plate detection at half resolution — the plate is dominant
    # so detail is unnecessary, and the morphology cost drops 4×.
    # The bbox we recover is scaled back to full resolution.
    scale = 2
    hsv_small = cv2.resize(hsv, (w_img // scale, h_img // scale),
                           interpolation=cv2.INTER_NEAREST)
    v = hsv_small[..., 2]
    s = hsv_small[..., 1]
    # Use the 75 th-percentile V as a relative "plate-bright" floor
    # rather than a fixed value — the workspace plate is the brightest
    # surface in frame but its absolute V varies wildly with overhead
    # lighting (180 in some captures, 240 in others). Capping at 220
    # keeps a bright-overlit lab from dragging the threshold above the
    # actual plate.
    hist, _ = np.histogram(v, bins=256, range=(0, 256))
    cum = np.cumsum(hist)
    p75 = int(np.searchsorted(cum, cum[-1] * 0.75))
    v_floor = min(220, max(120, p75))
    bright = ((v >= v_floor) & (s <= 50)).astype(np.uint8) * 255
    # Smaller kernel to match the half-resolution image (effective
    # 14 px at full res).
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))
    bright = cv2.morphologyEx(bright, cv2.MORPH_CLOSE, kernel, iterations=1)
    contours, _ = cv2.findContours(bright, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    largest = max(contours, key=cv2.contourArea)
    h_s, w_s = bright.shape
    if cv2.contourArea(largest) < 0.25 * h_s * w_s:
        return None
    x, y, w, h = cv2.boundingRect(largest)
    # Scale bbox back to full resolution.
    x *= scale; y *= scale; w *= scale; h *= scale
    if x < 8 or y < 8 or x + w > w_img - 8 or y + h > h_img - 8:
        return None  # plate touches frame edge — probably bg colour
    return (x, y, w, h)


def _build_mask(hsv: np.ndarray, v_thr: int, k: int) -> np.ndarray:
    v = hsv[..., 2]
    mask = (v < v_thr).astype(np.uint8) * 255

    # Zero the top / bottom border bands of the dark mask. Off-screen
    # cables, equipment boxes, floor seams etc. leak into the frame's
    # edge strips, and morphological closing then bridges them to the
    # battery silhouette in the workspace. The merged contour ends up
    # touching y=0 / y=h-1, the edge-touch reject drops it, and the
    # battery becomes invisible. A workspace battery is always ≥ ~30
    # px in from the top of the frame on this rig, so erasing the band
    # is a clean fix; the wider 5x5 open below is no longer enough on
    # its own when the bridge is 10–15 px wide.
    h_img = mask.shape[0]
    edge_band = max(20, h_img // 16)
    mask[:edge_band, :] = 0
    mask[h_img - edge_band:, :] = 0

    close_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (k, k))
    # Closing fills the battery's label cutouts and contact seams.
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, close_kernel, iterations=2)
    # Use a *larger* open kernel than the close kernel so any thin
    # bridges the close just bridged get split apart again. Kernel
    # (k+2)² removes connections ≤ ⌊(k+2)/2⌋ ≈ 2 px while only
    # nibbling 2 px off the battery body (insignificant against a
    # 100×150 px silhouette).
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

    # All per-contour stats live inside the contour's axis-aligned
    # bbox — there's no point allocating a full-frame ROI mask just
    # to mark <5 % of its pixels. Working on the bbox sub-image cuts
    # this function from ~3.7 ms to ~0.5 ms on a 640×480 frame.
    sub_mask = np.zeros((bh, bw), dtype=np.uint8)
    shifted = contour - np.array([[x, y]], dtype=contour.dtype)
    cv2.drawContours(sub_mask, [shifted], -1, 255, thickness=cv2.FILLED)
    sub_inside = sub_mask > 0
    sub_hsv  = hsv[y:y + bh, x:x + bw]
    sub_edge = edge_map[y:y + bh, x:x + bw]

    sat  = sub_hsv[..., 1][sub_inside]
    vch  = sub_hsv[..., 2][sub_inside]
    edges_in = sub_edge[sub_inside]

    mean_sat = float(sat.mean()) if sat.size else 255.0
    mean_v   = float(vch.mean()) if vch.size else 255.0
    edge_density = float((edges_in > 0).sum()) / max(1.0, area)

    # Inside-bright pixels (printed text + metal contacts). PCA of
    # their positions gives the printed-face long axis when present.
    bright_mask = (sub_hsv[..., 2] >= TEXT_V_FLOOR) \
                  & (sub_hsv[..., 1] <= TEXT_S_CEIL) \
                  & sub_inside
    ys_b, xs_b = np.where(bright_mask)
    bright_count = int(xs_b.size)
    bright_frac  = bright_count / max(1.0, area)

    text_angle_deg = None
    text_dispersion = 0.0
    if bright_count >= 8:
        # Add the bbox offset back so the angle is in frame coords —
        # PCA on relative coords gives the same eigenvectors but a
        # consumer reading text_angle_deg expects frame-aligned axes.
        pts = np.column_stack([xs_b + x, ys_b + y]).astype(np.float64)
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
        # Strong-shape escape: rectangularity ≥ 0.75 AND solidity
        # ≥ 0.90. The 0.90 floor is calibrated against an arm-camera
        # capture where the workspace plate ROI cropped a battery
        # silhouette down to 2.1 % area, solidity 0.942 — solid enough
        # to be unambiguously battery-shaped but failing the older
        # 0.95 floor by a hair. The smooth-block rule below still
        # rejects the matt-black accessory shapes (which pair high
        # solidity with low edge density and a colour cast), so this
        # only widens the gate for real batteries.
        strong_shape = (f["rectangularity"] >= 0.75
                        and f["solidity"]    >= 0.90)
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


def _expand_top_to_dim(hsv: np.ndarray,
                       x1: int, y1: int, x2: int,
                       max_extend: int = 18,
                       v_dim: int = 130) -> int:
    """Walk upward from a contour's top edge, extending it across rows
    where the bbox-column median V is still relatively dim. The dark
    mask uses V<80 to chase pure black; battery metal contacts and
    seam highlights typically sit at V≈90–130 and get cut off — this
    recovers them so the bbox encloses the whole battery, not just
    its body. Capped at max_extend px so it can't run into off-plate
    background junk that may be sitting just above the workspace.
    """
    if y1 <= 0:
        return y1
    v = hsv[..., 2]
    x1c = max(0, x1)
    x2c = min(v.shape[1], x2)
    if x2c <= x1c:
        return y1
    new_y1 = y1
    for offset in range(1, max_extend + 1):
        cand = y1 - offset
        if cand < 0:
            break
        row = v[cand, x1c:x2c]
        if int(np.median(row)) < v_dim:
            new_y1 = cand
        else:
            break
    return new_y1


def _find_candidates(bgr: np.ndarray, hsv: np.ndarray, edge_map: np.ndarray,
                     v_thr: int, k: int,
                     plate=None) -> list[tuple[float, BatteryBox, dict]]:
    """`plate` may be supplied by the caller (so detect() can compute
    it once and reuse across primary + fallback v_thr passes). When
    omitted we fall back to the per-call detection."""
    mask = _build_mask(hsv, v_thr, k)
    h_img, w_img = bgr.shape[:2]
    frame_area = float(h_img * w_img)
    if plate is None:
        plate = _find_workspace_bbox(hsv)
    # When the plate is found, mask off everything outside it before
    # contour search. The off-screen background (boxes, cables) lives
    # *outside* the plate and otherwise gets bridged into in-workspace
    # contours by morphological closing. Restricting attention to the
    # plate area is the only fix that handles wide (≥ battery-width)
    # bridges between the workspace object and ambient junk.
    if plate is not None:
        px, py, pw, ph = plate
        # Pad the ROI vertically so a battery standing on the plate
        # whose top juts above the plate's bright edge still gets its
        # full silhouette captured. Without padding, the bbox stops at
        # the plate top and HostGUI shows the battery cut off at the
        # midline. Background junk usually sits well above 30 px clear
        # of the plate so this padding doesn't reintroduce bridges.
        pad_top = 30
        py2 = max(0, py - pad_top)
        ph2 = ph + (py - py2)
        roi = np.zeros_like(mask)
        roi[py2:py2 + ph2, px:px + pw] = 255
        mask = cv2.bitwise_and(mask, roi)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
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
        # Extend top of the bbox to cover battery sections (metal
        # contacts, label highlights) whose V is too bright for the
        # 0.05–0.10 V_FLOOR strict mask. Limited to the height of
        # the bbox itself so a sea of darkness above can't pull the
        # bbox into off-screen junk.
        x2 = x + bw
        y_top = _expand_top_to_dim(hsv, x, y, x2, max_extend=min(20, bh // 4))
        out.append((score, (x, y_top, x2, y + bh, score), feats))
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

    # Compute the workspace plate ROI once and feed it into every
    # v_thr pass — the plate doesn't change between passes and the
    # detection costs ~1.4 ms even at half resolution. Same applies
    # to the V channel histogram, which all three percentile lookups
    # share.
    plate = _find_workspace_bbox(hsv)
    v_channel = hsv[..., 2]
    v_cum = np.cumsum(np.bincount(v_channel.ravel(), minlength=256))

    def _percentile_via_hist(percentile, hard_ceiling):
        target = v_cum[-1] * percentile / 100.0
        p = int(np.searchsorted(v_cum, target))
        return max(20, min(hard_ceiling, p + 5))

    # Primary pass — adaptive threshold from the 12 th percentile.
    v_thr = _percentile_via_hist(12.0, 80)
    candidates = _find_candidates(bgr, hsv, edge_map, v_thr, k, plate=plate)

    # Fallback — if nothing passes, try a stricter (darker) threshold.
    # Helps when the battery shares the scene with a mid-grey object:
    # the 12 th percentile is dragged up, merging blobs in the mask.
    if not candidates:
        v_thr2 = _percentile_via_hist(7.0, 60)
        if v_thr2 < v_thr:
            candidates = _find_candidates(bgr, hsv, edge_map, v_thr2, k,
                                          plate=plate)

    # Deep fallback — very lenient mask for cases where the battery sits
    # on a dark background and the top percentile is already noisy. We
    # still require SCORE_MIN so this only saves borderline misses.
    if not candidates:
        v_thr3 = _percentile_via_hist(18.0, 95)
        if v_thr3 > v_thr:
            candidates = _find_candidates(bgr, hsv, edge_map, v_thr3, k,
                                          plate=plate)

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
