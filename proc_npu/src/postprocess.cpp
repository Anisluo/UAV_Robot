#include "postprocess.h"

#include <cmath>
#include <cstdint>
#include <algorithm>

// ── Depth validity range (meters) ────────────────────────────────────────────
static constexpr float DEPTH_MIN_M = 0.1f;
static constexpr float DEPTH_MAX_M = 4.0f;

// Read uint16 depth value (little-endian) at pixel (px, py) from depth map.
// Returns depth in meters, or 0.0 if out of range / invalid.
static float read_depth_m(const InferenceFrame &frame,
                           uint32_t px, uint32_t py)
{
    if (frame.slot.depth_scale == 0.0f) return 0.0f;

    const uint32_t w = frame.slot.width;
    const uint32_t h = frame.slot.height;

    if (px >= w || py >= h) return 0.0f;

    const size_t depth_offset = frame.slot.depth_offset
                                + static_cast<size_t>(py * w + px) * sizeof(uint16_t);
    if (depth_offset + sizeof(uint16_t) > frame.payload.size()) return 0.0f;

    const uint8_t *p = frame.payload.data() + depth_offset;
    const uint16_t raw = static_cast<uint16_t>(p[0]) |
                         (static_cast<uint16_t>(p[1]) << 8);
    if (raw == 0) return 0.0f;

    const float depth_m = static_cast<float>(raw) * frame.slot.depth_scale;
    return (depth_m >= DEPTH_MIN_M && depth_m <= DEPTH_MAX_M) ? depth_m : 0.0f;
}

// Sample a small window around (cx, py) and return the median non-zero depth.
// More robust than a single-pixel lookup (RealSense has holes at edges).
static float robust_depth_m(const InferenceFrame &frame,
                             uint32_t cx, uint32_t cy, int radius = 3)
{
    float samples[64];
    int   n = 0;

    const int w = static_cast<int>(frame.slot.width);
    const int h = static_cast<int>(frame.slot.height);
    const int icx = static_cast<int>(cx);
    const int icy = static_cast<int>(cy);

    for (int dy = -radius; dy <= radius; ++dy) {
        for (int dx = -radius; dx <= radius; ++dx) {
            int x = icx + dx, y = icy + dy;
            if (x < 0 || x >= w || y < 0 || y >= h) continue;
            float d = read_depth_m(frame,
                                   static_cast<uint32_t>(x),
                                   static_cast<uint32_t>(y));
            if (d > 0.0f && n < 64) samples[n++] = d;
        }
    }
    if (n == 0) return 0.0f;

    std::sort(samples, samples + n);
    return samples[n / 2]; // median
}

// ── Workspace plate detection (no opencv) ───────────────────────────────────
// Locate the bright workspace surface (foam plate / desk) so we can clip
// drone bbox detections to the actual work area and reject phantom
// "airplane" hits that the COCO model fires on background junk above
// or beside the plate.
//
// We sample a 64×48 grid (every 10th pixel of a 640×480 frame) and
// return the bbox of the connected "bright + low-saturation" cluster
// that touches the most samples. Approximates a connected-component
// scan well enough for a single dominant white surface, with no
// opencv dependency. Returns false if no convincing plate is visible
// — caller falls back to the full frame.
struct PlateBbox {
    uint32_t x1, y1, x2, y2;
};

static bool find_plate_bbox(const InferenceFrame &frame, PlateBbox &out)
{
    const uint32_t w = frame.slot.width;
    const uint32_t h = frame.slot.height;
    if (frame.slot.color_offset >= frame.payload.size()) return false;
    const uint8_t *bgr = frame.payload.data() + frame.slot.color_offset;
    const uint32_t stride = frame.slot.stride;
    if (stride < w * 3U) return false;

    constexpr uint32_t step = 8;          // sample every 8th pixel
    constexpr uint8_t  v_floor = 165;     // bright enough
    constexpr uint8_t  s_max = 60;        // close to white (low saturation)

    uint32_t pmin_x = w, pmin_y = h, pmax_x = 0, pmax_y = 0;
    uint32_t bright_samples = 0;
    uint32_t total_samples  = 0;

    for (uint32_t y = 0; y < h; y += step) {
        const uint8_t *row = bgr + static_cast<size_t>(y) * stride;
        for (uint32_t x = 0; x < w; x += step) {
            const uint8_t b = row[x * 3 + 0];
            const uint8_t g = row[x * 3 + 1];
            const uint8_t r = row[x * 3 + 2];
            const uint8_t v_max = std::max({b, g, r});
            const uint8_t v_min = std::min({b, g, r});
            ++total_samples;
            if (v_max >= v_floor && static_cast<uint8_t>(v_max - v_min) <= s_max) {
                ++bright_samples;
                if (x < pmin_x) pmin_x = x;
                if (y < pmin_y) pmin_y = y;
                if (x > pmax_x) pmax_x = x;
                if (y > pmax_y) pmax_y = y;
            }
        }
    }

    // Need a substantial bright cluster — > 25 % of samples — and a
    // plate that doesn't pin against every frame edge (that would
    // mean we picked up the bg colour).
    if (bright_samples * 4U < total_samples) return false;
    if (pmax_x <= pmin_x || pmax_y <= pmin_y) return false;
    if (pmin_x < step && pmin_y < step
        && pmax_x + step >= w && pmax_y + step >= h) return false;

    out.x1 = pmin_x;
    out.y1 = pmin_y;
    out.x2 = std::min(w, pmax_x + step);
    out.y2 = std::min(h, pmax_y + step);
    return true;
}

// ── Main postprocess ──────────────────────────────────────────────────────────

UavCResult Postprocess::run(const InferenceFrame &frame,
                            const std::vector<RawDet> &raw,
                            float threshold) const
{
    UavCResult out{};
    out.frame_id       = frame.slot.frame_id;
    out.num_detections = 0;

    // ── Anchor-based per-class union ─────────────────────────────────────
    // The COCO-pretrained mavic3_drone.rknn fires multiple class=4
    // boxes on a single drone (body / propeller / arm) that survive
    // NMS at IoU 0.45. We need ALL of those merged into one bbox
    // covering the whole airframe, but mustn't sweep in stray
    // background "airplane" hits at the edges of the frame.
    // Algorithm:
    //   1. Find anchor = highest-score detection per class.
    //   2. Union the anchor with same-class detections whose IoU
    //      against the anchor's *original* bbox is ≥ kAnchorIou.
    //   3. Cap the resulting bbox at kMaxAreaFrac of the frame to
    //      keep cumulative drift in check.
    // Detections that don't IoU with the anchor are treated as a
    // separate object and dropped (no second cluster — the per-class
    // top-1 filter then keeps just the anchor cluster's bbox).
    struct Merged {
        int   class_id;
        float score;
        float x1, y1, x2, y2;
    };
    constexpr float kAnchorIou     = 0.30F;
    constexpr float kMaxAreaFrac   = 0.45F;
    const float frame_max_area = static_cast<float>(frame.slot.width)
                               * static_cast<float>(frame.slot.height)
                               * kMaxAreaFrac;
    auto box_area_xyxy = [](float x1, float y1, float x2, float y2) {
        return std::max(0.0f, x2 - x1) * std::max(0.0f, y2 - y1);
    };
    auto box_iou = [&](float ax1, float ay1, float ax2, float ay2,
                       float bx1, float by1, float bx2, float by2) {
        const float ix1 = std::max(ax1, bx1);
        const float iy1 = std::max(ay1, by1);
        const float ix2 = std::min(ax2, bx2);
        const float iy2 = std::min(ay2, by2);
        const float inter = std::max(0.0f, ix2 - ix1)
                          * std::max(0.0f, iy2 - iy1);
        if (inter <= 0.0f) return 0.0f;
        return inter / (box_area_xyxy(ax1, ay1, ax2, ay2)
                       + box_area_xyxy(bx1, by1, bx2, by2)
                       - inter + 1e-6f);
    };

    // First pass: pick the anchor (highest score) per class.
    std::vector<Merged> anchors;
    for (const RawDet &d : raw) {
        if (d.score < threshold) continue;
        Merged *cur = nullptr;
        for (auto &m : anchors) {
            if (m.class_id == d.class_id) { cur = &m; break; }
        }
        if (cur == nullptr) {
            anchors.push_back({d.class_id, d.score, d.x1, d.y1, d.x2, d.y2});
        } else if (d.score > cur->score) {
            *cur = {d.class_id, d.score, d.x1, d.y1, d.x2, d.y2};
        }
    }

    // Second pass: union each anchor with same-class detections whose
    // bbox overlaps the *anchor* (not the running union — stops drift).
    std::vector<Merged> grouped;
    grouped.reserve(anchors.size());
    for (const Merged &a : anchors) {
        Merged out{a};  // copy of anchor; we'll grow it
        for (const RawDet &d : raw) {
            if (d.score < threshold) continue;
            if (d.class_id != a.class_id) continue;
            // Skip the anchor itself (matched by score+bbox identity).
            if (d.score == a.score
                && d.x1 == a.x1 && d.y1 == a.y1
                && d.x2 == a.x2 && d.y2 == a.y2) continue;
            if (box_iou(d.x1, d.y1, d.x2, d.y2,
                        a.x1, a.y1, a.x2, a.y2) < kAnchorIou) continue;
            // Tentative new bbox after union — accept only if under cap.
            const float nx1 = std::min(out.x1, d.x1);
            const float ny1 = std::min(out.y1, d.y1);
            const float nx2 = std::max(out.x2, d.x2);
            const float ny2 = std::max(out.y2, d.y2);
            if (box_area_xyxy(nx1, ny1, nx2, ny2) > frame_max_area) continue;
            out.x1 = nx1; out.y1 = ny1; out.x2 = nx2; out.y2 = ny2;
            // Anchor's score is already the max for this class.
        }
        grouped.push_back(out);
    }

    // ── Workspace plate + depth gates ────────────────────────────────────
    // Constraint #1 (geometric): clip drone bbox to the white-plate
    // region. The COCO model often expands "airplane" detections off
    // the plate into the floor / cardboard background — those have
    // nothing useful to grasp, so chop them. If less than half the
    // bbox area survives the clip, drop the detection entirely.
    // Constraint #2 (depth): the workspace sits at ~0.3–1.0 m from
    // the arm camera. RKNN bboxes whose centre depth lies outside
    // [0.20, 1.50] m are background hallucinations — discard.
    PlateBbox plate{};
    const bool have_plate = find_plate_bbox(frame, plate);
    constexpr int32_t plate_pad = 30;          // px slack above the plate so
                                                // a standing drone fits in
    constexpr float depth_min_workspace_m = 0.20f;
    constexpr float depth_max_workspace_m = 1.50f;

    for (const Merged &g : grouped) {
        if (out.num_detections >= UAV_MAX_DETECTIONS) break;
        // mavic3_drone.rknn ships as a generic COCO YOLOv8 (nc=80), so
        // a static workspace lights up many classes (tv, keyboard, …).
        // For drone detection we only care about COCO class 4
        // (airplane) — everything else is noise on this scene. If a
        // future strategy needs other classes, drop this guard.
        if (g.class_id != 4) continue;

        // Apply plate clip (drone class only — keep face / battery
        // pass-throughs untouched).
        float cx1 = g.x1, cy1 = g.y1, cx2 = g.x2, cy2 = g.y2;
        if (have_plate) {
            const float px1 = static_cast<float>(plate.x1) - plate_pad;
            const float py1 = static_cast<float>(plate.y1) - plate_pad;
            const float px2 = static_cast<float>(plate.x2) + plate_pad;
            const float py2 = static_cast<float>(plate.y2) + plate_pad;
            const float orig_area = (g.x2 - g.x1) * (g.y2 - g.y1);
            cx1 = std::max(g.x1, px1);
            cy1 = std::max(g.y1, py1);
            cx2 = std::min(g.x2, px2);
            cy2 = std::min(g.y2, py2);
            const float clip_area = std::max(0.0f, cx2 - cx1)
                                  * std::max(0.0f, cy2 - cy1);
            if (orig_area <= 0.0f || clip_area / orig_area < 0.50f) {
                // Mostly outside the plate — almost certainly a
                // phantom airplane detection on background junk.
                continue;
            }
        }

        UavDetection det{};
        det.class_id = g.class_id;
        det.score    = g.score;
        det.x1       = cx1;
        det.y1       = cy1;
        det.x2       = cx2;
        det.y2       = cy2;
        det.has_xyz  = 0;

        // ── 3-D pose from RealSense depth ─────────────────────────────────
        const float cx_px = (cx1 + cx2) * 0.5f;
        const float cy_px = (cy1 + cy2) * 0.5f;
        const uint32_t px = static_cast<uint32_t>(cx_px + 0.5f);
        const uint32_t py = static_cast<uint32_t>(cy_px + 0.5f);

        const float depth_m = robust_depth_m(frame, px, py);
        // Depth gate: drop detections sitting on the wall / outside
        // the grasp working volume. Only enforced when depth is valid
        // — a drone shrouded in propellers can read 0 depth at the
        // bbox centre, in which case we trust the geometric checks.
        if (depth_m > 0.0f
            && (depth_m < depth_min_workspace_m
             || depth_m > depth_max_workspace_m)) {
            continue;
        }
        if (depth_m > 0.0f) {
            const float fx = frame.slot.fx;
            const float fy = frame.slot.fy;
            const float ppx = frame.slot.cx;  // principal point X
            const float ppy = frame.slot.cy;  // principal point Y

            if (fx > 0.0f && fy > 0.0f) {
                // Camera model: Z forward, X right, Y down
                const float Z_mm = depth_m * 1000.0f;
                const float X_mm = (cx_px - ppx) / fx * Z_mm;
                const float Y_mm = (cy_px - ppy) / fy * Z_mm;

        det.x_mm   = X_mm;
        det.y_mm   = Y_mm;
        det.z_mm   = Z_mm;
        det.roll_deg = 0.0f;
        det.pitch_deg = 0.0f;
        det.yaw_deg = 0.0f;
        det.has_xyz = 1;
        det.has_rpy = 0;
        det.grasp_mode = UAV_GRASP_MODE_3D;
            }
        }

        out.detections[out.num_detections++] = det;
    }

    return out;
}
