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

// ── Main postprocess ──────────────────────────────────────────────────────────

UavCResult Postprocess::run(const InferenceFrame &frame,
                            const std::vector<RawDet> &raw,
                            float threshold) const
{
    UavCResult out{};
    out.frame_id       = frame.slot.frame_id;
    out.num_detections = 0;

    for (const RawDet &d : raw) {
        if (out.num_detections >= UAV_MAX_DETECTIONS) break;
        if (d.score < threshold) continue;

        UavDetection det{};
        det.class_id = d.class_id;
        det.score    = d.score;
        det.x1       = d.x1;
        det.y1       = d.y1;
        det.x2       = d.x2;
        det.y2       = d.y2;
        det.has_xyz  = 0;

        // ── 3-D pose from RealSense depth ─────────────────────────────────
        const float cx_px = (d.x1 + d.x2) * 0.5f;
        const float cy_px = (d.y1 + d.y2) * 0.5f;
        const uint32_t px = static_cast<uint32_t>(cx_px + 0.5f);
        const uint32_t py = static_cast<uint32_t>(cy_px + 0.5f);

        const float depth_m = robust_depth_m(frame, px, py);
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
                det.has_xyz = 1;
            }
        }

        out.detections[out.num_detections++] = det;
    }

    return out;
}
