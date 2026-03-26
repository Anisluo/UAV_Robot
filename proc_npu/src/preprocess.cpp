#include "preprocess.h"

#include <cstring>
#include <algorithm>

static constexpr uint32_t MODEL_W = 640;
static constexpr uint32_t MODEL_H = 640;
static constexpr uint8_t  PAD_VAL = 114; // YOLOv8 standard letterbox gray

bool Preprocess::run(const InferenceFrame &frame, TensorInput &input) const
{
    const uint32_t src_w = frame.slot.width;
    const uint32_t src_h = frame.slot.height;

    if (src_w == 0 || src_h == 0) return false;

    // ── Letterbox scale & padding ─────────────────────────────────────────
    const float scale = std::min(
        static_cast<float>(MODEL_W) / static_cast<float>(src_w),
        static_cast<float>(MODEL_H) / static_cast<float>(src_h));

    const uint32_t scaled_w = static_cast<uint32_t>(src_w * scale + 0.5f);
    const uint32_t scaled_h = static_cast<uint32_t>(src_h * scale + 0.5f);

    const float pad_x = (static_cast<float>(MODEL_W) - static_cast<float>(scaled_w)) * 0.5f;
    const float pad_y = (static_cast<float>(MODEL_H) - static_cast<float>(scaled_h)) * 0.5f;

    const int32_t off_x = static_cast<int32_t>(pad_x + 0.5f);
    const int32_t off_y = static_cast<int32_t>(pad_y + 0.5f);

    input.width      = MODEL_W;
    input.height     = MODEL_H;
    input.orig_width  = src_w;
    input.orig_height = src_h;
    input.lb_scale   = scale;
    input.lb_pad_x   = static_cast<float>(off_x);
    input.lb_pad_y   = static_cast<float>(off_y);

    input.data.assign(MODEL_W * MODEL_H * 3U, PAD_VAL);

    if (frame.slot.color_offset >= frame.payload.size()) {
        return true; // gray frame – no source data
    }
    const uint8_t *src = frame.payload.data() + frame.slot.color_offset;
    const size_t   src_bytes = frame.payload.size() - frame.slot.color_offset;

    // ── Nearest-neighbour resize into letterbox canvas ────────────────────
    uint8_t *dst = input.data.data();

    for (uint32_t dy = 0; dy < scaled_h; ++dy) {
        // Map model row → source row
        const uint32_t sy = static_cast<uint32_t>(
            static_cast<float>(dy) / scale);

        const uint32_t sy_clamped = sy < src_h ? sy : src_h - 1;
        const uint32_t src_row_off = sy_clamped * frame.slot.stride;

        for (uint32_t dx = 0; dx < scaled_w; ++dx) {
            const uint32_t sx = static_cast<uint32_t>(
                static_cast<float>(dx) / scale);
            const uint32_t sx_clamped = sx < src_w ? sx : src_w - 1;

            const size_t src_idx = src_row_off + sx_clamped * 3U;
            if (src_idx + 2 >= src_bytes) continue;

            const uint32_t model_x = static_cast<uint32_t>(off_x) + dx;
            const uint32_t model_y = static_cast<uint32_t>(off_y) + dy;
            const size_t dst_idx = (model_y * MODEL_W + model_x) * 3U;

            dst[dst_idx + 0] = src[src_idx + 0]; // R
            dst[dst_idx + 1] = src[src_idx + 1]; // G
            dst[dst_idx + 2] = src[src_idx + 2]; // B
        }
    }

    return true;
}
