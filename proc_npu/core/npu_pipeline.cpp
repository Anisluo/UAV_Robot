#include "npu_pipeline.h"

#include <algorithm>
#include <chrono>
#include <cstdio>

namespace {
void print_result(const UavCResult &result) {
    std::printf("C_RESULT frame=%llu det=%u\n",
                static_cast<unsigned long long>(result.frame_id),
                result.num_detections);
}

float det_iou(const UavDetection &a, const UavDetection &b) {
    const float ix1 = std::max(a.x1, b.x1);
    const float iy1 = std::max(a.y1, b.y1);
    const float ix2 = std::min(a.x2, b.x2);
    const float iy2 = std::min(a.y2, b.y2);
    const float iw = std::max(0.0F, ix2 - ix1);
    const float ih = std::max(0.0F, iy2 - iy1);
    const float inter = iw * ih;
    if (inter <= 0.0F) return 0.0F;
    const float area_a = std::max(0.0F, (a.x2 - a.x1)) * std::max(0.0F, (a.y2 - a.y1));
    const float area_b = std::max(0.0F, (b.x2 - b.x1)) * std::max(0.0F, (b.y2 - b.y1));
    return inter / (area_a + area_b - inter + 1e-6F);
}

UavDetection ema_blend(const UavDetection &prev, const UavDetection &curr, float a) {
    UavDetection out = curr;
    out.x1 = prev.x1 * (1.0F - a) + curr.x1 * a;
    out.y1 = prev.y1 * (1.0F - a) + curr.y1 * a;
    out.x2 = prev.x2 * (1.0F - a) + curr.x2 * a;
    out.y2 = prev.y2 * (1.0F - a) + curr.y2 * a;
    out.score = std::max(prev.score, curr.score);
    return out;
}
}

NpuPipeline::NpuPipeline(NpuInfer *infer, ResultPublisher *publisher)
    : infer_(infer), publisher_(publisher) {}

bool NpuPipeline::process_frame(const InferenceFrame &frame,
                                float threshold,
                                uint64_t now_ns,
                                UavCHeartbeat *heartbeat_out) {
    if (heartbeat_out != nullptr) {
        *heartbeat_out = {};
    }
    if (infer_ == nullptr || publisher_ == nullptr) {
        return false;
    }

    const auto started = std::chrono::steady_clock::now();
    TensorInput tensor{};
    if (!preprocess_.run(frame, tensor)) {
        return false;
    }

    std::vector<RawDet> raw;
    if (!infer_->infer(tensor, raw)) {
        return false;
    }

    UavCResult result = postprocess_.run(frame, raw, threshold);

    // ── Temporal smoothing ───────────────────────────────────────────────
    // EMA-blend overlapping detections against the sticky cache, then
    // either accept the new result (and refresh the hold timer) or
    // replay the sticky one if we're still inside the hold window.
    if (result.num_detections > 0) {
        if (sticky_valid_) {
            for (uint32_t i = 0; i < result.num_detections; ++i) {
                UavDetection &cur = result.detections[i];
                for (uint32_t j = 0; j < sticky_.num_detections; ++j) {
                    const UavDetection &prev = sticky_.detections[j];
                    if (prev.class_id != cur.class_id) continue;
                    if (det_iou(prev, cur) >= kIouSnap) {
                        cur = ema_blend(prev, cur, kEmaAlpha);
                    }
                    break;
                }
            }
        }
        sticky_ = result;
        sticky_valid_ = true;
        sticky_until_ns_ = now_ns + kStickyHoldNs;
    } else if (sticky_valid_ && now_ns < sticky_until_ns_) {
        // Re-publish the previous result with its real frame_id but
        // updated to the current frame so downstream depth lookups
        // line up with the live image.
        result = sticky_;
        result.frame_id = frame.slot.frame_id;
    } else {
        // Hold expired — let the empty result through and reset cache.
        sticky_valid_ = false;
    }

    print_result(result);
    publisher_->publish(result);

    const auto finished = std::chrono::steady_clock::now();
    const float latency_ms =
        std::chrono::duration<float, std::milli>(finished - started).count();
    health_.on_infer_done(latency_ms);
    if (health_.should_emit_heartbeat(now_ns) && heartbeat_out != nullptr) {
        *heartbeat_out = health_.make_heartbeat(now_ns);
    }
    return true;
}
