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

    // ── Temporal smoothing + 2-frame consensus ───────────────────────────
    // Three states for the detection lifecycle:
    //   (a) STEADY  — sticky_valid_ is true; EMA-blend incoming, refresh
    //                 hold, fall back to sticky on empty frames.
    //   (b) WARMUP  — pending_valid_ is true (one prior non-empty frame
    //                 not yet shown). Need a second consecutive matching
    //                 detection to promote into STEADY and start
    //                 publishing. A single-frame phantom never reaches
    //                 STEADY.
    //   (c) IDLE    — neither cache holds anything; every empty frame
    //                 just re-publishes empty.
    if (result.num_detections > 0) {
        if (sticky_valid_) {
            // STEADY: EMA-blend bbox vs sticky, then refresh hold.
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
            sticky_ = result;
            sticky_until_ns_ = now_ns + kStickyHoldNs;
        } else if (pending_valid_) {
            // WARMUP → STEADY: confirm second consecutive frame.
            // Require a tighter IoU (kIouConfirm 0.40) than the EMA
            // snap threshold so a wandering "full-frame airplane"
            // phantom whose bbox jitters across the workspace doesn't
            // ride one through. The real drone, even at 5 fps, keeps
            // bbox overlap well above 0.4 between adjacent frames.
            constexpr float kIouConfirm = 0.40F;
            bool confirmed = false;
            for (uint32_t i = 0; i < result.num_detections && !confirmed; ++i) {
                const UavDetection &cur = result.detections[i];
                for (uint32_t j = 0; j < pending_.num_detections; ++j) {
                    const UavDetection &prev = pending_.detections[j];
                    if (prev.class_id != cur.class_id) continue;
                    if (det_iou(prev, cur) >= kIouConfirm) {
                        confirmed = true;
                        break;
                    }
                }
            }
            if (confirmed) {
                sticky_ = result;
                sticky_valid_ = true;
                sticky_until_ns_ = now_ns + kStickyHoldNs;
                pending_valid_ = false;
            } else {
                // Different bbox / class — reset pending to current.
                pending_ = result;
            }
        } else {
            // IDLE → WARMUP: stash, do not publish yet.
            pending_ = result;
            pending_valid_ = true;
        }
    } else {
        // Empty frame: the stream broke; drop pending warmup state and
        // either fall back to sticky (if hold not yet expired) or emit
        // empty.
        pending_valid_ = false;
        if (sticky_valid_ && now_ns < sticky_until_ns_) {
            result = sticky_;
            result.frame_id = frame.slot.frame_id;
        } else {
            sticky_valid_ = false;
        }
    }

    // While in WARMUP we still hold the sticky cache visible if its
    // own timer hasn't expired (smooth coverage during a brief miss
    // before the next confirmation arrives).
    if (result.num_detections == 0 && sticky_valid_ && now_ns < sticky_until_ns_) {
        result = sticky_;
        result.frame_id = frame.slot.frame_id;
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
