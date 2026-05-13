#include "npu_pipeline.h"

#include <algorithm>
#include <chrono>
#include <cstdio>

namespace {
void print_result(const UavCResult &result) {
    // Per-frame trace — only emitted when explicitly enabled. journald
    // can keep up at this rate (5 fps, 30 chars/line) but the cleanup
    // pass treats every per-iteration printf the same way so the
    // process can be left running for hours without filling the
    // journal. Set UAV_NPU_TRACE=1 to re-enable for diagnostics.
    static const bool s_trace = []() {
        const char *e = std::getenv("UAV_NPU_TRACE");
        return e != nullptr && e[0] != '\0' && e[0] != '0';
    }();
    if (!s_trace) return;
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

    // ── PRESENT/ABSENT state machine (Schmitt-trigger hysteresis) ────────
    // The user needs a clean binary platform state, not just per-frame
    // boxes. The COCO model flicks single-frame "airplane" phantoms on
    // background junk and conversely drops the real drone for one frame
    // every couple of seconds, so we use asymmetric counters:
    //
    //   ABSENT  state — empty published frames. To leave: need
    //                   kEnterFrames consecutive hits that IoU-match the
    //                   prior hit (same drone, not a moving phantom).
    //   PRESENT state — sticky bbox published. To leave: need kExitFrames
    //                   consecutive empty frames. Any hit mid-exit
    //                   resets exit_count_ — drone re-found.
    //
    // The bbox itself is EMA-blended while PRESENT so it tracks motion
    // without snapping on every frame.
    constexpr float kIouConfirm = 0.40F;

    auto detections_match = [&](const UavCResult &a, const UavCResult &b) {
        for (uint32_t i = 0; i < a.num_detections; ++i) {
            for (uint32_t j = 0; j < b.num_detections; ++j) {
                if (a.detections[i].class_id != b.detections[j].class_id) continue;
                if (det_iou(a.detections[i], b.detections[j]) >= kIouConfirm) {
                    return true;
                }
            }
        }
        return false;
    };

    if (result.num_detections > 0) {
        // Hit this frame.
        exit_count_ = 0;
        if (present_) {
            // PRESENT: EMA-blend bbox with sticky to kill jitter.
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
        } else {
            // ABSENT: count consecutive matching hits.
            if (pending_valid_ && detections_match(result, pending_)) {
                ++enter_count_;
            } else {
                enter_count_ = 1;       // first hit (or hit moved too much)
            }
            pending_       = result;
            pending_valid_ = true;
            if (enter_count_ >= kEnterFrames) {
                // Promote to PRESENT.
                sticky_           = result;
                sticky_valid_     = true;
                present_          = true;
                enter_count_      = 0;
                pending_valid_    = false;
                std::printf("C_PRESENCE drone=1\n");
                std::fflush(stdout);
            }
        }
    } else {
        // Empty frame.
        enter_count_   = 0;
        pending_valid_ = false;
        if (present_) {
            ++exit_count_;
            // Keep showing the sticky bbox until we confirm absence.
            if (exit_count_ < kExitFrames && sticky_valid_) {
                result = sticky_;
                result.frame_id = frame.slot.frame_id;
            } else {
                // PRESENT → ABSENT transition.
                present_       = false;
                sticky_valid_  = false;
                exit_count_    = 0;
                std::printf("C_PRESENCE drone=0\n");
                std::fflush(stdout);
            }
        }
        // else: stay ABSENT, emit empty result.
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
