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

    // Separate the synthetic platform marker (class_id >= UAV_CLASS_META_BASE)
    // from real drone hits so the Schmitt-trigger below counts ONLY drones.
    // The marker still ends up published unchanged — HostGUI uses it to
    // render the "platform visible" badge.
    auto has_drone_hit = [](const UavCResult &r) {
        for (uint32_t i = 0; i < r.num_detections; ++i) {
            if (r.detections[i].class_id < UAV_CLASS_META_BASE) return true;
        }
        return false;
    };
    auto has_platform_marker = [](const UavCResult &r) {
        for (uint32_t i = 0; i < r.num_detections; ++i) {
            if (r.detections[i].class_id == UAV_CLASS_PLATFORM) return true;
        }
        return false;
    };

    const bool drone_hit_this_frame = has_drone_hit(result);
    const bool platform_now         = has_platform_marker(result);

    // Platform state has no flicker (large bright region), so a single-frame
    // edge is the truth. Emit C_PRESENCE on every transition so journald
    // shows a clean ABSENT-from-the-start signal even before any drone
    // ever appears.
    if (platform_now != platform_visible_) {
        platform_visible_ = platform_now;
        std::printf("C_PRESENCE drone=%d platform=%d\n",
                    present_ ? 1 : 0, platform_visible_ ? 1 : 0);
        std::fflush(stdout);
    }

    // ── PRESENT/ABSENT (drone) state machine — Schmitt-trigger ──────────
    //   ABSENT  → PRESENT: kEnterFrames consecutive matching drone hits.
    //   PRESENT → ABSENT:  kExitFrames consecutive frames with no drone hit
    //                      (platform marker is ignored).
    // The bbox is EMA-blended while PRESENT.
    // Tight IoU (0.55) — real drone holds 0.7+ between adjacent frames even
    // at 5 fps; sporadic phantoms jitter around at 0.2–0.3 and can't make
    // a 5-in-a-row run.
    constexpr float kIouConfirm = 0.55F;

    auto drone_iou = [&](const UavCResult &a, const UavCResult &b) {
        for (uint32_t i = 0; i < a.num_detections; ++i) {
            if (a.detections[i].class_id >= UAV_CLASS_META_BASE) continue;
            for (uint32_t j = 0; j < b.num_detections; ++j) {
                if (b.detections[j].class_id >= UAV_CLASS_META_BASE) continue;
                if (a.detections[i].class_id != b.detections[j].class_id) continue;
                if (det_iou(a.detections[i], b.detections[j]) >= kIouConfirm) {
                    return true;
                }
            }
        }
        return false;
    };

    if (drone_hit_this_frame) {
        exit_count_ = 0;
        if (present_) {
            // PRESENT: EMA-blend each drone bbox against sticky.
            for (uint32_t i = 0; i < result.num_detections; ++i) {
                UavDetection &cur = result.detections[i];
                if (cur.class_id >= UAV_CLASS_META_BASE) continue;
                for (uint32_t j = 0; j < sticky_.num_detections; ++j) {
                    const UavDetection &prev = sticky_.detections[j];
                    if (prev.class_id >= UAV_CLASS_META_BASE) continue;
                    if (prev.class_id != cur.class_id) continue;
                    if (det_iou(prev, cur) >= kIouSnap) {
                        cur = ema_blend(prev, cur, kEmaAlpha);
                    }
                    break;
                }
            }
            sticky_ = result;
        } else {
            if (pending_valid_ && drone_iou(result, pending_)) {
                ++enter_count_;
            } else {
                enter_count_ = 1;
            }
            pending_       = result;
            pending_valid_ = true;
            if (enter_count_ >= kEnterFrames) {
                sticky_           = result;
                sticky_valid_     = true;
                present_          = true;
                enter_count_      = 0;
                pending_valid_    = false;
                std::printf("C_PRESENCE drone=1 platform=%d\n",
                            platform_visible_ ? 1 : 0);
                std::fflush(stdout);
            }
        }
    } else {
        // No drone this frame. The platform marker (if any) is still in
        // `result` and will be published — that's the "empty platform"
        // positive signal.
        enter_count_   = 0;
        pending_valid_ = false;
        if (present_) {
            ++exit_count_;
            if (exit_count_ < kExitFrames && sticky_valid_) {
                // Re-attach the sticky drone bbox; keep the platform
                // marker from `result` so the GUI shows BOTH.
                UavCResult merged = sticky_;
                merged.frame_id = frame.slot.frame_id;
                if (platform_now && merged.num_detections < UAV_MAX_DETECTIONS) {
                    for (uint32_t i = 0; i < result.num_detections; ++i) {
                        if (result.detections[i].class_id == UAV_CLASS_PLATFORM) {
                            merged.detections[merged.num_detections++] =
                                result.detections[i];
                            break;
                        }
                    }
                }
                result = merged;
            } else {
                present_       = false;
                sticky_valid_  = false;
                exit_count_    = 0;
                std::printf("C_PRESENCE drone=0 platform=%d\n",
                            platform_visible_ ? 1 : 0);
                std::fflush(stdout);
            }
        }
        // else: ABSENT — publish whatever postprocess returned (platform
        // marker only, or fully empty if camera is blind).
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
