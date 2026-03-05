#include "health.h"

CHealth::CHealth() : last_hb_ns_(0), infer_count_(0), window_start_ns_(0), sum_latency_ms_(0.0F) {}

void CHealth::on_infer_done(float latency_ms) {
    infer_count_++;
    sum_latency_ms_ += latency_ms;
}

bool CHealth::should_emit_heartbeat(uint64_t now_ns) const {
    return (now_ns - last_hb_ns_) >= 1000000000ULL;
}

UavCHeartbeat CHealth::make_heartbeat(uint64_t now_ns) const {
    if (window_start_ns_ == 0) {
        window_start_ns_ = now_ns;
    }
    const uint64_t elapsed_ns = now_ns - window_start_ns_;
    float fps = 0.0F;
    if (elapsed_ns > 0) {
        fps = static_cast<float>(infer_count_) * 1e9F / static_cast<float>(elapsed_ns);
    }
    float avg_latency = 0.0F;
    if (infer_count_ > 0) {
        avg_latency = sum_latency_ms_ / static_cast<float>(infer_count_);
    }

    UavCHeartbeat hb{};
    hb.timestamp_ns = now_ns;
    hb.inference_fps = fps;
    hb.avg_latency_ms = avg_latency;

    last_hb_ns_ = now_ns;
    infer_count_ = 0;
    sum_latency_ms_ = 0.0F;
    window_start_ns_ = now_ns;
    return hb;
}
