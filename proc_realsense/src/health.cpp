#include "health.h"

BHealth::BHealth() : last_hb_ns_(0), frame_count_(0), window_start_ns_(0), drops_(0) {}

void BHealth::on_frame_published() {
    frame_count_++;
}

void BHealth::set_drop_count(uint32_t drops) {
    drops_ = drops;
}

bool BHealth::should_emit_heartbeat(uint64_t now_ns) const {
    return (now_ns - last_hb_ns_) >= 1000000000ULL;
}

UavBHeartbeat BHealth::make_heartbeat(uint64_t now_ns) const {
    if (window_start_ns_ == 0) {
        window_start_ns_ = now_ns;
    }
    const uint64_t elapsed_ns = now_ns - window_start_ns_;
    float fps = 0.0F;
    if (elapsed_ns > 0) {
        fps = static_cast<float>(frame_count_) * 1e9F / static_cast<float>(elapsed_ns);
    }

    UavBHeartbeat hb{};
    hb.timestamp_ns = now_ns;
    hb.fps = fps;
    hb.drop_count = drops_;

    last_hb_ns_ = now_ns;
    frame_count_ = 0;
    window_start_ns_ = now_ns;
    return hb;
}
