#ifndef UAV_PROC_NPU_HEALTH_H
#define UAV_PROC_NPU_HEALTH_H

#include <cstdint>

#include "abi/msg_types.h"

class CHealth {
public:
    CHealth();
    void on_infer_done(float latency_ms);
    bool should_emit_heartbeat(uint64_t now_ns) const;
    UavCHeartbeat make_heartbeat(uint64_t now_ns) const;

private:
    mutable uint64_t last_hb_ns_;
    mutable uint64_t infer_count_;
    mutable uint64_t window_start_ns_;
    mutable float sum_latency_ms_;
};

#endif
