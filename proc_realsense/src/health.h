#ifndef UAV_PROC_REALSENSE_HEALTH_H
#define UAV_PROC_REALSENSE_HEALTH_H

#include <cstdint>

#include "abi/msg_types.h"

class BHealth {
public:
    BHealth();
    void on_frame_published();
    void set_drop_count(uint32_t drops);
    bool should_emit_heartbeat(uint64_t now_ns) const;
    UavBHeartbeat make_heartbeat(uint64_t now_ns) const;

private:
    mutable uint64_t last_hb_ns_;
    mutable uint64_t frame_count_;
    mutable uint64_t window_start_ns_;
    uint32_t drops_;
};

#endif
