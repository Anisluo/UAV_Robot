#ifndef UAV_PROC_REALSENSE_PIPELINE_H
#define UAV_PROC_REALSENSE_PIPELINE_H

#include <cstdint>

#include "health.h"
#include "rs_capture.h"
#include "shm_writer.h"

class RealsensePipeline {
public:
    explicit RealsensePipeline(ShmWriter *writer);

    bool publish_frame(const CaptureFrame &frame,
                       uint64_t now_ns,
                       UavBHeartbeat *heartbeat_out);
    void emit_idle_heartbeat(uint64_t now_ns, UavBHeartbeat *heartbeat_out);

private:
    ShmWriter *writer_;
    BHealth health_;
};

#endif
