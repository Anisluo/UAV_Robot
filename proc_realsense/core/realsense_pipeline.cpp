#include "realsense_pipeline.h"

RealsensePipeline::RealsensePipeline(ShmWriter *writer) : writer_(writer) {}

bool RealsensePipeline::publish_frame(const CaptureFrame &frame,
                                      uint64_t now_ns,
                                      UavBHeartbeat *heartbeat_out) {
    if (heartbeat_out != nullptr) {
        *heartbeat_out = {};
    }
    if (writer_ == nullptr) {
        return false;
    }

    if (writer_->write_frame(frame)) {
        health_.on_frame_published();
    }
    health_.set_drop_count(writer_->drop_count());
    if (health_.should_emit_heartbeat(now_ns) && heartbeat_out != nullptr) {
        *heartbeat_out = health_.make_heartbeat(now_ns);
    }
    return true;
}

void RealsensePipeline::emit_idle_heartbeat(uint64_t now_ns, UavBHeartbeat *heartbeat_out) {
    if (heartbeat_out != nullptr) {
        *heartbeat_out = {};
        if (health_.should_emit_heartbeat(now_ns)) {
            *heartbeat_out = health_.make_heartbeat(now_ns);
        }
    }
}
