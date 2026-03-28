#include "npu_pipeline.h"

#include <chrono>
#include <cstdio>

namespace {
void print_result(const UavCResult &result) {
    std::printf("C_RESULT frame=%llu det=%u\n",
                static_cast<unsigned long long>(result.frame_id),
                result.num_detections);
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

    const UavCResult result = postprocess_.run(frame, raw, threshold);
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
