#ifndef UAV_PROC_NPU_PIPELINE_H
#define UAV_PROC_NPU_PIPELINE_H

#include <cstdint>
#include <vector>

#include "abi/msg_types.h"
#include "health.h"
#include "npu_infer.h"
#include "postprocess.h"
#include "preprocess.h"
#include "result_publisher.h"
#include "shm_reader.h"

class NpuPipeline {
public:
    NpuPipeline(NpuInfer *infer, ResultPublisher *publisher);

    bool process_frame(const InferenceFrame &frame,
                       float threshold,
                       uint64_t now_ns,
                       UavCHeartbeat *heartbeat_out);

private:
    NpuInfer *infer_;
    ResultPublisher *publisher_;
    CHealth health_;
    Preprocess preprocess_;
    Postprocess postprocess_;

    // Temporal smoothing for the published result. The COCO-pretrained
    // mavic3_drone.rknn produces class=4 hits right at the score gate
    // (0.45–0.52), so single-frame drop-outs make the HostGUI bbox
    // flicker. Hold the last non-empty result for kStickyHoldNs after
    // detection drops, and EMA-smooth bbox between overlapping
    // consecutive detections to kill jitter.
    UavCResult sticky_{};
    bool       sticky_valid_{false};
    uint64_t   sticky_until_ns_{0};
    static constexpr uint64_t kStickyHoldNs = 600ULL * 1000ULL * 1000ULL;
    // 0.3 ⇒ 70 % previous, 30 % current — bbox keeps up with motion
    // but doesn't snap on every frame. Tested with the COCO-pretrained
    // mavic3_drone.rknn whose raw bbox jumps ~150 px between frames.
    static constexpr float    kEmaAlpha     = 0.3F;
    static constexpr float    kIouSnap      = 0.20F;
};

#endif
