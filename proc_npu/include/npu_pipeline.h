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

    // PRESENT-state cache. Updated every frame while present_=true so
    // the bbox shown in HostGUI follows the drone. EMA-blended against
    // the incoming detection to kill the ~150 px frame-to-frame jitter
    // of the COCO-pretrained model.
    UavCResult sticky_{};
    bool       sticky_valid_{false};
    // 0.3 ⇒ 70 % previous, 30 % current — bbox keeps up with motion
    // but doesn't snap on every frame.
    static constexpr float    kEmaAlpha     = 0.3F;
    static constexpr float    kIouSnap      = 0.20F;

    // Schmitt-trigger PRESENT/ABSENT machine. The COCO-pretrained
    // mavic3_drone.rknn produces ~0.50 score "airplane" phantoms on the
    // empty workbench every few frames, and conversely drops the real
    // drone for one frame here and there. To get a clean binary "drone
    // on platform yes/no", we use asymmetric hysteresis:
    //
    //   ABSENT → PRESENT: need kEnterFrames consecutive matching hits
    //                     (same class, IoU ≥ kIouConfirm between frames)
    //   PRESENT → ABSENT: need kExitFrames consecutive empty frames
    //
    // Single-frame phantoms never reach PRESENT; single-frame drops
    // never lose PRESENT. The bbox shown during PRESENT comes from
    // sticky_ (EMA-blended). Both counters reset on contradicting
    // evidence (an empty frame mid-enter, or a hit mid-exit).
    UavCResult pending_{};
    bool       pending_valid_{false};
    int        enter_count_{0};   // consecutive matching drone frames
    int        exit_count_{0};    // consecutive frames with no drone hit
    bool       present_{false};   // drone state — fires C_PRESENCE on edge
    bool       platform_visible_{false};  // plate visibility, edge-triggered too
    // Enter is harder than exit on purpose — the user's default scene is
    // "empty platform" and they want NO false PRESENT. 5 frames at 5 fps
    // ≈ 1 s of continuous confirmed hits before flipping. Real drone hits
    // every frame; sporadic COCO phantoms can't sustain a 5-in-a-row run.
    static constexpr int kEnterFrames = 5;
    static constexpr int kExitFrames  = 5;
};

#endif
