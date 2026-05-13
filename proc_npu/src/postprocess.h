#ifndef UAV_PROC_NPU_POSTPROCESS_H
#define UAV_PROC_NPU_POSTPROCESS_H

#include <vector>

#include "abi/msg_types.h"
#include "npu_infer.h"
#include "shm_reader.h"

// ── Class-ID sentinels that ride the same UavDetection stream ────────────
// Real detections from RKNN use the model's class IDs (COCO 0..79). We add
// two synthetic values for application-level signals that downstream
// consumers (npu_pipeline state machine, HostGUI overlay) can recognize:
//
//   UAV_CLASS_DRONE     = 4   — COCO "airplane", the real drone hits.
//   UAV_CLASS_PLATFORM  = 900 — workspace plate marker. Emitted whenever
//                                the white work plate is visible in
//                                frame, so the GUI can distinguish
//                                "camera sees an empty platform" from
//                                "camera can't see anything".
//   UAV_CLASS_BATTERY   = 200 — already in use by battery_tracker.py.
//
// Anything class_id >= UAV_CLASS_META_BASE is metadata, not a target.
static constexpr int32_t UAV_CLASS_DRONE     = 4;
static constexpr int32_t UAV_CLASS_BATTERY   = 200;
static constexpr int32_t UAV_CLASS_META_BASE = 900;
static constexpr int32_t UAV_CLASS_PLATFORM  = 900;

class Postprocess {
public:
    UavCResult run(const InferenceFrame &frame, const std::vector<RawDet> &raw, float threshold) const;
};

#endif
