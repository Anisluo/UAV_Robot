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
};

#endif
