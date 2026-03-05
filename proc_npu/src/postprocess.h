#ifndef UAV_PROC_NPU_POSTPROCESS_H
#define UAV_PROC_NPU_POSTPROCESS_H

#include <vector>

#include "abi/msg_types.h"
#include "npu_infer.h"
#include "shm_reader.h"

class Postprocess {
public:
    UavCResult run(const InferenceFrame &frame, const std::vector<RawDet> &raw, float threshold) const;
};

#endif
