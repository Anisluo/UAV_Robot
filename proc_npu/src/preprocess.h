#ifndef UAV_PROC_NPU_PREPROCESS_H
#define UAV_PROC_NPU_PREPROCESS_H

#include <cstdint>
#include <vector>

#include "shm_reader.h"

struct TensorInput {
    uint32_t width;
    uint32_t height;
    std::vector<float> data;
};

class Preprocess {
public:
    bool run(const InferenceFrame &frame, TensorInput &input) const;
};

#endif
