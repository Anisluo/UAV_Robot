#ifndef UAV_PROC_NPU_PREPROCESS_H
#define UAV_PROC_NPU_PREPROCESS_H

#include <cstdint>
#include <vector>

#include "shm_reader.h"

// Letterbox-resized tensor ready for RKNN input (uint8 RGB HWC, 640×640).
// Letterbox parameters are stored so postprocess can map back to image coords.
struct TensorInput {
    uint32_t width;          // model input width  (640)
    uint32_t height;         // model input height (640)
    uint32_t orig_width;     // source frame width
    uint32_t orig_height;    // source frame height
    float    lb_scale;       // scale:  src_px * lb_scale = model_px
    float    lb_pad_x;       // left padding in model pixels
    float    lb_pad_y;       // top  padding in model pixels
    // HWC uint8 RGB, size = width * height * 3
    std::vector<uint8_t> data;
};

class Preprocess {
public:
    bool run(const InferenceFrame &frame, TensorInput &input) const;
};

#endif // UAV_PROC_NPU_PREPROCESS_H
