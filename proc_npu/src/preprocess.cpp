#include "preprocess.h"

bool Preprocess::run(const InferenceFrame &frame, TensorInput &input) const {
    input.width = frame.slot.width;
    input.height = frame.slot.height;
    const uint32_t pixels = frame.slot.width * frame.slot.height;
    input.data.assign(static_cast<size_t>(pixels) * 3U, 0.0F);

    if (frame.slot.color_offset >= frame.payload.size()) {
        return true;
    }
    const uint8_t *color = frame.payload.data() + frame.slot.color_offset;
    const size_t max_bytes = frame.payload.size() - frame.slot.color_offset;
    const size_t expect = static_cast<size_t>(pixels) * 3U;
    const size_t n = max_bytes < expect ? max_bytes : expect;
    for (size_t i = 0; i < n; ++i) {
        input.data[i] = static_cast<float>(color[i]) / 255.0F;
    }
    return true;
}
