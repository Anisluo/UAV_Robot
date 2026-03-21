#ifndef UAV_PROC_GATEWAY_JPEG_ENCODER_H
#define UAV_PROC_GATEWAY_JPEG_ENCODER_H

#include <cstdint>
#include <vector>

class JpegEncoder {
public:
    // Encode BGR8 image to JPEG.  Returns empty vector on failure.
    std::vector<uint8_t> encode(const uint8_t *bgr,
                                uint32_t width,
                                uint32_t height,
                                uint32_t row_stride,
                                int      quality = 80);
};

#endif
