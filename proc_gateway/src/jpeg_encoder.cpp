#include "jpeg_encoder.h"

#include <csetjmp>
#include <cstdio>
#include <cstdlib>
#include <vector>

#include <jpeglib.h>

namespace {

struct ErrMgr {
    jpeg_error_mgr pub;
    jmp_buf        jbuf;
};

void on_error(j_common_ptr c) {
    longjmp(reinterpret_cast<ErrMgr *>(c->err)->jbuf, 1);
}

} // namespace

std::vector<uint8_t> JpegEncoder::encode(const uint8_t *bgr,
                                          uint32_t       w,
                                          uint32_t       h,
                                          uint32_t       stride,
                                          int            quality) {
    jpeg_compress_struct c{};
    ErrMgr               err{};

    c.err             = jpeg_std_error(&err.pub);
    err.pub.error_exit = on_error;

    if (setjmp(err.jbuf)) {
        jpeg_destroy_compress(&c);
        return {};
    }

    jpeg_create_compress(&c);

    unsigned char *outbuf = nullptr;
    unsigned long  outsz  = 0;
    jpeg_mem_dest(&c, &outbuf, &outsz);

    c.image_width      = w;
    c.image_height     = h;
    c.input_components = 3;
    c.in_color_space   = JCS_RGB;

    jpeg_set_defaults(&c);
    jpeg_set_quality(&c, quality, TRUE);
    jpeg_start_compress(&c, TRUE);

    // Convert BGR → RGB row by row (standard libjpeg expects RGB)
    std::vector<uint8_t> row_buf(w * 3);
    while (c.next_scanline < c.image_height) {
        const uint8_t *src = bgr + c.next_scanline * stride;
        for (uint32_t x = 0; x < w; ++x) {
            row_buf[x * 3 + 0] = src[x * 3 + 2];  // R ← B channel
            row_buf[x * 3 + 1] = src[x * 3 + 1];  // G
            row_buf[x * 3 + 2] = src[x * 3 + 0];  // B ← R channel
        }
        JSAMPROW row = row_buf.data();
        jpeg_write_scanlines(&c, &row, 1);
    }

    jpeg_finish_compress(&c);
    jpeg_destroy_compress(&c);

    std::vector<uint8_t> result(outbuf, outbuf + outsz);
    free(outbuf);
    return result;
}
