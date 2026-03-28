#include "proto_gripper_uart.h"

#include <string.h>

bool proto_gripper_uart_encode_open(bool open,
                                    uint8_t *out_buf,
                                    size_t out_buf_size,
                                    size_t *out_len) {
    static const uint8_t k_open_cmd[] = {
        0x55, 0x55, 0x01, 0x07, 0x01, 0x90, 0x01, 0xE8, 0x03, 0x7A
    };
    static const uint8_t k_close_cmd[] = {
        0x55, 0x55, 0x01, 0x07, 0x01, 0x84, 0x03, 0xE8, 0x03, 0x84
    };
    const uint8_t *src = open ? k_open_cmd : k_close_cmd;
    const size_t len = open ? sizeof(k_open_cmd) : sizeof(k_close_cmd);

    if (out_len == NULL) {
        return false;
    }
    *out_len = 0U;

    if (out_buf == NULL || out_buf_size < len) {
        return false;
    }

    memcpy(out_buf, src, len);
    *out_len = len;
    return true;
}
