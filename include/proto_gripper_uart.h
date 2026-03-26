#ifndef UAV_PROTO_GRIPPER_UART_H
#define UAV_PROTO_GRIPPER_UART_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

bool proto_gripper_uart_encode_open(bool open,
                                    uint8_t *out_buf,
                                    size_t out_buf_size,
                                    size_t *out_len);

#ifdef __cplusplus
}
#endif

#endif
