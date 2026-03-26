#ifndef UAV_CAN_SOCKETCAN_H
#define UAV_CAN_SOCKETCAN_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct {
    uint32_t can_id;
    uint8_t data[8];
    size_t len;
    bool is_extended_id;
} CanSocketFrame;

int can_socketcan_init(void);
void can_socketcan_close(void);
bool can_socketcan_send(uint32_t can_id,
                        bool is_extended_id,
                        const uint8_t *data,
                        size_t len);
bool can_socketcan_receive(CanSocketFrame *out_frame);
bool can_socketcan_send_std(uint16_t can_id, const uint8_t data[8]);
bool can_socketcan_send_ext(uint32_t can_id, const uint8_t *data, size_t len);

#endif
