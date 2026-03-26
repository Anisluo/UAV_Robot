#ifndef UAV_PROTO_PLATFORM_LOCK_H
#define UAV_PROTO_PLATFORM_LOCK_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define PLATFORM_LOCK_CAN_MAX_FRAMES 2U

typedef struct {
    uint32_t can_id;
    uint8_t data[8];
    uint8_t len;
    bool is_extended_id;
} PlatformLockCanFrame;

typedef struct {
    PlatformLockCanFrame frames[PLATFORM_LOCK_CAN_MAX_FRAMES];
    size_t count;
} PlatformLockCanBatch;

typedef enum {
    PLATFORM_LOCK_RESP_UNKNOWN = 0,
    PLATFORM_LOCK_RESP_ACK_OK,
    PLATFORM_LOCK_RESP_ACK_REJECTED,
    PLATFORM_LOCK_RESP_ACK_INVALID,
    PLATFORM_LOCK_RESP_POSITION_REACHED
} PlatformLockResponseType;

typedef struct {
    uint8_t addr;
    uint8_t cmd;
    uint8_t status;
    PlatformLockResponseType type;
} PlatformLockResponse;

bool proto_platform_lock_encode_enable(uint8_t addr,
                                       bool enable,
                                       bool sync,
                                       PlatformLockCanBatch *out_batch);
bool proto_platform_lock_encode_position(uint8_t addr,
                                         bool ccw,
                                         uint16_t rpm,
                                         uint8_t acc,
                                         uint32_t pulses,
                                         bool absolute_mode,
                                         bool sync,
                                         PlatformLockCanBatch *out_batch);
bool proto_platform_lock_encode_stop(uint8_t addr,
                                     bool sync,
                                     PlatformLockCanBatch *out_batch);
bool proto_platform_lock_decode(const uint8_t *payload,
                                size_t len,
                                PlatformLockResponse *out_resp);

#endif
