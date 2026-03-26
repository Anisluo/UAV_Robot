#include "proto_platform_lock.h"

#include <stddef.h>
#include <string.h>

#include "log.h"

#define PLATFORM_LOCK_CHECKSUM 0x6BU
#define PLATFORM_LOCK_CMD_ENABLE 0xF3U
#define PLATFORM_LOCK_CMD_POSITION 0xFDU
#define PLATFORM_LOCK_CMD_STOP 0xFEU
#define PLATFORM_LOCK_STATUS_OK 0x02U
#define PLATFORM_LOCK_STATUS_REJECTED 0xE2U
#define PLATFORM_LOCK_STATUS_INVALID 0xEEU
#define PLATFORM_LOCK_STATUS_REACHED 0x9FU

static bool platform_lock_build_batch(uint8_t addr,
                                      const uint8_t *payload,
                                      size_t payload_len,
                                      PlatformLockCanBatch *out_batch) {
    size_t offset = 0;
    size_t frame_index = 0;

    if (payload == NULL || out_batch == NULL) {
        return false;
    }

    memset(out_batch, 0, sizeof(*out_batch));
    while (offset < payload_len) {
        size_t chunk_len = payload_len - offset;

        if (frame_index >= PLATFORM_LOCK_CAN_MAX_FRAMES) {
            log_error("core.proto.platform", "payload too large: %u", (unsigned int)payload_len);
            return false;
        }
        if (chunk_len > 8U) {
            chunk_len = 8U;
        }

        out_batch->frames[frame_index].can_id = ((uint32_t)addr << 8) | (uint32_t)frame_index;
        out_batch->frames[frame_index].len = (uint8_t)chunk_len;
        out_batch->frames[frame_index].is_extended_id = true;
        memcpy(out_batch->frames[frame_index].data, payload + offset, chunk_len);
        ++frame_index;
        offset += chunk_len;
    }

    out_batch->count = frame_index;
    return true;
}

bool proto_platform_lock_encode_enable(uint8_t addr,
                                       bool enable,
                                       bool sync,
                                       PlatformLockCanBatch *out_batch) {
    const uint8_t payload[] = {
        addr, PLATFORM_LOCK_CMD_ENABLE, 0xABU, enable ? 0x01U : 0x00U, sync ? 0x01U : 0x00U, PLATFORM_LOCK_CHECKSUM
    };
    return platform_lock_build_batch(addr, payload, sizeof(payload), out_batch);
}

bool proto_platform_lock_encode_position(uint8_t addr,
                                         bool ccw,
                                         uint16_t rpm,
                                         uint8_t acc,
                                         uint32_t pulses,
                                         bool absolute_mode,
                                         bool sync,
                                         PlatformLockCanBatch *out_batch) {
    const uint8_t payload[] = {
        addr,
        PLATFORM_LOCK_CMD_POSITION,
        ccw ? 0x01U : 0x00U,
        (uint8_t)((rpm >> 8) & 0xFFU),
        (uint8_t)(rpm & 0xFFU),
        acc,
        (uint8_t)((pulses >> 24) & 0xFFU),
        (uint8_t)((pulses >> 16) & 0xFFU),
        (uint8_t)((pulses >> 8) & 0xFFU),
        (uint8_t)(pulses & 0xFFU),
        absolute_mode ? 0x01U : 0x00U,
        sync ? 0x01U : 0x00U,
        PLATFORM_LOCK_CHECKSUM
    };
    return platform_lock_build_batch(addr, payload, sizeof(payload), out_batch);
}

bool proto_platform_lock_encode_stop(uint8_t addr,
                                     bool sync,
                                     PlatformLockCanBatch *out_batch) {
    const uint8_t payload[] = {
        addr, PLATFORM_LOCK_CMD_STOP, 0x98U, sync ? 0x01U : 0x00U, PLATFORM_LOCK_CHECKSUM
    };
    return platform_lock_build_batch(addr, payload, sizeof(payload), out_batch);
}

bool proto_platform_lock_decode(const uint8_t *payload,
                                size_t len,
                                PlatformLockResponse *out_resp) {
    if (payload == NULL || out_resp == NULL || len < 4U) {
        return false;
    }
    if (payload[len - 1] != PLATFORM_LOCK_CHECKSUM) {
        return false;
    }

    memset(out_resp, 0, sizeof(*out_resp));
    out_resp->addr = payload[0];
    out_resp->cmd = payload[1];
    out_resp->status = payload[2];
    out_resp->type = PLATFORM_LOCK_RESP_UNKNOWN;

    if (payload[1] == PLATFORM_LOCK_CMD_POSITION && payload[2] == PLATFORM_LOCK_STATUS_REACHED && len == 4U) {
        out_resp->type = PLATFORM_LOCK_RESP_POSITION_REACHED;
        return true;
    }
    if (payload[2] == PLATFORM_LOCK_STATUS_OK) {
        out_resp->type = PLATFORM_LOCK_RESP_ACK_OK;
        return true;
    }
    if (payload[2] == PLATFORM_LOCK_STATUS_REJECTED) {
        out_resp->type = PLATFORM_LOCK_RESP_ACK_REJECTED;
        return true;
    }
    if (payload[2] == PLATFORM_LOCK_STATUS_INVALID || payload[1] == 0x00U) {
        out_resp->type = PLATFORM_LOCK_RESP_ACK_INVALID;
        return true;
    }

    return true;
}
