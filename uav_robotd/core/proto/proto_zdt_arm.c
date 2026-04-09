#include "proto_zdt_arm.h"

#include <stddef.h>
#include <string.h>

#include "log.h"

#define ZDT_ARM_CHECKSUM 0x6BU
#define ZDT_ARM_CMD_ENABLE 0xF3U
#define ZDT_ARM_CMD_SPEED 0xF6U
#define ZDT_ARM_CMD_POSITION 0xFDU
#define ZDT_ARM_CMD_STOP 0xFEU
#define ZDT_ARM_CMD_SYNC 0xFFU
#define ZDT_ARM_CMD_TRIGGER_HOME 0x9AU
#define ZDT_ARM_STATUS_OK 0x02U
#define ZDT_ARM_STATUS_REJECTED 0xE2U
#define ZDT_ARM_STATUS_INVALID 0xEEU
#define ZDT_ARM_STATUS_REACHED 0x9FU

static bool zdt_arm_validate_addr(uint8_t addr) {
    (void)addr;
    return true;
}

static bool zdt_arm_build_batch(uint8_t addr,
                                const uint8_t *payload,
                                size_t payload_len,
                                ZdtArmCanBatch *out_batch) {
    size_t frame_index = 0;
    size_t offset = 0;

    if (payload == NULL || out_batch == NULL) {
        return false;
    }
    if (!zdt_arm_validate_addr(addr)) {
        return false;
    }
    if (payload_len == 0U) {
        return false;
    }

    memset(out_batch, 0, sizeof(*out_batch));
    if (payload_len <= 8U) {
        out_batch->frames[0].can_id = ((uint32_t)addr << 8);
        out_batch->frames[0].len = (uint8_t)payload_len;
        out_batch->frames[0].is_extended_id = true;
        memcpy(out_batch->frames[0].data, payload, payload_len);
        out_batch->count = 1U;
        return true;
    }

    /* For CAN packets longer than 8 bytes, repeat the function byte on every frame.
     * Example from the Emm V5.0 manual:
     *   FD xx xx xx xx xx xx xx
     *   FD xx xx xx xx
     */
    offset = 1U;
    while (offset < payload_len) {
        size_t chunk_len = payload_len - offset;
        if (frame_index >= ZDT_ARM_CAN_MAX_FRAMES) {
            log_error("core.proto.zdt_arm", "payload too large: %u", (unsigned int)payload_len);
            return false;
        }
        if (chunk_len > 7U) {
            chunk_len = 7U;
        }

        out_batch->frames[frame_index].can_id = ((uint32_t)addr << 8) | (uint32_t)frame_index;
        out_batch->frames[frame_index].len = (uint8_t)(chunk_len + 1U);
        out_batch->frames[frame_index].is_extended_id = true;
        out_batch->frames[frame_index].data[0] = payload[0];
        memcpy(out_batch->frames[frame_index].data + 1, payload + offset, chunk_len);
        ++frame_index;
        offset += chunk_len;
    }

    out_batch->count = frame_index;
    return true;
}

bool proto_zdt_arm_encode_enable(uint8_t addr,
                                 bool enable,
                                 bool sync,
                                 ZdtArmCanBatch *out_batch) {
    const uint8_t payload[] = {
        ZDT_ARM_CMD_ENABLE, 0xABU, enable ? 0x01U : 0x00U, sync ? 0x01U : 0x00U, ZDT_ARM_CHECKSUM
    };
    return zdt_arm_build_batch(addr, payload, sizeof(payload), out_batch);
}

bool proto_zdt_arm_encode_speed(uint8_t addr,
                                bool ccw,
                                uint16_t rpm,
                                uint8_t acc,
                                bool sync,
                                ZdtArmCanBatch *out_batch) {
    const uint8_t payload[] = {
        ZDT_ARM_CMD_SPEED,
        ccw ? 0x01U : 0x00U,
        (uint8_t)((rpm >> 8) & 0xFFU),
        (uint8_t)(rpm & 0xFFU),
        acc,
        sync ? 0x01U : 0x00U,
        ZDT_ARM_CHECKSUM
    };
    return zdt_arm_build_batch(addr, payload, sizeof(payload), out_batch);
}

bool proto_zdt_arm_encode_position(uint8_t addr,
                                   bool ccw,
                                   uint16_t rpm,
                                   uint8_t acc,
                                   uint32_t pulses,
                                   bool absolute_mode,
                                   bool sync,
                                   ZdtArmCanBatch *out_batch) {
    const uint8_t payload[] = {
        ZDT_ARM_CMD_POSITION,
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
        ZDT_ARM_CHECKSUM
    };
    return zdt_arm_build_batch(addr, payload, sizeof(payload), out_batch);
}

bool proto_zdt_arm_encode_stop(uint8_t addr,
                               bool sync,
                               ZdtArmCanBatch *out_batch) {
    const uint8_t payload[] = {
        ZDT_ARM_CMD_STOP, 0x98U, sync ? 0x01U : 0x00U, ZDT_ARM_CHECKSUM
    };
    return zdt_arm_build_batch(addr, payload, sizeof(payload), out_batch);
}

bool proto_zdt_arm_encode_sync_start(ZdtArmCanBatch *out_batch) {
    const uint8_t payload[] = {ZDT_ARM_CMD_SYNC, 0x66U, ZDT_ARM_CHECKSUM};
    return zdt_arm_build_batch(0U, payload, sizeof(payload), out_batch);
}

bool proto_zdt_arm_encode_trigger_home(uint8_t addr,
                                       uint8_t home_mode,
                                       bool sync,
                                       ZdtArmCanBatch *out_batch) {
    const uint8_t payload[] = {
        ZDT_ARM_CMD_TRIGGER_HOME, home_mode, sync ? 0x01U : 0x00U, ZDT_ARM_CHECKSUM
    };
    return zdt_arm_build_batch(addr, payload, sizeof(payload), out_batch);
}

bool proto_zdt_arm_encode_write_zero_params(uint8_t addr,
                                            bool save,
                                            const ZdtArmZeroParams *params,
                                            ZdtArmCanBatch *out_batch) {
    const uint8_t payload[] = {
        0x4CU,
        0xAEU,
        save ? 0x01U : 0x00U,
        params != NULL ? params->zero_mode : 0x00U,
        params != NULL ? params->zero_direction : 0x00U,
        params != NULL ? (uint8_t)((params->zero_speed_rpm >> 8) & 0xFFU) : 0x00U,
        params != NULL ? (uint8_t)(params->zero_speed_rpm & 0xFFU) : 0x00U,
        params != NULL ? (uint8_t)((params->zero_timeout_ms >> 24) & 0xFFU) : 0x00U,
        params != NULL ? (uint8_t)((params->zero_timeout_ms >> 16) & 0xFFU) : 0x00U,
        params != NULL ? (uint8_t)((params->zero_timeout_ms >> 8) & 0xFFU) : 0x00U,
        params != NULL ? (uint8_t)(params->zero_timeout_ms & 0xFFU) : 0x00U,
        params != NULL ? (uint8_t)((params->collision_zero_speed_rpm >> 8) & 0xFFU) : 0x00U,
        params != NULL ? (uint8_t)(params->collision_zero_speed_rpm & 0xFFU) : 0x00U,
        params != NULL ? (uint8_t)((params->collision_zero_current_ma >> 8) & 0xFFU) : 0x00U,
        params != NULL ? (uint8_t)(params->collision_zero_current_ma & 0xFFU) : 0x00U,
        params != NULL ? (uint8_t)((params->collision_zero_time_ms >> 8) & 0xFFU) : 0x00U,
        params != NULL ? (uint8_t)(params->collision_zero_time_ms & 0xFFU) : 0x00U,
        (params != NULL && params->enable_auto_zero) ? 0x01U : 0x00U,
        ZDT_ARM_CHECKSUM
    };

    if (params == NULL) {
        return false;
    }
    return zdt_arm_build_batch(addr, payload, sizeof(payload), out_batch);
}

bool proto_zdt_arm_decode(const uint8_t *payload,
                          size_t len,
                          ZdtArmResponse *out_resp) {
    if (payload == NULL || out_resp == NULL || len < 3U) {
        return false;
    }
    if (payload[len - 1] != ZDT_ARM_CHECKSUM) {
        return false;
    }

    memset(out_resp, 0, sizeof(*out_resp));
    if (len >= 4U) {
        out_resp->addr = payload[0];
        out_resp->cmd = payload[1];
        out_resp->status = payload[2];
    } else {
        out_resp->addr = 0U;
        out_resp->cmd = payload[0];
        out_resp->status = payload[1];
    }
    out_resp->type = ZDT_ARM_RESP_UNKNOWN;

    if (out_resp->cmd == ZDT_ARM_CMD_POSITION && out_resp->status == ZDT_ARM_STATUS_REACHED) {
        out_resp->type = ZDT_ARM_RESP_POSITION_REACHED;
        return true;
    }
    if (out_resp->status == ZDT_ARM_STATUS_OK) {
        out_resp->type = ZDT_ARM_RESP_ACK_OK;
        return true;
    }
    if (out_resp->status == ZDT_ARM_STATUS_REJECTED) {
        out_resp->type = ZDT_ARM_RESP_ACK_REJECTED;
        return true;
    }
    if (out_resp->status == ZDT_ARM_STATUS_INVALID || out_resp->cmd == 0x00U) {
        out_resp->type = ZDT_ARM_RESP_ACK_INVALID;
        return true;
    }

    return true;
}
