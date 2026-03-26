#ifndef UAV_PROTO_ZDT_ARM_H
#define UAV_PROTO_ZDT_ARM_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define ZDT_ARM_CAN_MAX_FRAMES 2U

typedef struct {
    uint32_t can_id;
    uint8_t data[8];
    uint8_t len;
    bool is_extended_id;
} ZdtArmCanFrame;

typedef struct {
    ZdtArmCanFrame frames[ZDT_ARM_CAN_MAX_FRAMES];
    size_t count;
} ZdtArmCanBatch;

typedef enum {
    ZDT_ARM_RESP_UNKNOWN = 0,
    ZDT_ARM_RESP_ACK_OK,
    ZDT_ARM_RESP_ACK_REJECTED,
    ZDT_ARM_RESP_ACK_INVALID,
    ZDT_ARM_RESP_POSITION_REACHED
} ZdtArmResponseType;

typedef struct {
    uint8_t addr;
    uint8_t cmd;
    uint8_t status;
    ZdtArmResponseType type;
} ZdtArmResponse;

bool proto_zdt_arm_encode_enable(uint8_t addr,
                                 bool enable,
                                 bool sync,
                                 ZdtArmCanBatch *out_batch);
bool proto_zdt_arm_encode_speed(uint8_t addr,
                                bool ccw,
                                uint16_t rpm,
                                uint8_t acc,
                                bool sync,
                                ZdtArmCanBatch *out_batch);
bool proto_zdt_arm_encode_position(uint8_t addr,
                                   bool ccw,
                                   uint16_t rpm,
                                   uint8_t acc,
                                   uint32_t pulses,
                                   bool absolute_mode,
                                   bool sync,
                                   ZdtArmCanBatch *out_batch);
bool proto_zdt_arm_encode_stop(uint8_t addr,
                               bool sync,
                               ZdtArmCanBatch *out_batch);
bool proto_zdt_arm_encode_sync_start(ZdtArmCanBatch *out_batch);
bool proto_zdt_arm_encode_trigger_home(uint8_t addr,
                                       uint8_t home_mode,
                                       bool sync,
                                       ZdtArmCanBatch *out_batch);
bool proto_zdt_arm_decode(const uint8_t *payload,
                          size_t len,
                          ZdtArmResponse *out_resp);

#endif
