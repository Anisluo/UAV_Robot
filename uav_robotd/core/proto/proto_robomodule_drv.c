#include "proto_robomodule_drv.h"

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "log.h"

#define ROBOMODULE_DRV_CMD_RESET 0x0U
#define ROBOMODULE_DRV_CMD_MODE 0x1U
#define ROBOMODULE_DRV_CMD_VELOCITY 0x4U
#define ROBOMODULE_DRV_FILL_BYTE 0x55U
#define ROBOMODULE_DRV_MAX_GROUP 7U
#define ROBOMODULE_DRV_MAX_MOTOR_NUM 15U
#define ROBOMODULE_DRV_MAX_LIMIT_PWM 5000
#define ROBOMODULE_DRV_MAX_RPM 4000

static bool proto_robomodule_drv_validate(uint8_t group,
                                          uint8_t motor_num,
                                          RoboModuleDrvFrame *out_frame) {
    if (out_frame == NULL) {
        log_error("core.proto.robomodule", "null out_frame");
        return false;
    }
    if (group > ROBOMODULE_DRV_MAX_GROUP) {
        log_error("core.proto.robomodule", "invalid group=%u", (unsigned int)group);
        return false;
    }
    if (motor_num > ROBOMODULE_DRV_MAX_MOTOR_NUM) {
        log_error("core.proto.robomodule",
                  "invalid motor_num=%u",
                  (unsigned int)motor_num);
        return false;
    }
    return true;
}

static uint16_t proto_robomodule_drv_make_can_id(uint8_t group,
                                                 uint8_t motor_num,
                                                 uint8_t cmd) {
    return (uint16_t)(((group & 0x7U) << 8) |
                      ((motor_num & 0xFU) << 4) |
                      (cmd & 0xFU));
}

static void proto_robomodule_drv_fill_payload(RoboModuleDrvFrame *frame,
                                              uint8_t cmd) {
    frame->can_id = (uint16_t)((frame->can_id & 0x7F0U) | (cmd & 0xFU));
    memset(frame->data, ROBOMODULE_DRV_FILL_BYTE, sizeof(frame->data));
}

static int proto_robomodule_drv_clamp_int(int value, int min_value, int max_value) {
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

bool proto_robomodule_drv_encode_reset(uint8_t group,
                                       uint8_t motor_num,
                                       RoboModuleDrvFrame *out_frame) {
    if (!proto_robomodule_drv_validate(group, motor_num, out_frame)) {
        return false;
    }

    out_frame->can_id = proto_robomodule_drv_make_can_id(group,
                                                         motor_num,
                                                         ROBOMODULE_DRV_CMD_RESET);
    proto_robomodule_drv_fill_payload(out_frame, ROBOMODULE_DRV_CMD_RESET);
    return true;
}

bool proto_robomodule_drv_encode_mode(uint8_t group,
                                      uint8_t motor_num,
                                      uint8_t mode,
                                      RoboModuleDrvFrame *out_frame) {
    if (!proto_robomodule_drv_validate(group, motor_num, out_frame)) {
        return false;
    }

    out_frame->can_id = proto_robomodule_drv_make_can_id(group,
                                                         motor_num,
                                                         ROBOMODULE_DRV_CMD_MODE);
    proto_robomodule_drv_fill_payload(out_frame, ROBOMODULE_DRV_CMD_MODE);
    out_frame->data[0] = mode;
    return true;
}

bool proto_robomodule_drv_encode_velocity(uint8_t group,
                                          uint8_t motor_num,
                                          int limit_pwm,
                                          int target_rpm,
                                          RoboModuleDrvFrame *out_frame) {
    uint16_t pwm;
    int16_t rpm;

    if (!proto_robomodule_drv_validate(group, motor_num, out_frame)) {
        return false;
    }

    pwm = (uint16_t)proto_robomodule_drv_clamp_int(limit_pwm,
                                                   0,
                                                   ROBOMODULE_DRV_MAX_LIMIT_PWM);
    rpm = (int16_t)proto_robomodule_drv_clamp_int(target_rpm,
                                                  -ROBOMODULE_DRV_MAX_RPM,
                                                  ROBOMODULE_DRV_MAX_RPM);

    out_frame->can_id = proto_robomodule_drv_make_can_id(group,
                                                         motor_num,
                                                         ROBOMODULE_DRV_CMD_VELOCITY);
    proto_robomodule_drv_fill_payload(out_frame, ROBOMODULE_DRV_CMD_VELOCITY);
    out_frame->data[0] = (uint8_t)((pwm >> 8) & 0xFFU);
    out_frame->data[1] = (uint8_t)(pwm & 0xFFU);
    out_frame->data[2] = (uint8_t)(((uint16_t)rpm >> 8) & 0xFFU);
    out_frame->data[3] = (uint8_t)((uint16_t)rpm & 0xFFU);
    return true;
}

int proto_robomodule_drv_decode(void) {
    log_info("core.proto.robomodule", "decode stub");
    return 0;
}
