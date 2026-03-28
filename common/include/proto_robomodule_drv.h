#ifndef UAV_PROTO_ROBOMODULE_DRV_H
#define UAV_PROTO_ROBOMODULE_DRV_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct {
    uint16_t can_id;
    uint8_t data[8];
} RoboModuleDrvFrame;

enum {
    ROBOMODULE_DRV_MODE_VELOCITY_CLOSED_LOOP = 0x03
};

bool proto_robomodule_drv_encode_reset(uint8_t group,
                                       uint8_t motor_num,
                                       RoboModuleDrvFrame *out_frame);
bool proto_robomodule_drv_encode_mode(uint8_t group,
                                      uint8_t motor_num,
                                      uint8_t mode,
                                      RoboModuleDrvFrame *out_frame);
bool proto_robomodule_drv_encode_velocity(uint8_t group,
                                          uint8_t motor_num,
                                          int limit_pwm,
                                          int target_rpm,
                                          RoboModuleDrvFrame *out_frame);
int proto_robomodule_drv_decode(void);

#endif
