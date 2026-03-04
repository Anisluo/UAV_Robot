#include <stdio.h>
#include "dev.h"
#include "log.h"

bool arm_move(const char *pose) {
    log_info("drv.dev.arm", "arm_move(%s)", pose);
    return true;
}

bool platform_lock(bool lock) {
    log_info("drv.dev.platform", "platform_lock(%s)", lock ? "true" : "false");
    return true;
}

bool car_set_velocity(int linear_mm_s, int angular_mdeg_s) {
    log_info("drv.dev.car", "car_set_velocity(linear=%d, angular=%d)", linear_mm_s, angular_mdeg_s);
    return true;
}

bool gripper_open(bool open) {
    log_info("drv.dev.gripper", "gripper_open(%s)", open ? "true" : "false");
    return true;
}

void dev_emergency_stop_all(void) {
    log_error("drv.dev", "EMERGENCY STOP: stop arm/car and force safe GPIO state");
}
