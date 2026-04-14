/*
 * dev_stub.c - Stub implementations for dev.h functions that proc_arm
 *              does not own.
 *
 * gripper  → future proc_gripper process
 * relay    → future proc_airport process
 * car      → proc_gateway (already separate)
 *
 * These stubs satisfy the linker. The arm_controller.cpp emergency_stop
 * path calls dev_emergency_stop_all() which only needs to stop the arm
 * motors — that is handled by arm_stop() in arm_can.c.
 */

#include "dev.h"

extern "C" {
#include "log.h"
}

bool platform_lock(bool lock) {
    (void)lock;
    return true;
}

bool car_set_velocity(int linear_mm_s, int angular_mdeg_s) {
    (void)linear_mm_s;
    (void)angular_mdeg_s;
    return true;
}

bool gripper_open(bool open) {
    /* TODO: forward to proc_gripper via RPC when it exists */
    log_warn("dev.stub", "gripper_open(%s) stub — no proc_gripper",
             open ? "true" : "false");
    return true;
}

void dev_emergency_stop_all(void) {
    log_warn("dev.stub", "emergency stop: stopping arm motors");
    arm_stop();
}
