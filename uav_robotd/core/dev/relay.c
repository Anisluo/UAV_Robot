#include "dev.h"
#include "log.h"

/* Called by both proc_arm (ArmController::emergencyStop) and uav_robotd
 * (scheduler EVT_EMERGENCY_STOP) when an estop is requested. Only touches
 * subsystems that are guaranteed to be linked into BOTH binaries:
 *
 *   - arm_stop()    : arm.c (proc_arm) / arm_rpc_proxy.c (uav_robotd)
 *   - gripper_open(): gripper.c (linked into both)
 *
 * The chassis (car_set_velocity) is NOT touched here because proc_arm
 * does not link car.c. Instead, the scheduler in uav_robotd zeroes the
 * chassis itself when it processes EVT_EMERGENCY_STOP, after invoking
 * this function. */
void dev_emergency_stop_all(void) {
    log_error("core.dev.relay",
              "EMERGENCY STOP: forwarding to arm/gripper subsystems");

    if (!arm_stop()) {
        log_warn("core.dev.relay",
                 "arm_stop returned false (proc_arm offline?)");
    }

    if (!gripper_open(true)) {
        log_warn("core.dev.relay", "gripper_open(true) failed (no gripper?)");
    }
}
