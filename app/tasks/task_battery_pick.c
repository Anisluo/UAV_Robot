#include <stdio.h>
#include <string.h>
#include "app_battery_pick.h"
#include "dev.h"
#include "log.h"
#include "npu_detect.h"

void app_battery_pick_start(BatteryPickTask *task) {
    task->state        = BP_PRECHECK;
    task->ticks_in_state = 0;
    task->detected     = 0;
    task->target_x_mm  = 0.0f;
    task->target_y_mm  = 0.0f;
    task->target_z_mm  = 0.0f;
    log_info("app.battery_pick", "start FSM");
}

bool app_battery_pick_done(const BatteryPickTask *task) {
    return task->state == BP_DONE;
}

static void next_state(BatteryPickTask *task, BatteryPickState st) {
    task->state = st;
    task->ticks_in_state = 0;
}

bool app_battery_pick_step(BatteryPickTask *task, const DeviceRegistry *reg, char *err, int err_len) {
    (void)err_len;
    task->ticks_in_state++;

    switch (task->state) {
        case BP_PRECHECK:
            if (!device_registry_check_ready(reg)) {
                strcpy(err, "device offline");
                next_state(task, BP_FAIL);
                return false;
            }
            next_state(task, BP_FIXTURE_LOCK);
            return true;

        case BP_FIXTURE_LOCK:
            if (!platform_lock(true)) {
                strcpy(err, "platform lock failed");
                next_state(task, BP_FAIL);
                return false;
            }
            next_state(task, BP_DETECT);
            return true;

        case BP_DETECT: {
            /* Poll proc_npu for a detection with 3-D pose. */
            float x = 0.0f, y = 0.0f, z = 0.0f;
            if (npu_read_latest_battery(&x, &y, &z)) {
                task->target_x_mm = x;
                task->target_y_mm = y;
                task->target_z_mm = z;
                task->detected    = 1;
                log_info("app.battery_pick",
                         "battery detected at (%.1f, %.1f, %.1f) mm",
                         x, y, z);
                next_state(task, BP_APPROACH);
            } else if (task->ticks_in_state >= BP_DETECT_TIMEOUT_TICKS) {
                strcpy(err, "detect timeout: no battery found");
                next_state(task, BP_FAIL);
                return false;
            }
            /* Stay in BP_DETECT until detection arrives or timeout. */
            return true;
        }

        case BP_APPROACH:
            if (task->detected) {
                /* Move arm to detected battery position */
                if (!arm_move_to_xyz(task->target_x_mm,
                                     task->target_y_mm,
                                     task->target_z_mm)) {
                    strcpy(err, "arm approach (xyz) failed");
                    next_state(task, BP_FAIL);
                    return false;
                }
            } else {
                /* Fallback to preset approach pose */
                if (!arm_move("approach")) {
                    strcpy(err, "arm approach failed");
                    next_state(task, BP_FAIL);
                    return false;
                }
            }
            next_state(task, BP_GRIPPER_OPEN);
            return true;

        case BP_GRIPPER_OPEN:
            if (!gripper_open(true)) {
                strcpy(err, "gripper open failed");
                next_state(task, BP_FAIL);
                return false;
            }
            next_state(task, BP_ALIGN);
            return true;

        case BP_ALIGN:
            if (!car_set_velocity(0, 0)) {
                strcpy(err, "car stop failed");
                next_state(task, BP_FAIL);
                return false;
            }
            next_state(task, BP_GRASP);
            return true;

        case BP_GRASP:
            if (!gripper_open(false)) {
                strcpy(err, "grasp failed");
                next_state(task, BP_FAIL);
                return false;
            }
            next_state(task, BP_LIFT);
            return true;

        case BP_LIFT:
            if (!arm_move("lift")) {
                strcpy(err, "lift failed");
                next_state(task, BP_FAIL);
                return false;
            }
            next_state(task, BP_DONE);
            return true;

        case BP_DONE:
            return true;

        case BP_FAIL:
        default:
            return false;
    }
}
