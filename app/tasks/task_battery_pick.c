#include <stdio.h>
#include <string.h>
#include "app_battery_pick.h"
#include "dev.h"
#include "log.h"

void app_battery_pick_start(BatteryPickTask *task) {
    task->state = BP_PRECHECK;
    task->ticks_in_state = 0;
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

        case BP_DETECT:
            next_state(task, BP_APPROACH);
            return true;

        case BP_APPROACH:
            if (!arm_move("approach")) {
                strcpy(err, "arm approach failed");
                next_state(task, BP_FAIL);
                return false;
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
