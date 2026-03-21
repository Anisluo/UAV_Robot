#ifndef UAV_APP_BATTERY_PICK_H
#define UAV_APP_BATTERY_PICK_H

#include <stdbool.h>
#include "device_registry.h"

#define BP_DETECT_TIMEOUT_TICKS 100  /* ~20s at 200ms poll; fail if no detection */

typedef enum {
    BP_PRECHECK = 0,
    BP_FIXTURE_LOCK,
    BP_DETECT,
    BP_APPROACH,
    BP_GRIPPER_OPEN,
    BP_ALIGN,
    BP_GRASP,
    BP_LIFT,
    BP_DONE,
    BP_FAIL
} BatteryPickState;

typedef struct {
    BatteryPickState state;
    int  ticks_in_state;
    /* Detected battery 3-D pose (camera frame, mm). Valid when detected=1. */
    float target_x_mm;
    float target_y_mm;
    float target_z_mm;
    int   detected;
} BatteryPickTask;

void app_battery_pick_start(BatteryPickTask *task);
bool app_battery_pick_step(BatteryPickTask *task, const DeviceRegistry *reg, char *err, int err_len);
bool app_battery_pick_done(const BatteryPickTask *task);

#endif
