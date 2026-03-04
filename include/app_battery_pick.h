#ifndef UAV_APP_BATTERY_PICK_H
#define UAV_APP_BATTERY_PICK_H

#include <stdbool.h>
#include "device_registry.h"

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
    int ticks_in_state;
} BatteryPickTask;

void app_battery_pick_start(BatteryPickTask *task);
bool app_battery_pick_step(BatteryPickTask *task, const DeviceRegistry *reg, char *err, int err_len);
bool app_battery_pick_done(const BatteryPickTask *task);

#endif
