#ifndef UAV_TASK_MANAGER_H
#define UAV_TASK_MANAGER_H

#include "event_bus.h"
#include "system.h"
#include "device_registry.h"
#include "app_battery_pick.h"

typedef struct {
    DeviceRegistry devices;
    BatteryPickTask battery_pick;
} TaskManager;

void task_manager_init(TaskManager *mgr, EventBus *bus, SystemState *state);
void task_manager_handle(const Event *ev, EventBus *bus, SystemState *state, TaskManager *mgr);

#endif
