#ifndef UAV_SCHEDULER_H
#define UAV_SCHEDULER_H

#include "event_bus.h"
#include "system.h"
#include "device_registry.h"
#include "app_battery_pick.h"

typedef struct {
    DeviceRegistry devices;
    BatteryPickTask battery_pick;
} Scheduler;

void scheduler_init(Scheduler *scheduler, EventBus *bus, SystemState *state);
void scheduler_handle(const Event *ev, EventBus *bus, SystemState *state, Scheduler *scheduler);

#endif
