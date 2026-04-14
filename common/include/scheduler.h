#ifndef UAV_SCHEDULER_H
#define UAV_SCHEDULER_H

#include "event_bus.h"
#include "system.h"
#include "device_registry.h"
#include "app_battery_pick.h"
#include "app_arm_demo.h"
#include "app_pick_place.h"
#include "app_face_track.h"

typedef struct {
    DeviceRegistry devices;
    BatteryPickTask battery_pick_3d;
    BatteryPickTask battery_pick_6d;
    ArmDemoTask     arm_demo;
    PickPlaceTask   pick_place;
    FaceTrackTask   face_track;
} Scheduler;

void scheduler_init(Scheduler *scheduler, EventBus *bus, SystemState *state);
void scheduler_handle(const Event *ev, EventBus *bus, SystemState *state, Scheduler *scheduler);

#endif
