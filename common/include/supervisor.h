#ifndef UAV_SUPERVISOR_H
#define UAV_SUPERVISOR_H

#include "event_bus.h"
#include "system.h"

void supervisor_handle(const Event *ev, EventBus *bus, SystemState *state);

#endif
