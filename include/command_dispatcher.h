#ifndef UAV_COMMAND_DISPATCHER_H
#define UAV_COMMAND_DISPATCHER_H

#include "event_bus.h"
#include "system.h"

void command_dispatcher_handle(const Event *ev, EventBus *bus, SystemState *state);

#endif
