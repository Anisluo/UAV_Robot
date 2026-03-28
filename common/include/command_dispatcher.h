#ifndef UAV_COMMAND_DISPATCHER_H
#define UAV_COMMAND_DISPATCHER_H

#include "supervisor.h"

static inline void command_dispatcher_handle(const Event *ev, EventBus *bus, SystemState *state) {
    supervisor_handle(ev, bus, state);
}

#endif
