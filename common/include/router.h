#ifndef UAV_ROUTER_H
#define UAV_ROUTER_H

#include "event_bus.h"

void router_handle(const Event *ev, EventBus *bus);

#endif
