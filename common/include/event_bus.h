#ifndef UAV_EVENT_BUS_H
#define UAV_EVENT_BUS_H

#include <stdbool.h>
#include "event.h"

#define EVENT_BUS_CAPACITY 256

typedef struct {
    Event queue[EVENT_BUS_CAPACITY];
    int head;
    int tail;
    int size;
} EventBus;

void event_bus_init(EventBus *bus);
bool event_bus_publish(EventBus *bus, const Event *ev);
bool event_bus_try_pop(EventBus *bus, Event *ev);

#endif
