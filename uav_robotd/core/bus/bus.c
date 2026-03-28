#include "event_bus.h"

void event_bus_init(EventBus *bus) {
    bus->head = 0;
    bus->tail = 0;
    bus->size = 0;
}

bool event_bus_publish(EventBus *bus, const Event *ev) {
    if (bus->size >= EVENT_BUS_CAPACITY) {
        return false;
    }
    bus->queue[bus->tail] = *ev;
    bus->tail = (bus->tail + 1) % EVENT_BUS_CAPACITY;
    bus->size++;
    return true;
}

bool event_bus_try_pop(EventBus *bus, Event *ev) {
    if (bus->size == 0) {
        return false;
    }
    *ev = bus->queue[bus->head];
    bus->head = (bus->head + 1) % EVENT_BUS_CAPACITY;
    bus->size--;
    return true;
}
