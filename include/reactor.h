#ifndef UAV_INCLUDE_REACTOR_H
#define UAV_INCLUDE_REACTOR_H

#include <stddef.h>
#include "event_bus.h"
#include "io.h"

typedef struct {
    IOAdapter io;
} Reactor;

void reactor_init(Reactor *reactor, const char **script, size_t script_len);
void reactor_poll(Reactor *reactor, EventBus *bus, int timeout_ms);

#endif
