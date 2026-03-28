#ifndef UAV_INCLUDE_REACTOR_H
#define UAV_INCLUDE_REACTOR_H

#include <stdbool.h>
#include <stddef.h>
#include "event_bus.h"
#include "io.h"

typedef struct {
    IOAdapter io;
} Reactor;

bool reactor_init(Reactor *reactor, const char **script, size_t script_len, unsigned short listen_port);
void reactor_close(Reactor *reactor);
void reactor_poll(Reactor *reactor, EventBus *bus, int timeout_ms);

#endif
