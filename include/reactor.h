#ifndef UAV_REACTOR_H
#define UAV_REACTOR_H

#include <stddef.h>
#include "event_bus.h"

typedef struct {
    const char **script;
    size_t script_len;
    size_t next_idx;
} Reactor;

void reactor_init(Reactor *reactor, const char **script, size_t script_len);
void reactor_poll(Reactor *reactor, EventBus *bus, int timeout_ms);

#endif
