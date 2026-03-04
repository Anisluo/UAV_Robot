#ifndef UAV_SYSTEM_H
#define UAV_SYSTEM_H

#include <stdbool.h>
#include "event.h"

typedef struct {
    bool fault;
    bool emergency_stop;
    bool shutdown;
    TaskType active_task;
} SystemState;

void system_state_init(SystemState *state);

#endif
