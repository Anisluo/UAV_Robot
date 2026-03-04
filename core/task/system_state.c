#include "system.h"

void system_state_init(SystemState *state) {
    state->fault = false;
    state->emergency_stop = false;
    state->shutdown = false;
    state->active_task = TASK_NONE;
}
