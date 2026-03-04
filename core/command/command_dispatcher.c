#include "command_dispatcher.h"
#include "proto.h"
#include "log.h"

void command_dispatcher_handle(const Event *ev, EventBus *bus, SystemState *state) {
    if (ev->type != EVT_CMD_REQUEST) {
        return;
    }

    CommandType cmd = ev->data.command.cmd;
    log_info("dispatcher", "receive command %s", proto_command_name(cmd));

    if (cmd == CMD_EMERGENCY_STOP) {
        Event out = {0};
        out.type = EVT_EMERGENCY_STOP;
        out.ts_ms = ev->ts_ms;
        event_bus_publish(bus, &out);
        return;
    }

    if (cmd == CMD_RESET_FAULT) {
        Event out = {0};
        out.type = EVT_RESET_FAULT;
        out.ts_ms = ev->ts_ms;
        event_bus_publish(bus, &out);
        return;
    }

    if (cmd == CMD_SHUTDOWN) {
        Event out = {0};
        out.type = EVT_SHUTDOWN;
        out.ts_ms = ev->ts_ms;
        event_bus_publish(bus, &out);
        return;
    }

    if (state->fault || state->emergency_stop) {
        log_warn("dispatcher", "reject command because system in fault/estop");
        return;
    }

    if (cmd == CMD_START_BATTERY_PICK) {
        if (state->active_task != TASK_NONE) {
            log_warn("dispatcher", "reject start because task %d is running", state->active_task);
            return;
        }

        Event out = {0};
        out.type = EVT_TASK_REQUEST;
        out.ts_ms = ev->ts_ms;
        out.data.task.task = TASK_BATTERY_PICK;
        event_bus_publish(bus, &out);
        return;
    }
}
