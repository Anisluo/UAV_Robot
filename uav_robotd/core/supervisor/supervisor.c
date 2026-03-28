#include <string.h>
#include "supervisor.h"
#include "device_registry.h"
#include "proto.h"
#include "log.h"

static void publish_feedback(EventBus *bus, uint64_t ts_ms, CommandType cmd, bool accepted, const char *reason) {
    Event out = {0};
    out.type = accepted ? EVT_CMD_ACK : EVT_CMD_NACK;
    out.ts_ms = ts_ms;
    out.data.feedback.cmd = cmd;
    if (reason != NULL) {
        const char *nul = memchr(reason, '\0', UAV_TEXT_MAX - 1);
        size_t n = (nul != NULL) ? (size_t)(nul - reason) : (UAV_TEXT_MAX - 1);
        memcpy(out.data.feedback.reason, reason, n);
        out.data.feedback.reason[n] = '\0';
    }
    event_bus_publish(bus, &out);
}

void supervisor_handle(const Event *ev, EventBus *bus, SystemState *state) {
    if (ev->type != EVT_CMD_REQUEST) {
        return;
    }

    CommandType cmd = ev->data.command.cmd;
    log_info("dispatcher", "receive command %s", proto_command_name(cmd));

    if (cmd == CMD_EMERGENCY_STOP) {
        publish_feedback(bus, ev->ts_ms, cmd, true, "accepted");
        Event out = {0};
        out.type = EVT_EMERGENCY_STOP;
        out.ts_ms = ev->ts_ms;
        event_bus_publish(bus, &out);
        return;
    }

    if (cmd == CMD_RESET_FAULT) {
        publish_feedback(bus, ev->ts_ms, cmd, true, "accepted");
        Event out = {0};
        out.type = EVT_RESET_FAULT;
        out.ts_ms = ev->ts_ms;
        event_bus_publish(bus, &out);
        return;
    }

    if (cmd == CMD_SHUTDOWN) {
        publish_feedback(bus, ev->ts_ms, cmd, true, "accepted");
        Event out = {0};
        out.type = EVT_SHUTDOWN;
        out.ts_ms = ev->ts_ms;
        event_bus_publish(bus, &out);
        return;
    }

    if (state->fault || state->emergency_stop) {
        publish_feedback(bus, ev->ts_ms, cmd, false, "system in fault/estop");
        log_warn("dispatcher", "reject command because system in fault/estop");
        return;
    }

    if (cmd == CMD_START_BATTERY_PICK ||
        cmd == CMD_START_BATTERY_PICK_3D ||
        cmd == CMD_START_BATTERY_PICK_6D) {
        if (state->active_task != TASK_NONE) {
            publish_feedback(bus, ev->ts_ms, cmd, false, "task already running");
            log_warn("dispatcher", "reject start because task %d is running", state->active_task);
            return;
        }

        publish_feedback(bus, ev->ts_ms, cmd, true, "accepted");
        Event out = {0};
        out.type = EVT_TASK_REQUEST;
        out.ts_ms = ev->ts_ms;
        if (cmd == CMD_START_BATTERY_PICK_6D) {
            out.data.task.task = TASK_BATTERY_PICK_6D;
        } else if (cmd == CMD_START_BATTERY_PICK_3D) {
            out.data.task.task = TASK_BATTERY_PICK_3D;
        } else {
            out.data.task.task = TASK_BATTERY_PICK;
        }
        event_bus_publish(bus, &out);
        return;
    }

    publish_feedback(bus, ev->ts_ms, cmd, false, "unsupported command");
}

void system_state_init(SystemState *state) {
    state->fault = false;
    state->emergency_stop = false;
    state->shutdown = false;
    state->active_task = TASK_NONE;
}

void device_registry_init(DeviceRegistry *reg) {
    reg->arm_online = true;
    reg->platform_online = true;
    reg->car_online = true;
    reg->gripper_online = true;
}

bool device_registry_check_ready(const DeviceRegistry *reg) {
    return reg->arm_online && reg->platform_online && reg->car_online && reg->gripper_online;
}
