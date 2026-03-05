#include <string.h>
#include "scheduler.h"
#include "dev.h"
#include "log.h"

void scheduler_init(Scheduler *scheduler, EventBus *bus, SystemState *state) {
    (void)bus;
    device_registry_init(&scheduler->devices);
    app_battery_pick_start(&scheduler->battery_pick);
    state->active_task = TASK_NONE;
}

static void publish_task_done(EventBus *bus, TaskType task) {
    Event ev = {0};
    ev.type = EVT_TASK_DONE;
    ev.data.task.task = task;
    event_bus_publish(bus, &ev);
}

static void publish_task_fail(EventBus *bus, TaskType task, const char *reason) {
    (void)task;
    Event ev = {0};
    ev.type = EVT_TASK_FAIL;
    const char *nul = memchr(reason, '\0', UAV_TEXT_MAX - 1);
    size_t n = (nul != NULL) ? (size_t)(nul - reason) : (UAV_TEXT_MAX - 1);
    memcpy(ev.data.failure.reason, reason, n);
    ev.data.failure.reason[n] = '\0';
    event_bus_publish(bus, &ev);
}

void scheduler_handle(const Event *ev, EventBus *bus, SystemState *state, Scheduler *scheduler) {
    if (ev->type == EVT_TASK_REQUEST) {
        if (ev->data.task.task == TASK_BATTERY_PICK) {
            app_battery_pick_start(&scheduler->battery_pick);
            state->active_task = TASK_BATTERY_PICK;
            log_info("task", "task start: BATTERY_PICK");
        }
        return;
    }

    if (ev->type == EVT_TIMER_TICK && state->active_task == TASK_BATTERY_PICK) {
        char err[UAV_TEXT_MAX] = {0};
        if (!app_battery_pick_step(&scheduler->battery_pick, &scheduler->devices, err, sizeof(err))) {
            publish_task_fail(bus, TASK_BATTERY_PICK, err[0] ? err : "unknown task failure");
            return;
        }

        if (app_battery_pick_done(&scheduler->battery_pick)) {
            publish_task_done(bus, TASK_BATTERY_PICK);
        }
        return;
    }

    if (ev->type == EVT_TASK_DONE) {
        log_info("task", "task done: %d", ev->data.task.task);
        state->active_task = TASK_NONE;
        return;
    }

    if (ev->type == EVT_TASK_FAIL) {
        log_error("task", "task failed: %s", ev->data.failure.reason);
        state->fault = true;
        state->active_task = TASK_NONE;
        return;
    }

    if (ev->type == EVT_EMERGENCY_STOP) {
        state->emergency_stop = true;
        state->active_task = TASK_NONE;
        dev_emergency_stop_all();
        log_error("task", "all tasks canceled due to emergency stop");
        return;
    }

    if (ev->type == EVT_RESET_FAULT) {
        state->fault = false;
        state->emergency_stop = false;
        log_warn("task", "system fault and estop reset by command");
        return;
    }

    if (ev->type == EVT_SHUTDOWN) {
        state->shutdown = true;
    }
}
