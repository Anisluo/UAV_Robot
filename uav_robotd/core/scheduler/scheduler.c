#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "scheduler.h"
#include "dev.h"
#include "log.h"

void scheduler_init(Scheduler *scheduler, EventBus *bus, SystemState *state) {
    (void)bus;
    device_registry_init(&scheduler->devices);
    app_battery_pick_start(&scheduler->battery_pick_3d, UAV_GRASP_MODE_3D);
    app_battery_pick_start(&scheduler->battery_pick_6d, UAV_GRASP_MODE_6D);
    state->active_task = TASK_NONE;
}

static const char *task_status_path(void) {
    const char *path = getenv("UAV_TASK_STATUS_FILE");
    if (path == NULL || path[0] == '\0') {
        return "/tmp/uav_task_status.json";
    }
    return path;
}

static const char *task_name(TaskType task) {
    switch (task) {
        case TASK_BATTERY_PICK:
            return "BATTERY_PICK_3D";
        case TASK_BATTERY_PICK_6D:
            return "BATTERY_PICK_6D";
        case TASK_NONE:
        default:
            return "NONE";
    }
}

static void task_status_escape(char *dst, size_t dst_size, const char *src) {
    size_t di = 0;

    if (dst == NULL || dst_size == 0) {
        return;
    }
    if (src == NULL) {
        dst[0] = '\0';
        return;
    }

    while (*src != '\0' && di + 2 < dst_size) {
        char ch = *src++;
        if (ch == '\\' || ch == '"') {
            dst[di++] = '\\';
            dst[di++] = ch;
        } else if ((unsigned char)ch < 0x20U) {
            dst[di++] = ' ';
        } else {
            dst[di++] = ch;
        }
    }
    dst[di] = '\0';
}

static void write_task_status(TaskType task, const char *status, const char *reason) {
    FILE *fp;
    char escaped_reason[UAV_TEXT_MAX * 2];
    int active;

    fp = fopen(task_status_path(), "w");
    if (fp == NULL) {
        return;
    }

    task_status_escape(escaped_reason, sizeof(escaped_reason), reason != NULL ? reason : "");
    active = (status != NULL && strcmp(status, "running") == 0) ? 1 : 0;
    fprintf(fp,
            "{\"active\":%s,\"task\":\"%s\",\"status\":\"%s\",\"reason\":\"%s\"}\n",
            active ? "true" : "false",
            task_name(task),
            status != NULL ? status : "idle",
            escaped_reason);
    fclose(fp);
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
        if (ev->data.task.task == TASK_BATTERY_PICK || ev->data.task.task == TASK_BATTERY_PICK_3D) {
            app_battery_pick_start(&scheduler->battery_pick_3d, UAV_GRASP_MODE_3D);
            state->active_task = TASK_BATTERY_PICK_3D;
            log_info("task", "task start: BATTERY_PICK_3D");
            write_task_status(TASK_BATTERY_PICK_3D, "running", "");
        } else if (ev->data.task.task == TASK_BATTERY_PICK_6D) {
            app_battery_pick_start(&scheduler->battery_pick_6d, UAV_GRASP_MODE_6D);
            state->active_task = TASK_BATTERY_PICK_6D;
            log_info("task", "task start: BATTERY_PICK_6D");
            write_task_status(TASK_BATTERY_PICK_6D, "running", "");
        }
        return;
    }

    if (ev->type == EVT_TIMER_TICK &&
        (state->active_task == TASK_BATTERY_PICK ||
         state->active_task == TASK_BATTERY_PICK_3D ||
         state->active_task == TASK_BATTERY_PICK_6D)) {
        char err[UAV_TEXT_MAX] = {0};
        BatteryPickTask *task = (state->active_task == TASK_BATTERY_PICK_6D)
                                ? &scheduler->battery_pick_6d
                                : &scheduler->battery_pick_3d;
        if (!app_battery_pick_step(task, &scheduler->devices, err, sizeof(err))) {
            publish_task_fail(bus, state->active_task, err[0] ? err : "unknown task failure");
            return;
        }

        if (app_battery_pick_done(task)) {
            publish_task_done(bus, state->active_task);
        }
        return;
    }

    if (ev->type == EVT_TASK_DONE) {
        log_info("task", "task done: %d", ev->data.task.task);
        write_task_status(ev->data.task.task, "done", "");
        state->active_task = TASK_NONE;
        return;
    }

    if (ev->type == EVT_TASK_FAIL) {
        log_error("task", "task failed: %s", ev->data.failure.reason);
        write_task_status(state->active_task, "failed", ev->data.failure.reason);
        state->fault = true;
        state->active_task = TASK_NONE;
        return;
    }

    if (ev->type == EVT_EMERGENCY_STOP) {
        state->emergency_stop = true;
        write_task_status(state->active_task, "stopped", "emergency stop");
        state->active_task = TASK_NONE;
        dev_emergency_stop_all();
        log_error("task", "all tasks canceled due to emergency stop");
        return;
    }

    if (ev->type == EVT_RESET_FAULT) {
        state->fault = false;
        state->emergency_stop = false;
        write_task_status(TASK_NONE, "idle", "");
        log_warn("task", "system fault and estop reset by command");
        return;
    }

    if (ev->type == EVT_SHUTDOWN) {
        state->shutdown = true;
    }
}
