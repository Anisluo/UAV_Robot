/* task_pick_place.c
 *
 * Vision-guided pick-and-place orchestrator.  uav_robotd delegates all
 * sensor → plan → motion work to proc_grasp (which owns the
 * proc_realsense → proc_npu → proc_arm chain).  This task just kicks
 * the pipeline and polls its status.
 */

#define _POSIX_C_SOURCE 200809L

#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/un.h>
#include <unistd.h>
#include <stdlib.h>

#include "app_pick_place.h"
#include "log.h"

#define PP_DEFAULT_TIMEOUT_TICKS 300   /* ~60 s at 200 ms scheduler period */

static const char *grasp_sock_path(void) {
    const char *p = getenv("UAV_PROC_GRASP_SOCK");
    if (p != NULL && p[0] != '\0') return p;
    return "/tmp/uav_proc_grasp.ctrl.sock";
}

/* Open Unix stream, send one JSON-RPC line, read one line of response.
 * Returns bytes read on success, -1 on error. out_buf is NUL-terminated. */
static int grasp_call(const char *method,
                      const char *params_json,
                      char *out_buf,
                      size_t out_size) {
    static int s_id = 9000;
    int id = ++s_id;

    int fd = socket(AF_UNIX, SOCK_STREAM, 0);
    if (fd < 0) {
        log_error("task.pick_place", "socket: %s", strerror(errno));
        return -1;
    }

    struct timeval tv = {3, 0};
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

    struct sockaddr_un addr;
    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, grasp_sock_path(), sizeof(addr.sun_path) - 1);

    if (connect(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        log_error("task.pick_place",
                  "connect %s failed: %s (proc_grasp running?)",
                  addr.sun_path, strerror(errno));
        close(fd);
        return -1;
    }

    char req[512];
    int req_len = snprintf(req, sizeof(req),
                           "{\"jsonrpc\":\"2.0\",\"id\":%d,\"method\":\"%s\",\"params\":%s}\n",
                           id, method, params_json ? params_json : "{}");
    if (req_len <= 0 || write(fd, req, (size_t)req_len) != req_len) {
        close(fd);
        return -1;
    }

    size_t total = 0;
    while (total + 1 < out_size) {
        ssize_t n = read(fd, out_buf + total, out_size - 1 - total);
        if (n <= 0) break;
        total += (size_t)n;
        if (memchr(out_buf, '\n', total)) break;
    }
    out_buf[total] = '\0';
    close(fd);
    return (int)total;
}

/* Extract a string value for "key":"value" from a JSON fragment.
 * Writes up to cap-1 chars into out, NUL-terminates.  Returns true if found. */
static bool json_find_string(const char *buf, const char *key,
                             char *out, size_t cap) {
    char needle[64];
    snprintf(needle, sizeof(needle), "\"%s\"", key);
    const char *p = strstr(buf, needle);
    if (p == NULL) return false;
    p = strchr(p, ':');
    if (p == NULL) return false;
    ++p;
    while (*p == ' ' || *p == '\t') ++p;
    if (*p != '"') return false;
    ++p;
    size_t i = 0;
    while (*p && *p != '"' && i + 1 < cap) out[i++] = *p++;
    out[i] = '\0';
    return true;
}

void app_pick_place_start(PickPlaceTask *task) {
    if (task == NULL) return;
    task->state              = PP_INIT;
    task->ticks_in_state     = 0;
    task->poll_timeout_ticks = PP_DEFAULT_TIMEOUT_TICKS;
    task->last_error[0]      = '\0';
    log_info("app.pick_place", "start FSM (delegates to proc_grasp)");
}

bool app_pick_place_done(const PickPlaceTask *task) {
    return task != NULL && (task->state == PP_DONE || task->state == PP_FAIL);
}

static void next_state(PickPlaceTask *task, PickPlaceState st) {
    task->state          = st;
    task->ticks_in_state = 0;
}

bool app_pick_place_step(PickPlaceTask *task,
                         const DeviceRegistry *reg,
                         char *err,
                         int err_len) {
    (void)reg;
    if (task == NULL) {
        snprintf(err, (size_t)err_len, "task null");
        return false;
    }
    task->ticks_in_state++;

    char resp[1024];

    switch (task->state) {
        case PP_INIT:
            next_state(task, PP_TRIGGER_GRASP);
            return true;

        case PP_TRIGGER_GRASP: {
            int n = grasp_call("grasp.start", "{\"mode\":\"3D\"}",
                               resp, sizeof(resp));
            if (n < 0 || strstr(resp, "\"ok\":true") == NULL) {
                snprintf(task->last_error, sizeof(task->last_error),
                         "grasp.start failed");
                log_warn("app.pick_place", "grasp.start rejected: %s",
                         n > 0 ? resp : "no response");
                next_state(task, PP_FAIL);
                return false;
            }
            log_info("app.pick_place", "grasp.start ok → waiting for pipeline");
            next_state(task, PP_WAIT_DONE);
            return true;
        }

        case PP_WAIT_DONE: {
            if (task->ticks_in_state > task->poll_timeout_ticks) {
                snprintf(task->last_error, sizeof(task->last_error),
                         "timeout waiting for grasp pipeline");
                next_state(task, PP_FAIL);
                return false;
            }
            int n = grasp_call("grasp.get_status", "{}", resp, sizeof(resp));
            if (n < 0) {
                /* transient; just retry on next tick */
                return true;
            }
            char state_buf[32] = {};
            if (!json_find_string(resp, "state", state_buf, sizeof(state_buf))) {
                return true;
            }
            if (strcmp(state_buf, "done") == 0) {
                log_info("app.pick_place", "grasp pipeline done");
                next_state(task, PP_DONE);
                return true;
            }
            if (strcmp(state_buf, "fail") == 0) {
                char err_buf[96] = {};
                json_find_string(resp, "error", err_buf, sizeof(err_buf));
                snprintf(task->last_error, sizeof(task->last_error),
                         "grasp failed: %s", err_buf);
                next_state(task, PP_FAIL);
                return false;
            }
            /* still running: idle, confirm, pre_approach, ... */
            return true;
        }

        case PP_DONE:
        case PP_FAIL:
            return task->state == PP_DONE;
    }

    snprintf(err, (size_t)err_len, "unknown state");
    return false;
}
