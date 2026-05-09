/* arm_rpc_proxy.c
 *
 * Implements the public dev.h arm_* API as JSON-RPC clients that talk to
 * proc_arm via the unix socket at /tmp/uav_proc_arm.sock.
 *
 * Why a proxy?
 *   uav_robotd's task scheduler used to link arm.c directly, which would
 *   open the can4 SocketCAN interface itself. proc_arm ALSO opens can4 to
 *   serve HostGUI requests, so the two daemons would race on the same bus.
 *   Routing all of uav_robotd's arm operations through proc_arm makes
 *   proc_arm the single CAN owner and removes the contention.
 *
 * This file is linked into uav_robotd in place of uav_robotd/core/dev/arm.c
 * (see Makefile). proc_arm and proc_gateway continue to link the original
 * arm.c since they need (proc_arm) or used to need (proc_gateway, now
 * forwarding) direct CAN access through that file.
 *
 * If proc_arm is not reachable (socket missing or refused), every call
 * here logs an error and returns false. The task FSM in uav_robotd treats
 * a false return as a step failure, which is the correct behavior:
 * "arm not available" -> task fails cleanly.
 */

#include "dev.h"

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/time.h>      /* struct timeval for SO_RCVTIMEO/SO_SNDTIMEO */
#include <sys/types.h>
#include <sys/un.h>
#include <time.h>
#include <unistd.h>

#include "log.h"

#define ARM_PROXY_TAG "core.dev.arm.proxy"

static const char *arm_proxy_sock_path(void) {
    const char *p = getenv("UAV_PROC_ARM_SOCK");
    if (p != NULL && p[0] != '\0') {
        return p;
    }
    return "/tmp/uav_proc_arm.sock";
}

/* Build the JSON-RPC line, send it, read the response, and return ok=true
 * iff the response contained "ok":true. The response body is also written
 * into out_resp[out_resp_len] (truncated, NUL-terminated) when out_resp is
 * non-NULL so callers can extract additional fields. */
static bool arm_proxy_call(const char *method,
                           const char *params_json,
                           char *out_resp,
                           size_t out_resp_len) {
    static int s_id = 1;
    int id = ++s_id;

    int fd = socket(AF_UNIX, SOCK_STREAM, 0);
    if (fd < 0) {
        log_error(ARM_PROXY_TAG, "socket: %s", strerror(errno));
        return false;
    }

    struct timeval tv;
    tv.tv_sec = 30;  /* arm operations may take a few seconds */
    tv.tv_usec = 0;
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

    struct sockaddr_un addr;
    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, arm_proxy_sock_path(), sizeof(addr.sun_path) - 1);

    if (connect(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        log_error(ARM_PROXY_TAG,
                  "connect %s failed: %s (is proc_arm running?)",
                  addr.sun_path,
                  strerror(errno));
        close(fd);
        return false;
    }

    char req[768];
    int req_len = snprintf(req, sizeof(req),
                           "{\"jsonrpc\":\"2.0\",\"id\":%d,\"method\":\"%s\",\"params\":%s}\n",
                           id, method, params_json != NULL ? params_json : "{}");
    if (req_len <= 0 || req_len >= (int)sizeof(req)) {
        log_error(ARM_PROXY_TAG, "request encode overflow for %s", method);
        close(fd);
        return false;
    }
    if (write(fd, req, (size_t)req_len) != req_len) {
        log_error(ARM_PROXY_TAG, "write %s failed: %s", method, strerror(errno));
        close(fd);
        return false;
    }

    char buf[2048];
    size_t total = 0;
    while (total + 1 < sizeof(buf)) {
        ssize_t n = read(fd, buf + total, sizeof(buf) - 1 - total);
        if (n <= 0) {
            break;
        }
        total += (size_t)n;
        if (memchr(buf, '\n', total) != NULL) {
            break;
        }
    }
    buf[total] = '\0';
    close(fd);

    if (out_resp != NULL && out_resp_len > 0) {
        size_t copy = total < out_resp_len - 1 ? total : out_resp_len - 1;
        memcpy(out_resp, buf, copy);
        out_resp[copy] = '\0';
    }

    if (total == 0) {
        log_error(ARM_PROXY_TAG, "no response for %s", method);
        return false;
    }

    /* Look for "ok":true OR "result":true. proc_arm always returns one. */
    if (strstr(buf, "\"ok\":true") != NULL ||
        strstr(buf, "\"result\":true") != NULL) {
        return true;
    }
    log_warn(ARM_PROXY_TAG, "%s -> %s", method, buf);
    return false;
}

/* ───────────────────────── Public dev.h API ──────────────────────── */

bool arm_move(const char *pose) {
    if (pose == NULL || pose[0] == '\0') {
        return false;
    }
    char params[160];
    snprintf(params, sizeof(params), "{\"pose\":\"%s\"}", pose);
    return arm_proxy_call("arm.move_pose", params, NULL, 0);
}

bool arm_move_to_xyz(float x_mm, float y_mm, float z_mm) {
    char params[192];
    snprintf(params, sizeof(params),
             "{\"x_mm\":%.4f,\"y_mm\":%.4f,\"z_mm\":%.4f}",
             (double)x_mm, (double)y_mm, (double)z_mm);
    return arm_proxy_call("arm.move_xyz", params, NULL, 0);
}

bool arm_move_to_pose6d(float x_mm,
                        float y_mm,
                        float z_mm,
                        float roll_deg,
                        float pitch_deg,
                        float yaw_deg) {
    char params[256];
    snprintf(params, sizeof(params),
             "{\"x_mm\":%.4f,\"y_mm\":%.4f,\"z_mm\":%.4f,"
             "\"roll_deg\":%.4f,\"pitch_deg\":%.4f,\"yaw_deg\":%.4f}",
             (double)x_mm, (double)y_mm, (double)z_mm,
             (double)roll_deg, (double)pitch_deg, (double)yaw_deg);
    return arm_proxy_call("arm.move_pose6d", params, NULL, 0);
}

bool arm_move_joint_deg(int joint_index, float target_deg) {
    char params[160];
    snprintf(params, sizeof(params),
             "{\"joint\":%d,\"joint_index\":%d,\"target_deg\":%.4f}",
             joint_index, joint_index, (double)target_deg);
    return arm_proxy_call("arm.move_joint", params, NULL, 0);
}

bool arm_move_joints_deg(const float joints_deg[6]) {
    if (joints_deg == NULL) {
        return false;
    }
    char params[256];
    snprintf(params, sizeof(params),
             "{\"j1_deg\":%.4f,\"j2_deg\":%.4f,\"j3_deg\":%.4f,"
             "\"j4_deg\":%.4f,\"j5_deg\":%.4f,\"j6_deg\":%.4f}",
             (double)joints_deg[0], (double)joints_deg[1], (double)joints_deg[2],
             (double)joints_deg[3], (double)joints_deg[4], (double)joints_deg[5]);
    return arm_proxy_call("arm.move_joints", params, NULL, 0);
}

bool arm_home(void) {
    return arm_proxy_call("arm.home", "{}", NULL, 0);
}

bool arm_home_joint(int joint_index) {
    char params[64];
    snprintf(params, sizeof(params),
             "{\"joint\":%d,\"joint_index\":%d}", joint_index, joint_index);
    return arm_proxy_call("arm.home_joint", params, NULL, 0);
}

bool arm_set_zero_params(int joint_index) {
    char params[64];
    snprintf(params, sizeof(params),
             "{\"joint\":%d,\"joint_index\":%d}", joint_index, joint_index);
    return arm_proxy_call("arm.set_zero_params", params, NULL, 0);
}

bool arm_stop_joint(int joint_index) {
    char params[64];
    snprintf(params, sizeof(params),
             "{\"joint\":%d,\"joint_index\":%d}", joint_index, joint_index);
    return arm_proxy_call("arm.stop_joint", params, NULL, 0);
}

bool arm_stop(void) {
    return arm_proxy_call("arm.stop", "{}", NULL, 0);
}

/* The estop flag and pipe live inside proc_arm's arm.c. Nothing in
 * uav_robotd directly polls them; the proxy just forwards stop requests
 * to proc_arm so its watchdog / arm_stop() handles the rest. */
void arm_request_estop(void) {
    (void)arm_proxy_call("arm.emergency_stop", "{\"enable\":true}", NULL, 0);
}

void arm_clear_estop(void) {
    (void)arm_proxy_call("arm.emergency_stop", "{\"enable\":false}", NULL, 0);
}

bool arm_estop_requested(void) {
    /* uav_robotd doesn't need a tight read of the flag - it manages its
     * own emergency_stop in SystemState. Always return false here. */
    return false;
}

void arm_set_move_rpm(int rpm) {
    char params[64];
    snprintf(params, sizeof(params),
             "{\"move_rpm\":%d,\"zero_rpm\":0}", rpm);
    (void)arm_proxy_call("arm.set_speeds", params, NULL, 0);
}

void arm_set_zero_rpm(int rpm) {
    char params[64];
    snprintf(params, sizeof(params),
             "{\"move_rpm\":0,\"zero_rpm\":%d}", rpm);
    (void)arm_proxy_call("arm.set_speeds", params, NULL, 0);
}

int arm_get_move_rpm(void) {
    /* Round-trip would require parsing the response. uav_robotd never
     * reads this back, so return the env-default to keep the API total. */
    const char *e = getenv("UAV_ARM_RPM");
    if (e != NULL && e[0] != '\0') {
        return atoi(e);
    }
    return 300;
}

int arm_get_zero_rpm(void) {
    const char *e = getenv("UAV_ARM_ZERO_SPEED_RPM");
    if (e != NULL && e[0] != '\0') {
        return atoi(e);
    }
    return 30;
}

void arm_set_joint_dps(double dps) {
    char params[64];
    snprintf(params, sizeof(params), "{\"joint_dps\":%.2f}", dps);
    (void)arm_proxy_call("arm.set_speeds", params, NULL, 0);
}

void arm_set_zero_dps(double dps) {
    char params[64];
    snprintf(params, sizeof(params), "{\"zero_dps\":%.2f}", dps);
    (void)arm_proxy_call("arm.set_speeds", params, NULL, 0);
}

double arm_get_joint_dps(void) {
    /* uav_robotd never reads this back today; surface the env default
     * so callers get something sensible without a round-trip. */
    const char *e = getenv("UAV_ARM_JOINT_DPS");
    if (e != NULL && e[0] != '\0') {
        return atof(e);
    }
    return 60.0;
}

double arm_get_zero_dps(void) {
    const char *e = getenv("UAV_ARM_ZERO_DPS");
    if (e != NULL && e[0] != '\0') {
        return atof(e);
    }
    return 5.0;
}
