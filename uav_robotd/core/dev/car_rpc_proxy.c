/* car_rpc_proxy.c
 *
 * Implements the public dev.h car_set_velocity() API as a JSON-RPC client
 * that talks to proc_car via /tmp/uav_proc_car.sock.
 *
 * Same pattern as arm_rpc_proxy.c: uav_robotd no longer opens the CAN bus
 * for the chassis directly. proc_car is the single CAN owner for can3.
 *
 * This file is linked into uav_robotd in place of car.c + can_socketcan.c.
 */

#include "dev.h"

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/un.h>
#include <unistd.h>

#include "log.h"

#define CAR_PROXY_TAG "core.dev.car.proxy"

static const char *car_proxy_sock_path(void) {
    const char *p = getenv("UAV_PROC_CAR_SOCK");
    if (p != NULL && p[0] != '\0') {
        return p;
    }
    return "/tmp/uav_proc_car.sock";
}

static bool car_proxy_call(const char *method, const char *params_json) {
    static int s_id = 1;
    int id = ++s_id;

    int fd = socket(AF_UNIX, SOCK_STREAM, 0);
    if (fd < 0) {
        log_error(CAR_PROXY_TAG, "socket: %s", strerror(errno));
        return false;
    }

    struct timeval tv;
    tv.tv_sec = 5;
    tv.tv_usec = 0;
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

    struct sockaddr_un addr;
    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, car_proxy_sock_path(), sizeof(addr.sun_path) - 1);

    if (connect(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        log_error(CAR_PROXY_TAG,
                  "connect %s failed: %s (is proc_car running?)",
                  addr.sun_path, strerror(errno));
        close(fd);
        return false;
    }

    char req[512];
    int req_len = snprintf(req, sizeof(req),
                           "{\"jsonrpc\":\"2.0\",\"id\":%d,\"method\":\"%s\",\"params\":%s}\n",
                           id, method, params_json != NULL ? params_json : "{}");
    if (req_len <= 0 || write(fd, req, (size_t)req_len) != req_len) {
        close(fd);
        return false;
    }

    char buf[1024];
    size_t total = 0;
    while (total + 1 < sizeof(buf)) {
        ssize_t n = read(fd, buf + total, sizeof(buf) - 1 - total);
        if (n <= 0) break;
        total += (size_t)n;
        if (memchr(buf, '\n', total) != NULL) break;
    }
    buf[total] = '\0';
    close(fd);

    if (total == 0) {
        log_error(CAR_PROXY_TAG, "no response for %s", method);
        return false;
    }

    return strstr(buf, "\"ok\":true") != NULL
        || strstr(buf, "\"result\":true") != NULL;
}

bool car_set_velocity(int linear_mm_s, int angular_mdeg_s) {
    char params[128];
    snprintf(params, sizeof(params),
             "{\"linear_mm_s\":%d,\"angular_mdeg_s\":%d}",
             linear_mm_s, angular_mdeg_s);
    return car_proxy_call("car.set_velocity", params);
}
