/* gripper_rpc_proxy.c
 *
 * Implements gripper_open() from dev.h as a JSON-RPC client that
 * forwards to proc_gripper via /tmp/uav_proc_gripper.sock.
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

#define TAG "core.dev.gripper.proxy"

static const char *gripper_sock_path(void) {
    const char *p = getenv("UAV_PROC_GRIPPER_SOCK");
    if (p != NULL && p[0] != '\0') return p;
    return "/tmp/uav_proc_gripper.sock";
}

static bool gripper_call(const char *method, const char *params_json) {
    static int s_id = 1;
    int id = ++s_id;
    int fd = socket(AF_UNIX, SOCK_STREAM, 0);
    if (fd < 0) { log_error(TAG, "socket: %s", strerror(errno)); return false; }

    struct timeval tv = {5, 0};
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

    struct sockaddr_un addr;
    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, gripper_sock_path(), sizeof(addr.sun_path) - 1);

    if (connect(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        log_error(TAG, "connect %s failed: %s", addr.sun_path, strerror(errno));
        close(fd);
        return false;
    }

    char req[512];
    int n = snprintf(req, sizeof(req),
                     "{\"jsonrpc\":\"2.0\",\"id\":%d,\"method\":\"%s\",\"params\":%s}\n",
                     id, method, params_json ? params_json : "{}");
    if (n <= 0 || write(fd, req, (size_t)n) != n) { close(fd); return false; }

    char buf[1024];
    size_t total = 0;
    while (total + 1 < sizeof(buf)) {
        ssize_t r = read(fd, buf + total, sizeof(buf) - 1 - total);
        if (r <= 0) break;
        total += (size_t)r;
        if (memchr(buf, '\n', total)) break;
    }
    buf[total] = '\0';
    close(fd);
    if (total == 0) return false;

    return strstr(buf, "\"ok\":true") != NULL
        || strstr(buf, "\"result\":true") != NULL;
}

bool gripper_open(bool open) {
    char params[64];
    snprintf(params, sizeof(params), "{\"open\":%s}", open ? "true" : "false");
    return gripper_call("gripper.set", params);
}
