/**
 * proc_gripper/src/main.cpp
 *
 * Gripper sub-process: JSON-RPC server over Unix SOCK_STREAM.
 * Socket  : /tmp/uav_proc_gripper.sock
 * Methods :
 *   gripper.set  { "open": true/false }  -> { "ok": true }
 *   system.ping                          -> { "ok": true }
 *
 * UART path read from env UAV_GRIPPER_UART_PATH, default /dev/ttyUSB0.
 * Each gripper operation opens a fresh UART fd and closes it afterwards.
 */

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cerrno>
#include <string>

#include <unistd.h>
#include <poll.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <fcntl.h>
#include <signal.h>

extern "C" {
#include "uart_posix.h"
#include "proto_gripper_uart.h"
#include "log.h"
}

static constexpr const char *TAG         = "proc_gripper";
static constexpr const char *SOCK_PATH   = "/tmp/uav_proc_gripper.sock";
static constexpr int         UART_BAUD   = 115200;
static constexpr size_t      BUF_SIZE    = 4096;
static constexpr int         MAX_CLIENTS = 8;

/* ------------------------------------------------------------------ */
/* Minimal JSON helpers (no external dependency)                       */
/* ------------------------------------------------------------------ */

/** Find the value string of a JSON key in a flat JSON object string.
 *  Returns true and fills val_out on success. */
static bool json_get_string(const char *json, const char *key, char *val_out,
                            size_t val_size) {
    // Search for "key":
    char needle[128];
    snprintf(needle, sizeof(needle), "\"%s\"", key);
    const char *p = strstr(json, needle);
    if (!p) return false;
    p += strlen(needle);
    while (*p == ' ' || *p == '\t') ++p;
    if (*p != ':') return false;
    ++p;
    while (*p == ' ' || *p == '\t') ++p;
    if (*p == '"') {
        ++p;
        size_t i = 0;
        while (*p && *p != '"' && i + 1 < val_size) val_out[i++] = *p++;
        val_out[i] = '\0';
        return true;
    }
    // non-string token (bool / number)
    size_t i = 0;
    while (*p && *p != ',' && *p != '}' && *p != '\n' && i + 1 < val_size)
        val_out[i++] = *p++;
    // trim trailing whitespace
    while (i > 0 && (val_out[i - 1] == ' ' || val_out[i - 1] == '\t')) --i;
    val_out[i] = '\0';
    return i > 0;
}

static bool json_get_bool(const char *json, const char *key, bool &out) {
    char v[16];
    if (!json_get_string(json, key, v, sizeof(v))) return false;
    if (strcmp(v, "true") == 0)  { out = true;  return true; }
    if (strcmp(v, "false") == 0) { out = false; return true; }
    return false;
}

/* ------------------------------------------------------------------ */
/* Gripper operation                                                    */
/* ------------------------------------------------------------------ */

static bool gripper_set(bool open_gripper, const char *uart_path) {
    int fd = uart_posix_open(uart_path, UART_BAUD);
    if (fd < 0) {
        log_error(TAG, "uart_posix_open(%s) failed: %s", uart_path,
                  strerror(errno));
        return false;
    }

    uint8_t buf[32];
    size_t  len = 0;
    if (!proto_gripper_uart_encode_open(open_gripper, buf, sizeof(buf), &len)) {
        log_error(TAG, "proto_gripper_uart_encode_open failed");
        uart_posix_close(fd);
        return false;
    }

    bool ok = uart_posix_write_all(fd, buf, len);
    uart_posix_close(fd);

    if (!ok)
        log_error(TAG, "uart_posix_write_all failed");
    else
        log_info(TAG, "gripper %s OK", open_gripper ? "open" : "close");

    return ok;
}

/* ------------------------------------------------------------------ */
/* JSON-RPC dispatch                                                    */
/* ------------------------------------------------------------------ */

static std::string handle_request(const char *line, const char *uart_path) {
    char method[64]  = {};
    char id_str[32]  = {};

    json_get_string(line, "method", method, sizeof(method));
    json_get_string(line, "id",     id_str, sizeof(id_str));

    // Build id fragment for response
    char id_frag[48];
    if (id_str[0] == '\0' || strcmp(id_str, "null") == 0)
        snprintf(id_frag, sizeof(id_frag), "\"id\":null");
    else
        snprintf(id_frag, sizeof(id_frag), "\"id\":%s", id_str);

    if (strcmp(method, "system.ping") == 0) {
        char resp[128];
        snprintf(resp, sizeof(resp),
                 "{\"jsonrpc\":\"2.0\",%s,\"result\":{\"ok\":true}}\n",
                 id_frag);
        return resp;
    }

    if (strcmp(method, "gripper.set") == 0) {
        bool open_val = false;
        const char *params_start = strstr(line, "\"params\"");
        if (!params_start || !json_get_bool(params_start, "open", open_val)) {
            char resp[192];
            snprintf(resp, sizeof(resp),
                     "{\"jsonrpc\":\"2.0\",%s,"
                     "\"error\":{\"code\":-32602,\"message\":\"missing params.open\"}}\n",
                     id_frag);
            return resp;
        }

        bool ok = gripper_set(open_val, uart_path);
        char resp[128];
        snprintf(resp, sizeof(resp),
                 "{\"jsonrpc\":\"2.0\",%s,\"result\":{\"ok\":%s}}\n",
                 id_frag, ok ? "true" : "false");
        return resp;
    }

    // Method not found
    char resp[192];
    snprintf(resp, sizeof(resp),
             "{\"jsonrpc\":\"2.0\",%s,"
             "\"error\":{\"code\":-32601,\"message\":\"method not found\"}}\n",
             id_frag);
    return resp;
}

/* ------------------------------------------------------------------ */
/* Main                                                                 */
/* ------------------------------------------------------------------ */

int main() {
    signal(SIGPIPE, SIG_IGN);

    const char *uart_path = getenv("UAV_GRIPPER_UART_PATH");
    if (!uart_path || uart_path[0] == '\0')
        uart_path = "/dev/ttyUSB0";

    log_info(TAG, "starting, uart=%s, sock=%s", uart_path, SOCK_PATH);

    // Create Unix domain socket
    int server_fd = socket(AF_UNIX, SOCK_STREAM, 0);
    if (server_fd < 0) {
        log_error(TAG, "socket: %s", strerror(errno));
        return 1;
    }

    unlink(SOCK_PATH);
    struct sockaddr_un addr{};
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, SOCK_PATH, sizeof(addr.sun_path) - 1);

    if (bind(server_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        log_error(TAG, "bind %s: %s", SOCK_PATH, strerror(errno));
        return 1;
    }
    if (listen(server_fd, MAX_CLIENTS) < 0) {
        log_error(TAG, "listen: %s", strerror(errno));
        return 1;
    }
    log_info(TAG, "listening on %s", SOCK_PATH);

    // Single-threaded poll loop
    struct pollfd pfds[1 + MAX_CLIENTS];
    char          bufs[MAX_CLIENTS][BUF_SIZE];
    size_t        buf_lens[MAX_CLIENTS];

    pfds[0].fd     = server_fd;
    pfds[0].events = POLLIN;
    int nclients   = 0;

    memset(buf_lens, 0, sizeof(buf_lens));

    while (true) {
        int nfds = 1 + nclients;
        int ret  = poll(pfds, nfds, -1);
        if (ret < 0) {
            if (errno == EINTR) continue;
            log_error(TAG, "poll: %s", strerror(errno));
            break;
        }

        // Accept new connections
        if (pfds[0].revents & POLLIN) {
            int cfd = accept(server_fd, nullptr, nullptr);
            if (cfd >= 0 && nclients < MAX_CLIENTS) {
                pfds[1 + nclients].fd     = cfd;
                pfds[1 + nclients].events = POLLIN;
                buf_lens[nclients]        = 0;
                ++nclients;
                log_info(TAG, "client connected (total %d)", nclients);
            } else if (cfd >= 0) {
                close(cfd); // too many clients
            }
        }

        // Service existing clients
        for (int i = 0; i < nclients; ) {
            int idx = 1 + i;
            if (!(pfds[idx].revents & (POLLIN | POLLHUP | POLLERR))) {
                ++i;
                continue;
            }

            ssize_t n = read(pfds[idx].fd,
                             bufs[i] + buf_lens[i],
                             BUF_SIZE - buf_lens[i] - 1);
            if (n <= 0) {
                // Client disconnected
                close(pfds[idx].fd);
                log_info(TAG, "client disconnected (total %d)", nclients - 1);
                // Compact arrays
                int last = nclients - 1;
                pfds[idx]    = pfds[1 + last];
                buf_lens[i]  = buf_lens[last];
                memcpy(bufs[i], bufs[last], buf_lens[last]);
                --nclients;
                continue; // re-check same slot
            }

            buf_lens[i] += (size_t)n;
            bufs[i][buf_lens[i]] = '\0';

            // Process all complete newline-delimited messages
            char *start = bufs[i];
            char *nl;
            while ((nl = (char *)memchr(start, '\n',
                                        buf_lens[i] - (size_t)(start - bufs[i]))) != nullptr) {
                *nl = '\0';
                std::string resp = handle_request(start, uart_path);
                write(pfds[idx].fd, resp.c_str(), resp.size());
                start = nl + 1;
            }

            // Move unconsumed bytes to front
            size_t consumed = (size_t)(start - bufs[i]);
            buf_lens[i] -= consumed;
            if (consumed > 0 && buf_lens[i] > 0)
                memmove(bufs[i], start, buf_lens[i]);

            ++i;
        }
    }

    close(server_fd);
    unlink(SOCK_PATH);
    return 0;
}
