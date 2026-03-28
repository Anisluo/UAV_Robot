#include <cerrno>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <poll.h>
#include <string>

#include <fcntl.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

extern "C" {
#include "dev.h"
#include "log.h"
}

namespace {
constexpr const char *kTag = "proc_car";
constexpr const char *kSockPath = "/tmp/uav_proc_car.sock";
constexpr int kMaxClients = 8;
constexpr size_t kBufSize = 2048;
volatile sig_atomic_t g_running = 1;

void on_signal(int) {
    g_running = 0;
}

bool json_get_token(const char *json, const char *key, char *out, size_t out_size) {
    char needle[128];
    const char *p;
    size_t i = 0;
    std::snprintf(needle, sizeof(needle), "\"%s\"", key);
    p = std::strstr(json, needle);
    if (p == nullptr || out == nullptr || out_size == 0U) {
        return false;
    }
    p += std::strlen(needle);
    while (*p == ' ' || *p == '\t') {
        ++p;
    }
    if (*p != ':') {
        return false;
    }
    ++p;
    while (*p == ' ' || *p == '\t') {
        ++p;
    }
    if (*p == '"') {
        ++p;
        while (*p != '\0' && *p != '"' && i + 1U < out_size) {
            out[i++] = *p++;
        }
    } else {
        while (*p != '\0' && *p != ',' && *p != '}' && *p != '\n' && i + 1U < out_size) {
            out[i++] = *p++;
        }
    }
    out[i] = '\0';
    return i > 0U;
}

bool json_get_int(const char *json, const char *key, int *out) {
    char token[64];
    char *end = nullptr;
    long value;
    if (out == nullptr || !json_get_token(json, key, token, sizeof(token))) {
        return false;
    }
    value = std::strtol(token, &end, 10);
    if (end == token) {
        return false;
    }
    *out = static_cast<int>(value);
    return true;
}

std::string id_fragment(const char *line) {
    char id[64];
    if (!json_get_token(line, "id", id, sizeof(id)) || std::strcmp(id, "null") == 0) {
        return "\"id\":null";
    }
    return std::string("\"id\":") + id;
}

std::string result_ok(const std::string &id, bool ok) {
    return std::string("{\"jsonrpc\":\"2.0\",") + id + ",\"result\":{\"ok\":" + (ok ? "true" : "false") + "}}\n";
}

std::string error_resp(const std::string &id, int code, const char *msg) {
    return std::string("{\"jsonrpc\":\"2.0\",") + id +
           ",\"error\":{\"code\":" + std::to_string(code) + ",\"message\":\"" + msg + "\"}}\n";
}

std::string handle_request(const char *line) {
    char method[64] = {};
    std::string id = id_fragment(line);
    if (!json_get_token(line, "method", method, sizeof(method))) {
        return error_resp(id, -32600, "missing method");
    }
    if (std::strcmp(method, "system.ping") == 0) {
        return result_ok(id, true);
    }
    if (std::strcmp(method, "car.set_velocity") == 0) {
        int linear_mm_s = 0;
        int angular_mdeg_s = 0;
        (void)json_get_int(line, "linear_mm_s", &linear_mm_s);
        (void)json_get_int(line, "angular_mdeg_s", &angular_mdeg_s);
        return result_ok(id, car_set_velocity(linear_mm_s, angular_mdeg_s));
    }
    if (std::strcmp(method, "car.stop") == 0 || std::strcmp(method, "car.estop") == 0) {
        return result_ok(id, car_set_velocity(0, 0));
    }
    return error_resp(id, -32601, "method not found");
}
}

int main() {
    int server_fd;
    struct sockaddr_un addr{};
    struct pollfd pfds[1 + kMaxClients];
    int client_fds[kMaxClients];
    char bufs[kMaxClients][kBufSize];
    size_t buf_lens[kMaxClients];
    int nclients = 0;

    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);
    std::signal(SIGPIPE, SIG_IGN);

    const char *iface = std::getenv("UAV_CAR_CAN_IFACE");
    if (iface != nullptr && iface[0] != '\0') {
        (void)setenv("UAV_CAN_IFACE", iface, 1);
    }

    server_fd = socket(AF_UNIX, SOCK_STREAM, 0);
    if (server_fd < 0) {
        log_error(kTag, "socket failed: %s", std::strerror(errno));
        return 1;
    }

    unlink(kSockPath);
    addr.sun_family = AF_UNIX;
    std::strncpy(addr.sun_path, kSockPath, sizeof(addr.sun_path) - 1);
    if (bind(server_fd, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
        log_error(kTag, "bind failed: %s", std::strerror(errno));
        close(server_fd);
        return 1;
    }
    if (listen(server_fd, kMaxClients) < 0) {
        log_error(kTag, "listen failed: %s", std::strerror(errno));
        close(server_fd);
        return 1;
    }

    std::memset(buf_lens, 0, sizeof(buf_lens));
    for (int i = 0; i < kMaxClients; ++i) {
        client_fds[i] = -1;
    }
    log_info(kTag, "listening on %s can=%s", kSockPath, iface != nullptr && iface[0] != '\0' ? iface : "can3");

    while (g_running) {
        pfds[0].fd = server_fd;
        pfds[0].events = POLLIN;
        for (int i = 0; i < nclients; ++i) {
            pfds[1 + i].fd = client_fds[i];
            pfds[1 + i].events = POLLIN;
        }

        if (poll(pfds, 1 + nclients, 500) < 0) {
            if (errno == EINTR) {
                continue;
            }
            log_error(kTag, "poll failed: %s", std::strerror(errno));
            break;
        }

        if ((pfds[0].revents & POLLIN) != 0) {
            int client_fd = accept(server_fd, nullptr, nullptr);
            if (client_fd >= 0 && nclients < kMaxClients) {
                client_fds[nclients] = client_fd;
                buf_lens[nclients] = 0U;
                ++nclients;
            } else if (client_fd >= 0) {
                close(client_fd);
            }
        }

        for (int i = 0; i < nclients;) {
            const int idx = 1 + i;
            if ((pfds[idx].revents & (POLLIN | POLLHUP | POLLERR)) == 0) {
                ++i;
                continue;
            }
            ssize_t n = read(pfds[idx].fd, bufs[i] + buf_lens[i], kBufSize - buf_lens[i] - 1U);
            if (n <= 0) {
                close(pfds[idx].fd);
                if (i != nclients - 1) {
                    client_fds[i] = client_fds[nclients - 1];
                    buf_lens[i] = buf_lens[nclients - 1];
                    std::memcpy(bufs[i], bufs[nclients - 1], buf_lens[i]);
                }
                client_fds[nclients - 1] = -1;
                --nclients;
                continue;
            }

            buf_lens[i] += static_cast<size_t>(n);
            bufs[i][buf_lens[i]] = '\0';

            char *start = bufs[i];
            char *newline = nullptr;
            while ((newline = static_cast<char *>(std::memchr(start, '\n',
                                                               buf_lens[i] - static_cast<size_t>(start - bufs[i])))) != nullptr) {
                *newline = '\0';
                const std::string resp = handle_request(start);
                ssize_t ignored = write(pfds[idx].fd, resp.c_str(), resp.size());
                (void)ignored;
                start = newline + 1;
            }

            const size_t consumed = static_cast<size_t>(start - bufs[i]);
            buf_lens[i] -= consumed;
            if (consumed > 0U && buf_lens[i] > 0U) {
                std::memmove(bufs[i], start, buf_lens[i]);
            }
            ++i;
        }
    }

    (void)car_set_velocity(0, 0);
    close(server_fd);
    unlink(kSockPath);
    return 0;
}
