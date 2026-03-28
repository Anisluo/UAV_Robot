#include "arm_runtime.h"

#include <cerrno>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <poll.h>
#include <string>

#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

extern "C" {
#include "dev.h"
}

#include "log.h"

namespace {
constexpr const char *kTag = "proc_arm";
constexpr const char *kSockPath = "/tmp/uav_proc_arm.sock";
constexpr int kMaxClients = 8;
constexpr size_t kBufSize = 4096;

bool json_get_token(const char *json, const char *key, char *out, size_t out_size) {
    char needle[128];
    std::snprintf(needle, sizeof(needle), "\"%s\"", key);
    const char *p = std::strstr(json, needle);
    size_t i = 0;
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
        while (i > 0U && (out[i - 1U] == ' ' || out[i - 1U] == '\t')) {
            --i;
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

bool json_get_float(const char *json, const char *key, float *out) {
    char token[64];
    char *end = nullptr;
    float value;
    if (out == nullptr || !json_get_token(json, key, token, sizeof(token))) {
        return false;
    }
    value = std::strtof(token, &end);
    if (end == token) {
        return false;
    }
    *out = value;
    return true;
}

std::string json_escape(const char *text) {
    std::string out = "\"";
    for (const char *p = text; p != nullptr && *p != '\0'; ++p) {
        if (*p == '"' || *p == '\\') {
            out.push_back('\\');
        }
        out.push_back(*p);
    }
    out.push_back('"');
    return out;
}

std::string extract_id_fragment(const char *line) {
    char id[64];
    if (!json_get_token(line, "id", id, sizeof(id)) || std::strcmp(id, "null") == 0) {
        return "\"id\":null";
    }
    return std::string("\"id\":") + id;
}

std::string make_result(const std::string &id_fragment, const char *payload) {
    return std::string("{\"jsonrpc\":\"2.0\",") + id_fragment + ",\"result\":" + payload + "}\n";
}

std::string make_error(const std::string &id_fragment, int code, const char *message) {
    return std::string("{\"jsonrpc\":\"2.0\",") + id_fragment +
           ",\"error\":{\"code\":" + std::to_string(code) +
           ",\"message\":" + json_escape(message) + "}}\n";
}

std::string handle_request(const char *line) {
    char method[64] = {};
    std::string id_fragment = extract_id_fragment(line);

    if (!json_get_token(line, "method", method, sizeof(method))) {
        return make_error(id_fragment, -32600, "missing method");
    }

    if (std::strcmp(method, "system.ping") == 0) {
        return make_result(id_fragment, "{\"ok\":true}");
    }
    if (std::strcmp(method, "arm.home") == 0) {
        return make_result(id_fragment, arm_home() ? "{\"ok\":true}" : "{\"ok\":false}");
    }
    if (std::strcmp(method, "arm.stop") == 0) {
        return make_result(id_fragment, arm_stop() ? "{\"ok\":true}" : "{\"ok\":false}");
    }
    if (std::strcmp(method, "arm.move_pose") == 0) {
        char pose[64] = {};
        if (!json_get_token(line, "pose", pose, sizeof(pose))) {
            return make_error(id_fragment, -32602, "missing pose");
        }
        return make_result(id_fragment, arm_move(pose) ? "{\"ok\":true}" : "{\"ok\":false}");
    }
    if (std::strcmp(method, "arm.move_xyz") == 0) {
        float x_mm = 0.0F;
        float y_mm = 0.0F;
        float z_mm = 0.0F;
        if (!json_get_float(line, "x_mm", &x_mm) ||
            !json_get_float(line, "y_mm", &y_mm) ||
            !json_get_float(line, "z_mm", &z_mm)) {
            return make_error(id_fragment, -32602, "missing xyz");
        }
        return make_result(id_fragment,
                           arm_move_to_xyz(x_mm, y_mm, z_mm) ? "{\"ok\":true}" : "{\"ok\":false}");
    }
    if (std::strcmp(method, "arm.move_pose6d") == 0) {
        float x_mm = 0.0F;
        float y_mm = 0.0F;
        float z_mm = 0.0F;
        float roll_deg = 0.0F;
        float pitch_deg = 0.0F;
        float yaw_deg = 0.0F;
        if (!json_get_float(line, "x_mm", &x_mm) ||
            !json_get_float(line, "y_mm", &y_mm) ||
            !json_get_float(line, "z_mm", &z_mm) ||
            !json_get_float(line, "roll_deg", &roll_deg) ||
            !json_get_float(line, "pitch_deg", &pitch_deg) ||
            !json_get_float(line, "yaw_deg", &yaw_deg)) {
            return make_error(id_fragment, -32602, "missing pose6d");
        }
        return make_result(
            id_fragment,
            arm_move_to_pose6d(x_mm, y_mm, z_mm, roll_deg, pitch_deg, yaw_deg)
                ? "{\"ok\":true}"
                : "{\"ok\":false}");
    }
    if (std::strcmp(method, "arm.move_joint_deg") == 0) {
        int joint_index = 0;
        float target_deg = 0.0F;
        if (!json_get_int(line, "joint_index", &joint_index) ||
            !json_get_float(line, "target_deg", &target_deg)) {
            return make_error(id_fragment, -32602, "missing joint command");
        }
        return make_result(id_fragment,
                           arm_move_joint_deg(joint_index, target_deg)
                               ? "{\"ok\":true}"
                               : "{\"ok\":false}");
    }
    if (std::strcmp(method, "arm.move_joints_deg") == 0) {
        float joints_deg[6];
        const char *keys[6] = {"j1_deg", "j2_deg", "j3_deg", "j4_deg", "j5_deg", "j6_deg"};
        for (int i = 0; i < 6; ++i) {
            if (!json_get_float(line, keys[i], &joints_deg[i])) {
                return make_error(id_fragment, -32602, "missing joint set");
            }
        }
        return make_result(id_fragment,
                           arm_move_joints_deg(joints_deg) ? "{\"ok\":true}" : "{\"ok\":false}");
    }

    return make_error(id_fragment, -32601, "method not found");
}
}

int run_arm_runtime() {
    int server_fd;
    struct sockaddr_un addr{};
    struct pollfd pfds[1 + kMaxClients];
    int client_fds[kMaxClients];
    char bufs[kMaxClients][kBufSize];
    size_t buf_lens[kMaxClients];
    int nclients = 0;

    std::signal(SIGPIPE, SIG_IGN);

    server_fd = socket(AF_UNIX, SOCK_STREAM, 0);
    if (server_fd < 0) {
        log_error(kTag, "socket failed: %s", std::strerror(errno));
        return 1;
    }

    unlink(kSockPath);
    addr.sun_family = AF_UNIX;
    std::strncpy(addr.sun_path, kSockPath, sizeof(addr.sun_path) - 1);
    if (bind(server_fd, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
        log_error(kTag, "bind %s failed: %s", kSockPath, std::strerror(errno));
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
    log_info(kTag, "listening on %s", kSockPath);

    while (true) {
        pfds[0].fd = server_fd;
        pfds[0].events = POLLIN;

        for (int i = 0; i < nclients; ++i) {
            pfds[1 + i].fd = client_fds[i];
            pfds[1 + i].events = POLLIN;
        }

        if (poll(pfds, 1 + nclients, -1) < 0) {
            if (errno == EINTR) {
                continue;
            }
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

            {
                const size_t consumed = static_cast<size_t>(start - bufs[i]);
                buf_lens[i] -= consumed;
                if (consumed > 0U && buf_lens[i] > 0U) {
                    std::memmove(bufs[i], start, buf_lens[i]);
                }
            }
            ++i;
        }
    }

    close(server_fd);
    unlink(kSockPath);
    return 0;
}
