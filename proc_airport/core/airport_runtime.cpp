#include "airport_runtime.h"

#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <poll.h>
#include <string>

#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

extern "C" {
#include "dev.h"
}

// can_socketcan.h and proto_platform_lock.h are compiled as C++ in this
// project (Makefile feeds the .c files through g++), so their symbols
// are C++-mangled. Don't wrap their includes in extern "C" — that would
// make these declarations C-linkage and break linking.
#include "can_socketcan.h"
#include "proto_platform_lock.h"

#include "log.h"

namespace {
constexpr const char *kTag = "proc_airport";
constexpr const char *kSockPath = "/tmp/uav_proc_airport.sock";
constexpr const char *kDefaultCanIface = "can1";
constexpr int kMaxClients = 8;
constexpr size_t kBufSize = 2048;

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

bool json_get_bool(const char *json, const char *key, bool *out) {
    char token[16];
    if (out == nullptr || !json_get_token(json, key, token, sizeof(token))) {
        return false;
    }
    if (std::strcmp(token, "true") == 0) {
        *out = true;
        return true;
    }
    if (std::strcmp(token, "false") == 0) {
        *out = false;
        return true;
    }
    return false;
}

std::string extract_id_fragment(const char *line) {
    char id[64];
    if (!json_get_token(line, "id", id, sizeof(id)) || std::strcmp(id, "null") == 0) {
        return "\"id\":null";
    }
    return std::string("\"id\":") + id;
}

std::string make_result(const std::string &id_fragment, bool ok) {
    return std::string("{\"jsonrpc\":\"2.0\",") + id_fragment +
           ",\"result\":{\"ok\":" + (ok ? "true" : "false") + "}}\n";
}

std::string make_lock_result(const std::string &id_fragment,
                             bool ok,
                             const char *reason,
                             int elapsed_ms) {
    std::string s = std::string("{\"jsonrpc\":\"2.0\",") + id_fragment +
                    ",\"result\":{\"ok\":" + (ok ? "true" : "false");
    if (reason != nullptr && reason[0] != '\0') {
        s += ",\"reason\":\"" + std::string(reason) + "\"";
    }
    s += ",\"elapsed_ms\":" + std::to_string(elapsed_ms);
    s += "}}\n";
    return s;
}

std::string make_error(const std::string &id_fragment, int code, const char *message) {
    return std::string("{\"jsonrpc\":\"2.0\",") + id_fragment +
           ",\"error\":{\"code\":" + std::to_string(code) +
           ",\"message\":\"" + message + "\"}}\n";
}

// ── Platform-lock stall detection ────────────────────────────────────────
//
// Architecture: after the host sends "enable + position(target_pulses)" to
// the ZDT-style lock motor, we listen for its replies on the same CAN bus
// and decide outcome:
//
//   - REACHED (status 0x9F) ........ motor finished cleanly → ok
//   - TIMEOUT (no REACHED in N ms) . motor stuck or unresponsive → stall
//   - PROGRESS-STALL ............... position hasn't advanced for K ms
//                                    while we still expect motion → stall
//
// On any stall we immediately push CMD_STOP (0xFE) so the motor stops
// fighting the obstacle (a drone parked wrong, debris, mechanical bind),
// then we return ok:false with a reason string back to HostGUI.
//
// All thresholds are env-tunable so the operator can dial them in without
// rebuilding:
//
//   UAV_PLATFORM_LOCK_TIMEOUT_MS    (default 5000)
//   UAV_PLATFORM_STALL_NO_PROG_MS   (default 600 — position-static window)
//   UAV_PLATFORM_POLL_INTERVAL_MS   (default 50 — read-position cadence)

namespace {

// ZDT Emm V5 cmd codes we use for the read-realtime path. These aren't
// in proto_platform_lock.h because that header only covers the encode/
// decode for enable/position/stop. The motor's reply CAN frame, when
// decoded as 'addr, cmd=0x36, status, b3, b2, b1, b0, checksum', carries
// the live pulse count in big-endian b3..b0.
constexpr uint8_t ZDT_CMD_READ_POSITION = 0x36U;
constexpr uint8_t ZDT_CMD_READ_CHECKSUM = 0x6BU;

int env_int_or(const char *name, int fallback) {
    const char *e = std::getenv(name);
    if (e == nullptr || e[0] == '\0') return fallback;
    char *end = nullptr;
    long v = std::strtol(e, &end, 10);
    if (end == e || *end != '\0') return fallback;
    return static_cast<int>(v);
}

int now_ms_steady() {
    using namespace std::chrono;
    return static_cast<int>(duration_cast<milliseconds>(
        steady_clock::now().time_since_epoch()).count());
}

// Send the "read realtime position" query to the lock motor. Reply,
// if the firmware understands the cmd, comes back on the next recv
// loop iteration. Unsupported firmware just stays silent — we fall
// back to the REACHED/timeout signal alone, no harm.
bool send_read_position(uint8_t addr) {
    const uint8_t payload[] = {addr, ZDT_CMD_READ_POSITION, ZDT_CMD_READ_CHECKSUM};
    return can_socketcan_send(static_cast<uint32_t>(addr) << 8U,
                              true,
                              payload,
                              sizeof(payload));
}

// Try to extract the live encoder count out of a frame the motor sent
// back. Returns true and writes *pos when the frame matches the read-
// position response format; otherwise returns false (caller treats the
// frame as "not relevant").
bool decode_read_position(const CanSocketFrame &f, uint8_t addr, int32_t *pos) {
    if (f.len < 7U) return false;
    if (f.data[0] != addr) return false;
    if (f.data[1] != ZDT_CMD_READ_POSITION) return false;
    if (f.data[f.len - 1U] != ZDT_CMD_READ_CHECKSUM) return false;
    // b3..b0 big-endian after [addr, cmd, sign?]
    uint32_t raw = (static_cast<uint32_t>(f.data[3]) << 24) |
                   (static_cast<uint32_t>(f.data[4]) << 16) |
                   (static_cast<uint32_t>(f.data[5]) <<  8) |
                   (static_cast<uint32_t>(f.data[6]));
    *pos = static_cast<int32_t>(raw);
    return true;
}

bool send_stop_to_lock(uint8_t addr) {
    PlatformLockCanBatch batch;
    if (!proto_platform_lock_encode_stop(addr, false, &batch)) {
        return false;
    }
    for (size_t i = 0; i < batch.count; ++i) {
        if (!can_socketcan_send(batch.frames[i].can_id,
                                batch.frames[i].is_extended_id,
                                batch.frames[i].data,
                                batch.frames[i].len)) {
            return false;
        }
    }
    return true;
}

// Returns:  ok=true  + reason=""              → REACHED cleanly
//           ok=false + reason="stall_static"  → position frozen mid-motion
//           ok=false + reason="stall_timeout" → no REACHED within window
//           ok=false + reason="cmd_rejected"  → motor rejected the cmd
// elapsed_ms is filled in either case.
struct WaitResult {
    bool ok;
    const char *reason;
    int elapsed_ms;
};

WaitResult wait_for_lock_reached(uint8_t addr,
                                 int timeout_ms,
                                 int poll_interval_ms,
                                 int no_progress_ms) {
    const int t0 = now_ms_steady();
    int last_progress_t = t0;
    int last_pos = INT32_MIN;          // sentinel for "no reading yet"
    int last_poll_t = 0;               // force first poll immediately
    CanSocketFrame f{};

    while (true) {
        int now = now_ms_steady();
        int elapsed = now - t0;
        if (elapsed >= timeout_ms) {
            return {false, "stall_timeout", elapsed};
        }

        // Drain whatever the motor has sent.
        while (can_socketcan_receive(&f)) {
            PlatformLockResponse rsp{};
            if (proto_platform_lock_decode(f.data, f.len, &rsp) && rsp.addr == addr) {
                if (rsp.type == PLATFORM_LOCK_RESP_POSITION_REACHED) {
                    return {true, "", elapsed};
                }
                if (rsp.type == PLATFORM_LOCK_RESP_ACK_REJECTED ||
                    rsp.type == PLATFORM_LOCK_RESP_ACK_INVALID) {
                    return {false, "cmd_rejected", elapsed};
                }
                // ACK_OK is just "we got the cmd" — keep waiting.
            } else {
                // Maybe it's a read-position reply — track encoder progress.
                int32_t pos = 0;
                if (decode_read_position(f, addr, &pos)) {
                    if (last_pos == INT32_MIN || std::abs(pos - last_pos) > 5) {
                        last_pos = pos;
                        last_progress_t = now;
                    } else if (now - last_progress_t > no_progress_ms) {
                        return {false, "stall_static", elapsed};
                    }
                }
            }
        }

        // Issue a "read position" query at the polling cadence so we
        // have data to drive the stall_static path. Idempotent if the
        // firmware doesn't support it.
        if (now - last_poll_t >= poll_interval_ms) {
            send_read_position(addr);
            last_poll_t = now;
        }

        // Cheap sleep — we don't need µs precision here.
        usleep(5000);
    }
}

}  // namespace

std::string handle_request(const char *line) {
    char method[64] = {};
    std::string id_fragment = extract_id_fragment(line);
    if (!json_get_token(line, "method", method, sizeof(method))) {
        return make_error(id_fragment, -32600, "missing method");
    }
    if (std::strcmp(method, "system.ping") == 0) {
        return make_result(id_fragment, true);
    }
    if (std::strcmp(method, "airport.lock") == 0 || std::strcmp(method, "platform.lock") == 0) {
        bool lock = false;
        if (!json_get_bool(line, "lock", &lock)) {
            return make_error(id_fragment, -32602, "missing lock");
        }
        // platform_lock() encodes + sends the enable/position cmds. It
        // returns false only on a CAN-layer send failure (interface
        // missing, bind error, etc.) — actual mechanical outcome comes
        // from the wait loop below.
        if (!platform_lock(lock)) {
            log_error(kTag, "platform_lock(%s) CAN send failed", lock ? "true" : "false");
            return make_error(id_fragment, -32000, "can send failed");
        }
        const int t0 = now_ms_steady();
        const uint8_t addr = static_cast<uint8_t>(
            env_int_or("UAV_PLATFORM_LOCK_ADDR", 4));
        const int timeout_ms = env_int_or("UAV_PLATFORM_LOCK_TIMEOUT_MS", 5000);
        const int poll_ms    = env_int_or("UAV_PLATFORM_POLL_INTERVAL_MS", 50);
        const int stall_ms   = env_int_or("UAV_PLATFORM_STALL_NO_PROG_MS", 600);

        WaitResult wr = wait_for_lock_reached(addr, timeout_ms, poll_ms, stall_ms);
        if (!wr.ok) {
            // Motor is stuck / unresponsive / rejecting → cut power to
            // it via STOP so it isn't grinding against an obstacle.
            (void)send_stop_to_lock(addr);
            log_warn(kTag,
                     "lock=%s stalled: reason=%s elapsed_ms=%d (sent STOP)",
                     lock ? "true" : "false",
                     wr.reason,
                     wr.elapsed_ms);
        } else {
            log_info(kTag,
                     "lock=%s reached in %d ms",
                     lock ? "true" : "false",
                     wr.elapsed_ms);
        }
        (void)t0;
        return make_lock_result(id_fragment, wr.ok, wr.reason, wr.elapsed_ms);
    }
    return make_error(id_fragment, -32601, "method not found");
}
}

int run_airport_runtime() {
    int server_fd;
    const char *airport_can_iface = std::getenv("UAV_AIRPORT_CAN_IFACE");
    struct sockaddr_un addr{};
    struct pollfd pfds[1 + kMaxClients];
    int client_fds[kMaxClients];
    char bufs[kMaxClients][kBufSize];
    size_t buf_lens[kMaxClients];
    int nclients = 0;

    std::signal(SIGPIPE, SIG_IGN);

    if (airport_can_iface == nullptr || airport_can_iface[0] == '\0') {
        airport_can_iface = kDefaultCanIface;
    }
    (void)setenv("UAV_CAN_IFACE", airport_can_iface, 1);

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
    log_info(kTag, "listening on %s using can=%s", kSockPath, airport_can_iface);

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
                // Initialise this pollfd slot now. poll() only wrote revents
                // for the indices it was handed, and this one was not among
                // them — it still holds this slot's values from an earlier
                // iteration (a different client's fd, and stale revents).
                // Without this the loop below reads that stale fd, treats
                // the failure as a disconnect, and drops the brand-new
                // client without ever closing it: an fd leak plus a caller
                // that hangs until its own timeout. Diagnosed in proc_door
                // on 2026-08-08, where it wedged the whole RPC chain.
                pfds[1 + nclients].fd      = client_fd;
                pfds[1 + nclients].events  = POLLIN;
                pfds[1 + nclients].revents = 0;
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
                const int last = nclients - 1;
                if (i != last) {
                    // Move the last client into this slot — pfds included,
                    // so the `continue` below re-examines the moved client
                    // with its own fd/revents rather than the closed one's
                    // (which would read a dead fd and evict a live client).
                    client_fds[i] = client_fds[last];
                    buf_lens[i] = buf_lens[last];
                    std::memcpy(bufs[i], bufs[last], buf_lens[i]);
                    pfds[idx] = pfds[1 + last];
                }
                client_fds[last] = -1;
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
