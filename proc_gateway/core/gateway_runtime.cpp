#include "gateway_runtime.h"

// proc_gateway  –  bridges proc_realsense shm ring → TCP
//   :7001  JSON-RPC (newline-delimited)
//   :7002  MJPEG stream  [4-byte BE size][JPEG bytes]

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cerrno>
#include <cctype>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <cmath>
#include <fstream>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <arpa/inet.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <netinet/in.h>
#include <net/if.h>
#include <netinet/tcp.h>
#include <netinet/udp.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include "abi/ipc_framing.h"
#include "abi/msg_types.h"
#include "ctrl_client.h"
#include "dev.h"
#include "jpeg_encoder.h"
#include "proto_gripper_uart.h"
#include "proto_zdt_arm.h"
#include "shm_reader.h"
#include "uart_posix.h"

// ─── NPU result receiver ─────────────────────────────────────────────────────
// Binds a Unix SOCK_DGRAM to UAV_NPU_RESULT_GW_PATH so proc_npu can push results.

// Two parallel detection sources can publish into the gateway:
//   battery_tracker.py (class_id == 200, always)  ← Python CV sidecar
//   proc_npu / mavic3_drone.rknn (class_id < 200) ← RKNN inference
// Naively keeping a single "latest" cache means the slower source's
// detections get clobbered by every packet from the faster one, so the
// HostGUI sees only one of them at a time. Maintain a slot per source
// (keyed by class_id == 200) and merge on `npu.get_detections`.
struct NpuResultStore {
    std::mutex             mu;
    UavCResult             battery{};        // class_id == 200 packets
    UavCResult             other{};          // everything else (drone / face / …)
    // Atomic timestamps so the get_detections handler can re-check
    // freshness *outside* the mutex critical section. With plain
    // uint64_t I observed the optimiser caching the loaded value of
    // until_ns across calls, returning empty even when drain had
    // refreshed the slot mere milliseconds earlier — release/acquire
    // semantics on these atomics fix the visibility.
    std::atomic<uint64_t>  battery_until_ns{0};
    std::atomic<uint64_t>  other_until_ns{0};
};

static NpuResultStore g_npu_result;

static uint64_t now_ns_steady() {
    using namespace std::chrono;
    return duration_cast<nanoseconds>(steady_clock::now().time_since_epoch()).count();
}

// Caches stay valid for this long after the last *non-empty* packet
// from each source. battery_tracker.py keeps its own 600 ms hold
// internally, so 1.5 s is comfortably longer than that without making
// stale boxes hang on screen for noticeable time.
static constexpr uint64_t kCacheValidNs = 1500ULL * 1000ULL * 1000ULL;

// ─── Video source selection (declared here so the RPC handler further
//     down can see it; implementation colorize_depth() lives near the
//     video-push loop). 0 = RGB (default), 1 = colorized depth.
enum : int { VIDEO_SRC_RGB = 0, VIDEO_SRC_DEPTH = 1 };
static std::atomic<int> g_video_source{VIDEO_SRC_RGB};

static int open_npu_result_socket() {
    ::unlink(UAV_NPU_RESULT_GW_PATH);
    int fd = ::socket(AF_UNIX, SOCK_DGRAM, 0);
    if (fd < 0) return -1;
    sockaddr_un addr{};
    addr.sun_family = AF_UNIX;
    std::strncpy(addr.sun_path, UAV_NPU_RESULT_GW_PATH,
                 sizeof(addr.sun_path) - 1);
    if (::bind(fd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
        ::close(fd); return -1;
    }
    // Allow non-root clients (face_tracker.py runs as ubuntu) to sendto.
    (void)::chmod(UAV_NPU_RESULT_GW_PATH, 0666);
    int flags = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, flags | O_NONBLOCK);
    return fd;
}

// Drain all pending datagrams from the npu result socket, route by class.
// battery_tracker.py emits a packet every frame (incl. empty when no
// battery), but an empty packet carries no class info — so we can't
// tell which source it came from. Skip empties: a source that goes
// quiet simply lets its cache age out via kCacheValidNs.
static void drain_npu_results(int fd) {
    if (fd < 0) return;
    UavCResult r{};
    ssize_t n;
    while ((n = ::recv(fd, &r, sizeof(r), MSG_DONTWAIT)) == static_cast<ssize_t>(sizeof(r))) {
        if (r.num_detections == 0) continue;
        const bool is_battery = (r.detections[0].class_id == 200);
        const uint64_t deadline = now_ns_steady() + kCacheValidNs;
        std::lock_guard<std::mutex> lk(g_npu_result.mu);
        if (is_battery) {
            g_npu_result.battery = r;
            g_npu_result.battery_until_ns.store(deadline, std::memory_order_release);
        } else {
            g_npu_result.other = r;
            g_npu_result.other_until_ns.store(deadline, std::memory_order_release);
        }
    }
}

// ─── proc_arm JSON-RPC forwarding (Unix stream socket) ───────────────────────
// proc_arm owns the CAN bus for the arm.  All arm commands from the gateway
// must be forwarded to proc_arm so they share one CAN socket, avoiding
// ACK-frame races when both processes are bound to the same interface.
static const char *proc_arm_sock_path() {
    // gen-2 routing: if UAV_ARM_BACKEND=piper, or proc_piper's socket
    // exists, point arm.* (and piper.*) forwards at it. Else fall back
    // to the legacy proc_arm socket. proc_piper's RPC accepts the same
    // arm.* method names, so callers don't notice.
    const char *forced = std::getenv("UAV_PROC_ARM_SOCK");
    if (forced && forced[0] != '\0') return forced;

    const char *piper_env = std::getenv("UAV_PROC_PIPER_SOCK");
    const char *piper_sock = (piper_env && piper_env[0] != '\0')
                                 ? piper_env
                                 : "/tmp/uav_proc_piper.sock";

    const char *backend = std::getenv("UAV_ARM_BACKEND");
    if (backend && std::strcmp(backend, "piper") == 0) return piper_sock;

    // Probe: if proc_piper's socket is reachable, use it.
    struct stat st{};
    if (stat(piper_sock, &st) == 0) return piper_sock;

    return "/tmp/uav_proc_arm.sock";
}

// Sends one JSON-RPC request to proc_arm and returns the parsed "ok" field.
// Returns false on socket error or if result.ok == false.
static bool proc_arm_call(const char *method, const char *params_json) {
    static int s_id = 1000;
    int id = ++s_id;

    int fd = ::socket(AF_UNIX, SOCK_STREAM, 0);
    if (fd < 0) return false;

    struct timeval tv{};
    tv.tv_sec  = 30;   // arm homing can take several seconds
    tv.tv_usec = 0;
    ::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    ::setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

    sockaddr_un addr{};
    addr.sun_family = AF_UNIX;
    std::strncpy(addr.sun_path, proc_arm_sock_path(), sizeof(addr.sun_path) - 1);

    if (::connect(fd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
        ::close(fd);
        return false;
    }

    char req[512];
    int req_len = std::snprintf(req, sizeof(req),
        "{\"jsonrpc\":\"2.0\",\"id\":%d,\"method\":\"%s\",\"params\":%s}\n",
        id, method, params_json);
    if (req_len <= 0 || ::write(fd, req, (size_t)req_len) != req_len) {
        ::close(fd);
        return false;
    }

    // Read until newline
    char buf[1024] = {};
    int total = 0;
    while (total < (int)sizeof(buf) - 1) {
        ssize_t n = ::read(fd, buf + total, sizeof(buf) - 1 - (size_t)total);
        if (n <= 0) break;
        total += (int)n;
        if (std::memchr(buf, '\n', (size_t)total)) break;
    }
    ::close(fd);
    if (total <= 0) return false;

    // Parse "ok" field: look for "ok":true
    return std::strstr(buf, "\"ok\":true") != nullptr
        || std::strstr(buf, "\"result\":true") != nullptr;
}

// Like proc_arm_call but returns the raw "result" value string instead of bool.
// Returns empty string on failure.
static std::string proc_arm_call_result(const char *method, const char *params_json) {
    static int s_id = 2000;
    int id = ++s_id;

    int fd = ::socket(AF_UNIX, SOCK_STREAM, 0);
    if (fd < 0) return {};

    struct timeval tv{};
    tv.tv_sec  = 5;
    tv.tv_usec = 0;
    ::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    ::setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

    sockaddr_un addr{};
    addr.sun_family = AF_UNIX;
    std::strncpy(addr.sun_path, proc_arm_sock_path(), sizeof(addr.sun_path) - 1);

    if (::connect(fd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
        ::close(fd);
        return {};
    }

    char req[512];
    int req_len = std::snprintf(req, sizeof(req),
        "{\"jsonrpc\":\"2.0\",\"id\":%d,\"method\":\"%s\",\"params\":%s}\n",
        id, method, params_json);
    if (req_len <= 0 || ::write(fd, req, (size_t)req_len) != req_len) {
        ::close(fd);
        return {};
    }

    char buf[2048] = {};
    int total = 0;
    while (total < (int)sizeof(buf) - 1) {
        ssize_t n = ::read(fd, buf + total, sizeof(buf) - 1 - (size_t)total);
        if (n <= 0) break;
        total += (int)n;
        if (std::memchr(buf, '\n', (size_t)total)) break;
    }
    ::close(fd);
    if (total <= 0) return {};

    // Extract the value after "result":
    const char *tag = "\"result\":";
    const char *pos = std::strstr(buf, tag);
    if (!pos) return {};
    return std::string(pos + std::strlen(tag));
}

// Transparent pass-through to the arm backend's Unix socket. Sends the
// full original request line and copies the reply back verbatim, so any
// arm.* / piper.* method that gateway doesn't handle explicitly reaches
// proc_piper (gen-2) or proc_arm (gen-1) without us having to learn its
// param/return schema first.
static bool proc_arm_passthrough(const char *original_line,
                                  char *resp, size_t resp_sz) {
    int fd = ::socket(AF_UNIX, SOCK_STREAM, 0);
    if (fd < 0) return false;
    struct timeval tv{}; tv.tv_sec = 5; tv.tv_usec = 0;
    ::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    ::setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
    sockaddr_un addr{}; addr.sun_family = AF_UNIX;
    std::strncpy(addr.sun_path, proc_arm_sock_path(), sizeof(addr.sun_path) - 1);
    if (::connect(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        ::close(fd);
        return false;
    }
    const size_t line_len = std::strlen(original_line);
    if (::write(fd, original_line, line_len) != (ssize_t)line_len) {
        ::close(fd); return false;
    }
    if (line_len == 0 || original_line[line_len - 1] != '\n') {
        ::write(fd, "\n", 1);
    }
    char buf[4096] = {};
    int total = 0;
    while (total < (int)sizeof(buf) - 1) {
        ssize_t n = ::read(fd, buf + total, sizeof(buf) - 1 - (size_t)total);
        if (n <= 0) break;
        total += (int)n;
        if (std::memchr(buf, '\n', (size_t)total)) break;
    }
    ::close(fd);
    if (total <= 0) return false;
    // Copy verbatim; proc_piper's reply id matches what the client sent
    // because we forwarded the original line unchanged.
    snprintf(resp, resp_sz, "%.*s", total, buf);
    return true;
}

// ─── proc_npu JSON-RPC forwarding (Unix stream socket) ─────────────────────
// The legacy binary ctrl channel (ctrl_c.send_cmd) never actually reached
// proc_npu's JSON-RPC SOCK_STREAM server, so strategy-switch commands from
// the HostGUI silently no-oped.  Forward them as real JSON-RPC instead, so
// proc_npu's npu.set_strategy / npu.start / npu.stop handlers — which
// toggle the Python-sidecar trigger flags — run as intended.
static const char *proc_npu_sock_path() {
    const char *e = std::getenv("UAV_PROC_NPU_SOCK");
    return (e && e[0] != '\0') ? e : "/tmp/uav_proc_npu.ctrl.sock";
}

static bool proc_npu_call(const char *method, const char *params_json) {
    static int s_id = 2000;
    int id = ++s_id;

    int fd = ::socket(AF_UNIX, SOCK_STREAM, 0);
    if (fd < 0) return false;

    struct timeval tv{};
    tv.tv_sec  = 3;
    tv.tv_usec = 0;
    ::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    ::setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

    sockaddr_un addr{};
    addr.sun_family = AF_UNIX;
    std::strncpy(addr.sun_path, proc_npu_sock_path(), sizeof(addr.sun_path) - 1);
    if (::connect(fd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
        ::close(fd);
        return false;
    }

    char req[256];
    int req_len = std::snprintf(req, sizeof(req),
        "{\"jsonrpc\":\"2.0\",\"id\":%d,\"method\":\"%s\",\"params\":%s}\n",
        id, method, params_json);
    if (req_len <= 0 || ::write(fd, req, (size_t)req_len) != req_len) {
        ::close(fd);
        return false;
    }

    char buf[512] = {};
    int total = 0;
    while (total < (int)sizeof(buf) - 1) {
        ssize_t n = ::read(fd, buf + total, sizeof(buf) - 1 - (size_t)total);
        if (n <= 0) break;
        total += (int)n;
        if (std::memchr(buf, '\n', (size_t)total)) break;
    }
    ::close(fd);
    return total > 0 && std::strstr(buf, "\"ok\":true") != nullptr;
}

// ─── proc_car JSON-RPC forwarding (Unix stream socket) ──────────────────────
// proc_car owns the CAN bus for the chassis motors.  All ugv commands from
// the gateway are forwarded to proc_car so there is a single CAN owner,
// same pattern as proc_arm.
static const char *proc_car_sock_path() {
    const char *e = std::getenv("UAV_PROC_CAR_SOCK");
    return (e && e[0] != '\0') ? e : "/tmp/uav_proc_car.sock";
}

static bool proc_car_call(const char *method, const char *params_json) {
    static int s_id = 4000;
    int id = ++s_id;

    int fd = ::socket(AF_UNIX, SOCK_STREAM, 0);
    if (fd < 0) return false;

    struct timeval tv{};
    tv.tv_sec  = 5;
    tv.tv_usec = 0;
    ::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    ::setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

    sockaddr_un addr{};
    addr.sun_family = AF_UNIX;
    std::strncpy(addr.sun_path, proc_car_sock_path(), sizeof(addr.sun_path) - 1);

    if (::connect(fd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
        ::close(fd);
        return false;
    }

    char req[512];
    int req_len = std::snprintf(req, sizeof(req),
        "{\"jsonrpc\":\"2.0\",\"id\":%d,\"method\":\"%s\",\"params\":%s}\n",
        id, method, params_json);
    if (req_len <= 0 || ::write(fd, req, (size_t)req_len) != req_len) {
        ::close(fd);
        return false;
    }

    char buf[1024] = {};
    int total = 0;
    while (total < (int)sizeof(buf) - 1) {
        ssize_t n = ::read(fd, buf + total, sizeof(buf) - 1 - (size_t)total);
        if (n <= 0) break;
        total += (int)n;
        if (std::memchr(buf, '\n', (size_t)total)) break;
    }
    ::close(fd);
    if (total <= 0) return false;

    return std::strstr(buf, "\"ok\":true") != nullptr
        || std::strstr(buf, "\"result\":true") != nullptr;
}

// ─── proc_grasp JSON-RPC forwarding (Unix stream socket) ─────────────────────
// proc_grasp exposes the same JSON-RPC framing as proc_arm / proc_npu at
// UAV_CTRL_PATH_D. We talk to it the same way we talk to proc_arm: open
// per-call, write one line, read one line, close.
static std::string proc_grasp_call_raw(const char *method, const char *params_json) {
    static int s_id = 7000;
    int id = ++s_id;

    int fd = ::socket(AF_UNIX, SOCK_STREAM, 0);
    if (fd < 0) return {};

    struct timeval tv{};
    tv.tv_sec  = 5;
    tv.tv_usec = 0;
    ::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    ::setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

    sockaddr_un addr{};
    addr.sun_family = AF_UNIX;
    std::strncpy(addr.sun_path, UAV_CTRL_PATH_D, sizeof(addr.sun_path) - 1);

    if (::connect(fd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
        ::close(fd);
        return {};
    }

    char req[512];
    int req_len = std::snprintf(req, sizeof(req),
        "{\"id\":%d,\"method\":\"%s\",\"params\":%s}\n",
        id, method, params_json);
    if (req_len <= 0 || ::write(fd, req, (size_t)req_len) != req_len) {
        ::close(fd);
        return {};
    }

    char buf[2048] = {};
    int total = 0;
    while (total < (int)sizeof(buf) - 1) {
        ssize_t n = ::read(fd, buf + total, sizeof(buf) - 1 - (size_t)total);
        if (n <= 0) break;
        total += (int)n;
        if (std::memchr(buf, '\n', (size_t)total)) break;
    }
    ::close(fd);
    if (total <= 0) return {};

    const char *tag = "\"result\":";
    const char *pos = std::strstr(buf, tag);
    if (!pos) return {};
    return std::string(pos + std::strlen(tag));
}

static bool proc_grasp_call_ok(const char *method, const char *params_json) {
    std::string r = proc_grasp_call_raw(method, params_json);
    return !r.empty() && r.find("\"ok\":true") != std::string::npos;
}

// ─── Task forwarding (UDP → uav_robotd --listen-port UAV_APP_CMD_PORT) ───────
static int open_task_udp_socket() {
    int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
    return fd;
}

static void send_task_cmd(int fd, const char *cmd) {
    if (fd < 0) return;
    sockaddr_in addr{};
    addr.sin_family      = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    addr.sin_port        = htons(UAV_APP_CMD_PORT);
    ::sendto(fd, cmd, std::strlen(cmd), 0,
             reinterpret_cast<const sockaddr *>(&addr), sizeof(addr));
}

// ─── Signal ──────────────────────────────────────────────────────────────────
static volatile sig_atomic_t g_running = 1;
static void on_sig(int) { g_running = 0; }

// ─── Time ────────────────────────────────────────────────────────────────────
static uint64_t now_ms() {
    using namespace std::chrono;
    return (uint64_t)duration_cast<milliseconds>(
               steady_clock::now().time_since_epoch()).count();
}

// ─── TCP helpers ─────────────────────────────────────────────────────────────
static int listen_on(uint16_t port) {
    int fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) return -1;
    int opt = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    int flags = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, flags | O_NONBLOCK);
    sockaddr_in a{};
    a.sin_family      = AF_INET;
    a.sin_addr.s_addr = INADDR_ANY;
    a.sin_port        = htons(port);
    if (bind(fd, (sockaddr*)&a, sizeof(a)) || listen(fd, 8)) {
        close(fd); return -1;
    }
    return fd;
}

static void set_nodelay(int fd) {
    int opt = 1;
    setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &opt, sizeof(opt));
}

// ─── Minimal JSON field extractors ───────────────────────────────────────────
// These handle the simple, flat JSON objects sent by HostGUI.

static int json_int(const char *s, const char *key, int def = -1) {
    char pat[80];
    snprintf(pat, sizeof(pat), "\"%s\"", key);
    const char *p = strstr(s, pat);
    if (!p) return def;
    p += strlen(pat);
    while (*p == ' ' || *p == '\t') ++p;
    if (*p != ':') return def;
    ++p;
    while (*p == ' ' || *p == '\t') ++p;
    char *end;
    long v = strtol(p, &end, 10);
    return (end != p) ? (int)v : def;
}

static std::string json_str(const char *s, const char *key) {
    char pat[80];
    snprintf(pat, sizeof(pat), "\"%s\"", key);
    const char *p = strstr(s, pat);
    if (!p) return {};
    p += strlen(pat);
    while (*p == ' ' || *p == '\t') ++p;
    if (*p != ':') return {};
    ++p;
    while (*p == ' ' || *p == '\t') ++p;
    if (*p != '"') return {};
    ++p;
    const char *e = strchr(p, '"');
    if (!e) return {};
    return std::string(p, e - p);
}

static double json_double(const char *s, const char *key, double def = 0.0) {
    char pat[80];
    snprintf(pat, sizeof(pat), "\"%s\"", key);
    const char *p = strstr(s, pat);
    if (!p) return def;
    p += strlen(pat);
    while (*p == ' ' || *p == '\t') ++p;
    if (*p != ':') return def;
    ++p;
    while (*p == ' ' || *p == '\t') ++p;
    char *end;
    double v = strtod(p, &end);
    return (end != p) ? v : def;
}

static bool json_bool(const char *s, const char *key, bool def = false) {
    char pat[80];
    snprintf(pat, sizeof(pat), "\"%s\"", key);
    const char *p = strstr(s, pat);
    if (!p) return def;
    p += strlen(pat);
    while (*p == ' ' || *p == '\t') ++p;
    if (*p != ':') return def;
    ++p;
    while (*p == ' ' || *p == '\t') ++p;
    if (strncmp(p, "true", 4) == 0) return true;
    if (strncmp(p, "false", 5) == 0) return false;
    if (*p == '1') return true;
    if (*p == '0') return false;
    return def;
}

static bool env_bool_value(const char *name, bool fallback) {
    const char *raw = getenv(name);
    if (raw == nullptr || raw[0] == '\0') return fallback;
    if (strcmp(raw, "1") == 0 || strcasecmp(raw, "true") == 0 || strcasecmp(raw, "yes") == 0) {
        return true;
    }
    if (strcmp(raw, "0") == 0 || strcasecmp(raw, "false") == 0 || strcasecmp(raw, "no") == 0) {
        return false;
    }
    return fallback;
}

static std::string env_string_value(const char *name, const char *fallback) {
    const char *raw = getenv(name);
    if (raw == nullptr || raw[0] == '\0') {
        return std::string(fallback != nullptr ? fallback : "");
    }
    return std::string(raw);
}

static std::string json_escape(const std::string &input) {
    std::string out;
    out.reserve(input.size() + 16);
    for (char ch : input) {
        switch (ch) {
            case '\\': out += "\\\\"; break;
            case '"': out += "\\\""; break;
            case '\n': out += "\\n"; break;
            case '\r': break;
            case '\t': out += "\\t"; break;
            default:
                if ((unsigned char)ch < 0x20U) out += ' ';
                else out += ch;
                break;
        }
    }
    return out;
}

// (External Python probe script path removed — angles are read from proc_arm directly)

static std::string read_text_file(const std::string &path) {
    std::ifstream ifs(path);
    if (!ifs.is_open()) return {};
    std::ostringstream oss;
    oss << ifs.rdbuf();
    return oss.str();
}

static std::string tail_lines_text(const std::string &text, int max_lines) {
    if (max_lines <= 0) return {};
    std::vector<std::string> lines;
    std::istringstream iss(text);
    std::string line;
    while (std::getline(iss, line)) {
        lines.push_back(line);
    }
    int start = (int)lines.size() > max_lines ? (int)lines.size() - max_lines : 0;
    std::ostringstream oss;
    for (int i = start; i < (int)lines.size(); ++i) {
        if (i > start) oss << '\n';
        oss << lines[(size_t)i];
    }
    return oss.str();
}

static bool video_stream_enabled_default() {
    return env_bool_value("UAV_VIDEO_STREAM_ENABLED", true);
}

static std::string task_status_path() {
    return env_string_value("UAV_TASK_STATUS_FILE", "/tmp/uav_task_status.json");
}

static std::string task_log_path() {
    return env_string_value("UAV_LOG_FILE", "/tmp/uav_robotd.log");
}

class ArmGripperSerial {
public:
    bool set_open(bool open) {
        static constexpr int kBaudrate = 115200;
        uint8_t cmd[16]{};
        size_t cmd_len = 0;
        if (!proto_gripper_uart_encode_open(open, cmd, sizeof(cmd), &cmd_len)) {
            std::fprintf(stderr,
                         "proc_gateway: gripper encode failed action=%s\n",
                         open ? "open" : "close");
            return false;
        }

        const std::string uart_path = env_string_value("UAV_GRIPPER_UART_PATH", "/dev/ttyUSB0");
        int fd = uart_posix_open(uart_path.c_str(), kBaudrate);
        if (fd < 0) {
            std::fprintf(stderr,
                         "proc_gateway: gripper open uart failed path=%s baud=%d\n",
                         uart_path.c_str(), kBaudrate);
            return false;
        }

        const bool ok = uart_posix_write_all(fd, cmd, cmd_len);
        uart_posix_close(fd);
        if (!ok) {
            std::fprintf(stderr,
                         "proc_gateway: gripper send failed path=%s action=%s\n",
                         uart_path.c_str(), open ? "open" : "close");
            return false;
        }

        std::fprintf(stderr,
                     "proc_gateway: gripper action=%s path=%s baud=%d bytes=%zu\n",
                     open ? "open" : "close",
                     uart_path.c_str(),
                     kBaudrate,
                     cmd_len);
        return true;
    }
};

// ─── Airport relay outputs (gpio-innohi sysfs path) ─────────────────────────
class AirportRelayBank {
public:
    bool set_channel(int channel, bool on) {
        const std::string relay_path = get_channel_path(channel);
        if (relay_path.empty()) {
            std::fprintf(stderr,
                         "proc_gateway: airport relay invalid channel=%d\n",
                         channel);
            return false;
        }

        return write_value(relay_path,
                           get_channel_active_high(channel),
                           on,
                           "airport relay",
                           channel);
    }

    bool set_gripper(bool open) {
        const std::string relay_path = get_gripper_path();
        if (relay_path.empty()) {
            std::fprintf(stderr, "proc_gateway: airport gripper relay path unavailable\n");
            return false;
        }

        return write_value(relay_path,
                           get_gripper_active_high(),
                           open,
                           "airport gripper relay",
                           get_gripper_channel());
    }

private:
    static bool path_exists(const std::string &path) {
        return !path.empty() && ::access(path.c_str(), F_OK) == 0;
    }

    static std::string gpio_sysfs_path(const std::string &name) {
        return "/sys/devices/platform/gpio-innohi/gpio_innohi/" + name + "/value";
    }

    static std::string relay_channel_sysfs_path(const std::string &suffix) {
        if (suffix.empty()) return {};

        const std::string gpio_name = "GPIO_" + suffix;
        const std::string gpio_path = gpio_sysfs_path(gpio_name);
        if (path_exists(gpio_path)) {
            return gpio_path;
        }

        const std::string io_name = "IO" + suffix;
        const std::string io_path = gpio_sysfs_path(io_name);
        if (path_exists(io_path)) {
            return io_path;
        }

        return gpio_path;
    }

    static std::string trim_ascii(const char *raw) {
        if (raw == nullptr) return std::string();
        const char *begin = raw;
        while (*begin != '\0' && std::isspace((unsigned char)*begin)) begin++;
        const char *end = raw + std::strlen(raw);
        while (end > begin && std::isspace((unsigned char)*(end - 1))) end--;
        return std::string(begin, (size_t)(end - begin));
    }

    static std::string normalize_relay_path(const char *raw) {
        std::string value = trim_ascii(raw);
        if (value.empty()) {
            return {};
        }

        std::string upper = value;
        for (char &ch : upper) {
            ch = (char)std::toupper((unsigned char)ch);
        }

        if (upper.rfind("GPIO_", 0) == 0) {
            const std::string suffix = upper.substr(5);
            if (suffix == "1" || suffix == "2" || suffix == "3" || suffix == "4") {
                return relay_channel_sysfs_path(suffix);
            }
        }

        if (upper.rfind("GPIO", 0) == 0) {
            const std::string suffix = upper.substr(4);
            if (suffix == "1" || suffix == "2" || suffix == "3" || suffix == "4") {
                return relay_channel_sysfs_path(suffix);
            }
        }

        if (upper.rfind("EXT_IO", 0) == 0) {
            const std::string suffix = upper.substr(6);
            if (suffix == "1" || suffix == "2" || suffix == "3" || suffix == "4") {
                return relay_channel_sysfs_path(suffix);
            }
        }

        if (upper.rfind("EXTIO", 0) == 0) {
            const std::string suffix = upper.substr(5);
            if (suffix == "1" || suffix == "2" || suffix == "3" || suffix == "4") {
                return relay_channel_sysfs_path(suffix);
            }
        }

        if (upper.rfind("IO_", 0) == 0) {
            const std::string suffix = upper.substr(3);
            if (suffix == "1" || suffix == "2" || suffix == "3" || suffix == "4") {
                return relay_channel_sysfs_path(suffix);
            }
        }

        if (upper.rfind("IO", 0) == 0) {
            const std::string suffix = upper.substr(2);
            if (suffix == "1" || suffix == "2" || suffix == "3" || suffix == "4") {
                return relay_channel_sysfs_path(suffix);
            }
        }

        return value;
    }

    static bool valid_channel(int channel) {
        return channel >= 1 && channel <= 4;
    }

    static bool write_value(const std::string &relay_path,
                            bool active_high,
                            bool on,
                            const char *label,
                            int channel) {
        const char value = ((on ? active_high : !active_high) ? '1' : '0');

        int fd = ::open(relay_path.c_str(), O_WRONLY | O_CLOEXEC);
        if (fd < 0) {
            std::fprintf(stderr,
                         "proc_gateway: %s channel=%d on=%d path=%s failed errno=%d (%s)\n",
                         label, channel, on ? 1 : 0, relay_path.c_str(), errno, std::strerror(errno));
            return false;
        }

        const ssize_t n = ::write(fd, &value, 1);
        ::close(fd);
        if (n != 1) {
            std::fprintf(stderr,
                         "proc_gateway: %s channel=%d on=%d path=%s short_write=%zd errno=%d (%s)\n",
                         label, channel, on ? 1 : 0, relay_path.c_str(), (ssize_t)n, errno, std::strerror(errno));
            return false;
        }

        std::fprintf(stderr,
                     "proc_gateway: %s channel=%d on=%d path=%s value=%c\n",
                     label, channel, on ? 1 : 0, relay_path.c_str(), value);
        return true;
    }

    static std::string channel_env_name(const char *prefix, int channel) {
        return std::string(prefix) + std::to_string(channel);
    }

    static std::string get_channel_path(int channel) {
        if (!valid_channel(channel)) return {};

        const std::string env_name = channel_env_name("UAV_AIRPORT_RELAY", channel) + "_PATH";
        const char *raw = getenv(env_name.c_str());
        if (raw != nullptr && raw[0] != '\0') {
            return normalize_relay_path(raw);
        }

        return relay_channel_sysfs_path(std::to_string(channel));
    }

    static bool get_channel_active_high(int channel) {
        if (!valid_channel(channel)) return true;

        const std::string env_name = channel_env_name("UAV_AIRPORT_RELAY", channel) + "_ACTIVE_HIGH";
        return env_bool_value(env_name.c_str(), true);
    }

    static std::string get_gripper_path() {
        const char *raw = getenv("UAV_AIRPORT_GRIPPER_RELAY_PATH");
        if (raw != nullptr && raw[0] != '\0') {
            return normalize_relay_path(raw);
        }
        return get_channel_path(get_gripper_channel());
    }

    static bool get_gripper_active_high() {
        const char *raw = getenv("UAV_AIRPORT_GRIPPER_ACTIVE_HIGH");
        if (raw != nullptr && raw[0] != '\0') {
            return env_bool_value("UAV_AIRPORT_GRIPPER_ACTIVE_HIGH", true);
        }
        return get_channel_active_high(get_gripper_channel());
    }

    static int get_gripper_channel() {
        const char *raw = getenv("UAV_AIRPORT_GRIPPER_RELAY_CHANNEL");
        if (raw != nullptr && raw[0] != '\0') {
            char *end = nullptr;
            long value = std::strtol(raw, &end, 10);
            if (end != raw && value >= 1 && value <= 4) {
                return (int)value;
            }
        }

        const char *legacy_path = getenv("UAV_AIRPORT_GRIPPER_RELAY_PATH");
        if (legacy_path != nullptr && legacy_path[0] != '\0') {
            const std::string normalized = normalize_relay_path(legacy_path);
            for (int channel = 1; channel <= 4; ++channel) {
                const std::string default_path = relay_channel_sysfs_path(std::to_string(channel));
                if (normalized == default_path) {
                    return channel;
                }
            }
        }

        return 4;
    }

public:
    bool can_drive_gripper() const {
        const std::string relay_path = get_gripper_path();
        return !relay_path.empty() && path_exists(relay_path);
    }
};

// ─── UGV CAN control (REMOVED) ──────────────────────────────────────────────
// Chassis CAN control has been moved to proc_car.  proc_gateway now forwards
// ugv.set_velocity / ugv.stop to /tmp/uav_proc_car.sock via proc_car_call().
// The ChassisCanController class below is kept commented out for reference.
#if 0
class ChassisCanController_REMOVED {
public:
    bool set_velocity(double vx_mps, double omega_rad_s) {
        if (!ensure_ready()) return false;

        const double mm_per_sec_per_rpm = get_env_double("UAV_CAR_MM_PER_SEC_PER_RPM", 5.0);
        const double track_width_mm     = get_env_double("UAV_CAR_TRACK_WIDTH_MM", 600.0);
        const int base_min_rpm          = clamp_int((int)std::lround(
                                          get_env_double("UAV_CAR_MIN_RPM", 60.0)), 0, 4000);
        double yaw_component_mm_s = omega_rad_s * (track_width_mm / 2.0);
        int left_rpm  = clamp_int((int)std::lround((vx_mps * 1000.0 - yaw_component_mm_s) / mm_per_sec_per_rpm), -4000, 4000);
        int right_rpm = clamp_int((int)std::lround((vx_mps * 1000.0 + yaw_component_mm_s) / mm_per_sec_per_rpm), -4000, 4000);

        const bool turning = (left_rpm != right_rpm);
        const int turn_min_rpm = clamp_int((int)std::lround(
                                   get_env_double("UAV_CAR_TURN_MIN_RPM",
                                                  static_cast<double>(base_min_rpm))),
                                   0,
                                   4000);
        const int min_command_rpm = turning ? turn_min_rpm : base_min_rpm;

        const double left_trim = get_env_double("UAV_CAR_LEFT_TRIM", 1.0);
        const double right_trim = get_env_double("UAV_CAR_RIGHT_TRIM", 1.0);
        left_rpm = clamp_int((int)std::lround((double)left_rpm * left_trim), -4000, 4000);
        right_rpm = clamp_int((int)std::lround((double)right_rpm * right_trim), -4000, 4000);

        left_rpm = apply_min_rpm(left_rpm, min_command_rpm);
        right_rpm = apply_min_rpm(right_rpm, min_command_rpm);

        if (last_valid_ && left_rpm == last_left_rpm_ && right_rpm == last_right_rpm_) {
            return true;
        }

        throttle_command(left_rpm, right_rpm);
        const double base_limit_pwm = get_env_double("UAV_CAR_LIMIT_PWM", 800.0);
        const double turn_limit_pwm = get_env_double("UAV_CAR_TURN_LIMIT_PWM", 1200.0);
        const int limit_pwm = clamp_int((int)std::lround(turning ? turn_limit_pwm : base_limit_pwm),
                                        0,
                                        5000);
        const int ramp_steps = clamp_int((int)std::lround(
                                 get_env_double("UAV_CAR_RAMP_STEPS", 4.0)),
                                 1,
                                 20);
        const int ramp_dt_ms = clamp_int((int)std::lround(
                                 get_env_double("UAV_CAR_RAMP_DT_MS", 50.0)),
                                 0,
                                 1000);

        if (!ramp_lr_rpm(last_valid_ ? last_left_rpm_ : 0,
                         last_valid_ ? last_right_rpm_ : 0,
                         left_rpm,
                         right_rpm,
                         limit_pwm,
                         ramp_steps,
                         ramp_dt_ms)) return false;

        last_left_rpm_ = left_rpm;
        last_right_rpm_ = right_rpm;
        last_valid_ = true;
        last_send_tp_ = std::chrono::steady_clock::now();
        return true;
    }

    bool stop() {
        return set_velocity(0.0, 0.0);
    }

private:
    int fd_{-1};
    bool ready_{false};
    bool last_valid_{false};
    int last_left_rpm_{0};
    int last_right_rpm_{0};
    std::chrono::steady_clock::time_point last_send_tp_{};

    static int clamp_int(int value, int min_value, int max_value) {
        if (value < min_value) return min_value;
        if (value > max_value) return max_value;
        return value;
    }

    static double get_env_double(const char *name, double fallback) {
        const char *raw = getenv(name);
        if (raw == nullptr || raw[0] == '\0') return fallback;
        char *end = nullptr;
        double value = strtod(raw, &end);
        return (end != raw) ? value : fallback;
    }

    static int apply_min_rpm(int rpm, int min_rpm) {
        if (rpm == 0 || min_rpm <= 0) return rpm;
        if (rpm > 0 && rpm < min_rpm) return min_rpm;
        if (rpm < 0 && rpm > -min_rpm) return -min_rpm;
        return rpm;
    }

    static void sleep_ms(int ms) {
        timespec ts{};
        ts.tv_sec = ms / 1000;
        ts.tv_nsec = (long)(ms % 1000) * 1000000L;
        nanosleep(&ts, &ts);
    }

    void throttle_command(int left_rpm, int right_rpm) {
        const int min_interval_ms = clamp_int(
            (int)std::lround(get_env_double("UAV_CAR_MIN_SEND_INTERVAL_MS", 120.0)),
            0,
            2000);
        if (min_interval_ms <= 0) return;

        // Let stop commands through immediately so the chassis can halt without delay.
        if (left_rpm == 0 && right_rpm == 0) return;

        const auto now = std::chrono::steady_clock::now();
        if (last_send_tp_.time_since_epoch().count() == 0) return;

        const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - last_send_tp_);
        if (elapsed.count() >= min_interval_ms) return;

        std::this_thread::sleep_for(std::chrono::milliseconds(min_interval_ms) - elapsed);
    }

    static uint16_t make_can_id(uint8_t motor_num, uint8_t cmd) {
        return (uint16_t)(((0U & 0x7U) << 8) | ((motor_num & 0xFU) << 4) | (cmd & 0xFU));
    }

    bool ensure_ready() {
        if (ready_) return true;

        const char *iface = getenv("UAV_CAR_CAN_IFACE");
        if (iface == nullptr || iface[0] == '\0') iface = "can3";

        fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (fd_ < 0) return false;

        ifreq ifr{};
        snprintf(ifr.ifr_name, sizeof(ifr.ifr_name), "%s", iface);
        if (ioctl(fd_, SIOCGIFINDEX, &ifr) < 0) return false;

        sockaddr_can addr{};
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;
        if (bind(fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) return false;

        for (uint8_t motor_num = 1; motor_num <= 6U; ++motor_num) {
            uint8_t reset[8];
            memset(reset, 0x55, sizeof(reset));
            if (!send_std(make_can_id(motor_num, 0x0U), reset, 8)) return false;
            sleep_ms(4);
        }
        sleep_ms(150);
        for (uint8_t motor_num = 1; motor_num <= 6U; ++motor_num) {
            uint8_t mode[8];
            memset(mode, 0x55, sizeof(mode));
            mode[0] = 0x03U;
            if (!send_std(make_can_id(motor_num, 0x1U), mode, 8)) return false;
            sleep_ms(4);
        }
        sleep_ms(150);

        ready_ = true;
        return true;
    }

    bool send_std(uint16_t can_id, const uint8_t *data, size_t len) {
        can_frame frame{};
        if (fd_ < 0 || data == nullptr || len > 8U) return false;
        frame.can_id = can_id & CAN_SFF_MASK;
        frame.can_dlc = static_cast<__u8>(len);
        memcpy(frame.data, data, len);
        return write(fd_, &frame, sizeof(frame)) == (ssize_t)sizeof(frame);
    }

    bool send_velocity(uint8_t motor_num, int rpm, int limit_pwm) {
        uint8_t payload[8];
        int actual_rpm = clamp_int(rpm, -4000, 4000);
        int pwm = clamp_int(limit_pwm, 0, 5000);

        memset(payload, 0x55, sizeof(payload));
        payload[0] = (uint8_t)((pwm >> 8) & 0xFF);
        payload[1] = (uint8_t)(pwm & 0xFF);
        payload[2] = (uint8_t)(((uint16_t)actual_rpm >> 8) & 0xFF);
        payload[3] = (uint8_t)((uint16_t)actual_rpm & 0xFF);
        return send_std(make_can_id(motor_num, 0x4U), payload, 8);
    }

    bool set_lr_rpm(int left_rpm, int right_rpm, int limit_pwm) {
        static const uint8_t left_motors[3] = {1, 5, 2};
        static const uint8_t right_motors[3] = {4, 6, 3};

        for (uint8_t motor_num : left_motors) {
            if (!send_velocity(motor_num, left_rpm, limit_pwm)) return false;
            sleep_ms(4);
        }
        for (uint8_t motor_num : right_motors) {
            if (!send_velocity(motor_num, -right_rpm, limit_pwm)) return false;
            sleep_ms(4);
        }
        return true;
    }

    bool ramp_lr_rpm(int from_left_rpm,
                     int from_right_rpm,
                     int to_left_rpm,
                     int to_right_rpm,
                     int limit_pwm,
                     int steps,
                     int step_dt_ms) {
        if (steps <= 1 || (from_left_rpm == to_left_rpm && from_right_rpm == to_right_rpm)) {
            return set_lr_rpm(to_left_rpm, to_right_rpm, limit_pwm);
        }

        for (int i = 1; i <= steps; ++i) {
            const int left_rpm = clamp_int(
                (int)std::lround(from_left_rpm +
                                 ((to_left_rpm - from_left_rpm) * (double)i / (double)steps)),
                -4000,
                4000);
            const int right_rpm = clamp_int(
                (int)std::lround(from_right_rpm +
                                 ((to_right_rpm - from_right_rpm) * (double)i / (double)steps)),
                -4000,
                4000);
            if (!set_lr_rpm(left_rpm, right_rpm, limit_pwm)) return false;
            if (i < steps && step_dt_ms > 0) sleep_ms(step_dt_ms);
        }
        return true;
    }
};

#endif // ChassisCanController_REMOVED

static AirportRelayBank g_airport_relays;
static ArmGripperSerial g_arm_gripper;

// ─── Airport rail control (ZDT / Emm V5.0 over CAN) ────────────────────────
class AirportRailController {
public:
    AirportRailController() {
        current_pos_mm_[0] = env_double("UAV_AIRPORT_RAIL1_INIT_MM", 0.0);
        current_pos_mm_[1] = env_double("UAV_AIRPORT_RAIL2_INIT_MM", 0.0);
        current_pos_mm_[2] = env_double("UAV_AIRPORT_RAIL3_INIT_MM", 0.0);
    }

    bool set_speed_rpm(int rail_index, int rpm) {
        if (rail_index < 0 || rail_index >= 3) return false;
        if (!ensure_ready()) return false;

        const uint8_t addr = rail_addr(rail_index);
        const int max_rpm = clamp_int(env_int("UAV_AIRPORT_RAIL_MAX_RPM", 1500), 1, 3000);
        const uint8_t acc = (uint8_t)clamp_int(env_int("UAV_AIRPORT_RAIL_SPEED_ACC", 10), 0, 255);
        const bool reverse = env_bool_for_rail(rail_index, "UAV_AIRPORT_RAIL_REVERSE", false);
        const int actual_rpm = clamp_int(std::abs(rpm), 0, max_rpm);
        const bool ccw = ((rpm < 0) ? !reverse : reverse);

        if (addr == 0) return false;
        if (!enable_if_needed(addr)) return false;

        ZdtArmCanBatch batch{};
        if (!proto_zdt_arm_encode_speed(addr, ccw, (uint16_t)actual_rpm, acc, false, &batch)) {
            return false;
        }
        if (!send_batch(batch)) {
            return false;
        }

        std::fprintf(stderr,
                     "proc_gateway: airport rail=%d addr=%u speed_rpm=%d iface=%s\n",
                     rail_index, (unsigned int)addr, rpm, iface_.c_str());

        // State transition: any new motion command moves the rail out of
        // IDLE / STALLED and into MOVING — the previous stall (if any) is
        // cleared the moment we commit to a fresh direction.
        rail_state_[rail_index].store(actual_rpm > 0 ? RAIL_MOVING : RAIL_IDLE);

        // Stall protection for rail 2 (the standalone 单独控制 rail).
        // Rails 0+2 are monitored by start_lock_pair's pair-monitor; rail
        // 1 (= UI "rail 2") has no such cover. Spawn one here whenever
        // the user kicks it into motion, so a mechanical jam stops the
        // motor instead of grinding into the obstruction. Bumping the
        // session also cancels any prior monitor — handles rapid 前进/
        // 后退 / stop cycling without double-monitoring.
        if (rail_index == 1) {
            const uint64_t session = rail2_motion_session_.fetch_add(1) + 1;
            if (actual_rpm > 0) {
                std::thread([this, session]() {
                    monitor_rail_until_stall(1, session);
                }).detach();
            }
        }
        return true;
    }

    bool start_lock_pair(bool forward, int rpm) {
        const int speed_rpm = clamp_int(std::abs(rpm), 0, clamp_int(env_int("UAV_AIRPORT_RAIL_MAX_RPM", 1500), 1, 3000));
        const uint64_t session = pair_motion_session_.fetch_add(1) + 1;

        (void)stop_rail(0);
        (void)stop_rail(2);
        if (!set_speed_rpm(0, forward ? speed_rpm : -speed_rpm)) return false;
        if (!set_speed_rpm(2, forward ? speed_rpm : -speed_rpm)) {
            (void)stop_rail(0);
            return false;
        }

        std::thread([this, session]() {
            monitor_pair_until_stall(session);
        }).detach();

        std::fprintf(stderr,
                     "proc_gateway: airport pair rails=1,3 mode=%s speed_rpm=%d iface=%s\n",
                     forward ? "lock" : "release", speed_rpm, iface_.c_str());
        return true;
    }

    bool move_absolute_mm(int rail_index, double pos_mm) {
        if (rail_index < 0 || rail_index >= 3) return false;
        if (!ensure_ready()) return false;

        const uint8_t addr = rail_addr(rail_index);
        const uint16_t rpm = (uint16_t)clamp_int(env_int("UAV_AIRPORT_RAIL_RPM", 200), 1, 3000);
        const uint8_t acc = (uint8_t)clamp_int(env_int("UAV_AIRPORT_RAIL_ACC", 20), 0, 255);
        const double pulses_per_mm = env_double_for_rail(rail_index, "UAV_AIRPORT_RAIL_PULSES_PER_MM", 100.0);
        const bool reverse = env_bool_for_rail(rail_index, "UAV_AIRPORT_RAIL_REVERSE", false);

        if (addr == 0 || pulses_per_mm <= 0.0) return false;

        const double clamped_mm = std::max(0.0, pos_mm);
        const double delta_mm = clamped_mm - current_pos_mm_[rail_index];
        const uint32_t pulses = (uint32_t)std::llround(std::fabs(delta_mm) * pulses_per_mm);
        if (pulses == 0U) {
            return true;
        }

        if (!enable_if_needed(addr)) return false;

        ZdtArmCanBatch batch{};
        if (!proto_zdt_arm_encode_position(addr,
                                           (delta_mm < 0.0) ? !reverse : reverse,
                                           rpm,
                                           acc,
                                           pulses,
                                           false,
                                           false,
                                           &batch)) {
            return false;
        }
        if (!send_batch(batch)) return false;
        current_pos_mm_[rail_index] = clamped_mm;

        std::fprintf(stderr,
                     "proc_gateway: airport rail=%d addr=%u pos_mm=%.1f delta_mm=%.1f pulses=%u iface=%s\n",
                     rail_index, (unsigned int)addr, clamped_mm, delta_mm, (unsigned int)pulses, iface_.c_str());
        return true;
    }

    bool move_relative_mm(int rail_index, double delta_mm) {
        if (rail_index < 0 || rail_index >= 3) return false;
        return move_absolute_mm(rail_index, std::max(0.0, current_pos_mm_[rail_index] + delta_mm));
    }

    // Open-loop fixed-distance move at a chosen speed. Used by stage
    // scripts in "distance" stop_mode — drive the rail N mm at speed_rpm,
    // motor self-stops on its internal pulse counter, GUI advances when
    // state goes IDLE. If the motor stalls before reaching distance,
    // monitor_distance_move catches that and transitions to STALLED so
    // GUI still advances.
    //   distance_mm: signed. positive=normal direction, negative=reverse.
    bool move_distance(int rail_index, double distance_mm, int speed_rpm) {
        if (rail_index < 0 || rail_index >= 3) return false;
        if (!ensure_ready()) return false;

        const uint8_t addr = rail_addr(rail_index);
        const int max_rpm = clamp_int(env_int("UAV_AIRPORT_RAIL_MAX_RPM", 1500), 1, 3000);
        const uint8_t acc = (uint8_t)clamp_int(env_int("UAV_AIRPORT_RAIL_SPEED_ACC", 10), 0, 255);
        const double pulses_per_mm = env_double_for_rail(rail_index, "UAV_AIRPORT_RAIL_PULSES_PER_MM", 100.0);
        const bool reverse = env_bool_for_rail(rail_index, "UAV_AIRPORT_RAIL_REVERSE", false);

        if (addr == 0 || pulses_per_mm <= 0.0) return false;
        if (!enable_if_needed(addr)) return false;

        const uint32_t pulses = (uint32_t)std::llround(std::fabs(distance_mm) * pulses_per_mm);
        if (pulses == 0U) return true;
        const uint16_t rpm = (uint16_t)clamp_int(std::abs(speed_rpm), 1, max_rpm);
        const bool ccw = (distance_mm < 0.0) ? !reverse : reverse;

        ZdtArmCanBatch batch{};
        if (!proto_zdt_arm_encode_position(addr, ccw, rpm, acc, pulses,
                                            /*absolute_mode=*/false,
                                            /*sync=*/false, &batch)) {
            return false;
        }
        if (!send_batch(batch)) return false;

        rail_state_[rail_index].store(RAIL_MOVING);
        // Bump rail-2 session if this is rail 1 — same cancel semantics
        // as set_speed_rpm so a subsequent user click cancels any active
        // distance-completion monitor.
        if (rail_index == 1) rail2_motion_session_.fetch_add(1);

        // Estimate motion duration as the safety cap: pulses / (rpm * 200 / 60)
        // assuming 200 microsteps/rev. We won't trust this to "complete" —
        // we trust the motor's status flag — but it bounds the monitor's
        // worst-case lifetime.
        const double pulses_per_s = double(rpm) * 200.0 / 60.0;
        const int est_ms = (pulses_per_s > 1.0)
            ? int(double(pulses) / pulses_per_s * 1000.0)
            : 5000;
        const int cap_ms = std::max(2000, est_ms + 2000);

        std::thread([this, rail_index, cap_ms]() {
            monitor_distance_completion(rail_index, cap_ms);
        }).detach();

        std::fprintf(stderr,
                     "proc_gateway: airport rail=%d addr=%u distance_mm=%.1f pulses=%u rpm=%u est=%dms\n",
                     rail_index, (unsigned int)addr, distance_mm,
                     (unsigned int)pulses, (unsigned int)rpm, est_ms);
        return true;
    }

    bool stop_rail(int rail_index) {
        if (rail_index < 0 || rail_index >= 3) return false;
        if (!ensure_ready()) return false;

        const uint8_t addr = rail_addr(rail_index);
        ZdtArmCanBatch batch{};
        if (!proto_zdt_arm_encode_stop(addr, false, &batch)) {
            return false;
        }
        if (!send_batch(batch)) {
            return false;
        }

        // Cancel any running rail-2 stall monitor so it doesn't keep
        // polling a stopped motor (and doesn't fire a redundant stop
        // a moment later).
        if (rail_index == 1) {
            rail2_motion_session_.fetch_add(1);
        }

        // Manual stop drops out of MOVING into IDLE. If the rail was
        // already STALLED, the monitor's own stop already transitioned
        // it (compare_exchange below) — don't clobber STALLED with IDLE
        // because HostGUI's poll loop is waiting to observe the STALLED
        // transition to advance the script step.
        int expected = RAIL_MOVING;
        rail_state_[rail_index].compare_exchange_strong(expected, RAIL_IDLE);

        std::fprintf(stderr, "proc_gateway: airport rail=%d addr=%u stop iface=%s\n",
                     rail_index, (unsigned int)addr, iface_.c_str());
        return true;
    }

    bool stop_all() {
        pair_motion_session_.fetch_add(1);
        bool ok = true;
        ok = stop_rail(0) && ok;
        ok = stop_rail(1) && ok;
        ok = stop_rail(2) && ok;
        return ok;
    }

private:
    int fd_{-1};
    bool ready_{false};
    std::string iface_;
    std::array<bool, 256> enabled_{};
    std::array<double, 3> current_pos_mm_{{0.0, 0.0, 0.0}};
    std::mutex io_mu_;
    std::atomic<uint64_t> pair_motion_session_{0};
    std::atomic<uint64_t> rail2_motion_session_{0};

    // Per-rail observable state — HostGUI polls this via airport.get_status
    // to advance script steps the instant the stall fires, instead of
    // waiting out the operator's max_ms upper bound. 0=IDLE, 1=MOVING,
    // 2=STALLED. set_speed flips IDLE/STALLED→MOVING; the stall monitors
    // flip MOVING→STALLED; stop_rail (manual) flips MOVING→IDLE.
    enum RailState : int { RAIL_IDLE = 0, RAIL_MOVING = 1, RAIL_STALLED = 2 };
    std::array<std::atomic<int>, 3> rail_state_{{
        std::atomic<int>{RAIL_IDLE},
        std::atomic<int>{RAIL_IDLE},
        std::atomic<int>{RAIL_IDLE}
    }};
public:
    int get_rail_state(int rail_index) const {
        if (rail_index < 0 || rail_index >= 3) return RAIL_IDLE;
        return rail_state_[rail_index].load();
    }
private:

    static int clamp_int(int value, int min_value, int max_value) {
        if (value < min_value) return min_value;
        if (value > max_value) return max_value;
        return value;
    }

    static void sleep_ms(int ms) {
        timespec ts{};
        ts.tv_sec = ms / 1000;
        ts.tv_nsec = (long)(ms % 1000) * 1000000L;
        nanosleep(&ts, &ts);
    }

    static int env_int(const char *name, int fallback) {
        const char *raw = getenv(name);
        if (raw == nullptr || raw[0] == '\0') return fallback;
        char *end = nullptr;
        long value = strtol(raw, &end, 10);
        return (end != raw) ? (int)value : fallback;
    }

    static double env_double(const char *name, double fallback) {
        const char *raw = getenv(name);
        if (raw == nullptr || raw[0] == '\0') return fallback;
        char *end = nullptr;
        double value = strtod(raw, &end);
        return (end != raw) ? value : fallback;
    }

    static bool env_bool(const char *name, bool fallback) {
        const char *raw = getenv(name);
        if (raw == nullptr || raw[0] == '\0') return fallback;
        if (strcmp(raw, "1") == 0 || strcasecmp(raw, "true") == 0 || strcasecmp(raw, "yes") == 0) {
            return true;
        }
        if (strcmp(raw, "0") == 0 || strcasecmp(raw, "false") == 0 || strcasecmp(raw, "no") == 0) {
            return false;
        }
        return fallback;
    }

    static double env_double_for_rail(int rail_index, const char *base_name, double fallback) {
        char specific[64];
        std::snprintf(specific, sizeof(specific), "UAV_AIRPORT_RAIL%d_PULSES_PER_MM", rail_index + 1);
        return env_double(specific, env_double(base_name, fallback));
    }

    static bool env_bool_for_rail(int rail_index, const char *base_name, bool fallback) {
        char specific[64];
        std::snprintf(specific, sizeof(specific), "UAV_AIRPORT_RAIL%d_REVERSE", rail_index + 1);
        return env_bool(specific, env_bool(base_name, fallback));
    }

    static uint8_t rail_addr(int rail_index) {
        static const char *kAddrVars[3] = {
            "UAV_AIRPORT_RAIL1_ADDR",
            "UAV_AIRPORT_RAIL2_ADDR",
            "UAV_AIRPORT_RAIL3_ADDR"
        };
        return (uint8_t)clamp_int(env_int(kAddrVars[rail_index], rail_index + 1), 0, 255);
    }

    bool ensure_ready() {
        if (ready_) return true;

        const char *iface = getenv("UAV_AIRPORT_CAN_IFACE");
        if (iface == nullptr || iface[0] == '\0') iface = "can3";
        iface_ = iface;

        fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (fd_ < 0) return false;

        ifreq ifr{};
        std::snprintf(ifr.ifr_name, sizeof(ifr.ifr_name), "%s", iface_.c_str());
        if (ioctl(fd_, SIOCGIFINDEX, &ifr) < 0) return false;

        sockaddr_can addr{};
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;
        if (bind(fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) return false;

        ready_ = true;
        return true;
    }

    bool send_frame(const ZdtArmCanFrame &src) {
        if (fd_ < 0 || src.len > 8U) return false;
        can_frame frame{};
        frame.can_id = src.is_extended_id ? ((canid_t)(src.can_id & CAN_EFF_MASK) | CAN_EFF_FLAG)
                                          : (canid_t)(src.can_id & CAN_SFF_MASK);
        frame.can_dlc = src.len;
        std::memcpy(frame.data, src.data, src.len);
        return write(fd_, &frame, sizeof(frame)) == (ssize_t)sizeof(frame);
    }

    bool send_batch(const ZdtArmCanBatch &batch) {
        std::lock_guard<std::mutex> lk(io_mu_);
        return send_batch_locked(batch);
    }

    bool send_batch_locked(const ZdtArmCanBatch &batch) {
        for (size_t i = 0; i < batch.count; ++i) {
            if (!send_frame(batch.frames[i])) return false;
            sleep_ms(4);
        }
        return true;
    }

    bool enable_if_needed(uint8_t addr) {
        if (enabled_[addr]) return true;
        ZdtArmCanBatch batch{};
        if (!proto_zdt_arm_encode_enable(addr, true, false, &batch)) return false;
        if (!send_batch(batch)) return false;
        enabled_[addr] = true;
        sleep_ms(20);
        return true;
    }

    // ZDT (Emm V5.0) drivers latch the stall-protection bit on overload:
    // status flag 0x04/0x08 stays set after motion halts, and the driver
    // silently REJECTS subsequent SPEED commands until ENABLE is toggled
    // off → wait → on. The first attempt at this (commit c61a503) only
    // sent DISABLE and let the next set_speed_rpm's enable_if_needed()
    // send ENABLE later. That wasn't enough — user reported still having
    // to hit 急停 to unstick. Strengthen to a full explicit cycle:
    //
    //   1. DISABLE                   (de-energize, clears stall latch)
    //   2. sleep 120 ms              (firmware needs >100ms to settle)
    //   3. ENABLE                    (re-arm before any motion arrives)
    //   4. sleep 50 ms               (driver internal init)
    //
    // After this the motor is in a clean enabled state, ready for the
    // next SPEED command without further setup. enabled_[addr] stays
    // true so enable_if_needed() short-circuits.
    void clear_stall_latch(uint8_t addr) {
        if (addr == 0U) return;
        // CRITICAL: hold io_mu_ across the entire DISABLE → sleep → ENABLE
        // → sleep window. Earlier version released the lock between each
        // send_batch() call, leaving a 170 ms window where a fast operator
        // click (e.g. "前进 stalls → click 后退 immediately") would have
        // set_speed_rpm sneak its SPEED frame onto the bus WHILE the motor
        // was disabled — the firmware silently dropped the SPEED, then the
        // monitor sent ENABLE which armed the motor but with nothing
        // queued, leaving the rail sitting until the next click. User
        // reported "间隔很久才真正后退" — that's this race.
        //
        // By holding the lock, any concurrent set_speed_rpm queues on
        // send_batch() internally and resumes only after we exit. The
        // motor is in a stable enabled state by then; the queued SPEED
        // takes effect immediately.
        std::lock_guard<std::mutex> lk(io_mu_);
        ZdtArmCanBatch batch{};
        if (proto_zdt_arm_encode_enable(addr, false, false, &batch)) {
            (void)send_batch_locked(batch);
        }
        sleep_ms(120);
        batch = {};
        if (proto_zdt_arm_encode_enable(addr, true, false, &batch)) {
            (void)send_batch_locked(batch);
        }
        sleep_ms(50);
        enabled_[addr] = true;
        std::fprintf(stderr,
                     "proc_gateway: airport addr=%u stall latch cleared (locked DISABLE→ENABLE cycle)\n",
                     (unsigned int)addr);
    }

    void monitor_pair_until_stall(uint64_t session) {
        const int poll_ms       = clamp_int(env_int("UAV_AIRPORT_LOCK_POLL_MS",      120), 20, 1000);
        const int confirm_hits  = clamp_int(env_int("UAV_AIRPORT_LOCK_CONFIRM_HITS",   2),  1,   10);
        // Hard cap on how long the pair is allowed to move. If we never
        // see a confirmed stall within this window the operation is
        // forcibly aborted — protects against silent CAN failure where
        // read_status_flags returns false forever and the motors keep
        // grinding into whatever's in the way.
        const int max_duration  = clamp_int(env_int("UAV_AIRPORT_LOCK_MAX_MS",      6000), 500, 60000);
        // If status reads fail this many times in a row for a given
        // rail, treat the silence itself as a stall — the motor is
        // either unplugged or its firmware locked up under load, and
        // neither case is one where we want it to keep trying.
        const int max_read_fail = clamp_int(env_int("UAV_AIRPORT_LOCK_READ_FAIL_HITS", 8),  1,   100);

        const int rails[2]  = {0, 2};
        bool stopped[2]     = {false, false};
        int  stall_hits[2]  = {0, 0};
        int  read_fails[2]  = {0, 0};

        const uint64_t t0 = now_ms();

        while (g_running && pair_motion_session_.load() == session) {
            // Hard timeout — slam both rails to a stop and bail.
            if ((int64_t)(now_ms() - t0) >= max_duration) {
                for (int i = 0; i < 2; ++i) {
                    if (!stopped[i]) {
                        std::fprintf(stderr,
                            "proc_gateway: airport lock max-duration %dms hit, force-stop rail %d\n",
                            max_duration, rails[i]);
                        rail_state_[rails[i]].store(RAIL_STALLED);
                        (void)stop_rail(rails[i]);
                        clear_stall_latch(rail_addr(rails[i]));
                        stopped[i] = true;
                    }
                }
                break;
            }

            bool all_stopped = true;
            for (int i = 0; i < 2; ++i) {
                if (stopped[i]) continue;
                all_stopped = false;

                uint8_t flags = 0;
                if (!read_status_flags(rail_addr(rails[i]), &flags)) {
                    // Repeated read failures imply CAN silence — treat
                    // as worst-case (stall) so the motor doesn't keep
                    // fighting whatever is jamming it.
                    if (++read_fails[i] >= max_read_fail) {
                        std::fprintf(stderr,
                            "proc_gateway: airport lock rail=%d %d consecutive status reads failed → force-stop\n",
                            rails[i], read_fails[i]);
                        rail_state_[rails[i]].store(RAIL_STALLED);
                        (void)stop_rail(rails[i]);
                        clear_stall_latch(rail_addr(rails[i]));
                        stopped[i] = true;
                    }
                    continue;
                }
                read_fails[i] = 0;

                const bool stalled = (flags & 0x04U) != 0U || (flags & 0x08U) != 0U;
                if (stalled) {
                    stall_hits[i] += 1;
                    if (stall_hits[i] >= confirm_hits) {
                        std::fprintf(stderr,
                            "proc_gateway: airport lock rail=%d stall flags=0x%02X → stop\n",
                            rails[i], flags);
                        rail_state_[rails[i]].store(RAIL_STALLED);
                        (void)stop_rail(rails[i]);
                        clear_stall_latch(rail_addr(rails[i]));
                        stopped[i] = true;
                    }
                } else {
                    stall_hits[i] = 0;
                }
            }

            if (all_stopped || (stopped[0] && stopped[1])) {
                break;
            }
            sleep_ms(poll_ms);
        }
    }

    // Single-rail variant of monitor_pair_until_stall, used by rail 2
    // (the 单独控制 rail). Same stall detection (status bits 0x04/0x08),
    // same "CAN silence == stall" fallback, same hard duration cap.
    // Defaults are independent so they can be tuned per rail without
    // affecting the pair-lock timings.
    void monitor_rail_until_stall(int rail_index, uint64_t session) {
        const int poll_ms       = clamp_int(env_int("UAV_AIRPORT_RAIL2_POLL_MS",      120), 20, 1000);
        const int confirm_hits  = clamp_int(env_int("UAV_AIRPORT_RAIL2_CONFIRM_HITS",   2),  1,   10);
        // Rail 2 can sweep further than the lock pair, so allow a longer
        // run before the safety cap kicks in. Override via env if the
        // physical travel time exceeds this.
        const int max_duration  = clamp_int(env_int("UAV_AIRPORT_RAIL2_MAX_MS",     30000), 500, 600000);
        const int max_read_fail = clamp_int(env_int("UAV_AIRPORT_RAIL2_READ_FAIL_HITS", 8),  1,   100);

        const uint8_t addr = rail_addr(rail_index);
        if (addr == 0U) return;

        int stall_hits = 0;
        int read_fails = 0;
        const uint64_t t0 = now_ms();

        while (g_running && rail2_motion_session_.load() == session) {
            if ((int64_t)(now_ms() - t0) >= max_duration) {
                std::fprintf(stderr,
                    "proc_gateway: airport rail=%d max-duration %dms hit, force-stop\n",
                    rail_index, max_duration);
                rail_state_[rail_index].store(RAIL_STALLED);
                (void)stop_rail(rail_index);
                clear_stall_latch(addr);
                break;
            }

            uint8_t flags = 0;
            if (!read_status_flags(addr, &flags)) {
                if (++read_fails >= max_read_fail) {
                    std::fprintf(stderr,
                        "proc_gateway: airport rail=%d %d consecutive status reads failed → force-stop\n",
                        rail_index, read_fails);
                    rail_state_[rail_index].store(RAIL_STALLED);
                    (void)stop_rail(rail_index);
                    clear_stall_latch(addr);
                    break;
                }
                sleep_ms(poll_ms);
                continue;
            }
            read_fails = 0;

            const bool stalled = (flags & 0x04U) != 0U || (flags & 0x08U) != 0U;
            if (stalled) {
                stall_hits += 1;
                if (stall_hits >= confirm_hits) {
                    std::fprintf(stderr,
                        "proc_gateway: airport rail=%d stall flags=0x%02X → stop\n",
                        rail_index, flags);
                    rail_state_[rail_index].store(RAIL_STALLED);
                    (void)stop_rail(rail_index);
                    clear_stall_latch(addr);
                    break;
                }
            } else {
                stall_hits = 0;
            }

            sleep_ms(poll_ms);
        }
    }

    // Watches a position-mode (move_distance) run until the motor's
    // status flag reports "reached" (bit 0x02) OR stalls (0x04/0x08) OR
    // we hit cap_ms (safety). Transitions rail_state_ accordingly so
    // HostGUI's poll loop can advance the script step.
    void monitor_distance_completion(int rail_index, int cap_ms) {
        if (rail_index < 0 || rail_index >= 3) return;
        const uint8_t addr = rail_addr(rail_index);
        if (addr == 0U) return;
        constexpr int poll_ms = 80;
        constexpr int reach_confirm = 2;     // 2 consecutive 0x02 hits
        constexpr int max_read_fail = 12;
        int reached_hits = 0;
        int stall_hits   = 0;
        int read_fails   = 0;
        const uint64_t t0 = now_ms();
        while (g_running) {
            if ((int64_t)(now_ms() - t0) >= cap_ms) {
                std::fprintf(stderr,
                    "proc_gateway: airport rail=%d distance monitor cap %dms hit, force-stop\n",
                    rail_index, cap_ms);
                rail_state_[rail_index].store(RAIL_STALLED);
                (void)stop_rail(rail_index);
                clear_stall_latch(addr);
                return;
            }
            uint8_t flags = 0;
            if (!read_status_flags(addr, &flags)) {
                if (++read_fails >= max_read_fail) {
                    std::fprintf(stderr,
                        "proc_gateway: airport rail=%d distance monitor %d CAN reads failed → stall\n",
                        rail_index, read_fails);
                    rail_state_[rail_index].store(RAIL_STALLED);
                    (void)stop_rail(rail_index);
                    clear_stall_latch(addr);
                    return;
                }
                sleep_ms(poll_ms);
                continue;
            }
            read_fails = 0;

            const bool stalled = (flags & 0x04U) != 0U || (flags & 0x08U) != 0U;
            if (stalled) {
                if (++stall_hits >= 2) {
                    std::fprintf(stderr,
                        "proc_gateway: airport rail=%d distance run stalled early flags=0x%02X\n",
                        rail_index, flags);
                    rail_state_[rail_index].store(RAIL_STALLED);
                    (void)stop_rail(rail_index);
                    clear_stall_latch(addr);
                    return;
                }
            } else {
                stall_hits = 0;
            }

            const bool reached = (flags & 0x02U) != 0U;
            if (reached) {
                if (++reached_hits >= reach_confirm) {
                    std::fprintf(stderr,
                        "proc_gateway: airport rail=%d distance run reached target flags=0x%02X\n",
                        rail_index, flags);
                    rail_state_[rail_index].store(RAIL_IDLE);
                    return;
                }
            } else {
                reached_hits = 0;
            }

            sleep_ms(poll_ms);
        }
    }

    bool read_status_flags(uint8_t addr, uint8_t *out_flags) {
        if (out_flags == nullptr || addr == 0U) return false;
        if (!ensure_ready()) return false;

        std::lock_guard<std::mutex> lk(io_mu_);
        drain_rx_locked();

        can_frame query{};
        query.can_id = ((canid_t)(((uint32_t)addr << 8) | 0U) & CAN_EFF_MASK) | CAN_EFF_FLAG;
        query.can_dlc = 2;
        query.data[0] = 0x3AU;
        query.data[1] = 0x6BU;
        if (write(fd_, &query, sizeof(query)) != (ssize_t)sizeof(query)) {
            return false;
        }

        const uint64_t deadline = now_ms() + (uint64_t)clamp_int(env_int("UAV_AIRPORT_STATUS_TIMEOUT_MS", 150), 20, 2000);
        while (now_ms() < deadline) {
            pollfd pfd{};
            pfd.fd = fd_;
            pfd.events = POLLIN;
            const int timeout = (int)std::max<int64_t>(1, (int64_t)(deadline - now_ms()));
            int rc = poll(&pfd, 1, timeout);
            if (rc <= 0) break;
            if ((pfd.revents & POLLIN) == 0) continue;

            can_frame frame{};
            ssize_t n = recv(fd_, &frame, sizeof(frame), 0);
            if (n != (ssize_t)sizeof(frame)) continue;
            if (frame.can_dlc < 3) continue;

            const uint32_t raw_id = frame.can_id & CAN_EFF_MASK;
            const uint8_t resp_addr = (uint8_t)((raw_id >> 8) & 0xFFU);
            if (resp_addr != addr) continue;

            if (frame.data[frame.can_dlc - 1] != 0x6BU) continue;
            if (frame.data[0] == 0x3AU) {
                *out_flags = frame.data[1];
                return true;
            }
            if (frame.can_dlc >= 4 && frame.data[1] == 0x3AU && frame.data[0] == addr) {
                *out_flags = frame.data[2];
                return true;
            }
        }
        return false;
    }

    void drain_rx_locked() {
        while (true) {
            can_frame frame{};
            ssize_t n = recv(fd_, &frame, sizeof(frame), MSG_DONTWAIT);
            if (n < 0) {
                if (errno == EAGAIN || errno == EWOULDBLOCK) return;
                return;
            }
            if (n == 0) return;
        }
    }
};

static AirportRailController g_airport;

// ─── RPC connection state ─────────────────────────────────────────────────────
struct RpcConn {
    int         fd;
    std::string buf;
};

// Blocking write (RPC responses are tiny ≤ 256 bytes; EINTR/EAGAIN loop is fine)
static bool write_all(int fd, const char *data, size_t len) {
    while (len > 0) {
        ssize_t n = write(fd, data, len);
        if (n <= 0) {
            if (errno == EINTR || errno == EAGAIN) continue;
            return false;
        }
        data += n;
        len  -= n;
    }
    return true;
}

// ─── Build JSON for a single detection ───────────────────────────────────────
static int fmt_detection(char *buf, int bufsz, const UavDetection &d) {
    return snprintf(buf, (size_t)bufsz,
        "{\"class_id\":%d,\"score\":%.4f,"
        "\"x1\":%.1f,\"y1\":%.1f,\"x2\":%.1f,\"y2\":%.1f,"
        "\"x_mm\":%.2f,\"y_mm\":%.2f,\"z_mm\":%.2f,"
        "\"roll_deg\":%.2f,\"pitch_deg\":%.2f,\"yaw_deg\":%.2f,"
        "\"has_xyz\":%d,\"has_rpy\":%d,\"grasp_mode\":%u}",
        d.class_id, d.score,
        d.x1, d.y1, d.x2, d.y2,
        d.x_mm, d.y_mm, d.z_mm,
        d.roll_deg, d.pitch_deg, d.yaw_deg,
        (int)d.has_xyz, (int)d.has_rpy, (unsigned int)d.grasp_mode);
}

// ─── Handle one JSON-RPC request line ────────────────────────────────────────
static void handle_rpc(int fd, const std::string &line,
                        CtrlClient &ctrl_b, CtrlClient &ctrl_c,
                        int task_udp_fd,
                        uint64_t start_ms,
                        bool *video_stream_enabled,
                        std::vector<int> *vid_clients) {
    const char *s  = line.c_str();
    int         id = json_int(s, "id", 0);
    std::string method = json_str(s, "method");

    char resp[8192];

    // Per-RPC trace logs — fine for one-off diagnostics, but at
    // HostGUI's 30 Hz poll rate (npu.get_detections + arm.get_angles
    // alternating, each response ~700 chars) journald falls behind
    // and the gateway main loop stalls, dragging the JPEG video push
    // from ~30 fps to <1 fps. Gate behind UAV_GATEWAY_RPC_LOG so a
    // dev can flip it on when needed (`UAV_GATEWAY_RPC_LOG=1` in
    // /etc/default/uav_robot) without paying the cost in normal use.
    static const bool s_rpc_trace = []() {
        const char *e = std::getenv("UAV_GATEWAY_RPC_LOG");
        return e != nullptr && e[0] != '\0' && e[0] != '0';
    }();
    if (s_rpc_trace) {
        fprintf(stderr, "proc_gateway: RPC fd=%d recv: %s\n", fd, line.c_str());
    }

    if (method == "system.ping") {
        uint64_t uptime = now_ms() - start_ms;
        snprintf(resp, sizeof(resp),
                 "{\"id\":%d,\"result\":{\"pong\":true,\"uptime_ms\":%llu}}\n",
                 id, (unsigned long long)uptime);

    } else if (method == "system.get_backend") {
        // Reports which arm backend is live. HostGUI uses this on
        // connect to decide whether to show PiperWidget (gen-2) or
        // legacy ArmWidget. Detection order:
        //   1. UAV_ARM_BACKEND env var ('piper' | 'legacy') wins.
        //   2. Else probe the proc_piper Unix socket — present ⇒ piper.
        //   3. Otherwise legacy.
        const char *backend_env = std::getenv("UAV_ARM_BACKEND");
        const char *piper_sock  = std::getenv("UAV_PROC_PIPER_SOCK");
        if (piper_sock == nullptr || piper_sock[0] == '\0') piper_sock = "/tmp/uav_proc_piper.sock";

        const char *backend = "legacy";
        const char *model   = "ZDT Emm V5 (gen-1)";
        if (backend_env != nullptr && backend_env[0] != '\0') {
            backend = backend_env;
            if (std::strcmp(backend, "piper") == 0) model = "AgileX Piper 6-DOF";
        } else {
            struct stat st{};
            if (stat(piper_sock, &st) == 0) {
                backend = "piper";
                model   = "AgileX Piper 6-DOF";
            }
        }
        snprintf(resp, sizeof(resp),
                 "{\"id\":%d,\"result\":{\"backend\":\"%s\",\"model\":\"%s\"}}\n",
                 id, backend, model);

    } else if (method == "camera.set_profile") {
        int w   = json_int(s, "width",  640);
        int h   = json_int(s, "height", 480);
        int fps = json_int(s, "fps",     30);
        ctrl_b.send_cmd(UAV_CTRL_B_SET_PROFILE, w, h, (float)fps);
        snprintf(resp, sizeof(resp),
                 "{\"id\":%d,\"result\":{\"ok\":true}}\n", id);

    } else if (method == "camera.set_exposure") {
        int us = json_int(s, "exposure_us", 0);
        ctrl_b.send_cmd(UAV_CTRL_B_SET_EXPOSURE, us);
        snprintf(resp, sizeof(resp),
                 "{\"id\":%d,\"result\":{\"ok\":true}}\n", id);

    // ── NPU control ──────────────────────────────────────────────────────
    } else if (method == "npu.start") {
        ctrl_c.send_cmd(UAV_CTRL_C_START);
        snprintf(resp, sizeof(resp),
                 "{\"id\":%d,\"result\":{\"ok\":true}}\n", id);

    } else if (method == "npu.stop") {
        ctrl_c.send_cmd(UAV_CTRL_C_STOP);
        snprintf(resp, sizeof(resp),
                 "{\"id\":%d,\"result\":{\"ok\":true}}\n", id);

    } else if (method == "npu.set_strategy") {
        int strategy = json_int(s, "strategy_id", 0);
        // Forward via JSON-RPC — this is the path that makes proc_npu
        // toggle the face_tracker / battery_tracker trigger flags.
        // Keep the legacy binary broadcast too for any other consumer
        // still listening for UAV_CTRL_C_SET_STRATEGY.
        char params[64];
        std::snprintf(params, sizeof(params), "{\"strategy\":%d}", strategy);
        bool ok = proc_npu_call("npu.set_strategy", params);
        ctrl_c.send_cmd(UAV_CTRL_C_SET_STRATEGY, strategy);
        snprintf(resp, sizeof(resp),
                 "{\"id\":%d,\"result\":{\"ok\":%s,\"strategy_id\":%d}}\n",
                 id, ok ? "true" : "false", strategy);

    } else if (method == "npu.set_threshold") {
        // threshold is a float in params; parse via a simple float extraction
        const char *tp = strstr(s, "\"threshold\":");
        float thr = 0.5F;
        if (tp) {
            tp += strlen("\"threshold\":");
            while (*tp == ' ' || *tp == '\t') ++tp;
            char *end;
            thr = strtof(tp, &end);
            if (end == tp) thr = 0.5F;
        }
        ctrl_c.send_cmd(UAV_CTRL_C_SET_THRESHOLD, 0, 0, thr);
        snprintf(resp, sizeof(resp),
                 "{\"id\":%d,\"result\":{\"ok\":true}}\n", id);

    } else if (method == "npu.get_detections") {
        // Snapshot both sources under the lock, then merge their
        // detection lists so HostGUI sees drone + battery in one
        // response. Per-source timeout (kCacheValidNs) silently drops
        // a slot once the producer has stopped sending non-empty
        // packets, keeping the GUI from holding stale boxes forever.
        UavCResult bat{}, oth{};
        bool has_bat = false, has_oth = false;
        {
            std::lock_guard<std::mutex> lk(g_npu_result.mu);
            const uint64_t now = now_ns_steady();
            if (g_npu_result.battery_until_ns.load(std::memory_order_acquire) > now) {
                bat = g_npu_result.battery; has_bat = true;
            }
            if (g_npu_result.other_until_ns.load(std::memory_order_acquire) > now) {
                oth = g_npu_result.other;   has_oth = true;
            }
        }
        const uint32_t total = (has_bat ? bat.num_detections : 0)
                             + (has_oth ? oth.num_detections : 0);
        const uint64_t fid = std::max(has_bat ? bat.frame_id : 0,
                                      has_oth ? oth.frame_id : 0);
        if (total == 0) {
            snprintf(resp, sizeof(resp),
                     "{\"id\":%d,\"result\":{\"frame_id\":0,\"num_detections\":0,"
                     "\"detections\":[]}}\n", id);
        } else {
            char dets_buf[6144];
            int  pos = 0;
            dets_buf[pos++] = '[';
            uint32_t emitted = 0;
            auto emit = [&](const UavDetection &d) {
                if (emitted >= UAV_MAX_DETECTIONS) return;
                if (emitted > 0) dets_buf[pos++] = ',';
                pos += fmt_detection(dets_buf + pos,
                                     (int)(sizeof(dets_buf) - (size_t)pos),
                                     d);
                emitted++;
            };
            if (has_oth) {
                for (uint32_t i = 0; i < oth.num_detections
                                   && i < UAV_MAX_DETECTIONS; ++i) {
                    emit(oth.detections[i]);
                }
            }
            if (has_bat) {
                for (uint32_t i = 0; i < bat.num_detections
                                   && i < UAV_MAX_DETECTIONS; ++i) {
                    emit(bat.detections[i]);
                }
            }
            dets_buf[pos++] = ']';
            dets_buf[pos]   = '\0';
            snprintf(resp, sizeof(resp),
                     "{\"id\":%d,\"result\":{\"frame_id\":%llu,"
                     "\"num_detections\":%u,\"detections\":%s}}\n",
                     id,
                     (unsigned long long)fid,
                     emitted,
                     dets_buf);
        }

    } else if (method == "npu.get_status") {
        snprintf(resp, sizeof(resp),
                 "{\"id\":%d,\"result\":{\"ok\":true}}\n", id);

    // ── Task forwarding ───────────────────────────────────────────────────
    } else if (method == "task.start") {
        std::string task_name = json_str(s, "task");
        const char *cmd_str = "START_BATTERY_PICK";
        if (task_name == "arm_demo")             cmd_str = "START_ARM_DEMO";
        else if (task_name == "arm_home")        cmd_str = "START_ARM_HOME";
        else if (task_name == "battery_pick")    cmd_str = "START_BATTERY_PICK";
        else if (task_name == "battery_pick_3d") cmd_str = "START_BATTERY_PICK_3D";
        else if (task_name == "battery_pick_6d") cmd_str = "START_BATTERY_PICK_6D";
        else if (task_name == "pick_place")      cmd_str = "START_PICK_PLACE";
        else if (task_name == "face_track")      cmd_str = "START_FACE_TRACK";
        else if (task_name == "platform_lock")   cmd_str = "START_BATTERY_PICK";
        send_task_cmd(task_udp_fd, cmd_str);
        snprintf(resp, sizeof(resp),
                 "{\"id\":%d,\"result\":{\"ok\":true,\"task\":\"%s\",\"cmd\":\"%s\"}}\n",
                 id, task_name.c_str(), cmd_str);

    } else if (method == "task.stop") {
        send_task_cmd(task_udp_fd, "ESTOP");
        snprintf(resp, sizeof(resp),
                 "{\"id\":%d,\"result\":{\"ok\":true}}\n", id);

    } else if (method == "task.reset") {
        send_task_cmd(task_udp_fd, "RESET");
        snprintf(resp, sizeof(resp),
                 "{\"id\":%d,\"result\":{\"ok\":true}}\n", id);

    } else if (method == "task.get_status") {
        std::string status = read_text_file(task_status_path());
        /* scheduler.c writes the status JSON with a trailing newline. If we
         * splice that straight into the JSON-RPC frame, the embedded \n
         * shows up before the outer closing '}' and the HostGUI line-based
         * parser splits one response into two malformed pieces. Strip any
         * trailing whitespace before embedding. */
        while (!status.empty() &&
               (status.back() == '\n' || status.back() == '\r' ||
                status.back() == ' '  || status.back() == '\t')) {
            status.pop_back();
        }
        if (status.empty()) {
            status = "{\"active\":false,\"task\":\"NONE\",\"status\":\"idle\",\"reason\":\"\"}";
        }
        snprintf(resp, sizeof(resp),
                 "{\"id\":%d,\"result\":%s}\n",
                 id, status.c_str());

    } else if (method == "system.get_logs") {
        int max_lines = json_int(s, "max_lines", 100);
        if (max_lines < 1) max_lines = 1;
        if (max_lines > 500) max_lines = 500;

        /* Optional `source` selects which service log to fetch. Default
         * "robotd" reads /tmp/uav_robotd.log (the file uav_robotd writes
         * directly). Other sources are systemd units whose stdout/stderr
         * goes to journald — we shell out to journalctl. proc_gateway runs
         * as root so it has full journal access. */
        std::string source = json_str(s, "source");
        if (source.empty()) source = "robotd";

        std::string source_label = source;
        std::string logs;

        if (source == "robotd" || source == "uav_robotd") {
            const std::string path = task_log_path();
            logs = tail_lines_text(read_text_file(path), max_lines);
            source_label = path;
        } else {
            /* Map friendly source name → systemd unit. Reject anything we
             * don't recognise so we can't be tricked into running arbitrary
             * journalctl filters. */
            const char *unit = nullptr;
            if      (source == "proc_arm")        unit = "uav-proc-arm.service";
            else if (source == "proc_gateway")    unit = "uav-proc-gateway.service";
            else if (source == "proc_npu")        unit = "uav-proc-npu.service";
            else if (source == "proc_realsense")  unit = "uav-proc-realsense.service";
            else if (source == "proc_car")        unit = "uav-proc-car.service";
            else if (source == "proc_gripper")    unit = "uav-proc-gripper.service";
            else if (source == "proc_airport")    unit = "uav-proc-airport.service";
            else if (source == "proc_grasp")      unit = "uav-proc-grasp.service";

            if (unit == nullptr) {
                logs = "[unknown log source: " + source + "]";
                source_label = source;
            } else {
                char cmd[256];
                snprintf(cmd, sizeof(cmd),
                         "journalctl -u %s -n %d --no-pager -o short 2>&1",
                         unit, max_lines);
                FILE *fp = popen(cmd, "r");
                if (fp == nullptr) {
                    logs = "[popen failed: ";
                    logs += strerror(errno);
                    logs += "]";
                } else {
                    char buf[4096];
                    while (fgets(buf, sizeof(buf), fp) != nullptr) {
                        logs.append(buf);
                    }
                    pclose(fp);
                }
                source_label = unit;
            }
        }

        const std::string escaped       = json_escape(logs);
        const std::string escaped_label = json_escape(source_label);

        /* Logs can easily exceed the 8 KB resp[] buffer, so build the
         * frame in a std::string and write it directly, then return so we
         * don't fall through to the bottom write_all that uses resp[]. */
        std::string frame = "{\"id\":";
        frame += std::to_string(id);
        frame += ",\"result\":{\"source\":\"";
        frame += escaped_label;
        frame += "\",\"logs\":\"";
        frame += escaped;
        frame += "\"}}\n";
        fprintf(stderr, "proc_gateway: RPC fd=%d resp: system.get_logs source=%s bytes=%zu\n",
                fd, source.c_str(), frame.size());
        write_all(fd, frame.data(), frame.size());
        return;

    } else if (method == "video.set_enabled") {
        bool enabled = json_bool(s, "enabled", true);
        if (video_stream_enabled != nullptr) {
            *video_stream_enabled = enabled;
        }
        if (!enabled && vid_clients != nullptr) {
            for (int client_fd : *vid_clients) {
                close(client_fd);
            }
            vid_clients->clear();
        }
        snprintf(resp, sizeof(resp),
                 "{\"id\":%d,\"result\":{\"ok\":true,\"enabled\":%s}}\n",
                 id, enabled ? "true" : "false");

    } else if (method == "video.get_status") {
        const bool enabled = (video_stream_enabled != nullptr) ? *video_stream_enabled : true;
        const int clients = (vid_clients != nullptr) ? (int)vid_clients->size() : 0;
        const char *src_str = (g_video_source.load() == VIDEO_SRC_DEPTH)
                                ? "depth" : "rgb";
        snprintf(resp, sizeof(resp),
                 "{\"id\":%d,\"result\":{\"enabled\":%s,\"clients\":%d,"
                 "\"source\":\"%s\"}}\n",
                 id, enabled ? "true" : "false", clients, src_str);

    } else if (method == "video.set_source") {
        // Accept either {"source":"depth"} / "rgb" or a raw int 0 / 1.
        // Only one stream is sent at a time to keep :7002 bandwidth flat —
        // the renderer on the RK3588 swaps source at the next frame.
        std::string src = json_str(s, "source");
        int new_src = VIDEO_SRC_RGB;
        if (src == "depth" || src == "DEPTH" || src == "1") {
            new_src = VIDEO_SRC_DEPTH;
        } else if (src == "rgb" || src == "RGB" || src == "color" || src == "0") {
            new_src = VIDEO_SRC_RGB;
        } else {
            // Fall back to an int param named "source" for convenience.
            const int iv = json_int(s, "source", -1);
            if (iv == 1) new_src = VIDEO_SRC_DEPTH;
            else if (iv == 0) new_src = VIDEO_SRC_RGB;
        }
        g_video_source.store(new_src);
        snprintf(resp, sizeof(resp),
                 "{\"id\":%d,\"result\":{\"ok\":true,\"source\":\"%s\"}}\n",
                 id, (new_src == VIDEO_SRC_DEPTH) ? "depth" : "rgb");

    } else if (method == "arm.home") {
        // Forward to proc_arm which owns the CAN bus
        bool ok = proc_arm_call("arm.home", "{}");
        snprintf(resp, sizeof(resp),
                 "{\"id\":%d,\"result\":{\"ok\":%s}}\n",
                 id, ok ? "true" : "false");

    } else if (method == "arm.stop") {
        // Forward to proc_arm which owns the CAN bus. proc_arm's estop
        // watchdog will see this on the unix-socket fd and abort any
        // currently-running blocking move on the proc_arm side.
        bool ok = proc_arm_call("arm.stop", "{}");
        snprintf(resp, sizeof(resp),
                 "{\"id\":%d,\"result\":{\"ok\":%s}}\n",
                 id, ok ? "true" : "false");

    } else if (method == "arm.emergency_stop") {
        // Forward to proc_arm which owns the CAN bus.
        bool enable = json_bool(s, "enable", true);
        char params_json[32];
        std::snprintf(params_json, sizeof(params_json),
                      "{\"enable\":%s}", enable ? "true" : "false");
        bool ok = proc_arm_call("arm.emergency_stop", params_json);
        snprintf(resp, sizeof(resp),
                 "{\"id\":%d,\"result\":{\"ok\":%s}}\n",
                 id, ok ? "true" : "false");

    } else if (method == "arm.set_speeds") {
        // Forward to proc_arm which owns the runtime RPM overrides.
        int move_rpm = json_int(s, "move_rpm", 0);
        int zero_rpm = json_int(s, "zero_rpm", 0);
        char params_json[64];
        std::snprintf(params_json, sizeof(params_json),
                      "{\"move_rpm\":%d,\"zero_rpm\":%d}", move_rpm, zero_rpm);
        bool ok = proc_arm_call("arm.set_speeds", params_json);
        snprintf(resp, sizeof(resp),
                 "{\"id\":%d,\"result\":{\"ok\":%s,\"move_rpm\":%d,\"zero_rpm\":%d}}\n",
                 id, ok ? "true" : "false", move_rpm, zero_rpm);

    } else if (method == "arm.move_pose") {
        // Forward to proc_arm.
        std::string pose = json_str(s, "pose");
        char params_json[128];
        std::snprintf(params_json, sizeof(params_json),
                      "{\"pose\":\"%s\"}", pose.c_str());
        bool ok = !pose.empty() && proc_arm_call("arm.move_pose", params_json);
        snprintf(resp, sizeof(resp),
                 "{\"id\":%d,\"result\":{\"ok\":%s,\"pose\":\"%s\"}}\n",
                 id, ok ? "true" : "false", pose.c_str());

    } else if (method == "arm.move_xyz") {
        // Forward to proc_arm.
        double x_mm = json_double(s, "x_mm", 0.0);
        double y_mm = json_double(s, "y_mm", 0.0);
        double z_mm = json_double(s, "z_mm", 0.0);
        char params_json[160];
        std::snprintf(params_json, sizeof(params_json),
                      "{\"x_mm\":%.4f,\"y_mm\":%.4f,\"z_mm\":%.4f}",
                      x_mm, y_mm, z_mm);
        bool ok = proc_arm_call("arm.move_xyz", params_json);
        snprintf(resp, sizeof(resp),
                 "{\"id\":%d,\"result\":{\"ok\":%s,\"x_mm\":%.1f,\"y_mm\":%.1f,\"z_mm\":%.1f}}\n",
                 id, ok ? "true" : "false", x_mm, y_mm, z_mm);

    } else if (method == "arm.home_joint") {
        int joint = json_int(s, "joint", -1);
        if (joint < 0) joint = json_int(s, "joint_index", -1);
        char params_json[64];
        std::snprintf(params_json, sizeof(params_json),
                      "{\"joint\":%d,\"joint_index\":%d}", joint, joint);
        bool ok = (joint >= 1 && joint <= 6) &&
                  proc_arm_call("arm.home_joint", params_json);
        snprintf(resp, sizeof(resp),
                 "{\"id\":%d,\"result\":{\"ok\":%s,\"joint\":%d}}\n",
                 id, ok ? "true" : "false", joint);

    } else if (method == "arm.move_joint") {
        // Accept either "joint" or "joint_index" — HostGUI / PiperWidget
        // uses joint_index, legacy callers use joint. Both pass through to
        // proc_piper / proc_arm.
        int joint = json_int(s, "joint", -1);
        if (joint < 0) joint = json_int(s, "joint_index", -1);
        double target_deg = json_double(s, "target_deg", 0.0);
        char params_json[128];
        std::snprintf(params_json, sizeof(params_json),
                      "{\"joint\":%d,\"joint_index\":%d,\"target_deg\":%.4f}",
                      joint, joint, target_deg);
        bool ok = (joint >= 1 && joint <= 6) &&
                  proc_arm_call("arm.move_joint", params_json);
        snprintf(resp, sizeof(resp),
                 "{\"id\":%d,\"result\":{\"ok\":%s,\"joint\":%d,\"target_deg\":%.1f}}\n",
                 id, ok ? "true" : "false", joint, target_deg);

    } else if (method == "arm.move_joints") {
        // Accept three input shapes, in priority order:
        //   1. "joints":  [j1, j2, j3, j4, j5, j6]      ← TeachWidget
        //   2. "angles":  [j1, j2, j3, j4, j5, j6]      ← PiperWidget
        //   3. "j1_deg":N, "j2_deg":N, …                ← legacy / direct
        // Whichever yields six numbers wins. Before the array shapes were
        // recognised, gateway silently dropped them and forwarded six zeros
        // to proc_piper, which made the arm go home regardless of intent.
        double j[6] = {0, 0, 0, 0, 0, 0};
        bool   filled = false;

        // Try array shapes first. json_int/json_double tools key on
        // "name":..., so for arrays we have to grep manually.
        for (const char *key : {"\"joints\"", "\"angles\""}) {
            const char *p = std::strstr(s, key);
            if (!p) continue;
            p = std::strchr(p, '[');
            if (!p) continue;
            ++p;
            int idx = 0;
            for (; idx < 6; ++idx) {
                while (*p == ' ' || *p == '\t' || *p == ',' || *p == '\n') ++p;
                if (*p == ']' || *p == '\0') break;
                char *end = nullptr;
                double v = std::strtod(p, &end);
                if (end == p) break;
                j[idx] = v;
                p = end;
            }
            if (idx == 6) { filled = true; break; }
        }
        // Fallback to per-joint keys if no array was present.
        if (!filled) {
            j[0] = json_double(s, "j1_deg", 0.0);
            j[1] = json_double(s, "j2_deg", 0.0);
            j[2] = json_double(s, "j3_deg", 0.0);
            j[3] = json_double(s, "j4_deg", 0.0);
            j[4] = json_double(s, "j5_deg", 0.0);
            j[5] = json_double(s, "j6_deg", 0.0);
        }
        // Forward in the array form proc_piper natively understands. We
        // include both "joints" (which proc_piper's _extract_joints prefers)
        // and per-joint keys (in case the backend is still the legacy
        // proc_arm which decodes those instead).
        char params_json[384];
        std::snprintf(params_json, sizeof(params_json),
                      "{\"joints\":[%.4f,%.4f,%.4f,%.4f,%.4f,%.4f],"
                      "\"j1_deg\":%.4f,\"j2_deg\":%.4f,\"j3_deg\":%.4f,"
                      "\"j4_deg\":%.4f,\"j5_deg\":%.4f,\"j6_deg\":%.4f}",
                      j[0], j[1], j[2], j[3], j[4], j[5],
                      j[0], j[1], j[2], j[3], j[4], j[5]);
        bool ok = proc_arm_call("arm.move_joints", params_json);
        snprintf(resp, sizeof(resp),
                 "{\"id\":%d,\"result\":{\"ok\":%s,"
                 "\"joints_deg\":[%.1f,%.1f,%.1f,%.1f,%.1f,%.1f]}}\n",
                 id, ok ? "true" : "false",
                 j[0], j[1], j[2], j[3], j[4], j[5]);

    } else if (method == "arm.get_angles") {
        // Forward to proc_arm which tracks signed current_joints_deg_ correctly.
        // arm.get_motor_angles returns a JSON array directly as the result value.
        std::string result_str = proc_arm_call_result("arm.get_motor_angles", "{}");
        // result_str is like "[1.0,-25.0,0.0,0.0,0.0,0.0]\n" or empty on failure.
        if (!result_str.empty() && result_str[0] == '[') {
            // Truncate at the closing ']' of the array — everything after it
            // (the JSON-RPC envelope's closing '}' and newline) must be stripped.
            size_t last_bracket = result_str.rfind(']');
            if (last_bracket != std::string::npos)
                result_str = result_str.substr(0, last_bracket + 1);
            snprintf(resp, sizeof(resp),
                     "{\"id\":%d,\"result\":{\"ok\":true,\"angles\":%s}}\n",
                     id, result_str.c_str());
        } else {
            snprintf(resp, sizeof(resp),
                     "{\"id\":%d,\"result\":{\"ok\":false,\"error\":\"proc_arm unavailable\"}}\n",
                     id);
        }

    } else if (method == "ugv.set_velocity") {
        double vx    = json_double(s, "vx", 0.0);
        double vy    = json_double(s, "vy", 0.0);
        double omega = json_double(s, "omega", 0.0);
        // Convert m/s to mm/s and rad/s to mdeg/s for proc_car
        int linear_mm_s  = static_cast<int>(vx * 1000.0);
        int angular_mdeg_s = static_cast<int>(omega * 180.0 / 3.14159265 * 1000.0);
        char params[128];
        std::snprintf(params, sizeof(params),
                      "{\"linear_mm_s\":%d,\"angular_mdeg_s\":%d}",
                      linear_mm_s, angular_mdeg_s);
        bool ok = proc_car_call("car.set_velocity", params);
        snprintf(resp, sizeof(resp),
                 "{\"id\":%d,\"result\":{\"ok\":%s,\"vx\":%.3f,\"vy\":%.3f,\"omega\":%.3f}}\n",
                 id, ok ? "true" : "false", vx, vy, omega);

    } else if (method == "ugv.stop") {
        bool ok = proc_car_call("car.stop", "{}");
        snprintf(resp, sizeof(resp),
                 "{\"id\":%d,\"result\":{\"ok\":%s}}\n",
                 id, ok ? "true" : "false");

    } else if (method == "airport.set_rail") {
        int rail = json_int(s, "rail", -1);
        double pos_mm = json_double(s, "pos_mm", 0.0);
        bool ok = g_airport.move_absolute_mm(rail, pos_mm);
        snprintf(resp, sizeof(resp),
                 "{\"id\":%d,\"result\":{\"ok\":%s,\"rail\":%d,\"pos_mm\":%.1f}}\n",
                 id, ok ? "true" : "false", rail, pos_mm);

    } else if (method == "airport.set_speed") {
        int rail = json_int(s, "rail", -1);
        int speed_rpm = json_int(s, "speed_rpm", 0);
        bool ok = g_airport.set_speed_rpm(rail, speed_rpm);
        snprintf(resp, sizeof(resp),
                 "{\"id\":%d,\"result\":{\"ok\":%s,\"rail\":%d,\"speed_rpm\":%d}}\n",
                 id, ok ? "true" : "false", rail, speed_rpm);

    } else if (method == "airport.lock") {
        int speed_rpm = json_int(s, "speed_rpm", 0);
        bool ok = g_airport.start_lock_pair(true, speed_rpm);
        snprintf(resp, sizeof(resp),
                 "{\"id\":%d,\"result\":{\"ok\":%s,\"mode\":\"lock\",\"speed_rpm\":%d,\"rails\":[0,2]}}\n",
                 id, ok ? "true" : "false", speed_rpm);

    } else if (method == "airport.release") {
        int speed_rpm = json_int(s, "speed_rpm", 0);
        bool ok = g_airport.start_lock_pair(false, speed_rpm);
        snprintf(resp, sizeof(resp),
                 "{\"id\":%d,\"result\":{\"ok\":%s,\"mode\":\"release\",\"speed_rpm\":%d,\"rails\":[0,2]}}\n",
                 id, ok ? "true" : "false", speed_rpm);

    } else if (method == "airport.stop") {
        int rail = json_int(s, "rail", -1);
        bool ok = g_airport.stop_rail(rail);
        snprintf(resp, sizeof(resp),
                 "{\"id\":%d,\"result\":{\"ok\":%s,\"rail\":%d}}\n",
                 id, ok ? "true" : "false", rail);

    } else if (method == "airport.stop_all") {
        bool ok = g_airport.stop_all();
        snprintf(resp, sizeof(resp),
                 "{\"id\":%d,\"result\":{\"ok\":%s}}\n",
                 id, ok ? "true" : "false");

    } else if (method == "airport.move_distance") {
        // Open-loop relative move at speed_rpm. Motor self-stops after the
        // pulse count is consumed. backend monitor flips rail_state_
        // IDLE/STALLED so HostGUI's poll loop advances the script step.
        int rail = json_int(s, "rail", -1);
        double dist_mm = json_double(s, "distance_mm", 0.0);
        int speed_rpm = json_int(s, "speed_rpm", 500);
        bool ok = g_airport.move_distance(rail, dist_mm, speed_rpm);
        snprintf(resp, sizeof(resp),
                 "{\"id\":%d,\"result\":{\"ok\":%s,\"rail\":%d,\"distance_mm\":%.2f,\"speed_rpm\":%d}}\n",
                 id, ok ? "true" : "false", rail, dist_mm, speed_rpm);

    } else if (method == "airport.get_status") {
        // Per-rail observable state. HostGUI Tab4 script orchestrator polls
        // this every ~200ms during AIRPORT_RAIL steps so it can advance to
        // the next step the instant a stall fires, instead of waiting out
        // the max_ms budget. state: 0=idle, 1=moving, 2=stalled.
        const int s0 = g_airport.get_rail_state(0);
        const int s1 = g_airport.get_rail_state(1);
        const int s2 = g_airport.get_rail_state(2);
        snprintf(resp, sizeof(resp),
                 "{\"id\":%d,\"result\":{\"ok\":true,"
                 "\"rails\":[{\"index\":0,\"state\":%d},"
                            "{\"index\":1,\"state\":%d},"
                            "{\"index\":2,\"state\":%d}]}}\n",
                 id, s0, s1, s2);

    } else if (method == "airport.relay") {
        int channel = json_int(s, "channel", -1);
        bool on = json_bool(s, "on", false);
        bool ok = g_airport_relays.set_channel(channel, on);
        snprintf(resp, sizeof(resp),
                 "{\"id\":%d,\"result\":{\"ok\":%s,\"channel\":%d,\"on\":%s}}\n",
                 id, ok ? "true" : "false", channel, on ? "true" : "false");

    } else if (method == "airport.gripper") {
        bool open = json_bool(s, "open", false);
        bool ok = g_airport_relays.set_gripper(open);
        snprintf(resp, sizeof(resp),
                 "{\"id\":%d,\"result\":{\"ok\":%s,\"open\":%s}}\n",
                 id, ok ? "true" : "false", open ? "true" : "false");

    } else if (method == "arm_gripper.set") {
        bool open = json_bool(s, "open", false);
        bool ok = g_arm_gripper.set_open(open);
        snprintf(resp, sizeof(resp),
                 "{\"id\":%d,\"result\":{\"ok\":%s,\"open\":%s}}\n",
                 id, ok ? "true" : "false", open ? "true" : "false");

    // ── proc_grasp forwarding ────────────────────────────────────────────
    } else if (method == "grasp.start" || method == "grasp.stop" ||
               method == "grasp.run_once" || method == "grasp.set_mode" ||
               method == "grasp.set_hand_eye") {
        /* Extract balanced {...} for "params" so we can forward them
         * verbatim to proc_grasp. */
        std::string params_slice = "{}";
        const char *pp = strstr(s, "\"params\":");
        if (pp != nullptr) {
            pp += strlen("\"params\":");
            while (*pp == ' ' || *pp == '\t') ++pp;
            if (*pp == '{') {
                int depth = 0;
                const char *start = pp;
                for (; *pp != '\0'; ++pp) {
                    if (*pp == '{') ++depth;
                    else if (*pp == '}') {
                        --depth;
                        if (depth == 0) { ++pp; break; }
                    }
                }
                params_slice.assign(start, (size_t)(pp - start));
            }
        }
        bool ok = proc_grasp_call_ok(method.c_str(), params_slice.c_str());
        snprintf(resp, sizeof(resp),
                 "{\"id\":%d,\"result\":{\"ok\":%s}}\n",
                 id, ok ? "true" : "false");

    } else if (method == "grasp.get_status") {
        std::string status = proc_grasp_call_raw("grasp.get_status", "{}");
        /* status is "{...}}\n" — we need just the inner body. strip the
         * outer "}}\n" that belongs to the proc_grasp frame envelope. */
        while (!status.empty() && (status.back() == '\n' || status.back() == '\r')) {
            status.pop_back();
        }
        if (status.size() >= 1 && status.back() == '}') {
            status.pop_back();  // drop envelope close-brace
        }
        if (status.empty()) {
            status = "{\"state\":\"unavailable\"}";
        }
        snprintf(resp, sizeof(resp),
                 "{\"id\":%d,\"result\":%s}\n", id, status.c_str());

    } else if (std::strncmp(method.c_str(), "arm.", 4) == 0 ||
               std::strncmp(method.c_str(), "piper.", 6) == 0) {
        // Transparent forward to the configured arm backend (proc_arm or
        // proc_piper). Covers arm.get_pose, arm.move_pose6d, all the
        // piper.* extensions, etc. — methods gateway doesn't decode itself.
        if (!proc_arm_passthrough(line.c_str(), resp, sizeof(resp))) {
            snprintf(resp, sizeof(resp),
                     "{\"id\":%d,\"result\":{\"ok\":false,\"error\":\"arm backend unavailable\"}}\n",
                     id);
        }
    } else {
        // Stub for ugv/airport/gripper unknowns – acknowledge without action
        snprintf(resp, sizeof(resp),
                 "{\"id\":%d,\"result\":{\"ok\":true}}\n", id);
    }

    if (s_rpc_trace) {
        fprintf(stderr, "proc_gateway: RPC fd=%d resp: %s", fd, resp);
    }
    write_all(fd, resp, strlen(resp));
}

// (VIDEO_SRC_* + g_video_source are declared above, near the top of the
//  file, so that the RPC handler — which sits earlier in the source —
//  can reference them too.)

// Depth range used for the colormap (metres).  Anything nearer / farther /
// invalid renders black, so the user sees a clean cut-off instead of a
// noisy band at the extremes.
static constexpr float DEPTH_MAP_MIN_M = 0.20F;   // ~20 cm
static constexpr float DEPTH_MAP_MAX_M = 1.50F;   // 150 cm

// Render a uint16 depth buffer to a BGR image using a cheap turbo-style
// colormap (blue → cyan → green → yellow → red).  Writes `out` in place,
// sized to w*h*3 bytes — downstream can hand this straight to
// JpegEncoder::encode() exactly like a colour frame, no special path.
static void colorize_depth(const uint8_t *depth16,
                            uint32_t w, uint32_t h,
                            float depth_scale_m,
                            std::vector<uint8_t> &out)
{
    out.resize(static_cast<size_t>(w) * h * 3U);
    const uint16_t *d = reinterpret_cast<const uint16_t *>(depth16);
    const float z_min = DEPTH_MAP_MIN_M;
    const float z_max = DEPTH_MAP_MAX_M;
    const float inv_span = 1.0F / (z_max - z_min);

    for (uint32_t i = 0; i < w * h; ++i) {
        const uint16_t raw = d[i];
        if (raw == 0U) {                      // invalid depth
            out[i * 3 + 0] = 0; out[i * 3 + 1] = 0; out[i * 3 + 2] = 0;
            continue;
        }
        const float z = static_cast<float>(raw) * depth_scale_m;
        if (z < z_min || z > z_max) {
            out[i * 3 + 0] = 0; out[i * 3 + 1] = 0; out[i * 3 + 2] = 0;
            continue;
        }
        float t = (z - z_min) * inv_span;   // near=0 (blue), far=1 (red)
        if (t < 0.0F) t = 0.0F; else if (t > 1.0F) t = 1.0F;

        float r, g, b;
        if (t < 0.25F) {                    // blue → cyan
            float s = t * 4.0F;
            r = 0.0F; g = s;        b = 1.0F;
        } else if (t < 0.50F) {             // cyan → green
            float s = (t - 0.25F) * 4.0F;
            r = 0.0F; g = 1.0F;     b = 1.0F - s;
        } else if (t < 0.75F) {             // green → yellow
            float s = (t - 0.50F) * 4.0F;
            r = s;    g = 1.0F;     b = 0.0F;
        } else {                            // yellow → red
            float s = (t - 0.75F) * 4.0F;
            r = 1.0F; g = 1.0F - s; b = 0.0F;
        }
        // Store BGR (the encoder swaps to RGB row-by-row).
        out[i * 3 + 0] = static_cast<uint8_t>(b * 255.0F);
        out[i * 3 + 1] = static_cast<uint8_t>(g * 255.0F);
        out[i * 3 + 2] = static_cast<uint8_t>(r * 255.0F);
    }
}

// ─── Push one JPEG frame to a video client ───────────────────────────────────
// Returns false if the client should be closed.
//
// Critical: gateway runs the RPC poll loop and the video push in the same
// thread. A blocking send() on a slow video client (full kernel send buffer
// because the GUI is behind) used to stall RPC traffic for seconds —
// operators saw the arm "respond late" to slider moves. We now probe each
// client with POLLOUT first and drop the entire frame if it would block.
// Once a send starts (POLLOUT says writable), we commit to finishing it;
// that's bounded by RTT in practice (~ms on a healthy LAN).
static bool push_frame(int fd, const std::vector<uint8_t> &jpeg) {
    pollfd pfd = {fd, POLLOUT, 0};
    if (poll(&pfd, 1, 0) <= 0) {
        return true;          // not writable yet — skip this frame, keep conn
    }
    if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) {
        return false;         // socket genuinely dead
    }
    if (!(pfd.revents & POLLOUT)) {
        return true;          // kernel buffer full — drop frame, retry next tick
    }

    // Build header + payload as a single contiguous buffer for one send() call.
    uint32_t sz = (uint32_t)jpeg.size();
    std::vector<uint8_t> pkt(4 + jpeg.size());
    pkt[0] = (sz >> 24) & 0xFF;
    pkt[1] = (sz >> 16) & 0xFF;
    pkt[2] = (sz >>  8) & 0xFF;
    pkt[3] = (sz      ) & 0xFF;
    memcpy(pkt.data() + 4, jpeg.data(), jpeg.size());

    size_t sent = 0;
    while (sent < pkt.size()) {
        ssize_t n = send(fd, pkt.data() + sent, pkt.size() - sent, 0);
        if (n < 0) {
            if (errno == EINTR) {
                continue;
            }
            return false;
        }
        if (n == 0) {
            return false;
        }
        sent += (size_t)n;
    }
    return true;
}

// ─── runtime ─────────────────────────────────────────────────────────────────
int run_gateway_runtime() {
    signal(SIGINT,  on_sig);
    signal(SIGTERM, on_sig);
    signal(SIGPIPE, SIG_IGN);

    const uint64_t start_ms = now_ms();

    ShmReader   shm;
    JpegEncoder encoder;
    CtrlClient  ctrl_b;   // → proc_realsense
    CtrlClient  ctrl_c;   // → proc_npu

    ctrl_b.open(UAV_CTRL_PATH_B);
    ctrl_c.open(UAV_CTRL_PATH_C);

    // NPU result receiver socket
    int npu_result_fd = open_npu_result_socket();
    if (npu_result_fd < 0) {
        fprintf(stderr, "proc_gateway: warning – could not bind NPU result socket\n");
    }

    // Task command UDP socket (sends to uav_robotd)
    int task_udp_fd = open_task_udp_socket();

    int rpc_listen = listen_on(7001);
    int vid_listen = listen_on(7002);
    if (rpc_listen < 0 || vid_listen < 0) {
        fprintf(stderr, "proc_gateway: failed to bind ports 7001/7002\n");
        return 1;
    }
    fprintf(stderr, "proc_gateway: JSON-RPC :7001   MJPEG :7002\n");

    std::vector<RpcConn> rpc_clients;
    std::vector<int>     vid_clients;
    bool video_stream_enabled = video_stream_enabled_default();

    uint64_t last_shm_try = 0;
    GatewayFrame frame;

    while (g_running) {

        // ── Retry opening shm every 2 s while proc_realsense is not running ──
        if (!shm.is_open()) {
            uint64_t now = now_ms();
            if (now - last_shm_try >= 2000) {
                last_shm_try = now;
                if (shm.open(UAV_SHM_RING_NAME)) {
                    fprintf(stderr, "proc_gateway: shm ring opened\n");
                }
            }
        }

        // Drain any detection results pushed by proc_npu
        drain_npu_results(npu_result_fd);

        // ── Build pollfd: [0] rpc_listen, [1] vid_listen, [2+] rpc clients ──
        std::vector<pollfd> pfds;
        pfds.push_back({rpc_listen, POLLIN, 0});
        pfds.push_back({vid_listen, POLLIN, 0});
        for (auto &c : rpc_clients)
            pfds.push_back({c.fd, POLLIN, 0});

        int ret = poll(pfds.data(), (nfds_t)pfds.size(), 10 /*ms*/);
        if (ret < 0) {
            if (errno == EINTR) continue;
            break;
        }

        // ── Accept new RPC connection ─────────────────────────────────────────
        if (pfds[0].revents & POLLIN) {
            int fd = accept(rpc_listen, nullptr, nullptr);
            if (fd >= 0) {
                set_nodelay(fd);
                rpc_clients.push_back({fd, {}});
                fprintf(stderr, "proc_gateway: RPC client connected fd=%d\n", fd);
            }
        }

        // ── Accept new Video connection ───────────────────────────────────────
        if (pfds[1].revents & POLLIN) {
            int fd = accept(vid_listen, nullptr, nullptr);
            if (fd >= 0) {
                set_nodelay(fd);
                if (video_stream_enabled) {
                    vid_clients.push_back(fd);
                    fprintf(stderr, "proc_gateway: Video client connected fd=%d\n", fd);
                } else {
                    fprintf(stderr, "proc_gateway: Video client rejected while disabled fd=%d\n", fd);
                    close(fd);
                }
            }
        }

        // ── Handle RPC client data ────────────────────────────────────────────
        std::vector<int> dead_rpc;
        for (size_t i = 0; i < rpc_clients.size(); ++i) {
            if (!(pfds[i + 2].revents & POLLIN)) continue;
            auto &c = rpc_clients[i];
            char tmp[4096];
            ssize_t n = read(c.fd, tmp, sizeof(tmp) - 1);
            if (n <= 0) {
                fprintf(stderr,
                        "proc_gateway: RPC fd=%d read=%zd errno=%d (%s)\n",
                        c.fd, n, errno, strerror(errno));
                dead_rpc.push_back(c.fd);
                continue;
            }
            tmp[n] = '\0';
            c.buf += tmp;
            // Process complete newline-delimited JSON lines
            size_t pos;
            while ((pos = c.buf.find('\n')) != std::string::npos) {
                std::string line = c.buf.substr(0, pos);
                c.buf.erase(0, pos + 1);
                if (!line.empty())
                    handle_rpc(c.fd,
                               line,
                               ctrl_b,
                               ctrl_c,
                               task_udp_fd,
                               start_ms,
                               &video_stream_enabled,
                               &vid_clients);
            }
        }
        for (int dfd : dead_rpc) {
            rpc_clients.erase(
                std::remove_if(rpc_clients.begin(), rpc_clients.end(),
                    [dfd](const RpcConn &c){ return c.fd == dfd; }),
                rpc_clients.end());
            close(dfd);
            fprintf(stderr, "proc_gateway: RPC client disconnected\n");
        }

        // ── Read latest frame from shm → encode → push to video clients ──────
        if (video_stream_enabled && shm.is_open() && shm.read_latest(frame) && !vid_clients.empty()) {
            // Runtime knobs for HostGUI display fps tuning. Detection
            // still uses the full-resolution shm frame; only the JPEG
            // we ship over :7002 is shrunk / lower-quality, so visual
            // quality drops independently of detector accuracy.
            //   UAV_GATEWAY_JPEG_QUALITY  (10..95, default 80)
            //   UAV_GATEWAY_VIDEO_SCALE   (1, 2, or 4 — divide W&H by N,
            //                              default 1 = native resolution)
            static const int s_jpeg_quality = []() {
                const char *e = std::getenv("UAV_GATEWAY_JPEG_QUALITY");
                if (!e || !*e) return 80;
                int v = std::atoi(e);
                return (v >= 10 && v <= 95) ? v : 80;
            }();
            static const int s_video_scale = []() {
                const char *e = std::getenv("UAV_GATEWAY_VIDEO_SCALE");
                if (!e || !*e) return 1;
                int v = std::atoi(e);
                return (v == 2 || v == 4) ? v : 1;
            }();

            std::vector<uint8_t> jpeg;
            const int src = g_video_source.load();
            const uint8_t *src_bgr;
            uint32_t enc_w, enc_h, enc_stride;
            static thread_local std::vector<uint8_t> depth_bgr;
            static thread_local std::vector<uint8_t> scaled_bgr;
            if (src == VIDEO_SRC_DEPTH && !frame.depth.empty()
                && frame.depth_scale > 0.0F) {
                // Colorize depth → BGR, then JPEG-encode via the same path
                // as the RGB stream so :7002 frame format stays uniform.
                colorize_depth(frame.depth.data(),
                               frame.width, frame.height,
                               frame.depth_scale, depth_bgr);
                src_bgr = depth_bgr.data();
                enc_w = frame.width;
                enc_h = frame.height;
                enc_stride = frame.width * 3U;
            } else {
                // Default / fallback: colour frame. Also used when the
                // user selected DEPTH but the current slot doesn't carry
                // a depth buffer (source ran without depth enabled).
                src_bgr = frame.color.data();
                enc_w = frame.width;
                enc_h = frame.height;
                enc_stride = frame.stride;
            }

            if (s_video_scale > 1) {
                // Cheap nearest-neighbour downscale — faster than libjpeg
                // resize and good enough for viewing. Detector path is
                // untouched; this only shrinks the JPEG that goes out.
                const uint32_t out_w = enc_w / s_video_scale;
                const uint32_t out_h = enc_h / s_video_scale;
                scaled_bgr.resize(out_w * out_h * 3U);
                for (uint32_t y = 0; y < out_h; ++y) {
                    const uint8_t *src_row = src_bgr + y * s_video_scale * enc_stride;
                    uint8_t *dst_row = scaled_bgr.data() + y * out_w * 3U;
                    for (uint32_t x = 0; x < out_w; ++x) {
                        const uint8_t *p = src_row + x * s_video_scale * 3U;
                        dst_row[x * 3U + 0] = p[0];
                        dst_row[x * 3U + 1] = p[1];
                        dst_row[x * 3U + 2] = p[2];
                    }
                }
                src_bgr = scaled_bgr.data();
                enc_w = out_w;
                enc_h = out_h;
                enc_stride = out_w * 3U;
            }
            jpeg = encoder.encode(src_bgr, enc_w, enc_h, enc_stride, s_jpeg_quality);
            if (!jpeg.empty()) {
                std::vector<int> dead_vid;
                for (int vfd : vid_clients) {
                    if (!push_frame(vfd, jpeg))
                        dead_vid.push_back(vfd);
                }
                for (int dfd : dead_vid) {
                    vid_clients.erase(
                        std::remove(vid_clients.begin(), vid_clients.end(), dfd),
                        vid_clients.end());
                    close(dfd);
                    fprintf(stderr, "proc_gateway: Video client disconnected\n");
                }
            }
        }
    }

    // ── Cleanup ───────────────────────────────────────────────────────────────
    for (auto &c : rpc_clients) close(c.fd);
    for (int fd  : vid_clients)  close(fd);
    close(rpc_listen);
    close(vid_listen);
    if (npu_result_fd >= 0) { close(npu_result_fd); ::unlink(UAV_NPU_RESULT_GW_PATH); }
    if (task_udp_fd   >= 0)   close(task_udp_fd);
    shm.close();
    ctrl_b.close();
    ctrl_c.close();
    fprintf(stderr, "proc_gateway: stopped\n");
    return 0;
}
