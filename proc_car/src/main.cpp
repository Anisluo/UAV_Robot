#include <cerrno>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <poll.h>
#include <string>

#include <fcntl.h>
#include <linux/joystick.h>
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

// ---------------------------------------------------------------------------
// Joystick tele-op (Linux joydev /dev/input/jsX, Xbox-360-mapped controllers)
//
// joydev axis layout for XInput-compatible pads:
//   axes[0]=LX  axes[1]=LY (push up = -32767)  axes[2]=LT
//   axes[3]=RX  axes[4]=RY                     axes[5]=RT
//   axes[6]=DPAD_X  axes[7]=DPAD_Y
// joydev buttons (xpad driver):
//   0=A 1=B 2=X 3=Y 4=LB 5=RB 6=BACK 7=START 8=GUIDE 9=LSTICK 10=RSTICK
//
// Safety model:
//   * dead-man's switch — motion only while the configured button is held;
//     release => immediate zero command via slew.
//   * disconnect / no data — POLLHUP/POLLERR or read failure closes the fd
//     and zeros the last sent velocity. Reopen is attempted periodically.
//   * slew-rate limit — change in commanded velocity per send tick is capped
//     to honour the chassis spec's "no step inputs" guidance.
// ---------------------------------------------------------------------------

constexpr int kJoyAxisMax = 32767;
constexpr size_t kJoyAxisCount = 8;
constexpr size_t kJoyButtonCount = 16;

// joydev xpad mapping for the inputs we care about:
constexpr int kBtnY = 3;   // Y (yellow, top) — panic stop

// Left stick = DIRECTION
//   LX (axes[0]): push right = +32767  → right turn
//   LY (axes[1]): push up    = -32767  → forward
constexpr int kAxisLX = 0;
constexpr int kAxisLY = 1;

// Right stick Y = THROTTLE MULTIPLIER (continuous, three reference points)
//   push UP    (RY = -32767) → throttle = kThrottleMax (1.00, full speed)
//   center     (RY = 0)      → throttle = kThrottleMid (0.50, half speed)
//   push DOWN  (RY = +32767) → throttle = kThrottleMin (0.10, crawl)
// Linear interpolation between breakpoints. A separate small deadzone keeps
// the rest position glued to 0.50 instead of drifting from stick noise.
constexpr int kAxisRY = 4;

// L2 (LT, axis 2) + R2 (RT, axis 5) analog triggers. Both held → full-speed
// unlock; either alone or none held → speed cap stays at the Normal value.
// xpad reports these as bipolar axes (-32767 rest, +32767 fully pressed).
constexpr int kAxisLT = 2;
constexpr int kAxisRT = 5;
constexpr int kTriggerThreshold = 16000;   // ~75% press = "held"

// Two speed caps. Normal applies always; Unlocked applies only when both
// triggers (L2 AND R2) are held simultaneously. Throttle stick still scales
// below whichever cap is active.
constexpr int kMaxLinearMmS_Normal     = 40000;    // 40 m/s commanded
constexpr int kMaxAngularMdegS_Normal  = 3000000;  // 3000°/s
constexpr int kMaxLinearMmS_Unlocked   = 80000;    // 80 m/s commanded = 4000 RPM = protocol cap
constexpr int kMaxAngularMdegS_Unlocked= 6000000;  // 6000°/s

// Throttle breakpoints.
constexpr double kThrottleMin = 0.10;
constexpr double kThrottleMid = 0.50;
constexpr double kThrottleMax = 1.00;

struct JoyConfig {
    const char *dev_path = "/dev/input/js0";
    int deadzone_raw = 3000;          // ~9% of full scale on the direction stick
    int throttle_deadzone_raw = 2500; // throttle stick rest-band → mid
    int send_interval_ms = 50;        // ~20 Hz send cadence
    int slew_linear_mm_s = 4000;      // ~1s 0→full at the 80000 mm/s protocol cap
    int slew_angular_mdeg_s = 800000; // max angular change per send tick
    int reopen_interval_ms = 2000;    // retry open every 2 s when disconnected
};

struct JoyState {
    int fd = -1;
    int axes[kJoyAxisCount] = {0};
    bool buttons[kJoyButtonCount] = {false};

    bool was_moving = false;        // edge-log "stick engaged / released"
    bool was_panic  = false;        // edge-log Y press
    bool was_unlocked = false;      // edge-log L2+R2 unlock state

    int last_sent_linear_mm_s = 0;
    int last_sent_angular_mdeg_s = 0;
    bool last_sent_valid = false;

    long last_send_ms = 0;
    long last_open_attempt_ms = 0;
};

long monotonic_ms() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<long>(ts.tv_sec) * 1000L + ts.tv_nsec / 1000000L;
}

int env_int(const char *name, int fallback) {
    const char *s = std::getenv(name);
    if (s == nullptr || s[0] == '\0') {
        return fallback;
    }
    char *end = nullptr;
    long v = std::strtol(s, &end, 10);
    if (end == s) {
        return fallback;
    }
    return static_cast<int>(v);
}

void joy_load_config_from_env(JoyConfig *cfg) {
    const char *dev = std::getenv("UAV_CAR_JOY_DEV");
    if (dev != nullptr && dev[0] != '\0') {
        cfg->dev_path = dev;
    }
    cfg->deadzone_raw        = env_int("UAV_CAR_JOY_DEADZONE",        cfg->deadzone_raw);
    cfg->send_interval_ms    = env_int("UAV_CAR_JOY_SEND_MS",         cfg->send_interval_ms);
    cfg->slew_linear_mm_s    = env_int("UAV_CAR_JOY_SLEW_LIN",        cfg->slew_linear_mm_s);
    cfg->slew_angular_mdeg_s = env_int("UAV_CAR_JOY_SLEW_ANG",        cfg->slew_angular_mdeg_s);
    cfg->reopen_interval_ms  = env_int("UAV_CAR_JOY_REOPEN_MS",       cfg->reopen_interval_ms);
    cfg->throttle_deadzone_raw = env_int("UAV_CAR_JOY_THROTTLE_DZ",   cfg->throttle_deadzone_raw);
}

void joy_reset_state(JoyState *s) {
    for (size_t i = 0; i < kJoyAxisCount; ++i) {
        s->axes[i] = 0;
    }
    for (size_t i = 0; i < kJoyButtonCount; ++i) {
        s->buttons[i] = false;
    }
    s->was_moving = false;
    s->was_panic  = false;
    s->was_unlocked = false;
}

void joy_close(JoyState *s, const char *reason) {
    if (s->fd >= 0) {
        log_warn(kTag, "joystick closed: %s", reason);
        close(s->fd);
        s->fd = -1;
    }
    joy_reset_state(s);
    if (s->last_sent_valid &&
        (s->last_sent_linear_mm_s != 0 || s->last_sent_angular_mdeg_s != 0)) {
        (void)car_set_velocity(0, 0);
        s->last_sent_linear_mm_s = 0;
        s->last_sent_angular_mdeg_s = 0;
        s->last_sent_valid = true;
    }
}

bool joy_drain(JoyState *s, const JoyConfig * /*cfg*/) {
    struct js_event ev;
    while (true) {
        ssize_t n = read(s->fd, &ev, sizeof(ev));
        if (n == static_cast<ssize_t>(sizeof(ev))) {
            uint8_t type = ev.type & ~JS_EVENT_INIT;
            if (type == JS_EVENT_AXIS && ev.number < kJoyAxisCount) {
                s->axes[ev.number] = ev.value;
            } else if (type == JS_EVENT_BUTTON && ev.number < kJoyButtonCount) {
                s->buttons[ev.number] = (ev.value != 0);
            }
            continue;
        }
        if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            return true;
        }
        return false;  // EOF or hard error => caller should close.
    }
}

int apply_deadzone_and_scale(int axis_raw, int deadzone, int max_out) {
    int absv = axis_raw < 0 ? -axis_raw : axis_raw;
    if (absv <= deadzone) {
        return 0;
    }
    long span = static_cast<long>(kJoyAxisMax) - deadzone;
    if (span <= 0) {
        return 0;
    }
    long scaled = static_cast<long>(absv - deadzone) * max_out / span;
    if (scaled > max_out) {
        scaled = max_out;
    }
    return axis_raw < 0 ? -static_cast<int>(scaled) : static_cast<int>(scaled);
}

int slew(int target, int current, int max_delta) {
    int diff = target - current;
    if (diff > max_delta) {
        return current + max_delta;
    }
    if (diff < -max_delta) {
        return current - max_delta;
    }
    return target;
}

// Throttle stick (right stick Y, axes[kAxisRY]) → throttle multiplier.
// Piecewise linear: push-up = Max, center = Mid, push-down = Min.
// A throttle deadzone glues the rest position firmly to Mid so noise on the
// stick doesn't oscillate the commanded velocity.
double joy_compute_throttle(int ry_raw, int dz) {
    int absv = ry_raw < 0 ? -ry_raw : ry_raw;
    if (absv <= dz) {
        return kThrottleMid;
    }
    // Strip the deadzone so the interpolation hits Mid exactly at the
    // boundary, not partway through.
    const double span = static_cast<double>(kJoyAxisMax) - dz;
    if (span <= 0.0) return kThrottleMid;
    if (ry_raw < 0) {
        // Up: mid → max as we push further up.
        double t = static_cast<double>(-ry_raw - dz) / span;
        if (t > 1.0) t = 1.0;
        return kThrottleMid + t * (kThrottleMax - kThrottleMid);
    }
    // Down: mid → min as we push further down.
    double t = static_cast<double>(ry_raw - dz) / span;
    if (t > 1.0) t = 1.0;
    return kThrottleMid - t * (kThrottleMid - kThrottleMin);
}

void joy_tick(JoyState *s, const JoyConfig *cfg) {
    long now = monotonic_ms();
    if (s->last_sent_valid && (now - s->last_send_ms) < cfg->send_interval_ms) {
        return;
    }

    int linear_target = 0;
    int angular_target = 0;
    bool panic = false;
    double throttle = kThrottleMid;
    bool unlocked = false;
    int lin_cap = kMaxLinearMmS_Normal;
    int ang_cap = kMaxAngularMdegS_Normal;

    if (s->fd >= 0) {
        // L2 + R2 both held → unlock full speed cap. Either alone → no effect.
        const bool l2_held = s->axes[kAxisLT] > kTriggerThreshold;
        const bool r2_held = s->axes[kAxisRT] > kTriggerThreshold;
        unlocked = l2_held && r2_held;
        lin_cap = unlocked ? kMaxLinearMmS_Unlocked   : kMaxLinearMmS_Normal;
        ang_cap = unlocked ? kMaxAngularMdegS_Unlocked: kMaxAngularMdegS_Normal;

        if (s->buttons[kBtnY]) {
            // Y panic-stop overrides everything — zero command regardless of sticks.
            panic = true;
        } else {
            throttle = joy_compute_throttle(s->axes[kAxisRY], cfg->throttle_deadzone_raw);

            // Direction stick scaled to *current cap* first; throttle multiplies after.
            // LY push-up = -32767 → negate so forward = positive.
            int dir_lin = apply_deadzone_and_scale(
                -s->axes[kAxisLY], cfg->deadzone_raw, lin_cap);
            // LX push-right = +32767. ROS REP-103: angular.z positive = CCW; so
            // negate so push-right yields negative angular → right turn.
            int dir_ang = apply_deadzone_and_scale(
                -s->axes[kAxisLX], cfg->deadzone_raw, ang_cap);

            linear_target  = static_cast<int>(dir_lin * throttle);
            angular_target = static_cast<int>(dir_ang * throttle);
        }
    }

    const bool moving = (linear_target != 0 || angular_target != 0);

    // Edge-log unlock state transitions so the operator can see L2+R2 take effect.
    if (unlocked != s->was_unlocked) {
        s->was_unlocked = unlocked;
        log_info(kTag, "joy: speed cap %s (cap=%dmm/s, %dmdeg/s)",
                 unlocked ? "UNLOCKED (L2+R2)" : "NORMAL (release L2/R2)",
                 lin_cap, ang_cap);
    }

    // Edge-log moving/panic state so the operator can audit input.
    if (panic != s->was_panic) {
        s->was_panic = panic;
        if (panic) log_info(kTag, "joy: Y panic-stop");
    }
    if (moving != s->was_moving) {
        s->was_moving = moving;
        if (moving) {
            log_info(kTag, "joy: stick engaged throttle=%.2f cap=%s (lin_max=%dmm/s ang_max=%dmdeg/s)",
                     throttle,
                     unlocked ? "UNLOCK" : "norm",
                     static_cast<int>(lin_cap * throttle),
                     static_cast<int>(ang_cap * throttle));
        } else if (!panic) {
            log_info(kTag, "joy: stick centered (slewing to zero)");
        }
    }

    int current_lin = s->last_sent_valid ? s->last_sent_linear_mm_s : 0;
    int current_ang = s->last_sent_valid ? s->last_sent_angular_mdeg_s : 0;
    int new_lin = slew(linear_target, current_lin, cfg->slew_linear_mm_s);
    int new_ang = slew(angular_target, current_ang, cfg->slew_angular_mdeg_s);

    if (s->last_sent_valid && new_lin == current_lin && new_ang == current_ang) {
        s->last_send_ms = now;
        return;
    }

    if (!car_set_velocity(new_lin, new_ang)) {
        log_warn(kTag, "joy: car_set_velocity(%d, %d) failed", new_lin, new_ang);
    }
    s->last_sent_linear_mm_s = new_lin;
    s->last_sent_angular_mdeg_s = new_ang;
    s->last_sent_valid = true;
    s->last_send_ms = now;
}

void joy_try_open(JoyState *s, const JoyConfig *cfg) {
    long now = monotonic_ms();
    if (s->fd >= 0) {
        return;
    }
    if (s->last_open_attempt_ms != 0 &&
        (now - s->last_open_attempt_ms) < cfg->reopen_interval_ms) {
        return;
    }
    s->last_open_attempt_ms = now;
    int fd = open(cfg->dev_path, O_RDONLY | O_NONBLOCK | O_CLOEXEC);
    if (fd < 0) {
        return;
    }
    s->fd = fd;
    joy_reset_state(s);
    log_info(kTag, "joystick opened: %s", cfg->dev_path);
}

}  // namespace

int main() {
    int server_fd;
    struct sockaddr_un addr{};
    struct pollfd pfds[2 + kMaxClients];  // server + clients + joystick
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

    JoyConfig joy_cfg;
    joy_load_config_from_env(&joy_cfg);
    JoyState joy_state;
    joy_try_open(&joy_state, &joy_cfg);

    log_info(kTag,
             "listening on %s can=%s joy=%s "
             "[LStick=direction (LY fwd/back, LX turn) | "
             "RStick-Y=throttle (up=%.0f%% mid=%.0f%% down=%.0f%%) | "
             "normal peak %dmm/s, L2+R2 unlock to %dmm/s | "
             "Y=panic-stop, release LStick=stop]",
             kSockPath,
             iface != nullptr && iface[0] != '\0' ? iface : "can3",
             joy_cfg.dev_path,
             kThrottleMax * 100.0,
             kThrottleMid * 100.0,
             kThrottleMin * 100.0,
             kMaxLinearMmS_Normal,
             kMaxLinearMmS_Unlocked);

    while (g_running) {
        int nfds = 0;
        pfds[nfds].fd = server_fd;
        pfds[nfds].events = POLLIN;
        ++nfds;
        const int client_pfd_start = nfds;
        for (int i = 0; i < nclients; ++i) {
            pfds[nfds].fd = client_fds[i];
            pfds[nfds].events = POLLIN;
            ++nfds;
        }
        const int joy_pfd_idx = (joy_state.fd >= 0) ? nfds : -1;
        if (joy_state.fd >= 0) {
            pfds[nfds].fd = joy_state.fd;
            pfds[nfds].events = POLLIN;
            ++nfds;
        }

        // Cap poll timeout at the joystick send interval so joy_tick runs
        // even when no events arrive (slew-down to zero, reopen attempts, etc.).
        int timeout_ms = joy_cfg.send_interval_ms;
        if (timeout_ms > 500) {
            timeout_ms = 500;
        }
        if (timeout_ms <= 0) {
            timeout_ms = 500;
        }

        if (poll(pfds, nfds, timeout_ms) < 0) {
            if (errno == EINTR) {
                continue;
            }
            log_error(kTag, "poll failed: %s", std::strerror(errno));
            break;
        }

        // Snapshot the joystick's revents now. A client accepted below takes
        // the pollfd slot right after the last client — which is exactly
        // where the joystick sits this iteration — so the accept path has to
        // overwrite that slot, and this value would be lost.
        const short joy_revents =
            (joy_pfd_idx >= 0) ? pfds[joy_pfd_idx].revents : static_cast<short>(0);

        if ((pfds[0].revents & POLLIN) != 0) {
            int client_fd = accept(server_fd, nullptr, nullptr);
            if (client_fd >= 0 && nclients < kMaxClients) {
                client_fds[nclients] = client_fd;
                buf_lens[nclients] = 0U;
                // Initialise this pollfd slot. poll() only wrote revents for
                // the indices it was handed, and this one was not among them
                // — here it still holds the JOYSTICK's fd and revents. Left
                // stale, the client loop below would read the joystick
                // device and then close() it on failure, killing tele-op,
                // while the brand-new client is dropped without its fd ever
                // being closed. Same defect diagnosed in proc_door
                // (2026-08-08), where it wedged the whole RPC chain.
                pfds[client_pfd_start + nclients].fd      = client_fd;
                pfds[client_pfd_start + nclients].events  = POLLIN;
                pfds[client_pfd_start + nclients].revents = 0;
                ++nclients;
            } else if (client_fd >= 0) {
                close(client_fd);
            }
        }

        for (int i = 0; i < nclients;) {
            const int idx = client_pfd_start + i;
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
                    // with its own fd/revents rather than the closed one's.
                    client_fds[i] = client_fds[last];
                    buf_lens[i] = buf_lens[last];
                    std::memcpy(bufs[i], bufs[last], buf_lens[i]);
                    pfds[idx] = pfds[client_pfd_start + last];
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

            const size_t consumed = static_cast<size_t>(start - bufs[i]);
            buf_lens[i] -= consumed;
            if (consumed > 0U && buf_lens[i] > 0U) {
                std::memmove(bufs[i], start, buf_lens[i]);
            }
            ++i;
        }

        if (joy_pfd_idx >= 0) {
            short rev = joy_revents;   // snapshot taken before accept()
            if ((rev & (POLLHUP | POLLERR | POLLNVAL)) != 0) {
                joy_close(&joy_state, "POLLHUP/POLLERR/POLLNVAL");
            } else if ((rev & POLLIN) != 0) {
                if (!joy_drain(&joy_state, &joy_cfg)) {
                    joy_close(&joy_state, "read failure");
                }
            }
        }

        joy_tick(&joy_state, &joy_cfg);
        if (joy_state.fd < 0) {
            joy_try_open(&joy_state, &joy_cfg);
        }
    }

    if (joy_state.fd >= 0) {
        close(joy_state.fd);
    }
    (void)car_set_velocity(0, 0);
    close(server_fd);
    unlink(kSockPath);
    return 0;
}
