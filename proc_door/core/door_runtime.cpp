#include "door_runtime.h"

#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include <fcntl.h>
#include <poll.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <unistd.h>

#include "modbus_rtu.h"

extern "C" {
#include "log.h"
}

// proc_door — RS485 / Modbus-RTU digital-IO module (中盛 DIO series).
//
// Owns one serial port and exposes it as a JSON-RPC service on a Unix
// stream socket, same shape as proc_airport / proc_gripper:
//
//   /tmp/uav_proc_door.sock, newline-delimited JSON
//
// ── Field wiring (RK3588 airport, confirmed 2026-08-08) ──────────────────
//
//   Y1 + Y2  停机坪升降电机   Y1=1,Y2=0 上升   Y1=0,Y2=1 下降   both 0 停
//   Y3       舱门电机使能     1 = 通电运动
//   Y4       舱门方向         1 = 正转(开舱门)   0 = 反转(关舱门)
//   X1       停机坪 上限位 (Top)
//   X2       停机坪 下限位 (Bottom)
//   X3       舱门 开到位
//   X4       舱门 关到位
//
// All coils de-energised is the safe/default state.
//
// Two motion styles fall out of that, so an Axis carries a mode:
//
//   kDualCoil — one coil per direction (停机坪). Never both at once.
//   kRunDir   — one enable coil + one direction coil (舱门). Direction is
//               only ever changed while the enable coil is OFF.
//
// ── Motion is asynchronous ───────────────────────────────────────────────
// Commands energise and return immediately; the poll loop supervises the
// limit switches and drops the coils on arrival / timeout. That is what
// makes a STOP request reachable *while* the axis is moving — a blocking
// wait inside the handler would deadlock the single-threaded server for
// the whole travel. Callers watch door.get_status (which HostGUI polls
// anyway for the sensor LEDs) to see the move finish.

namespace {

constexpr const char *kTag = "proc_door";
constexpr int         kMaxClients = 8;
constexpr size_t      kBufSize = 4096;
constexpr int         kReopenIntervalMs = 2000;

volatile std::sig_atomic_t g_stop = 0;

void on_signal(int /*sig*/) { g_stop = 1; }

int now_ms() {
    using namespace std::chrono;
    return static_cast<int>(
        duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count());
}

/* ---------------------------------------------------------------- env ---- */

const char *env_str_or(const char *name, const char *fallback) {
    const char *e = std::getenv(name);
    return (e != nullptr && e[0] != '\0') ? e : fallback;
}

int env_int_or(const char *name, int fallback) {
    const char *e = std::getenv(name);
    if (e == nullptr || e[0] == '\0') return fallback;
    char *end = nullptr;
    const long v = std::strtol(e, &end, 10);
    if (end == e || *end != '\0') return fallback;
    return static_cast<int>(v);
}

bool env_bool_or(const char *name, bool fallback) {
    const char *e = std::getenv(name);
    if (e == nullptr || e[0] == '\0') return fallback;
    return (std::strcmp(e, "1") == 0 || std::strcmp(e, "true") == 0 ||
            std::strcmp(e, "yes") == 0 || std::strcmp(e, "on") == 0);
}

/* --------------------------------------------------------------- json ---- */
// Same flat-scan approach the other proc_* services use: good enough for
// the small, machine-generated requests we get, and dependency-free. The
// scan retries on the next occurrence of a key when the first one turns
// out not to be a real "key": value pair (e.g. a string value that happens
// to spell the key name).

bool json_get_token(const char *json, const char *key, char *out, size_t out_size) {
    if (json == nullptr || out == nullptr || out_size == 0U) return false;

    char needle[128];
    std::snprintf(needle, sizeof(needle), "\"%s\"", key);
    const size_t needle_len = std::strlen(needle);

    const char *search = json;
    while ((search = std::strstr(search, needle)) != nullptr) {
        const char *p = search + needle_len;
        search = p;
        while (*p == ' ' || *p == '\t') ++p;
        if (*p != ':') continue;
        ++p;
        while (*p == ' ' || *p == '\t') ++p;

        size_t i = 0;
        if (*p == '"') {
            ++p;
            while (*p != '\0' && *p != '"' && i + 1U < out_size) out[i++] = *p++;
        } else {
            while (*p != '\0' && *p != ',' && *p != '}' && *p != ']' &&
                   *p != '\n' && *p != '\r' && i + 1U < out_size) {
                out[i++] = *p++;
            }
            while (i > 0U && (out[i - 1U] == ' ' || out[i - 1U] == '\t')) --i;
        }
        out[i] = '\0';
        return i > 0U;
    }
    return false;
}

bool json_get_bool(const char *json, const char *key, bool *out) {
    char token[16];
    if (out == nullptr || !json_get_token(json, key, token, sizeof(token))) return false;
    if (std::strcmp(token, "true") == 0 || std::strcmp(token, "1") == 0) {
        *out = true;
        return true;
    }
    if (std::strcmp(token, "false") == 0 || std::strcmp(token, "0") == 0) {
        *out = false;
        return true;
    }
    return false;
}

int json_get_int(const char *json, const char *key, int fallback) {
    char token[32];
    if (!json_get_token(json, key, token, sizeof(token))) return fallback;
    char *end = nullptr;
    const long v = std::strtol(token, &end, 10);
    if (end == token) return fallback;
    return static_cast<int>(v);
}

std::string json_escape(const std::string &in) {
    std::string out;
    out.reserve(in.size() + 8U);
    for (const char c : in) {
        switch (c) {
            case '"':  out += "\\\""; break;
            case '\\': out += "\\\\"; break;
            case '\n': out += "\\n";  break;
            case '\r': out += "\\r";  break;
            case '\t': out += "\\t";  break;
            default:
                if (static_cast<unsigned char>(c) < 0x20U) {
                    char buf[8];
                    std::snprintf(buf, sizeof(buf), "\\u%04x",
                                  static_cast<unsigned>(static_cast<unsigned char>(c)));
                    out += buf;
                } else {
                    out += c;
                }
        }
    }
    return out;
}

std::string jbool(bool v) { return v ? "true" : "false"; }

std::string extract_id_fragment(const char *line) {
    char id[64];
    if (!json_get_token(line, "id", id, sizeof(id)) || std::strcmp(id, "null") == 0) {
        return "\"id\":null";
    }
    return std::string("\"id\":") + id;
}

std::string make_error(const std::string &id_fragment, int code, const std::string &message) {
    return std::string("{\"jsonrpc\":\"2.0\",") + id_fragment +
           ",\"error\":{\"code\":" + std::to_string(code) +
           ",\"message\":\"" + json_escape(message) + "\"}}\n";
}

// body is the inner content of "result", without the surrounding braces.
std::string make_result(const std::string &id_fragment, const std::string &body) {
    return std::string("{\"jsonrpc\":\"2.0\",") + id_fragment +
           ",\"result\":{" + body + "}}\n";
}

std::string make_ok(const std::string &id_fragment, bool ok, const std::string &extra = {}) {
    std::string body = std::string("\"ok\":") + jbool(ok);
    if (!extra.empty()) body += "," + extra;
    return make_result(id_fragment, body);
}

std::string bits_json(const std::vector<uint8_t> &bits, bool valid) {
    if (!valid || bits.empty()) return "[]";
    std::string s = "[";
    for (size_t i = 0; i < bits.size(); ++i) {
        if (i != 0U) s += ",";
        s += (bits[i] != 0U) ? "1" : "0";
    }
    s += "]";
    return s;
}

std::string hex_json(const uint8_t *data, size_t len) {
    std::string s;
    char buf[4];
    for (size_t i = 0; i < len; ++i) {
        if (i != 0U) s += " ";
        std::snprintf(buf, sizeof(buf), "%02X", data[i]);
        s += buf;
    }
    return s;
}

/* -------------------------------------------------------------- config --- */

// Motion style of one axis.
enum class AxisMode {
    kDualCoil,   // coil_a = direction A, coil_b = direction B (停机坪)
    kRunDir,     // run coil + direction coil (舱门)
};

struct AxisConfig {
    const char *key = "";          // json key / log tag: "hatch" | "helipad"
    AxisMode    mode = AxisMode::kDualCoil;

    // kDualCoil: ch_a / ch_b are the two direction coils (1-based Y number).
    // kRunDir:   ch_a is the RUN coil, ch_b is the DIRECTION coil.
    int  ch_a = 0;
    int  ch_b = 0;
    bool dir_level_a = true;       // kRunDir: direction-coil level meaning "A"

    int  in_a = 0;                 // limit input for end A (1-based X number)
    int  in_b = 0;                 // limit input for end B
    int  timeout_ms = 20000;

    // Human-readable state names, so the JSON says "opened"/"top" rather
    // than a generic A/B the GUI would have to translate.
    const char *name_a = "a";
    const char *name_b = "b";
    const char *name_moving_a = "moving_a";
    const char *name_moving_b = "moving_b";
    const char *verb_a = "A";      // for logs: 开 / 上升
    const char *verb_b = "B";
};

struct DoorConfig {
    std::string uart_path;
    int         baud      = 38400;
    uint8_t     slave     = 1U;
    int         out_count = 8;
    int         in_count  = 8;
    int         poll_ms   = 150;
    std::string sock_path;
    bool        in_active_low = false;
    int         dir_settle_ms = 150;   // gap between direction set and RUN on
    int         modbus_timeout_ms = 150;
    bool        release_on_exit = true;

    AxisConfig hatch;     // 舱门
    AxisConfig pad;       // 停机坪

    static DoorConfig from_env() {
        DoorConfig c;
        c.uart_path = env_str_or("UAV_DOOR_UART_PATH", "/dev/ttyUSB0");
        c.sock_path = env_str_or("UAV_DOOR_SOCK", "/tmp/uav_proc_door.sock");
        c.baud      = env_int_or("UAV_DOOR_BAUD", 38400);
        c.out_count = env_int_or("UAV_DOOR_OUT_COUNT", 8);
        c.in_count  = env_int_or("UAV_DOOR_IN_COUNT", 8);
        c.poll_ms   = env_int_or("UAV_DOOR_POLL_MS", 150);

        int addr = env_int_or("UAV_DOOR_ADDR", 1);
        if (addr < 1) addr = 1;
        if (addr > 255) addr = 255;
        c.slave = static_cast<uint8_t>(addr);

        if (c.out_count < 0) c.out_count = 0;
        if (c.out_count > 64) c.out_count = 64;
        if (c.in_count  < 0) c.in_count  = 0;
        if (c.in_count  > 64) c.in_count  = 64;
        if (c.poll_ms   < 0) c.poll_ms   = 0;
        // Limit-switch overshoot is bounded by the poll period, so don't
        // let it be configured slower than ~2 Hz.
        if (c.poll_ms   > 0 && c.poll_ms < 50)  c.poll_ms = 50;
        if (c.poll_ms   > 500) c.poll_ms = 500;

        c.in_active_low   = env_bool_or("UAV_DOOR_IN_ACTIVE_LOW", false);
        c.dir_settle_ms   = env_int_or("UAV_DOOR_DIR_SETTLE_MS", 150);
        // Paid in full on every read when the module is silent, so keep it
        // small — the module normally answers in ~5 ms at 38400.
        c.modbus_timeout_ms = env_int_or("UAV_DOOR_MODBUS_TIMEOUT_MS", 150);
        if (c.modbus_timeout_ms < 30)   c.modbus_timeout_ms = 30;
        if (c.modbus_timeout_ms > 1000) c.modbus_timeout_ms = 1000;
        c.release_on_exit = env_bool_or("UAV_DOOR_RELEASE_ON_EXIT", true);
        if (c.dir_settle_ms < 0) c.dir_settle_ms = 0;
        if (c.dir_settle_ms > 3000) c.dir_settle_ms = 3000;

        // ── 舱门: Y3 = 使能, Y4 = 方向 (1 正转 = 开) ──
        c.hatch.key           = "hatch";
        c.hatch.mode          = AxisMode::kRunDir;
        c.hatch.ch_a          = env_int_or("UAV_DOOR_HATCH_RUN_CH", 3);
        c.hatch.ch_b          = env_int_or("UAV_DOOR_HATCH_DIR_CH", 4);
        c.hatch.dir_level_a   = (env_int_or("UAV_DOOR_HATCH_OPEN_LEVEL", 1) != 0);
        c.hatch.in_a          = env_int_or("UAV_DOOR_HATCH_OPEN_IN", 3);
        c.hatch.in_b          = env_int_or("UAV_DOOR_HATCH_CLOSE_IN", 4);
        c.hatch.timeout_ms    = env_int_or("UAV_DOOR_HATCH_TIMEOUT_MS", 20000);
        c.hatch.name_a        = "opened";
        c.hatch.name_b        = "closed";
        c.hatch.name_moving_a = "opening";
        c.hatch.name_moving_b = "closing";
        c.hatch.verb_a        = "open";
        c.hatch.verb_b        = "close";

        // ── 停机坪: Y1 = 上升, Y2 = 下降 ──
        c.pad.key           = "helipad";
        c.pad.mode          = AxisMode::kDualCoil;
        c.pad.ch_a          = env_int_or("UAV_DOOR_PAD_UP_CH", 1);
        c.pad.ch_b          = env_int_or("UAV_DOOR_PAD_DOWN_CH", 2);
        c.pad.in_a          = env_int_or("UAV_DOOR_PAD_TOP_IN", 1);
        c.pad.in_b          = env_int_or("UAV_DOOR_PAD_BOTTOM_IN", 2);
        c.pad.timeout_ms    = env_int_or("UAV_DOOR_PAD_TIMEOUT_MS", 30000);
        c.pad.name_a        = "top";
        c.pad.name_b        = "bottom";
        c.pad.name_moving_a = "rising";
        c.pad.name_moving_b = "lowering";
        c.pad.verb_a        = "up";
        c.pad.verb_b        = "down";

        if (c.hatch.timeout_ms < 500) c.hatch.timeout_ms = 500;
        if (c.pad.timeout_ms   < 500) c.pad.timeout_ms   = 500;
        return c;
    }
};

/* -------------------------------------------------------------- device --- */

// Everything the RPC layer needs to know about one axis after a command.
struct MoveResult {
    bool        ok = false;
    std::string reason;     // started | already | busy_reversed | no_link | error text
};

class DoorDevice {
 public:
    explicit DoorDevice(const DoorConfig &cfg) : cfg_(cfg) {
        inputs_.assign(static_cast<size_t>(cfg_.in_count), 0U);
        outputs_.assign(static_cast<size_t>(cfg_.out_count), 0U);
        bus_.set_timeout_ms(cfg_.modbus_timeout_ms);
    }

    const DoorConfig &cfg() const { return cfg_; }
    bool connected() const { return bus_.is_open() && link_ok_; }
    const std::string &last_error() const { return last_error_; }

    /* ---- link management ---- */

    bool ensure_open() {
        if (bus_.is_open()) return true;
        const int now = now_ms();
        if (next_reopen_ms_ != 0 && now - next_reopen_ms_ < 0) return false;
        next_reopen_ms_ = now + kReopenIntervalMs;

        if (!bus_.open(cfg_.uart_path, cfg_.baud)) {
            last_error_ = bus_.last_error();
            if (!open_failure_logged_) {
                log_error(kTag, "open %s @%d failed: %s", cfg_.uart_path.c_str(),
                          cfg_.baud, last_error_.c_str());
                open_failure_logged_ = true;
            }
            return false;
        }
        open_failure_logged_ = false;
        link_ok_ = false;
        fail_streak_ = 0;
        log_info(kTag, "serial open %s @%d 8N1 addr=%u", cfg_.uart_path.c_str(),
                 cfg_.baud, static_cast<unsigned>(cfg_.slave));

        // Handshake exactly like the panel does: holding register 0x32
        // carries the module's own station id. Silence here means wrong
        // baud / station / A-B swapped, and is worth saying out loud once.
        uint16_t reg = 0;
        if (bus_.read_holding(cfg_.slave, 0x0032U, 1U, &reg)) {
            log_info(kTag, "module online, station id = %u", static_cast<unsigned>(reg));
            link_ok_ = true;
        } else {
            // Port is there, module is silent. Distinct from "node missing"
            // and needs a different fix, so keep the wording distinct too —
            // it is what the GUI shows the operator.
            last_error_ = "no handshake: " + bus_.last_error();
            log_warn(kTag, "no handshake response (%s) - check A/B wiring, baud, station",
                     bus_.last_error().c_str());
        }
        return true;
    }

    void reconnect() {
        abort_all("reconnect");
        bus_.close();
        next_reopen_ms_ = 0;
        inputs_valid_ = false;
        outputs_valid_ = false;
        link_ok_ = false;
        (void)ensure_open();
    }

    /* ---- background tick: refresh cache, then supervise both axes ---- */

    void poll_tick() {
        if (!ensure_open()) {
            // Link is down while an axis believes it is moving: we can no
            // longer see limits or drop coils, so give up on the move
            // rather than reporting motion that nobody is supervising.
            abort_all("link down");
            return;
        }

        // Port is open but the module is not answering (board powered off,
        // A/B unplugged). Every read then costs a full serial timeout, and
        // at the 150 ms supervision cadence that starves this
        // single-threaded loop: requests pile up on the socket, proc_gateway
        // blocks on its forward, and HostGUI's 300 ms status poll turns a
        // dead relay board into a wedged RPC chain for the whole console.
        // Probe at the reconnect cadence instead until it talks again.
        if (!link_ok_) {
            const int now = now_ms();
            if (next_probe_ms_ != 0 && now - next_probe_ms_ < 0) {
                abort_all("link down");
                return;
            }
            next_probe_ms_ = now + kReopenIntervalMs;
            (void)refresh_inputs();
            if (!link_ok_) {
                abort_all("link down");
                return;
            }
            // Module just came back — fall through to a normal scan.
        }

        (void)refresh_inputs();
        if (++output_tick_ >= 4) {          // outputs change rarely — 1 in 4
            output_tick_ = 0;
            (void)refresh_outputs();
        }
        supervise(hatch_, cfg_.hatch);
        supervise(pad_,   cfg_.pad);
    }

    bool refresh_inputs() {
        if (cfg_.in_count <= 0) return true;
        if (!ensure_open()) return false;

        std::vector<uint8_t> bits(static_cast<size_t>(cfg_.in_count), 0U);
        if (!bus_.read_discrete_inputs(cfg_.slave, 0U,
                                       static_cast<uint16_t>(cfg_.in_count), bits.data())) {
            note_failure("read inputs");
            return false;
        }
        note_success();

        if (!inputs_valid_ || bits != inputs_) {
            if (inputs_valid_) {
                for (size_t i = 0; i < bits.size(); ++i) {
                    if (bits[i] != inputs_[i]) {
                        log_info(kTag, "input X%zu %s", i + 1U, (bits[i] != 0U) ? "1" : "0");
                    }
                }
            }
            input_change_seq_++;
            last_input_change_ms_ = now_ms();
        }
        inputs_ = bits;
        inputs_valid_ = true;
        last_input_read_ms_ = now_ms();
        return true;
    }

    bool refresh_outputs() {
        if (cfg_.out_count <= 0) return true;
        if (!ensure_open()) return false;

        std::vector<uint8_t> bits(static_cast<size_t>(cfg_.out_count), 0U);
        if (!bus_.read_coils(cfg_.slave, 0U,
                             static_cast<uint16_t>(cfg_.out_count), bits.data())) {
            note_failure("read outputs");
            return false;
        }
        note_success();
        outputs_ = bits;
        outputs_valid_ = true;
        return true;
    }

    /* ---- raw output control (channel is 1-based, like Y1..Yn) ---- */

    bool set_relay(int channel, bool on) {
        if (channel < 1 || channel > cfg_.out_count) {
            last_error_ = "channel out of range (1.." + std::to_string(cfg_.out_count) + ")";
            return false;
        }
        // A manual write to a coil an axis is currently driving would make
        // the supervisor and the operator fight over it. The operator wins;
        // the axis gives up its claim.
        release_axis_claim(channel);
        return write_coil(channel, on);
    }

    bool set_all(bool on) {
        if (cfg_.out_count <= 0) {
            last_error_ = "no output channels configured";
            return false;
        }
        if (!ensure_open()) return false;
        if (!bus_.write_coils(cfg_.slave, 0U,
                              static_cast<uint16_t>(cfg_.out_count), on)) {
            note_failure("write coils");
            return false;
        }
        note_success();
        outputs_.assign(static_cast<size_t>(cfg_.out_count), on ? 1U : 0U);
        outputs_valid_ = true;
        abort_all(on ? "set_all(on)" : "set_all(off)");
        log_info(kTag, "all relays -> %s", on ? "ON" : "OFF");
        return true;
    }

    // Momentary close: ON, hold, OFF. Blocks the service loop for `ms`, so
    // it is clamped hard. Only meant for auxiliary outputs, never for the
    // motor axes (they are supervised asynchronously instead).
    bool pulse(int channel, int ms) {
        if (ms < 20) ms = 20;
        if (ms > 3000) ms = 3000;
        if (!set_relay(channel, true)) return false;
        usleep(static_cast<useconds_t>(ms) * 1000U);
        return set_relay(channel, false);
    }

    /* ---- axis commands ---- */

    MoveResult hatch_open()  { return start(hatch_, cfg_.hatch, true); }
    MoveResult hatch_close() { return start(hatch_, cfg_.hatch, false); }
    bool       hatch_stop()  { return stop(hatch_, cfg_.hatch, "rpc"); }

    MoveResult pad_up()   { return start(pad_, cfg_.pad, true); }
    MoveResult pad_down() { return start(pad_, cfg_.pad, false); }
    bool       pad_stop() { return stop(pad_, cfg_.pad, "rpc"); }

    // Emergency: drop every coil the two axes own, in one 0x0F when we can.
    bool stop_all() {
        bool ok = true;
        if (!stop(hatch_, cfg_.hatch, "stop_all")) ok = false;
        if (!stop(pad_,   cfg_.pad,   "stop_all")) ok = false;
        log_info(kTag, "stop_all (ok=%d)", ok ? 1 : 0);
        return ok;
    }

    // Best-effort de-energise at shutdown, so a service restart can never
    // leave a motor running into a hard stop.
    void release_outputs() {
        if (!cfg_.release_on_exit || !bus_.is_open()) return;
        for (int ch : {cfg_.hatch.ch_a, cfg_.hatch.ch_b, cfg_.pad.ch_a, cfg_.pad.ch_b}) {
            if (ch > 0) (void)bus_.write_coil(cfg_.slave, static_cast<uint16_t>(ch - 1), false);
        }
    }

    /* ---- observable state ---- */

    bool input_active(int index_1based) const {
        if (!inputs_valid_) return false;
        if (index_1based < 1 || index_1based > static_cast<int>(inputs_.size())) return false;
        const bool bit = inputs_[static_cast<size_t>(index_1based - 1)] != 0U;
        return cfg_.in_active_low ? !bit : bit;
    }

    std::string axis_json(const char *key) const {
        if (std::strcmp(key, cfg_.hatch.key) == 0) return axis_body(hatch_, cfg_.hatch);
        return axis_body(pad_, cfg_.pad);
    }

    const std::vector<uint8_t> &inputs() const { return inputs_; }
    const std::vector<uint8_t> &outputs() const { return outputs_; }
    bool inputs_valid() const { return inputs_valid_; }
    bool outputs_valid() const { return outputs_valid_; }
    int  input_change_seq() const { return input_change_seq_; }
    int  input_age_ms() const { return inputs_valid_ ? (now_ms() - last_input_read_ms_) : -1; }
    int  last_change_age_ms() const {
        return (last_input_change_ms_ == 0) ? -1 : (now_ms() - last_input_change_ms_);
    }

    int raw(const uint8_t *body, size_t len, uint8_t *rx, size_t rx_cap, size_t expect) {
        if (!ensure_open()) return -1;
        const int n = bus_.transact_raw(body, len, rx, rx_cap, expect);
        if (n < 0) note_failure("raw"); else note_success();
        return n;
    }

 private:
    // Live state of one axis.
    struct AxisState {
        bool        moving = false;
        bool        to_a = true;        // direction of the current/last move
        int         start_ms = 0;
        std::string reason;             // outcome of the last finished move
        int         elapsed_ms = 0;
    };

    /* ---- coil helpers ---- */

    bool write_coil(int channel, bool on) {
        if (channel < 1) return true;               // not wired → nothing to do
        if (!ensure_open()) return false;
        if (!bus_.write_coil(cfg_.slave, static_cast<uint16_t>(channel - 1), on)) {
            note_failure("write coil");
            return false;
        }
        note_success();
        if (outputs_valid_ && channel <= static_cast<int>(outputs_.size())) {
            outputs_[static_cast<size_t>(channel - 1)] = on ? 1U : 0U;
        }
        log_info(kTag, "relay Y%d -> %s", channel, on ? "ON" : "OFF");
        return true;
    }

    // Cut power to an axis without touching its direction coil. For
    // kRunDir that is the RUN coil only; for kDualCoil, both coils.
    bool deenergise(const AxisConfig &c) {
        bool ok = true;
        if (c.mode == AxisMode::kRunDir) {
            if (!write_coil(c.ch_a, false)) ok = false;
        } else {
            if (!write_coil(c.ch_a, false)) ok = false;
            if (!write_coil(c.ch_b, false)) ok = false;
        }
        return ok;
    }

    bool owns_channel(const AxisConfig &c, int channel) const {
        return channel > 0 && (channel == c.ch_a || channel == c.ch_b);
    }

    void release_axis_claim(int channel) {
        if (hatch_.moving && owns_channel(cfg_.hatch, channel)) {
            hatch_.moving = false;
            hatch_.reason = "manual_override";
            log_warn(kTag, "hatch move dropped: Y%d written manually", channel);
        }
        if (pad_.moving && owns_channel(cfg_.pad, channel)) {
            pad_.moving = false;
            pad_.reason = "manual_override";
            log_warn(kTag, "helipad move dropped: Y%d written manually", channel);
        }
    }

    void abort_all(const char *why) {
        if (hatch_.moving) {
            hatch_.moving = false;
            hatch_.reason = why;
        }
        if (pad_.moving) {
            pad_.moving = false;
            pad_.reason = why;
        }
    }

    /* ---- axis motion ---- */

    MoveResult start(AxisState &st, const AxisConfig &c, bool to_a) {
        MoveResult r;
        const char *verb = to_a ? c.verb_a : c.verb_b;

        if (c.ch_a <= 0) {
            r.reason = "axis not configured";
            return r;
        }
        if (!ensure_open() || !connected()) {
            r.reason = last_error_.empty() ? "module offline" : last_error_;
            return r;
        }

        // Refresh limits before deciding — the cache may be a poll old and
        // we do not want to drive into an already-reached hard stop.
        (void)refresh_inputs();
        const int target_in = to_a ? c.in_a : c.in_b;
        if (target_in > 0 && input_active(target_in)) {
            // Already there. Make sure nothing is still energised and say so.
            (void)deenergise(c);
            st.moving = false;
            st.reason = "already";
            st.elapsed_ms = 0;
            log_info(kTag, "%s %s ignored: already at limit X%d", c.key, verb, target_in);
            r.ok = true;
            r.reason = "already";
            return r;
        }

        // Reversing mid-travel: always cut power first, and for kRunDir let
        // the contactor settle before flipping the direction coil. Driving
        // a direction change through a live motor is how contactors weld.
        const bool was_moving = st.moving;
        if (!deenergise(c)) {
            st.moving = false;
            r.reason = last_error_;
            return r;
        }
        if (was_moving && st.to_a != to_a && cfg_.dir_settle_ms > 0) {
            usleep(static_cast<useconds_t>(cfg_.dir_settle_ms) * 1000U);
        }

        if (c.mode == AxisMode::kRunDir) {
            const bool dir_level = to_a ? c.dir_level_a : !c.dir_level_a;
            if (!write_coil(c.ch_b, dir_level)) {
                r.reason = last_error_;
                return r;
            }
            if (cfg_.dir_settle_ms > 0) {
                usleep(static_cast<useconds_t>(cfg_.dir_settle_ms) * 1000U);
            }
            if (!write_coil(c.ch_a, true)) {    // RUN
                (void)write_coil(c.ch_b, false);
                r.reason = last_error_;
                return r;
            }
        } else {
            if (!write_coil(to_a ? c.ch_a : c.ch_b, true)) {
                (void)deenergise(c);
                r.reason = last_error_;
                return r;
            }
        }

        st.moving = true;
        st.to_a = to_a;
        st.start_ms = now_ms();
        st.reason = "started";
        st.elapsed_ms = 0;

        if (target_in <= 0) {
            log_warn(kTag, "%s %s started with no limit input configured - "
                           "will run until timeout (%d ms) or stop",
                     c.key, verb, c.timeout_ms);
        } else {
            log_info(kTag, "%s %s started (limit X%d, timeout %d ms)",
                     c.key, verb, target_in, c.timeout_ms);
        }
        r.ok = true;
        r.reason = "started";
        return r;
    }

    bool stop(AxisState &st, const AxisConfig &c, const char *why) {
        const bool was_moving = st.moving;
        st.moving = false;
        bool ok = deenergise(c);
        // Park the direction coil back at the default (all-off) state.
        if (c.mode == AxisMode::kRunDir && !write_coil(c.ch_b, false)) ok = false;
        if (was_moving) {
            st.elapsed_ms = now_ms() - st.start_ms;
            st.reason = "stopped";
            log_info(kTag, "%s stopped by %s after %d ms", c.key, why, st.elapsed_ms);
        }
        return ok;
    }

    // Called every poll tick for each axis: watch the target limit and the
    // timeout, and cut power on whichever fires first.
    void supervise(AxisState &st, const AxisConfig &c) {
        if (!st.moving) return;

        const int elapsed = now_ms() - st.start_ms;
        const int target_in = st.to_a ? c.in_a : c.in_b;

        if (target_in > 0 && input_active(target_in)) {
            st.moving = false;
            st.elapsed_ms = elapsed;
            st.reason = "reached";
            (void)deenergise(c);
            if (c.mode == AxisMode::kRunDir) (void)write_coil(c.ch_b, false);
            log_info(kTag, "%s %s reached limit X%d in %d ms",
                     c.key, st.to_a ? c.verb_a : c.verb_b, target_in, elapsed);
            return;
        }

        if (elapsed >= c.timeout_ms) {
            st.moving = false;
            st.elapsed_ms = elapsed;
            st.reason = "timeout";
            (void)deenergise(c);
            if (c.mode == AxisMode::kRunDir) (void)write_coil(c.ch_b, false);
            log_warn(kTag, "%s %s TIMEOUT after %d ms - power cut",
                     c.key, st.to_a ? c.verb_a : c.verb_b, elapsed);
        }
    }

    const char *state_text(const AxisState &st, const AxisConfig &c) const {
        if (st.moving) return st.to_a ? c.name_moving_a : c.name_moving_b;
        if (!inputs_valid_) return "unknown";
        const bool a = (c.in_a > 0) && input_active(c.in_a);
        const bool b = (c.in_b > 0) && input_active(c.in_b);
        if (a && b) return "fault";          // both limits → sensor/wiring problem
        if (a) return c.name_a;
        if (b) return c.name_b;
        if (c.in_a <= 0 && c.in_b <= 0) return "unknown";
        return "between";
    }

    std::string axis_body(const AxisState &st, const AxisConfig &c) const {
        std::string s = "{";
        s += "\"state\":\"" + std::string(state_text(st, c)) + "\"";
        s += ",\"moving\":" + jbool(st.moving);
        s += ",\"" + std::string(c.name_a) + "\":" +
             jbool((c.in_a > 0) && input_active(c.in_a));
        s += ",\"" + std::string(c.name_b) + "\":" +
             jbool((c.in_b > 0) && input_active(c.in_b));
        s += ",\"reason\":\"" + json_escape(st.reason) + "\"";
        s += ",\"elapsed_ms\":" +
             std::to_string(st.moving ? (now_ms() - st.start_ms) : st.elapsed_ms);
        s += ",\"timeout_ms\":" + std::to_string(c.timeout_ms);
        s += "}";
        return s;
    }

    void note_failure(const char *what) {
        last_error_ = std::string(what) + ": " + bus_.last_error();
        ++fail_streak_;
        // A handful of consecutive failures means the link is gone (adapter
        // unplugged, module powered off). Drop the fd so ensure_open() does
        // a full reopen instead of writing into a dead handle forever.
        if (fail_streak_ >= 5) {
            if (link_ok_) {
                log_error(kTag, "link lost after %d failures (%s) - reopening",
                          fail_streak_, last_error_.c_str());
            }
            link_ok_ = false;
            bus_.close();
            inputs_valid_ = false;
            outputs_valid_ = false;
            next_reopen_ms_ = now_ms() + kReopenIntervalMs;
            fail_streak_ = 0;
            abort_all("link lost");
        }
    }

    void note_success() {
        if (!link_ok_) log_info(kTag, "link up");
        link_ok_ = true;
        fail_streak_ = 0;
        last_error_.clear();
    }

    DoorConfig           cfg_;
    ModbusRtu            bus_;
    std::vector<uint8_t> inputs_;
    std::vector<uint8_t> outputs_;
    AxisState            hatch_;
    AxisState            pad_;
    bool                 inputs_valid_  = false;
    bool                 outputs_valid_ = false;
    bool                 link_ok_       = false;
    bool                 open_failure_logged_ = false;
    int                  fail_streak_   = 0;
    int                  next_reopen_ms_ = 0;
    int                  next_probe_ms_ = 0;   // link-down probe backoff
    int                  last_input_read_ms_ = 0;
    int                  last_input_change_ms_ = 0;
    int                  input_change_seq_ = 0;
    int                  output_tick_ = 0;
    std::string          last_error_;
};

/* ------------------------------------------------------------ dispatch --- */

std::string status_body(DoorDevice &dev) {
    const DoorConfig &c = dev.cfg();
    std::string body;
    body += "\"ok\":true";
    body += ",\"connected\":" + jbool(dev.connected());
    body += ",\"port\":\"" + json_escape(c.uart_path) + "\"";
    body += ",\"baud\":" + std::to_string(c.baud);
    body += ",\"addr\":" + std::to_string(static_cast<int>(c.slave));
    body += ",\"hatch\":" + dev.axis_json("hatch");
    body += ",\"helipad\":" + dev.axis_json("helipad");
    body += ",\"in_count\":" + std::to_string(c.in_count);
    body += ",\"out_count\":" + std::to_string(c.out_count);
    body += ",\"inputs\":" + bits_json(dev.inputs(), dev.inputs_valid());
    body += ",\"outputs\":" + bits_json(dev.outputs(), dev.outputs_valid());
    body += ",\"input_age_ms\":" + std::to_string(dev.input_age_ms());
    body += ",\"last_change_ms\":" + std::to_string(dev.last_change_age_ms());
    body += ",\"change_seq\":" + std::to_string(dev.input_change_seq());
    if (!dev.connected() && !dev.last_error().empty()) {
        body += ",\"error\":\"" + json_escape(dev.last_error()) + "\"";
    }
    return body;
}

std::string move_reply(const std::string &id, const MoveResult &r,
                       DoorDevice &dev, const char *axis_key) {
    std::string extra = "\"reason\":\"" + json_escape(r.reason) + "\"";
    extra += ",\"" + std::string(axis_key) + "\":" + dev.axis_json(axis_key);
    return make_ok(id, r.ok, extra);
}

std::string handle_request(const char *line, DoorDevice &dev) {
    const std::string id = extract_id_fragment(line);

    char method[64] = {};
    if (!json_get_token(line, "method", method, sizeof(method))) {
        return make_error(id, -32600, "missing method");
    }

    if (std::strcmp(method, "system.ping") == 0) {
        return make_ok(id, true);
    }

    /* ── 舱门 ─────────────────────────────────────────────────────────── */
    if (std::strcmp(method, "door.open") == 0) {
        return move_reply(id, dev.hatch_open(), dev, "hatch");
    }
    if (std::strcmp(method, "door.close") == 0) {
        return move_reply(id, dev.hatch_close(), dev, "hatch");
    }
    if (std::strcmp(method, "door.stop") == 0) {
        const bool ok = dev.hatch_stop();
        return make_ok(id, ok, "\"hatch\":" + dev.axis_json("hatch"));
    }

    /* ── 停机坪 ───────────────────────────────────────────────────────── */
    if (std::strcmp(method, "helipad.up") == 0) {
        return move_reply(id, dev.pad_up(), dev, "helipad");
    }
    if (std::strcmp(method, "helipad.down") == 0) {
        return move_reply(id, dev.pad_down(), dev, "helipad");
    }
    if (std::strcmp(method, "helipad.stop") == 0) {
        const bool ok = dev.pad_stop();
        return make_ok(id, ok, "\"helipad\":" + dev.axis_json("helipad"));
    }

    /* ── 急停 / 状态 ─────────────────────────────────────────────────── */
    if (std::strcmp(method, "door.stop_all") == 0 ||
        std::strcmp(method, "helipad.stop_all") == 0) {
        const bool ok = dev.stop_all();
        return make_ok(id, ok, "\"hatch\":" + dev.axis_json("hatch") +
                               ",\"helipad\":" + dev.axis_json("helipad"));
    }

    if (std::strcmp(method, "door.get_status") == 0 ||
        std::strcmp(method, "helipad.get_status") == 0) {
        return make_result(id, status_body(dev));
    }

    if (std::strcmp(method, "door.get_inputs") == 0) {
        bool refresh = true;
        (void)json_get_bool(line, "refresh", &refresh);
        const bool ok = refresh ? dev.refresh_inputs() : dev.inputs_valid();
        std::string extra = "\"inputs\":" + bits_json(dev.inputs(), dev.inputs_valid());
        extra += ",\"hatch\":" + dev.axis_json("hatch");
        extra += ",\"helipad\":" + dev.axis_json("helipad");
        if (!ok && !dev.last_error().empty()) {
            extra += ",\"error\":\"" + json_escape(dev.last_error()) + "\"";
        }
        return make_ok(id, ok, extra);
    }

    if (std::strcmp(method, "door.get_outputs") == 0) {
        const bool ok = dev.refresh_outputs();
        std::string extra = "\"outputs\":" + bits_json(dev.outputs(), dev.outputs_valid());
        if (!ok && !dev.last_error().empty()) {
            extra += ",\"error\":\"" + json_escape(dev.last_error()) + "\"";
        }
        return make_ok(id, ok, extra);
    }

    /* ── 裸继电器操作（调试 / 其他负载） ────────────────────────────── */
    if (std::strcmp(method, "door.set_relay") == 0) {
        const int channel = json_get_int(line, "channel", -1);
        bool on = false;
        if (channel < 0 || !json_get_bool(line, "on", &on)) {
            return make_error(id, -32602, "missing channel/on");
        }
        const bool ok = dev.set_relay(channel, on);
        std::string extra = "\"channel\":" + std::to_string(channel) + ",\"on\":" + jbool(on);
        if (!ok) extra += ",\"error\":\"" + json_escape(dev.last_error()) + "\"";
        return make_ok(id, ok, extra);
    }

    if (std::strcmp(method, "door.set_all") == 0) {
        bool on = false;
        if (!json_get_bool(line, "on", &on)) {
            return make_error(id, -32602, "missing on");
        }
        const bool ok = dev.set_all(on);
        std::string extra = "\"on\":" + jbool(on);
        if (!ok) extra += ",\"error\":\"" + json_escape(dev.last_error()) + "\"";
        return make_ok(id, ok, extra);
    }

    if (std::strcmp(method, "door.pulse") == 0) {
        const int channel = json_get_int(line, "channel", -1);
        const int ms      = json_get_int(line, "ms", 500);
        if (channel < 0) return make_error(id, -32602, "missing channel");
        const bool ok = dev.pulse(channel, ms);
        std::string extra = "\"channel\":" + std::to_string(channel) +
                            ",\"ms\":" + std::to_string(ms);
        if (!ok) extra += ",\"error\":\"" + json_escape(dev.last_error()) + "\"";
        return make_ok(id, ok, extra);
    }

    if (std::strcmp(method, "door.reconnect") == 0) {
        dev.reconnect();
        // Report *why* it failed, not just that it did. The two failures
        // look identical from the GUI but need different actions:
        //   open(...) ENOENT → the CH340 itself is gone (it is powered from
        //                      the relay board, so board off = node gone)
        //   handshake silent → node is there, module not answering
        std::string extra = "\"connected\":" + jbool(dev.connected());
        if (!dev.connected() && !dev.last_error().empty()) {
            extra += ",\"error\":\"" + json_escape(dev.last_error()) + "\"";
        }
        return make_ok(id, dev.connected(), extra);
    }

    if (std::strcmp(method, "door.raw") == 0) {
        char hex[256] = {};
        if (!json_get_token(line, "hex", hex, sizeof(hex))) {
            return make_error(id, -32602, "missing hex");
        }
        uint8_t body[128];
        size_t  body_len = 0;
        int     nibble = -1;
        for (const char *p = hex; *p != '\0' && body_len < sizeof(body); ++p) {
            int v;
            if (*p >= '0' && *p <= '9')      v = *p - '0';
            else if (*p >= 'a' && *p <= 'f') v = *p - 'a' + 10;
            else if (*p >= 'A' && *p <= 'F') v = *p - 'A' + 10;
            else continue;
            if (nibble < 0) {
                nibble = v;
            } else {
                body[body_len++] = static_cast<uint8_t>((nibble << 4) | v);
                nibble = -1;
            }
        }
        if (body_len < 2U || nibble >= 0) {
            return make_error(id, -32602, "bad hex payload");
        }
        int expect = json_get_int(line, "expect", 0);
        if (expect < 0) expect = 0;
        if (expect > 128) expect = 128;

        uint8_t rx[256];
        const int n = dev.raw(body, body_len, rx, sizeof(rx), static_cast<size_t>(expect));
        std::string extra = "\"tx\":\"" + hex_json(body, body_len) + "\"";
        extra += ",\"rx\":\"" + ((n > 0) ? hex_json(rx, static_cast<size_t>(n)) : std::string()) + "\"";
        if (n < 0) extra += ",\"error\":\"" + json_escape(dev.last_error()) + "\"";
        return make_ok(id, n > 0, extra);
    }

    return make_error(id, -32601, "method not found");
}

}  // namespace

/* ---------------------------------------------------------------- main --- */

int run_door_runtime() {
    std::signal(SIGPIPE, SIG_IGN);
    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);

    const DoorConfig cfg = DoorConfig::from_env();
    DoorDevice dev(cfg);

    const int server_fd = socket(AF_UNIX, SOCK_STREAM, 0);
    if (server_fd < 0) {
        log_error(kTag, "socket failed: %s", std::strerror(errno));
        return 1;
    }

    unlink(cfg.sock_path.c_str());
    struct sockaddr_un addr {};
    addr.sun_family = AF_UNIX;
    std::strncpy(addr.sun_path, cfg.sock_path.c_str(), sizeof(addr.sun_path) - 1);
    if (bind(server_fd, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
        log_error(kTag, "bind %s failed: %s", cfg.sock_path.c_str(), std::strerror(errno));
        close(server_fd);
        return 1;
    }
    if (listen(server_fd, kMaxClients) < 0) {
        log_error(kTag, "listen failed: %s", std::strerror(errno));
        close(server_fd);
        return 1;
    }
    (void)chmod(cfg.sock_path.c_str(), 0666);

    // Non-blocking listener so the accept-drain loop below can stop on
    // EAGAIN instead of blocking forever once the queue is empty.
    {
        const int flags = fcntl(server_fd, F_GETFL, 0);
        if (flags >= 0) (void)fcntl(server_fd, F_SETFL, flags | O_NONBLOCK);
    }

    log_info(kTag, "listening on %s (uart=%s baud=%d addr=%u out=%d in=%d poll=%dms)",
             cfg.sock_path.c_str(), cfg.uart_path.c_str(), cfg.baud,
             static_cast<unsigned>(cfg.slave), cfg.out_count, cfg.in_count, cfg.poll_ms);
    log_info(kTag, "hatch: run=Y%d dir=Y%d(open=%d) limits X%d/X%d timeout=%dms",
             cfg.hatch.ch_a, cfg.hatch.ch_b, cfg.hatch.dir_level_a ? 1 : 0,
             cfg.hatch.in_a, cfg.hatch.in_b, cfg.hatch.timeout_ms);
    log_info(kTag, "helipad: up=Y%d down=Y%d limits X%d/X%d timeout=%dms",
             cfg.pad.ch_a, cfg.pad.ch_b, cfg.pad.in_a, cfg.pad.in_b, cfg.pad.timeout_ms);

    (void)dev.ensure_open();

    struct pollfd pfds[1 + kMaxClients];
    int    client_fds[kMaxClients];
    char   bufs[kMaxClients][kBufSize];
    size_t buf_lens[kMaxClients];
    int    nclients = 0;

    std::memset(buf_lens, 0, sizeof(buf_lens));
    for (int i = 0; i < kMaxClients; ++i) client_fds[i] = -1;

    int next_poll_ms = now_ms();

    while (g_stop == 0) {
        pfds[0].fd = server_fd;
        pfds[0].events = POLLIN;
        for (int i = 0; i < nclients; ++i) {
            pfds[1 + i].fd = client_fds[i];
            pfds[1 + i].events = POLLIN;
        }

        // Wake in time for the next background scan; with polling disabled
        // just block on the socket with a 1 s tick so SIGTERM is noticed.
        int wait_ms = 1000;
        if (cfg.poll_ms > 0) {
            wait_ms = next_poll_ms - now_ms();
            if (wait_ms < 0) wait_ms = 0;
        }

        const int pr = poll(pfds, static_cast<nfds_t>(1 + nclients), wait_ms);
        if (pr < 0) {
            if (errno == EINTR) continue;
            log_error(kTag, "poll failed: %s", std::strerror(errno));
            break;
        }

        if (cfg.poll_ms > 0 && now_ms() - next_poll_ms >= 0) {
            dev.poll_tick();
            next_poll_ms = now_ms() + cfg.poll_ms;
        }

        if (pr == 0) continue;

        if ((pfds[0].revents & POLLIN) != 0) {
            // Drain the whole accept queue, not one per iteration: HostGUI
            // polls status at 300 ms and the gateway opens a fresh
            // connection per call, so a burst would otherwise take several
            // supervision periods to clear.
            for (;;) {
                const int client_fd = accept(server_fd, nullptr, nullptr);
                if (client_fd < 0) break;
                if (nclients >= kMaxClients) {
                    close(client_fd);
                    log_warn(kTag, "client limit %d reached, connection refused", kMaxClients);
                    break;
                }
                client_fds[nclients] = client_fd;
                buf_lens[nclients] = 0U;
                // Initialise the WHOLE pollfd entry here. poll() only wrote
                // revents for the indices it was handed, and this slot was
                // not one of them — it still holds garbage from an earlier
                // iteration. Leaving it stale made the client loop below
                // read a stale fd, treat the failure as a disconnect, and
                // drop this brand-new client without closing it: an fd leak
                // plus a caller that hangs until its own timeout. That is
                // what wedged the RPC chain on 2026-08-08.
                pfds[1 + nclients].fd      = client_fd;
                pfds[1 + nclients].events  = POLLIN;
                pfds[1 + nclients].revents = 0;
                ++nclients;
                if (nclients >= kMaxClients) break;
            }
        }

        for (int i = 0; i < nclients;) {
            const int idx = 1 + i;
            if ((pfds[idx].revents & (POLLIN | POLLHUP | POLLERR)) == 0) {
                ++i;
                continue;
            }

            const ssize_t n = read(pfds[idx].fd, bufs[i] + buf_lens[i],
                                   kBufSize - buf_lens[i] - 1U);
            if (n <= 0) {
                close(pfds[idx].fd);
                const int last = nclients - 1;
                if (i != last) {
                    // Move the last client into this slot — pfds included,
                    // so the `continue` below re-examines the moved client
                    // with its own fd/revents instead of the closed one's.
                    client_fds[i] = client_fds[last];
                    buf_lens[i]   = buf_lens[last];
                    std::memcpy(bufs[i], bufs[last], buf_lens[i]);
                    pfds[idx]     = pfds[1 + last];
                }
                client_fds[last] = -1;
                --nclients;
                continue;
            }

            buf_lens[i] += static_cast<size_t>(n);
            bufs[i][buf_lens[i]] = '\0';

            char *start = bufs[i];
            char *newline = nullptr;
            while ((newline = static_cast<char *>(
                        std::memchr(start, '\n',
                                    buf_lens[i] - static_cast<size_t>(start - bufs[i])))) != nullptr) {
                *newline = '\0';
                const std::string resp = handle_request(start, dev);
                const ssize_t wrote = write(pfds[idx].fd, resp.c_str(), resp.size());
                (void)wrote;
                start = newline + 1;
            }

            const size_t consumed = static_cast<size_t>(start - bufs[i]);
            buf_lens[i] -= consumed;
            if (consumed > 0U && buf_lens[i] > 0U) {
                std::memmove(bufs[i], start, buf_lens[i]);
            }
            // An over-long line with no newline would otherwise wedge the
            // buffer full forever; drop it and resync on the next frame.
            if (buf_lens[i] >= kBufSize - 1U) {
                log_warn(kTag, "oversized request dropped");
                buf_lens[i] = 0U;
            }
            ++i;
        }
    }

    log_info(kTag, "shutting down - releasing all motor coils");
    dev.release_outputs();

    for (int i = 0; i < nclients; ++i) {
        if (client_fds[i] >= 0) close(client_fds[i]);
    }
    close(server_fd);
    unlink(cfg.sock_path.c_str());
    return 0;
}
