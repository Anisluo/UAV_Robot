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

struct NpuResultStore {
    std::mutex      mu;
    UavCResult      latest{};
    bool            has_result{false};
};

static NpuResultStore g_npu_result;

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
    int flags = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, flags | O_NONBLOCK);
    return fd;
}

// Drain all pending datagrams from the npu result socket, cache the latest.
static void drain_npu_results(int fd) {
    if (fd < 0) return;
    UavCResult r{};
    ssize_t n;
    while ((n = ::recv(fd, &r, sizeof(r), MSG_DONTWAIT)) == static_cast<ssize_t>(sizeof(r))) {
        std::lock_guard<std::mutex> lk(g_npu_result.mu);
        g_npu_result.latest = r;
        g_npu_result.has_result = true;
    }
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

static void set_nonblock(int fd) {
    int f = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, f | O_NONBLOCK);
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

static std::string trim_copy(const std::string &input) {
    size_t start = 0;
    size_t end = input.size();
    while (start < end && std::isspace(static_cast<unsigned char>(input[start]))) {
        ++start;
    }
    while (end > start && std::isspace(static_cast<unsigned char>(input[end - 1U]))) {
        --end;
    }
    return input.substr(start, end - start);
}

static std::string shell_quote(const std::string &input) {
    std::string out = "'";
    for (char ch : input) {
        if (ch == '\'') {
            out += "'\\''";
        } else {
            out.push_back(ch);
        }
    }
    out.push_back('\'');
    return out;
}

static bool run_arm_probe_json(const char *action,
                               std::string &out_json,
                               std::string &out_error) {
    const char *script_env = std::getenv("UAV_ARM_ANGLE_PROBE_SCRIPT");
    const char *sdk_env = std::getenv("UAV_ARM_SDK_SO_PATH");
    const char *iface_env = std::getenv("UAV_ARM_CAN_IFACE");
    const std::string script_path = (script_env && script_env[0] != '\0')
        ? script_env
        : "/home/ubuntu/UAV_Robot/tools/socketcan_pcanbasic_probe.py";
    const std::string sdk_so_path = (sdk_env && sdk_env[0] != '\0')
        ? sdk_env
        : "/home/ubuntu/arm_sdk_probe/controller_core.cpython-38-aarch64-linux-gnu.so";
    const std::string iface = (iface_env && iface_env[0] != '\0') ? iface_env : "can4";
    int usb_id = 1;
    if (const char *usb_env = std::getenv("UAV_ARM_SDK_USB_ID")) {
        char *end = nullptr;
        long parsed = std::strtol(usb_env, &end, 10);
        if (end != usb_env && end != nullptr && *end == '\0') {
            usb_id = static_cast<int>(parsed);
        }
    }
    int probe_timeout_sec = 3;
    if (const char *timeout_env = std::getenv("UAV_ARM_PROBE_TIMEOUT_SEC")) {
        char *end = nullptr;
        long parsed = std::strtol(timeout_env, &end, 10);
        if (end != timeout_env && end != nullptr && *end == '\0' && parsed > 0 && parsed < 60) {
            probe_timeout_sec = static_cast<int>(parsed);
        }
    }
    const std::string cmd =
        "timeout " + std::to_string(probe_timeout_sec) + "s python3 " + shell_quote(script_path) +
        " --json --action " + shell_quote((action && action[0] != '\0') ? action : "angles") +
        " --iface " + shell_quote(iface) +
        " --sdk-so " + shell_quote(sdk_so_path) +
        " --usb-id " + std::to_string(usb_id);
    FILE *pipe = popen(cmd.c_str(), "r");
    std::string output;
    char buffer[256];

    if (pipe == nullptr) {
        out_error = "popen failed";
        return false;
    }
    while (std::fgets(buffer, sizeof(buffer), pipe) != nullptr) {
        output += buffer;
    }
    int status = pclose(pipe);

    output = trim_copy(output);
    if (status != 0 && output.empty()) {
        out_error = "arm probe timeout or failure";
        return false;
    }
    if (output.empty()) {
        out_error = "empty probe output";
        return false;
    }
    size_t json_pos = output.rfind('{');
    if (json_pos == std::string::npos) {
        out_error = output;
        return false;
    }
    out_json = trim_copy(output.substr(json_pos));
    return true;
}

static bool read_arm_angles_json(std::string &out_json, std::string &out_error) {
    return run_arm_probe_json("angles", out_json, out_error);
}

static bool run_arm_home_json(std::string &out_json, std::string &out_error) {
    return run_arm_probe_json("home", out_json, out_error);
}

static bool start_arm_home_background(std::string &out_error) {
    const char *script_env = std::getenv("UAV_ARM_ANGLE_PROBE_SCRIPT");
    const char *sdk_env = std::getenv("UAV_ARM_SDK_SO_PATH");
    const char *iface_env = std::getenv("UAV_ARM_CAN_IFACE");
    const std::string script_path = (script_env && script_env[0] != '\0')
        ? script_env
        : "/home/ubuntu/UAV_Robot/tools/socketcan_pcanbasic_probe.py";
    const std::string sdk_so_path = (sdk_env && sdk_env[0] != '\0')
        ? sdk_env
        : "/home/ubuntu/arm_sdk_probe/controller_core.cpython-38-aarch64-linux-gnu.so";
    const std::string iface = (iface_env && iface_env[0] != '\0') ? iface_env : "can4";
    int usb_id = 1;
    if (const char *usb_env = std::getenv("UAV_ARM_SDK_USB_ID")) {
        char *end = nullptr;
        long parsed = std::strtol(usb_env, &end, 10);
        if (end != usb_env && end != nullptr && *end == '\0') {
            usb_id = static_cast<int>(parsed);
        }
    }
    int home_timeout_sec = 120;
    if (const char *timeout_env = std::getenv("UAV_ARM_HOME_TIMEOUT_SEC")) {
        char *end = nullptr;
        long parsed = std::strtol(timeout_env, &end, 10);
        if (end != timeout_env && end != nullptr && *end == '\0' && parsed > 0 && parsed < 3600) {
            home_timeout_sec = static_cast<int>(parsed);
        }
    }

    const std::string cmd =
        "nohup timeout " + std::to_string(home_timeout_sec) + "s python3 " + shell_quote(script_path) +
        " --json --action home" +
        " --iface " + shell_quote(iface) +
        " --sdk-so " + shell_quote(sdk_so_path) +
        " --usb-id " + std::to_string(usb_id) +
        " >/tmp/uav_arm_home.log 2>&1 </dev/null &";
    int rc = std::system(cmd.c_str());
    if (rc != 0) {
        out_error = "failed to start arm home background task";
        return false;
    }
    return true;
}

static bool get_cached_arm_angles_json(std::string &out_json, std::string &out_error) {
    static std::mutex cache_mu;
    static std::string cached_json =
        "{\"ok\":true,\"initialized\":false,\"action\":\"angles\",\"angles\":[0.0,0.0,0.0,0.0,0.0,0.0]}";
    static uint64_t cached_at_ms = 0;
    static std::atomic<bool> refresh_in_flight{false};
    int cache_ms = 1500;
    if (const char *cache_env = std::getenv("UAV_ARM_PROBE_CACHE_MS")) {
        char *end = nullptr;
        long parsed = std::strtol(cache_env, &end, 10);
        if (end != cache_env && end != nullptr && *end == '\0' && parsed >= 0 && parsed < 60000) {
            cache_ms = static_cast<int>(parsed);
        }
    }

    const uint64_t now = now_ms();
    {
        std::lock_guard<std::mutex> lk(cache_mu);
        if (!cached_json.empty() && (now - cached_at_ms) < static_cast<uint64_t>(cache_ms)) {
            out_json = cached_json;
            return true;
        }
    }

    bool expected = false;
    if (refresh_in_flight.compare_exchange_strong(expected, true)) {
        std::thread([&cache_mu, &cached_json, &cached_at_ms, &refresh_in_flight]() {
            std::string fresh_json;
            std::string fresh_error;
            bool ok = read_arm_angles_json(fresh_json, fresh_error);
            if (ok && !fresh_json.empty()) {
                std::lock_guard<std::mutex> lk(cache_mu);
                cached_json = fresh_json;
                cached_at_ms = now_ms();
            }
            refresh_in_flight.store(false);
        }).detach();
    }

    {
        std::lock_guard<std::mutex> lk(cache_mu);
        out_json = cached_json;
        return true;
    }
}

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

// ─── UGV CAN control ────────────────────────────────────────────────────────
class ChassisCanController {
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

static ChassisCanController g_chassis;
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

    void monitor_pair_until_stall(uint64_t session) {
        const int poll_ms = clamp_int(env_int("UAV_AIRPORT_LOCK_POLL_MS", 120), 20, 1000);
        const int confirm_hits = clamp_int(env_int("UAV_AIRPORT_LOCK_CONFIRM_HITS", 2), 1, 10);
        const int rails[2] = {0, 2};
        bool stopped[2] = {false, false};
        int stall_hits[2] = {0, 0};

        while (g_running && pair_motion_session_.load() == session) {
            bool all_stopped = true;
            for (int i = 0; i < 2; ++i) {
                if (stopped[i]) continue;
                all_stopped = false;

                uint8_t flags = 0;
                if (!read_status_flags(rail_addr(rails[i]), &flags)) {
                    continue;
                }

                const bool stalled = (flags & 0x04U) != 0U || (flags & 0x08U) != 0U;
                if (stalled) {
                    stall_hits[i] += 1;
                    if (stall_hits[i] >= confirm_hits) {
                        (void)stop_rail(rails[i]);
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

    fprintf(stderr, "proc_gateway: RPC fd=%d recv: %s\n", fd, line.c_str());

    if (method == "system.ping") {
        uint64_t uptime = now_ms() - start_ms;
        snprintf(resp, sizeof(resp),
                 "{\"id\":%d,\"result\":{\"pong\":true,\"uptime_ms\":%llu}}\n",
                 id, (unsigned long long)uptime);

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
        ctrl_c.send_cmd(UAV_CTRL_C_SET_STRATEGY, strategy);
        snprintf(resp, sizeof(resp),
                 "{\"id\":%d,\"result\":{\"ok\":true,\"strategy_id\":%d}}\n",
                 id, strategy);

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
        UavCResult r{};
        bool has = false;
        {
            std::lock_guard<std::mutex> lk(g_npu_result.mu);
            r   = g_npu_result.latest;
            has = g_npu_result.has_result;
        }
        if (!has) {
            snprintf(resp, sizeof(resp),
                     "{\"id\":%d,\"result\":{\"frame_id\":0,\"num_detections\":0,"
                     "\"detections\":[]}}\n", id);
        } else {
            // Build detections JSON array
            char dets_buf[6144];
            int  pos = 0;
            dets_buf[pos++] = '[';
            for (uint32_t i = 0; i < r.num_detections && i < UAV_MAX_DETECTIONS; ++i) {
                if (i > 0) dets_buf[pos++] = ',';
                pos += fmt_detection(dets_buf + pos,
                                     (int)(sizeof(dets_buf) - (size_t)pos),
                                     r.detections[i]);
            }
            dets_buf[pos++] = ']';
            dets_buf[pos]   = '\0';
            snprintf(resp, sizeof(resp),
                     "{\"id\":%d,\"result\":{\"frame_id\":%llu,"
                     "\"num_detections\":%u,\"detections\":%s}}\n",
                     id,
                     (unsigned long long)r.frame_id,
                     r.num_detections,
                     dets_buf);
        }

    } else if (method == "npu.get_status") {
        snprintf(resp, sizeof(resp),
                 "{\"id\":%d,\"result\":{\"ok\":true}}\n", id);

    // ── Task forwarding ───────────────────────────────────────────────────
    } else if (method == "task.start") {
        std::string task_name = json_str(s, "task");
        const char *cmd_str = "START_BATTERY_PICK";
        if (task_name == "arm_home")       cmd_str = "RESET";
        else if (task_name == "battery_pick_3d") cmd_str = "START_BATTERY_PICK_3D";
        else if (task_name == "battery_pick_6d") cmd_str = "START_BATTERY_PICK_6D";
        else if (task_name == "platform_lock") cmd_str = "START_BATTERY_PICK";
        send_task_cmd(task_udp_fd, cmd_str);
        snprintf(resp, sizeof(resp),
                 "{\"id\":%d,\"result\":{\"ok\":true,\"task\":\"%s\"}}\n",
                 id, task_name.c_str());

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
        const std::string path = task_log_path();
        const std::string logs = tail_lines_text(read_text_file(path), max_lines);
        const std::string escaped = json_escape(logs);
        snprintf(resp, sizeof(resp),
                 "{\"id\":%d,\"result\":{\"path\":\"%s\",\"logs\":\"%s\"}}\n",
                 id, path.c_str(), escaped.c_str());

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
        snprintf(resp, sizeof(resp),
                 "{\"id\":%d,\"result\":{\"enabled\":%s,\"clients\":%d}}\n",
                 id, enabled ? "true" : "false", clients);

    } else if (method == "arm.home") {
        std::string probe_error;
        bool ok = start_arm_home_background(probe_error);
        if (ok) {
            snprintf(resp, sizeof(resp),
                     "{\"id\":%d,\"result\":{\"ok\":true,\"started\":true}}\n",
                     id);
        } else {
            const std::string escaped = json_escape(probe_error);
            snprintf(resp, sizeof(resp),
                     "{\"id\":%d,\"result\":{\"ok\":false,\"error\":\"%s\"}}\n",
                     id,
                     escaped.c_str());
        }

    } else if (method == "arm.stop") {
        bool ok = arm_stop();
        snprintf(resp, sizeof(resp),
                 "{\"id\":%d,\"result\":{\"ok\":%s}}\n",
                 id, ok ? "true" : "false");

    } else if (method == "arm.move_pose") {
        std::string pose = json_str(s, "pose");
        bool ok = !pose.empty() && arm_move(pose.c_str());
        snprintf(resp, sizeof(resp),
                 "{\"id\":%d,\"result\":{\"ok\":%s,\"pose\":\"%s\"}}\n",
                 id, ok ? "true" : "false", pose.c_str());

    } else if (method == "arm.move_xyz") {
        double x_mm = json_double(s, "x_mm", 0.0);
        double y_mm = json_double(s, "y_mm", 0.0);
        double z_mm = json_double(s, "z_mm", 0.0);
        bool ok = arm_move_to_xyz((float)x_mm, (float)y_mm, (float)z_mm);
        snprintf(resp, sizeof(resp),
                 "{\"id\":%d,\"result\":{\"ok\":%s,\"x_mm\":%.1f,\"y_mm\":%.1f,\"z_mm\":%.1f}}\n",
                 id, ok ? "true" : "false", x_mm, y_mm, z_mm);

    } else if (method == "arm.move_joint") {
        int joint = json_int(s, "joint", -1);
        double target_deg = json_double(s, "target_deg", 0.0);
        bool ok = arm_move_joint_deg(joint, (float)target_deg);
        snprintf(resp, sizeof(resp),
                 "{\"id\":%d,\"result\":{\"ok\":%s,\"joint\":%d,\"target_deg\":%.1f}}\n",
                 id, ok ? "true" : "false", joint, target_deg);

    } else if (method == "arm.move_joints") {
        float joints_deg[6] = {
            (float)json_double(s, "j1_deg", 0.0),
            (float)json_double(s, "j2_deg", 0.0),
            (float)json_double(s, "j3_deg", 0.0),
            (float)json_double(s, "j4_deg", 0.0),
            (float)json_double(s, "j5_deg", 0.0),
            (float)json_double(s, "j6_deg", 0.0)
        };
        bool ok = arm_move_joints_deg(joints_deg);
        snprintf(resp, sizeof(resp),
                 "{\"id\":%d,\"result\":{\"ok\":%s,"
                 "\"joints_deg\":[%.1f,%.1f,%.1f,%.1f,%.1f,%.1f]}}\n",
                 id, ok ? "true" : "false",
                 (double)joints_deg[0], (double)joints_deg[1], (double)joints_deg[2],
                 (double)joints_deg[3], (double)joints_deg[4], (double)joints_deg[5]);

    } else if (method == "arm.get_angles") {
        std::string angles_json;
        std::string angles_error;
        bool ok = get_cached_arm_angles_json(angles_json, angles_error);
        if (ok && !angles_json.empty()) {
            snprintf(resp, sizeof(resp),
                     "{\"id\":%d,\"result\":%s}\n",
                     id, angles_json.c_str());
        } else {
            const std::string escaped = json_escape(angles_error.empty() ? "arm.get_angles failed" : angles_error);
            snprintf(resp, sizeof(resp),
                     "{\"id\":%d,\"result\":{\"ok\":false,\"error\":\"%s\"}}\n",
                     id, escaped.c_str());
        }

    } else if (method == "ugv.set_velocity") {
        double vx    = json_double(s, "vx", 0.0);
        double vy    = json_double(s, "vy", 0.0);
        double omega = json_double(s, "omega", 0.0);
        bool ok = g_chassis.set_velocity(vx, omega);
        snprintf(resp, sizeof(resp),
                 "{\"id\":%d,\"result\":{\"ok\":%s,\"vx\":%.3f,\"vy\":%.3f,\"omega\":%.3f}}\n",
                 id, ok ? "true" : "false", vx, vy, omega);

    } else if (method == "ugv.stop") {
        bool ok = g_chassis.stop();
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

    } else {
        // Stub for arm/ugv/airport/gripper – acknowledge without hardware action
        snprintf(resp, sizeof(resp),
                 "{\"id\":%d,\"result\":{\"ok\":true}}\n", id);
    }

    fprintf(stderr, "proc_gateway: RPC fd=%d resp: %s", fd, resp);
    write_all(fd, resp, strlen(resp));
}

// ─── Push one JPEG frame to a video client ───────────────────────────────────
// Returns false if the client should be closed.
static bool push_frame(int fd, const std::vector<uint8_t> &jpeg) {
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
            std::vector<uint8_t> jpeg = encoder.encode(
                frame.color.data(), frame.width, frame.height, frame.stride);
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
