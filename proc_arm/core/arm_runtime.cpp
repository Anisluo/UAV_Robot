#include "arm_runtime.h"

#include <array>
#include <atomic>
#include <cerrno>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <poll.h>
#include <string>
#include <thread>
#include <unordered_set>
#include <vector>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include "arm_controller.h"

extern "C" {
#include "dev.h"
#include "log.h"
}

namespace {
/* Global registry of currently-active client fds (legacy TCP + unix JSON-RPC).
 * The estop watchdog thread iterates over this set, peeks at each fd's
 * receive buffer with MSG_PEEK, and signals an emergency stop the moment it
 * sees any stop request - even though the main thread is busy executing a
 * blocking move command and cannot drain the buffer itself. */
std::mutex g_client_fds_mutex;
std::unordered_set<int> g_client_fds;
std::atomic<bool> g_estop_watchdog_running{false};
std::thread g_estop_watchdog_thread;

void register_client_fd(int fd) {
    std::lock_guard<std::mutex> lock(g_client_fds_mutex);
    g_client_fds.insert(fd);
}

void unregister_client_fd(int fd) {
    std::lock_guard<std::mutex> lock(g_client_fds_mutex);
    g_client_fds.erase(fd);
}

bool buffer_contains_stop(const char *buf, size_t len) {
    /* Search for any of the known stop tokens. We accept both legacy
     * action-style ("action":"emergency_stop") and JSON-RPC method names
     * ("arm.stop" / "arm.emergency_stop"). */
    static const char *kTokens[] = {
        "emergency_stop",
        "arm.stop",
        "arm.emergency_stop",
    };
    for (size_t i = 0; i < sizeof(kTokens) / sizeof(kTokens[0]); ++i) {
        const char *needle = kTokens[i];
        size_t nlen = std::strlen(needle);
        if (nlen > len) {
            continue;
        }
        for (size_t j = 0; j + nlen <= len; ++j) {
            if (std::memcmp(buf + j, needle, nlen) == 0) {
                return true;
            }
        }
    }
    return false;
}

void estop_watchdog_loop() {
    log_info("proc_arm.estop", "watchdog started");
    char peek_buf[2048];
    while (g_estop_watchdog_running.load()) {
        std::vector<int> snapshot;
        {
            std::lock_guard<std::mutex> lock(g_client_fds_mutex);
            snapshot.assign(g_client_fds.begin(), g_client_fds.end());
        }
        for (int fd : snapshot) {
            ssize_t n = recv(fd, peek_buf, sizeof(peek_buf), MSG_PEEK | MSG_DONTWAIT);
            if (n > 0) {
                if (buffer_contains_stop(peek_buf, static_cast<size_t>(n))) {
                    if (!arm_estop_requested()) {
                        log_warn("proc_arm.estop",
                                 "stop token detected on fd=%d, signaling estop",
                                 fd);
                        arm_request_estop();
                    }
                }
            }
        }
        usleep(20000);  /* 20 ms poll interval */
    }
    log_info("proc_arm.estop", "watchdog stopped");
}

void start_estop_watchdog() {
    if (g_estop_watchdog_running.exchange(true)) {
        return;
    }
    g_estop_watchdog_thread = std::thread(estop_watchdog_loop);
}

}  // namespace

namespace {
constexpr const char *kTag = "proc_arm";
constexpr const char *kSockPath = "/tmp/uav_proc_arm.sock";
constexpr int kLegacyPortDefault = 12345;
constexpr int kMaxClients = 8;
constexpr size_t kBufSize = 4096;
constexpr size_t kLegacyMaxPacketSize = 1U << 20;

bool json_get_token(const char *json, const char *key, char *out, size_t out_size) {
    char needle[128];
    const char *p;
    size_t i = 0;

    if (json == nullptr || out == nullptr || out_size == 0U) {
        return false;
    }

    std::snprintf(needle, sizeof(needle), "\"%s\"", key);
    p = std::strstr(json, needle);
    if (p == nullptr) {
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

bool json_get_double(const char *json, const char *key, double *out) {
    char token[64];
    char *end = nullptr;
    double value;

    if (out == nullptr || !json_get_token(json, key, token, sizeof(token))) {
        return false;
    }
    value = std::strtod(token, &end);
    if (end == token) {
        return false;
    }
    *out = value;
    return true;
}

bool json_get_bool(const char *json, const char *key, bool *out) {
    char token[16];

    if (out == nullptr || !json_get_token(json, key, token, sizeof(token))) {
        return false;
    }
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

std::string make_result(const std::string &id_fragment, const std::string &payload) {
    return std::string("{\"jsonrpc\":\"2.0\",") + id_fragment + ",\"result\":" + payload + "}\n";
}

std::string make_error(const std::string &id_fragment, int code, const char *message) {
    return std::string("{\"jsonrpc\":\"2.0\",") + id_fragment +
           ",\"error\":{\"code\":" + std::to_string(code) +
           ",\"message\":" + json_escape(message) + "}}\n";
}

std::string bool_result(const std::string &id_fragment, bool ok) {
    return make_result(id_fragment, ok ? "{\"ok\":true}" : "{\"ok\":false}");
}

std::string number_payload(double value) {
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%.2f", value);
    return buf;
}

std::string array_payload(const std::array<double, 6> &values) {
    char buf[256];
    std::snprintf(buf,
                  sizeof(buf),
                  "[%.6f,%.6f,%.6f,%.6f,%.6f,%.6f]",
                  values[0],
                  values[1],
                  values[2],
                  values[3],
                  values[4],
                  values[5]);
    return buf;
}

std::string array16_payload(const std::array<double, 16> &values) {
    std::string out = "[";
    char buf[64];
    for (size_t i = 0; i < values.size(); ++i) {
        std::snprintf(buf, sizeof(buf), "%.6f", values[i]);
        if (i > 0U) {
            out += ",";
        }
        out += buf;
    }
    out += "]";
    return out;
}

bool extract_param_slice(const char *json, std::string *out) {
    const char *p = std::strstr(json, "\"params\"");
    const char *start;
    int depth = 0;

    if (out == nullptr || p == nullptr) {
        return false;
    }
    p = std::strchr(p, ':');
    if (p == nullptr) {
        return false;
    }
    ++p;
    while (*p == ' ' || *p == '\t') {
        ++p;
    }
    start = p;
    if (*p == '{' || *p == '[') {
        const char open_ch = *p;
        const char close_ch = (open_ch == '{') ? '}' : ']';
        while (*p != '\0') {
            if (*p == open_ch) {
                ++depth;
            } else if (*p == close_ch) {
                --depth;
                if (depth == 0) {
                    ++p;
                    out->assign(start, static_cast<size_t>(p - start));
                    return true;
                }
            }
            ++p;
        }
        return false;
    }

    while (*p != '\0' && *p != ',' && *p != '}') {
        ++p;
    }
    out->assign(start, static_cast<size_t>(p - start));
    return !out->empty();
}

bool parse_keyed_joints(const char *json, std::array<double, 6> *out) {
    const char *keys[6] = {"j1_deg", "j2_deg", "j3_deg", "j4_deg", "j5_deg", "j6_deg"};
    if (out == nullptr) {
        return false;
    }
    for (size_t i = 0; i < out->size(); ++i) {
        if (!json_get_double(json, keys[i], &(*out)[i])) {
            return false;
        }
    }
    return true;
}

bool parse_legacy_number_array(const std::string &text, std::array<double, 6> *out) {
    const char *p = text.c_str();
    char *end = nullptr;

    if (out == nullptr) {
        return false;
    }
    while (*p != '\0' && *p != '[') {
        ++p;
    }
    if (*p != '[') {
        return false;
    }
    ++p;
    for (size_t i = 0; i < out->size(); ++i) {
        while (*p == ' ' || *p == '\t') {
            ++p;
        }
        (*out)[i] = std::strtod(p, &end);
        if (end == p) {
            return false;
        }
        p = end;
        while (*p == ' ' || *p == '\t') {
            ++p;
        }
        if (i + 1U < out->size()) {
            if (*p != ',') {
                return false;
            }
            ++p;
        }
    }
    return true;
}

bool parse_legacy_angle_mode_params(const std::string &params,
                                    std::array<double, 6> *joints_deg,
                                    double *speed_ratio) {
    const char *first_bracket;
    const char *after_first;
    char *end = nullptr;

    if (joints_deg == nullptr || speed_ratio == nullptr) {
        return false;
    }
    first_bracket = std::strchr(params.c_str(), '[');
    if (first_bracket == nullptr) {
        return false;
    }
    after_first = std::strchr(first_bracket + 1, ']');
    if (after_first == nullptr) {
        return false;
    }
    if (!parse_legacy_number_array(std::string(first_bracket, static_cast<size_t>(after_first - first_bracket + 1)),
                                   joints_deg)) {
        return false;
    }
    *speed_ratio = std::strtod(after_first + 1, &end);
    if (end == after_first + 1) {
        *speed_ratio = 1.0;
    }
    return true;
}

bool parse_legacy_pose_params(const std::string &params,
                              double *x_mm,
                              double *y_mm,
                              double *z_mm,
                              double *roll_deg,
                              double *pitch_deg,
                              double *yaw_deg,
                              std::string *rotation_order,
                              double *speed_ratio,
                              bool require_speed) {
    const char *p = params.c_str();
    char *end = nullptr;
    double *values[6] = {x_mm, y_mm, z_mm, roll_deg, pitch_deg, yaw_deg};

    if (rotation_order == nullptr) {
        return false;
    }
    while (*p != '\0' && *p != '[') {
        ++p;
    }
    if (*p != '[') {
        return false;
    }
    ++p;
    for (size_t i = 0; i < 6; ++i) {
        while (*p == ' ' || *p == '\t') {
            ++p;
        }
        *values[i] = std::strtod(p, &end);
        if (end == p) {
            return false;
        }
        p = end;
        while (*p == ' ' || *p == '\t') {
            ++p;
        }
        if (*p != ',') {
            return false;
        }
        ++p;
    }
    while (*p == ' ' || *p == '\t') {
        ++p;
    }
    if (*p != '"') {
        return false;
    }
    ++p;
    {
        const char *start = p;
        while (*p != '\0' && *p != '"') {
            ++p;
        }
        if (*p != '"') {
            return false;
        }
        rotation_order->assign(start, static_cast<size_t>(p - start));
        ++p;
    }
    if (speed_ratio != nullptr) {
        while (*p == ' ' || *p == '\t' || *p == ',') {
            ++p;
        }
        *speed_ratio = 1.0;
        if (*p != ']') {
            *speed_ratio = std::strtod(p, &end);
            if (end == p && require_speed) {
                return false;
            }
        } else if (require_speed) {
            return false;
        }
    }
    return true;
}

bool recv_all(int fd, void *buf, size_t len) {
    size_t total = 0;
    while (total < len) {
        ssize_t n = recv(fd, static_cast<char *>(buf) + total, len - total, 0);
        if (n <= 0) {
            return false;
        }
        total += static_cast<size_t>(n);
    }
    return true;
}

bool send_all(int fd, const void *buf, size_t len) {
    size_t total = 0;
    while (total < len) {
        ssize_t n = send(fd, static_cast<const char *>(buf) + total, len - total, MSG_NOSIGNAL);
        if (n <= 0) {
            return false;
        }
        total += static_cast<size_t>(n);
    }
    return true;
}

bool send_legacy_message(int fd, const std::string &payload) {
    uint8_t hdr[8];
    uint64_t len = static_cast<uint64_t>(payload.size());

    for (int i = 7; i >= 0; --i) {
        hdr[i] = static_cast<uint8_t>(len & 0xFFU);
        len >>= 8;
    }
    return send_all(fd, hdr, sizeof(hdr)) && send_all(fd, payload.data(), payload.size());
}

bool recv_legacy_message(int fd, std::string *out) {
    uint8_t hdr[8];
    uint64_t len = 0;
    std::string payload;

    if (out == nullptr || !recv_all(fd, hdr, sizeof(hdr))) {
        return false;
    }
    for (size_t i = 0; i < sizeof(hdr); ++i) {
        len = (len << 8) | hdr[i];
    }
    if (len == 0U || len > kLegacyMaxPacketSize) {
        return false;
    }
    payload.resize(static_cast<size_t>(len));
    if (!recv_all(fd, payload.data(), payload.size())) {
        return false;
    }
    *out = payload;
    return true;
}

std::string handle_jsonrpc_request(const char *line) {
    char method[64] = {};
    std::string id_fragment = extract_id_fragment(line);
    ArmController &controller = ArmController::instance();

    if (!json_get_token(line, "method", method, sizeof(method))) {
        return make_error(id_fragment, -32600, "missing method");
    }

    if (std::strcmp(method, "system.ping") == 0) {
        return make_result(id_fragment, "{\"ok\":true}");
    }
    if (std::strcmp(method, "arm.home") == 0) {
        return bool_result(id_fragment, controller.home());
    }
    if (std::strcmp(method, "arm.home_joint") == 0) {
        int joint_index = 0;
        if (!json_get_int(line, "joint_index", &joint_index) &&
            !json_get_int(line, "joint", &joint_index)) {
            return make_error(id_fragment, -32602, "missing joint_index");
        }
        return bool_result(id_fragment, controller.homeJoint(joint_index));
    }
    if (std::strcmp(method, "arm.stop") == 0) {
        return bool_result(id_fragment, controller.stop());
    }
    if (std::strcmp(method, "arm.emergency_stop") == 0) {
        bool enable = false;
        if (!json_get_bool(line, "enable", &enable)) {
            return make_error(id_fragment, -32602, "missing enable");
        }
        return bool_result(id_fragment, controller.emergencyStop(enable));
    }
    if (std::strcmp(method, "arm.set_free_mode") == 0) {
        bool enable = false;
        if (!json_get_bool(line, "enable", &enable)) {
            return make_error(id_fragment, -32602, "missing enable");
        }
        return bool_result(id_fragment, controller.setFreeMode(enable));
    }
    if (std::strcmp(method, "arm.set_speeds") == 0) {
        // Two speed conventions are accepted:
        //   joint_dps / zero_dps: joint-output deg/s (uniform across joints,
        //     converted to per-joint motor RPM via gear ratio at command time)
        //   move_rpm / zero_rpm:  legacy motor-side RPM (uniform RPM, makes
        //     low-ratio joints spin much faster than high-ratio ones)
        // dps takes priority when both are sent in the same call.
        int move_rpm = 0;
        int zero_rpm = 0;
        double joint_dps = 0.0;
        double zero_dps = 0.0;
        const bool has_move      = json_get_int(line, "move_rpm",  &move_rpm);
        const bool has_zero      = json_get_int(line, "zero_rpm",  &zero_rpm);
        const bool has_joint_dps = json_get_double(line, "joint_dps", &joint_dps);
        const bool has_zero_dps  = json_get_double(line, "zero_dps",  &zero_dps);
        if (has_joint_dps) {
            arm_set_joint_dps(joint_dps);
        } else if (has_move) {
            arm_set_move_rpm(move_rpm);
        }
        if (has_zero_dps) {
            arm_set_zero_dps(zero_dps);
        } else if (has_zero) {
            arm_set_zero_rpm(zero_rpm);
        }
        char payload[192];
        std::snprintf(payload, sizeof(payload),
                      "{\"ok\":true,\"joint_dps\":%.2f,\"zero_dps\":%.2f,"
                      "\"move_rpm\":%d,\"zero_rpm\":%d}",
                      arm_get_joint_dps(), arm_get_zero_dps(),
                      arm_get_move_rpm(),  arm_get_zero_rpm());
        return make_result(id_fragment, payload);
    }
    if (std::strcmp(method, "arm.get_speeds") == 0) {
        char payload[192];
        std::snprintf(payload, sizeof(payload),
                      "{\"ok\":true,\"joint_dps\":%.2f,\"zero_dps\":%.2f,"
                      "\"move_rpm\":%d,\"zero_rpm\":%d}",
                      arm_get_joint_dps(), arm_get_zero_dps(),
                      arm_get_move_rpm(),  arm_get_zero_rpm());
        return make_result(id_fragment, payload);
    }
    if (std::strcmp(method, "arm.move_pose") == 0) {
        char pose[64] = {};
        if (!json_get_token(line, "pose", pose, sizeof(pose))) {
            return make_error(id_fragment, -32602, "missing pose");
        }
        return bool_result(id_fragment, controller.movePose(pose));
    }
    if (std::strcmp(method, "arm.move_xyz") == 0) {
        double x_mm = 0.0;
        double y_mm = 0.0;
        double z_mm = 0.0;
        double eta_s = 0.0;
        if (!json_get_double(line, "x_mm", &x_mm) ||
            !json_get_double(line, "y_mm", &y_mm) ||
            !json_get_double(line, "z_mm", &z_mm)) {
            return make_error(id_fragment, -32602, "missing xyz");
        }
        if (!controller.moveToXyz(x_mm, y_mm, z_mm, &eta_s)) {
            return bool_result(id_fragment, false);
        }
        return make_result(id_fragment, std::string("{\"ok\":true,\"eta_s\":") + number_payload(eta_s) + "}");
    }
    if (std::strcmp(method, "arm.move_pose6d") == 0 || std::strcmp(method, "arm.move_xyz_rotation") == 0) {
        double x_mm = 0.0;
        double y_mm = 0.0;
        double z_mm = 0.0;
        double roll_deg = 0.0;
        double pitch_deg = 0.0;
        double yaw_deg = 0.0;
        double speed_ratio = 1.0;
        double eta_s = 0.0;
        char rotation_order[16] = "zyx";
        if (!json_get_double(line, "x_mm", &x_mm) ||
            !json_get_double(line, "y_mm", &y_mm) ||
            !json_get_double(line, "z_mm", &z_mm) ||
            !json_get_double(line, "roll_deg", &roll_deg) ||
            !json_get_double(line, "pitch_deg", &pitch_deg) ||
            !json_get_double(line, "yaw_deg", &yaw_deg)) {
            return make_error(id_fragment, -32602, "missing pose6d");
        }
        (void)json_get_double(line, "speed_ratio", &speed_ratio);
        (void)json_get_token(line, "rotation_order", rotation_order, sizeof(rotation_order));
        if (!controller.moveToPose6d(x_mm, y_mm, z_mm, roll_deg, pitch_deg, yaw_deg, rotation_order, speed_ratio, &eta_s)) {
            return bool_result(id_fragment, false);
        }
        return make_result(id_fragment, std::string("{\"ok\":true,\"eta_s\":") + number_payload(eta_s) + "}");
    }
    if (std::strcmp(method, "arm.move_linear_xyz_rotation") == 0) {
        double x_mm = 0.0;
        double y_mm = 0.0;
        double z_mm = 0.0;
        double roll_deg = 0.0;
        double pitch_deg = 0.0;
        double yaw_deg = 0.0;
        double eta_s = 0.0;
        char rotation_order[16] = "zyx";
        if (!json_get_double(line, "x_mm", &x_mm) ||
            !json_get_double(line, "y_mm", &y_mm) ||
            !json_get_double(line, "z_mm", &z_mm) ||
            !json_get_double(line, "roll_deg", &roll_deg) ||
            !json_get_double(line, "pitch_deg", &pitch_deg) ||
            !json_get_double(line, "yaw_deg", &yaw_deg)) {
            return make_error(id_fragment, -32602, "missing pose6d");
        }
        (void)json_get_token(line, "rotation_order", rotation_order, sizeof(rotation_order));
        if (!controller.moveLinearPose6d(x_mm, y_mm, z_mm, roll_deg, pitch_deg, yaw_deg, rotation_order, &eta_s)) {
            return bool_result(id_fragment, false);
        }
        return make_result(id_fragment, std::string("{\"ok\":true,\"eta_s\":") + number_payload(eta_s) + "}");
    }
    if (std::strcmp(method, "arm.move_joint") == 0 || std::strcmp(method, "arm.move_joint_deg") == 0) {
        int joint_index = 0;
        double target_deg = 0.0;
        if ((!json_get_int(line, "joint_index", &joint_index) && !json_get_int(line, "joint", &joint_index)) ||
            !json_get_double(line, "target_deg", &target_deg)) {
            return make_error(id_fragment, -32602, "missing joint command");
        }
        return bool_result(id_fragment, controller.moveJointDeg(joint_index, target_deg));
    }
    if (std::strcmp(method, "arm.move_joints") == 0 ||
        std::strcmp(method, "arm.move_joints_deg") == 0 ||
        std::strcmp(method, "arm.angle_mode") == 0) {
        std::array<double, 6> joints_deg{};
        double speed_ratio = 1.0;
        double eta_s = 0.0;
        if (!parse_keyed_joints(line, &joints_deg)) {
            return make_error(id_fragment, -32602, "missing joint set");
        }
        (void)json_get_double(line, "speed_ratio", &speed_ratio);
        if (!controller.moveJointsDeg(joints_deg, speed_ratio, &eta_s)) {
            return bool_result(id_fragment, false);
        }
        return make_result(id_fragment, std::string("{\"ok\":true,\"eta_s\":") + number_payload(eta_s) + "}");
    }
    if (std::strcmp(method, "arm.dynamic_move") == 0) {
        return make_error(id_fragment, -32601, "use legacy protocol for dynamic_move");
    }
    if (std::strcmp(method, "arm.get_motor_angles") == 0) {
        return make_result(id_fragment, array_payload(controller.getMotorAngles()));
    }
    if (std::strcmp(method, "arm.set_current_zero") == 0) {
        int joint_index = 0;
        if (!json_get_int(line, "joint_index", &joint_index) &&
            !json_get_int(line, "joint", &joint_index)) {
            return make_error(id_fragment, -32602, "missing joint_index");
        }
        return bool_result(id_fragment, controller.setCurrentZero(joint_index));
    }
    if (std::strcmp(method, "arm.reset_current_zero") == 0) {
        int joint_index = 0;
        if (!json_get_int(line, "joint_index", &joint_index) &&
            !json_get_int(line, "joint", &joint_index)) {
            return make_error(id_fragment, -32602, "missing joint_index");
        }
        return bool_result(id_fragment, controller.resetCurrentZero(joint_index));
    }
    if (std::strcmp(method, "arm.get_pose") == 0) {
        std::array<double, 6> pose{};
        char rotation_order[16] = "zyx";
        (void)json_get_token(line, "rotation_order", rotation_order, sizeof(rotation_order));
        if (!controller.getPose(rotation_order, &pose)) {
            return bool_result(id_fragment, false);
        }
        return make_result(id_fragment, array_payload(pose));
    }
    if (std::strcmp(method, "arm.get_transform") == 0 || std::strcmp(method, "arm.get_T") == 0) {
        std::array<double, 16> transform{};
        if (!controller.getTransform(&transform)) {
            return bool_result(id_fragment, false);
        }
        return make_result(id_fragment, array16_payload(transform));
    }
    if (std::strcmp(method, "gripper.set") == 0) {
        bool open = false;
        if (!json_get_bool(line, "open", &open)) {
            return make_error(id_fragment, -32602, "missing open");
        }
        return bool_result(id_fragment, controller.setGripperOpen(open));
    }
    if (std::strcmp(method, "arm.servo_gripper") == 0) {
        int angle_deg = 0;
        double eta_s = 0.0;
        if (!json_get_int(line, "angle_deg", &angle_deg)) {
            return make_error(id_fragment, -32602, "missing angle_deg");
        }
        if (!controller.setServoGripper(angle_deg, &eta_s)) {
            return bool_result(id_fragment, false);
        }
        return make_result(id_fragment, std::string("{\"ok\":true,\"eta_s\":") + number_payload(eta_s) + "}");
    }

    return make_error(id_fragment, -32601, "method not found");
}

std::string handle_legacy_request(const std::string &request, bool *skip_default_response) {
    char action[64] = {};
    ArmController &controller = ArmController::instance();
    std::string params;

    if (skip_default_response != nullptr) {
        *skip_default_response = false;
    }
    if (!json_get_token(request.c_str(), "action", action, sizeof(action))) {
        return "null";
    }
    (void)extract_param_slice(request.c_str(), &params);

    if (std::strcmp(action, "emergency_stop") == 0) {
        double enable = 0.0;
        (void)std::strtod(params.c_str(), nullptr);
        json_get_double(request.c_str(), "params", &enable);
        controller.emergencyStop(std::strstr(params.c_str(), "1") != nullptr);
        return "0.05";
    }
    if (std::strcmp(action, "home_joint") == 0) {
        int joint_index = static_cast<int>(std::strtol(params.c_str(), nullptr, 10));
        return controller.homeJoint(joint_index) ? "0.05" : "-1";
    }
    if (std::strcmp(action, "angle_mode") == 0) {
        std::array<double, 6> joints_deg{};
        double speed_ratio = 1.0;
        double eta_s = -1.0;
        if (!parse_legacy_angle_mode_params(params, &joints_deg, &speed_ratio) ||
            !controller.moveJointsDeg(joints_deg, speed_ratio, &eta_s)) {
            return "-1";
        }
        return number_payload(eta_s);
    }
    if (std::strcmp(action, "move_xyz_rotation") == 0) {
        double x_mm = 0.0;
        double y_mm = 0.0;
        double z_mm = 0.0;
        double roll_deg = 0.0;
        double pitch_deg = 0.0;
        double yaw_deg = 0.0;
        double speed_ratio = 1.0;
        double eta_s = -1.0;
        std::string rotation_order = "zyx";
        if (!parse_legacy_pose_params(params,
                                      &x_mm,
                                      &y_mm,
                                      &z_mm,
                                      &roll_deg,
                                      &pitch_deg,
                                      &yaw_deg,
                                      &rotation_order,
                                      &speed_ratio,
                                      true) ||
            !controller.moveToPose6d(x_mm, y_mm, z_mm, roll_deg, pitch_deg, yaw_deg, rotation_order, speed_ratio, &eta_s)) {
            return "-1";
        }
        return number_payload(eta_s);
    }
    if (std::strcmp(action, "move_linear_xyz_rotation") == 0) {
        double x_mm = 0.0;
        double y_mm = 0.0;
        double z_mm = 0.0;
        double roll_deg = 0.0;
        double pitch_deg = 0.0;
        double yaw_deg = 0.0;
        double eta_s = -1.0;
        std::string rotation_order = "zyx";
        if (!parse_legacy_pose_params(params,
                                      &x_mm,
                                      &y_mm,
                                      &z_mm,
                                      &roll_deg,
                                      &pitch_deg,
                                      &yaw_deg,
                                      &rotation_order,
                                      nullptr,
                                      false) ||
            !controller.moveLinearPose6d(x_mm, y_mm, z_mm, roll_deg, pitch_deg, yaw_deg, rotation_order, &eta_s)) {
            return "-1";
        }
        if (skip_default_response != nullptr) {
            *skip_default_response = false;
        }
        return number_payload(eta_s);
    }
    if (std::strcmp(action, "gripper_on") == 0) {
        return controller.setGripperOpen(false) ? "0.05" : "null";
    }
    if (std::strcmp(action, "gripper_off") == 0) {
        return controller.setGripperOpen(true) ? "0.05" : "null";
    }
    if (std::strcmp(action, "servo_gripper") == 0) {
        double eta_s = 1.0;
        int angle_deg = static_cast<int>(std::strtol(params.c_str(), nullptr, 10));
        return controller.setServoGripper(angle_deg, &eta_s) ? number_payload(eta_s) : "null";
    }
    if (std::strcmp(action, "robodk_simu") == 0) {
        return "0.05";
    }
    if (std::strcmp(action, "set_free_mode") == 0) {
        controller.setFreeMode(std::strstr(params.c_str(), "1") != nullptr);
        return "0.10";
    }
    if (std::strcmp(action, "get_motor_angles") == 0) {
        return array_payload(controller.getMotorAngles());
    }
    if (std::strcmp(action, "set_current_zero") == 0) {
        int joint_index = static_cast<int>(std::strtol(params.c_str(), nullptr, 10));
        return controller.setCurrentZero(joint_index) ? "0.05" : "-1";
    }
    if (std::strcmp(action, "reset_current_zero") == 0) {
        int joint_index = static_cast<int>(std::strtol(params.c_str(), nullptr, 10));
        return controller.resetCurrentZero(joint_index) ? "0.05" : "-1";
    }
    if (std::strcmp(action, "get_pose") == 0) {
        std::array<double, 6> pose{};
        std::string order = "zyx";
        char token[16] = {};
        if (json_get_token(request.c_str(), "params", token, sizeof(token)) && token[0] != '\0') {
            order = token;
        } else if (!params.empty() && params[0] == '"') {
            order.assign(params.c_str() + 1, params.size() >= 2 ? params.size() - 2 : 0);
        }
        if (!controller.getPose(order, &pose)) {
            return "null";
        }
        return array_payload(pose);
    }
    if (std::strcmp(action, "get_T") == 0) {
        std::array<double, 16> transform{};
        if (!controller.getTransform(&transform)) {
            return "null";
        }
        transform[3] *= 1000.0;
        transform[7] *= 1000.0;
        transform[11] *= 1000.0;
        return array16_payload(transform);
    }
    if (std::strcmp(action, "dynamic_move") == 0) {
        std::array<double, 6> joints_deg{};
        std::array<double, 6> current_pulses{};
        std::array<double, 6> max_speed{};
        double eta_s = 0.05;
        size_t first_open = params.find('[');
        size_t first_close = params.find(']');
        if (first_open == std::string::npos || first_close == std::string::npos) {
            return "-1";
        }
        if (!parse_legacy_number_array(params.substr(first_open, first_close - first_open + 1), &joints_deg)) {
            return "-1";
        }
        {
            size_t second_open = params.find('[', first_close + 1);
            size_t second_close = params.find(']', second_open);
            size_t third_open = params.find('[', second_close + 1);
            size_t third_close = params.find(']', third_open);
            if (second_open == std::string::npos || second_close == std::string::npos ||
                third_open == std::string::npos || third_close == std::string::npos) {
                return "-1";
            }
            if (!parse_legacy_number_array(params.substr(second_open, second_close - second_open + 1), &current_pulses) ||
                !parse_legacy_number_array(params.substr(third_open, third_close - third_open + 1), &max_speed)) {
                return "-1";
            }
        }
        if (!controller.dynamicMove(joints_deg, current_pulses, max_speed, &eta_s)) {
            return "-1";
        }
        return number_payload(eta_s);
    }

    return "null";
}
}

int run_arm_runtime() {
    int server_fd = -1;
    int legacy_fd = -1;
    struct sockaddr_un addr{};
    struct sockaddr_in legacy_addr{};
    struct pollfd pfds[1 + kMaxClients];
    int client_fds[kMaxClients];
    char bufs[kMaxClients][kBufSize];
    size_t buf_lens[kMaxClients];
    int nclients = 0;
    int legacy_port = kLegacyPortDefault;

    std::signal(SIGPIPE, SIG_IGN);

    server_fd = socket(AF_UNIX, SOCK_STREAM, 0);
    if (server_fd < 0) {
        log_error(kTag, "unix socket failed: %s", std::strerror(errno));
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
        log_error(kTag, "listen unix failed: %s", std::strerror(errno));
        close(server_fd);
        return 1;
    }

    if (const char *port_text = std::getenv("UAV_PROC_ARM_PORT")) {
        legacy_port = std::atoi(port_text);
        if (legacy_port <= 0) {
            legacy_port = kLegacyPortDefault;
        }
    }
    legacy_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (legacy_fd >= 0) {
        int reuse = 1;
        setsockopt(legacy_fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
        std::memset(&legacy_addr, 0, sizeof(legacy_addr));
        legacy_addr.sin_family = AF_INET;
        legacy_addr.sin_port = htons(static_cast<uint16_t>(legacy_port));
        legacy_addr.sin_addr.s_addr = htonl(INADDR_ANY);
        if (bind(legacy_fd, reinterpret_cast<struct sockaddr *>(&legacy_addr), sizeof(legacy_addr)) < 0 ||
            listen(legacy_fd, 2) < 0) {
            log_warn(kTag, "legacy tcp bind/listen %d failed: %s", legacy_port, std::strerror(errno));
            close(legacy_fd);
            legacy_fd = -1;
        } else {
            log_info(kTag, "legacy tcp listening on 0.0.0.0:%d", legacy_port);
        }
    }

    std::memset(buf_lens, 0, sizeof(buf_lens));
    for (int i = 0; i < kMaxClients; ++i) {
        client_fds[i] = -1;
    }
    log_info(kTag, "jsonrpc listening on %s", kSockPath);

    start_estop_watchdog();

    while (true) {
        pfds[0].fd = server_fd;
        pfds[0].events = POLLIN;
        for (int i = 0; i < nclients; ++i) {
            pfds[1 + i].fd = client_fds[i];
            pfds[1 + i].events = POLLIN;
        }

        if (poll(pfds, 1 + nclients, 100) < 0) {
            if (errno == EINTR) {
                continue;
            }
            log_error(kTag, "poll failed: %s", std::strerror(errno));
            break;
        }

        if ((pfds[0].revents & POLLIN) != 0) {
            int client_fd = accept(server_fd, nullptr, nullptr);
            if (client_fd >= 0) {
                if (nclients >= kMaxClients) {
                    close(client_fd);
                } else {
                    client_fds[nclients] = client_fd;
                    pfds[1 + nclients].fd = client_fd;
                    pfds[1 + nclients].events = POLLIN;
                    // poll() did not write revents for this slot (it wasn't
                    // in the set), so it still holds a previous iteration's
                    // value. Left stale, the loop below recv()s from a
                    // client that has not sent anything yet and blocks.
                    pfds[1 + nclients].revents = 0;
                    buf_lens[nclients] = 0U;
                    ++nclients;
                    register_client_fd(client_fd);
                }
            }
        }

        if (legacy_fd >= 0) {
            struct pollfd legacy_pfd{};
            legacy_pfd.fd = legacy_fd;
            legacy_pfd.events = POLLIN;
            if (poll(&legacy_pfd, 1, 0) > 0 && (legacy_pfd.revents & POLLIN) != 0) {
                int client_fd = accept(legacy_fd, nullptr, nullptr);
                if (client_fd >= 0) {
                    register_client_fd(client_fd);
                    std::string request;
                    while (recv_legacy_message(client_fd, &request)) {
                        bool skip_default = false;
                        std::string response = handle_legacy_request(request, &skip_default);
                        if (!skip_default) {
                            if (!send_legacy_message(client_fd, response)) {
                                break;
                            }
                        }
                    }
                    unregister_client_fd(client_fd);
                    close(client_fd);
                }
            }
        }

        for (int i = 0; i < nclients;) {
            bool removed = false;
            if ((pfds[1 + i].revents & (POLLIN | POLLHUP | POLLERR)) != 0) {
                ssize_t n = recv(client_fds[i],
                                 bufs[i] + buf_lens[i],
                                 sizeof(bufs[i]) - buf_lens[i] - 1U,
                                 0);
                if (n <= 0) {
                    unregister_client_fd(client_fds[i]);
                    close(client_fds[i]);
                    for (int j = i; j + 1 < nclients; ++j) {
                        client_fds[j] = client_fds[j + 1];
                        buf_lens[j] = buf_lens[j + 1];
                        std::memcpy(bufs[j], bufs[j + 1], sizeof(bufs[j]));
                    }
                    --nclients;
                    removed = true;
                } else {
                    buf_lens[i] += static_cast<size_t>(n);
                    bufs[i][buf_lens[i]] = '\0';

                    while (true) {
                        char *newline = static_cast<char *>(std::memchr(bufs[i], '\n', buf_lens[i]));
                        if (newline == nullptr) {
                            break;
                        }
                        size_t line_len = static_cast<size_t>(newline - bufs[i]);
                        std::string request(bufs[i], line_len);
                        std::string response = handle_jsonrpc_request(request.c_str());
                        if (!send_all(client_fds[i], response.data(), response.size())) {
                            unregister_client_fd(client_fds[i]);
                            close(client_fds[i]);
                            for (int j = i; j + 1 < nclients; ++j) {
                                client_fds[j] = client_fds[j + 1];
                                buf_lens[j] = buf_lens[j + 1];
                                std::memcpy(bufs[j], bufs[j + 1], sizeof(bufs[j]));
                            }
                            --nclients;
                            removed = true;
                            break;
                        }

                        {
                            size_t remaining = buf_lens[i] - line_len - 1U;
                            std::memmove(bufs[i], newline + 1, remaining);
                            buf_lens[i] = remaining;
                            bufs[i][buf_lens[i]] = '\0';
                        }
                    }

                    if (!removed && buf_lens[i] + 1U >= sizeof(bufs[i])) {
                        unregister_client_fd(client_fds[i]);
                        close(client_fds[i]);
                        for (int j = i; j + 1 < nclients; ++j) {
                            client_fds[j] = client_fds[j + 1];
                            buf_lens[j] = buf_lens[j + 1];
                            std::memcpy(bufs[j], bufs[j + 1], sizeof(bufs[j]));
                        }
                        --nclients;
                        removed = true;
                    }
                }
            }
            if (!removed) {
                ++i;
            }
        }
    }

    for (int i = 0; i < nclients; ++i) {
        unregister_client_fd(client_fds[i]);
        close(client_fds[i]);
    }
    if (legacy_fd >= 0) {
        close(legacy_fd);
    }
    close(server_fd);
    unlink(kSockPath);
    return 0;
}
