#include "arm_client.h"

#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <chrono>
#include <thread>

#include <sys/socket.h>
#include <sys/time.h>
#include <sys/un.h>
#include <unistd.h>

namespace {

const char *default_sock_path()
{
    const char *e = std::getenv("UAV_PROC_ARM_SOCK");
    return (e != nullptr && e[0] != '\0') ? e : "/tmp/uav_proc_arm.sock";
}

// Parse a double value for a given key from a JSON string (minimal parser).
bool parse_json_double(const char *json, const char *key, double *out)
{
    if (json == nullptr || key == nullptr || out == nullptr) return false;
    char needle[64];
    std::snprintf(needle, sizeof(needle), "\"%s\"", key);
    const char *pos = std::strstr(json, needle);
    if (pos == nullptr) return false;
    pos = std::strchr(pos + std::strlen(needle), ':');
    if (pos == nullptr) return false;
    ++pos;
    while (*pos == ' ' || *pos == '\t') ++pos;
    char *endp = nullptr;
    double v = std::strtod(pos, &endp);
    if (endp == pos) return false;
    *out = v;
    return true;
}

// Parse a JSON array of doubles: [v0,v1,...]. Returns count parsed.
int parse_json_double_array(const char *json, double *out, int max_n)
{
    if (json == nullptr || out == nullptr) return 0;
    const char *p = std::strchr(json, '[');
    if (p == nullptr) return 0;
    ++p;
    int count = 0;
    while (count < max_n) {
        while (*p == ' ' || *p == '\t' || *p == ',' || *p == '\n') ++p;
        if (*p == ']' || *p == '\0') break;
        char *endp = nullptr;
        double v = std::strtod(p, &endp);
        if (endp == p) break;
        out[count++] = v;
        p = endp;
    }
    return count;
}

constexpr double kPoseTolMm    = 2.0;   // mm tolerance for position convergence
constexpr double kPollIntervalS = 0.15;  // seconds between pose polls
constexpr double kEtaMargin     = 2.0;   // extra seconds beyond eta_s

}  // namespace

ArmClient::ArmClient(std::string sock_path)
    : sock_path_(std::move(sock_path))
{
    if (sock_path_.empty()) {
        sock_path_ = default_sock_path();
    }
}

// ---------------------------------------------------------------------------
// raw_request: open socket, send JSON-RPC, return full response string.
// ---------------------------------------------------------------------------
std::string ArmClient::raw_request(const char *method, const char *params_json, int timeout_sec)
{
    if (method == nullptr || params_json == nullptr) return {};

    static int s_id = 5000;
    int id = ++s_id;

    int fd = ::socket(AF_UNIX, SOCK_STREAM, 0);
    if (fd < 0) {
        std::fprintf(stderr, "proc_grasp: arm socket failed: %s\n", std::strerror(errno));
        return {};
    }

    struct timeval tv{};
    tv.tv_sec  = timeout_sec;
    tv.tv_usec = 0;
    ::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    ::setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

    sockaddr_un addr{};
    addr.sun_family = AF_UNIX;
    std::strncpy(addr.sun_path, sock_path_.c_str(), sizeof(addr.sun_path) - 1);

    if (::connect(fd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
        std::fprintf(stderr, "proc_grasp: arm connect %s failed: %s\n",
                     sock_path_.c_str(), std::strerror(errno));
        ::close(fd);
        return {};
    }

    char req[512];
    int req_len = std::snprintf(req, sizeof(req),
        "{\"jsonrpc\":\"2.0\",\"id\":%d,\"method\":\"%s\",\"params\":%s}\n",
        id, method, params_json);
    if (req_len <= 0 || ::write(fd, req, static_cast<size_t>(req_len)) != req_len) {
        ::close(fd);
        return {};
    }

    char buf[1024] = {};
    int total = 0;
    while (total < static_cast<int>(sizeof(buf)) - 1) {
        ssize_t n = ::read(fd, buf + total, sizeof(buf) - 1 - static_cast<size_t>(total));
        if (n <= 0) break;
        total += static_cast<int>(n);
        if (std::memchr(buf, '\n', static_cast<size_t>(total)) != nullptr) break;
    }
    ::close(fd);
    if (total <= 0) return {};

    return std::string(buf, static_cast<size_t>(total));
}

// ---------------------------------------------------------------------------
// call_ok: fire-and-forget — returns true iff "ok":true in response.
// ---------------------------------------------------------------------------
bool ArmClient::call_ok(const char *method, const char *params_json, int timeout_sec)
{
    std::string resp = raw_request(method, params_json, timeout_sec);
    if (resp.empty()) return false;
    return resp.find("\"ok\":true") != std::string::npos
        || resp.find("\"result\":true") != std::string::npos;
}

// ---------------------------------------------------------------------------
// call_move: like call_ok but also parses eta_s from the response.
// ---------------------------------------------------------------------------
bool ArmClient::call_move(const char *method, const char *params_json,
                          double *out_eta_s, int timeout_sec)
{
    std::string resp = raw_request(method, params_json, timeout_sec);
    if (resp.empty()) return false;
    if (resp.find("\"ok\":true") == std::string::npos) return false;
    if (out_eta_s != nullptr) {
        if (!parse_json_double(resp.c_str(), "eta_s", out_eta_s)) {
            *out_eta_s = 0.0;
        }
    }
    return true;
}

// ---------------------------------------------------------------------------
// get_pose: read current [x, y, z, roll, pitch, yaw] from proc_arm.
// ---------------------------------------------------------------------------
bool ArmClient::get_pose(std::array<double, 6> &out)
{
    std::string resp = raw_request("arm.get_pose", "{\"rotation_order\":\"zyx\"}", 5);
    if (resp.empty()) return false;
    // Response: {"id":N,"result":[x,y,z,roll,pitch,yaw]}
    double vals[6] = {};
    if (parse_json_double_array(resp.c_str(), vals, 6) < 6) return false;
    for (int i = 0; i < 6; ++i) out[static_cast<size_t>(i)] = vals[i];
    return true;
}

// ---------------------------------------------------------------------------
// wait_pose: poll arm.get_pose until position converges or timeout.
// ---------------------------------------------------------------------------
bool ArmClient::wait_pose(const double target[6], double tol_mm, double timeout_s)
{
    auto t0 = std::chrono::steady_clock::now();
    double deadline_s = timeout_s > 0.0 ? timeout_s : 10.0;

    while (true) {
        std::this_thread::sleep_for(
            std::chrono::milliseconds(static_cast<int>(kPollIntervalS * 1000)));

        std::array<double, 6> pose{};
        if (get_pose(pose)) {
            double dx = pose[0] - target[0];
            double dy = pose[1] - target[1];
            double dz = pose[2] - target[2];
            double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
            if (dist <= tol_mm) {
                return true;
            }
        }

        auto elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - t0).count();
        if (elapsed >= deadline_s) {
            std::fprintf(stderr, "proc_grasp: wait_pose timeout (%.1fs)\n", elapsed);
            return false;
        }
    }
}

// ---------------------------------------------------------------------------
// move_pose6d: blocking move to 6D pose.
// ---------------------------------------------------------------------------
bool ArmClient::move_pose6d(double x_mm, double y_mm, double z_mm,
                            double roll_deg, double pitch_deg, double yaw_deg,
                            double speed_ratio)
{
    char params[256];
    std::snprintf(params, sizeof(params),
                  "{\"x_mm\":%.3f,\"y_mm\":%.3f,\"z_mm\":%.3f,"
                  "\"roll_deg\":%.3f,\"pitch_deg\":%.3f,\"yaw_deg\":%.3f,"
                  "\"speed_ratio\":%.3f,\"rotation_order\":\"zyx\"}",
                  x_mm, y_mm, z_mm, roll_deg, pitch_deg, yaw_deg, speed_ratio);

    double eta_s = 0.0;
    if (!call_move("arm.move_pose6d", params, &eta_s)) {
        return false;
    }

    std::fprintf(stderr, "proc_grasp: move_pose6d -> (%.1f,%.1f,%.1f) eta=%.1fs\n",
                 x_mm, y_mm, z_mm, eta_s);

    double target[6] = {x_mm, y_mm, z_mm, roll_deg, pitch_deg, yaw_deg};
    double timeout = eta_s + kEtaMargin;
    if (timeout < 3.0) timeout = 3.0;
    return wait_pose(target, kPoseTolMm, timeout);
}

// ---------------------------------------------------------------------------
// move_xyz: blocking move to XYZ (orientation unchanged).
// ---------------------------------------------------------------------------
bool ArmClient::move_xyz(double x_mm, double y_mm, double z_mm)
{
    char params[128];
    std::snprintf(params, sizeof(params),
                  "{\"x_mm\":%.3f,\"y_mm\":%.3f,\"z_mm\":%.3f}",
                  x_mm, y_mm, z_mm);

    double eta_s = 0.0;
    if (!call_move("arm.move_xyz", params, &eta_s)) {
        return false;
    }

    std::fprintf(stderr, "proc_grasp: move_xyz -> (%.1f,%.1f,%.1f) eta=%.1fs\n",
                 x_mm, y_mm, z_mm, eta_s);

    // For move_xyz we only check position convergence (no orientation target).
    // Read current pose to get orientation, then wait for XYZ convergence.
    std::array<double, 6> cur{};
    get_pose(cur);
    double target[6] = {x_mm, y_mm, z_mm, cur[3], cur[4], cur[5]};
    double timeout = eta_s + kEtaMargin;
    if (timeout < 3.0) timeout = 3.0;
    return wait_pose(target, kPoseTolMm, timeout);
}

// ---------------------------------------------------------------------------
// home: blocking home. Wait for arm to reach home position.
// ---------------------------------------------------------------------------
bool ArmClient::home()
{
    if (!call_ok("arm.home", "{}")) return false;

    // Home can take a long time — poll pose until it stabilises.
    std::fprintf(stderr, "proc_grasp: home started, waiting...\n");
    auto t0 = std::chrono::steady_clock::now();
    std::array<double, 6> prev{};
    int stable_count = 0;

    while (true) {
        std::this_thread::sleep_for(std::chrono::milliseconds(300));

        std::array<double, 6> pose{};
        if (get_pose(pose)) {
            double dx = pose[0] - prev[0];
            double dy = pose[1] - prev[1];
            double dz = pose[2] - prev[2];
            double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
            if (dist < 0.5) {
                ++stable_count;
                if (stable_count >= 3) {
                    std::fprintf(stderr, "proc_grasp: home complete\n");
                    return true;
                }
            } else {
                stable_count = 0;
            }
            prev = pose;
        }

        auto elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - t0).count();
        if (elapsed > 60.0) {
            std::fprintf(stderr, "proc_grasp: home timeout\n");
            return false;
        }
    }
}

bool ArmClient::stop()            { return call_ok("arm.stop", "{}", 5); }
bool ArmClient::emergency_stop()  { return call_ok("arm.emergency_stop", "{}", 5); }

// ---------------------------------------------------------------------------
// servo_gripper: blocking gripper move.
// ---------------------------------------------------------------------------
bool ArmClient::servo_gripper(int angle_deg)
{
    char params[64];
    std::snprintf(params, sizeof(params), "{\"angle_deg\":%d}", angle_deg);

    double eta_s = 0.0;
    if (!call_move("arm.servo_gripper", params, &eta_s)) {
        return false;
    }

    std::fprintf(stderr, "proc_grasp: servo_gripper(%d) eta=%.1fs\n", angle_deg, eta_s);

    // Gripper has no pose feedback — sleep for eta_s + margin.
    if (eta_s > 0.0) {
        int wait_ms = static_cast<int>((eta_s + 0.5) * 1000);
        std::this_thread::sleep_for(std::chrono::milliseconds(wait_ms));
    } else {
        // Fallback: gripper typically takes ~0.5s
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    return true;
}
