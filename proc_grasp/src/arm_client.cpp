#include "arm_client.h"

#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>

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
}  // namespace

ArmClient::ArmClient(std::string sock_path)
    : sock_path_(std::move(sock_path))
{
    if (sock_path_.empty()) {
        sock_path_ = default_sock_path();
    }
}

bool ArmClient::call_ok(const char *method, const char *params_json, int timeout_sec)
{
    if (method == nullptr || params_json == nullptr) return false;

    static int s_id = 5000;
    int id = ++s_id;

    int fd = ::socket(AF_UNIX, SOCK_STREAM, 0);
    if (fd < 0) {
        std::fprintf(stderr, "proc_grasp: arm socket failed: %s\n", std::strerror(errno));
        return false;
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
        return false;
    }

    char req[512];
    int req_len = std::snprintf(req, sizeof(req),
        "{\"jsonrpc\":\"2.0\",\"id\":%d,\"method\":\"%s\",\"params\":%s}\n",
        id, method, params_json);
    if (req_len <= 0 || ::write(fd, req, static_cast<size_t>(req_len)) != req_len) {
        ::close(fd);
        return false;
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
    if (total <= 0) return false;

    return std::strstr(buf, "\"ok\":true") != nullptr
        || std::strstr(buf, "\"result\":true") != nullptr;
}

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
    return call_ok("arm.move_pose6d", params);
}

bool ArmClient::move_xyz(double x_mm, double y_mm, double z_mm)
{
    char params[128];
    std::snprintf(params, sizeof(params),
                  "{\"x_mm\":%.3f,\"y_mm\":%.3f,\"z_mm\":%.3f}",
                  x_mm, y_mm, z_mm);
    return call_ok("arm.move_xyz", params);
}

bool ArmClient::home()            { return call_ok("arm.home", "{}"); }
bool ArmClient::stop()            { return call_ok("arm.stop", "{}", 5); }
bool ArmClient::emergency_stop()  { return call_ok("arm.emergency_stop", "{}", 5); }

bool ArmClient::servo_gripper(int angle_deg)
{
    char params[64];
    std::snprintf(params, sizeof(params), "{\"angle_deg\":%d}", angle_deg);
    return call_ok("arm.servo_gripper", params);
}
