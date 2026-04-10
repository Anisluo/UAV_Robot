#include "result_listener.h"

#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstring>

#include <fcntl.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include "abi/ipc_framing.h"

namespace {
uint64_t now_ms_steady()
{
    using namespace std::chrono;
    return static_cast<uint64_t>(
        duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count());
}
}  // namespace

ResultListener::ResultListener()
    : fd_(-1), has_result_(false), last_rx_ms_(0) {}

ResultListener::~ResultListener()
{
    close();
}

bool ResultListener::open()
{
    close();

    fd_ = ::socket(AF_UNIX, SOCK_DGRAM, 0);
    if (fd_ < 0) {
        std::fprintf(stderr, "proc_grasp: result socket failed: %s\n", std::strerror(errno));
        return false;
    }

    sockaddr_un addr{};
    addr.sun_family = AF_UNIX;
    std::strncpy(addr.sun_path, UAV_NPU_RESULT_GRASP_PATH, sizeof(addr.sun_path) - 1);
    ::unlink(UAV_NPU_RESULT_GRASP_PATH);

    if (::bind(fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
        std::fprintf(stderr, "proc_grasp: result bind failed: %s\n", std::strerror(errno));
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    int flags = fcntl(fd_, F_GETFL, 0);
    if (flags >= 0) {
        fcntl(fd_, F_SETFL, flags | O_NONBLOCK);
    }
    return true;
}

void ResultListener::close()
{
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
        ::unlink(UAV_NPU_RESULT_GRASP_PATH);
    }
}

int ResultListener::poll()
{
    if (fd_ < 0) return 0;
    int drained = 0;
    UavCResult r{};
    ssize_t n;
    while ((n = ::recv(fd_, &r, sizeof(r), MSG_DONTWAIT)) == static_cast<ssize_t>(sizeof(r))) {
        latest_ = r;
        has_result_ = true;
        last_rx_ms_ = now_ms_steady();
        ++drained;
    }
    return drained;
}

bool ResultListener::latest(UavCResult *out, uint64_t *age_ms_out) const
{
    if (!has_result_ || out == nullptr) return false;
    *out = latest_;
    if (age_ms_out != nullptr) {
        const uint64_t now = now_ms_steady();
        *age_ms_out = (now > last_rx_ms_) ? (now - last_rx_ms_) : 0U;
    }
    return true;
}

void ResultListener::clear()
{
    has_result_ = false;
}
