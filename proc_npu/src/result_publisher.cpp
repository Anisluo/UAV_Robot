#include "result_publisher.h"

#include <cerrno>
#include <cstring>
#include <cstdio>

#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include "abi/ipc_framing.h"  // UAV_NPU_RESULT_GW_PATH, UAV_NPU_RESULT_APP_PATH

ResultPublisher::ResultPublisher() = default;

ResultPublisher::~ResultPublisher()
{
    close();
}

bool ResultPublisher::open()
{
    fd_ = ::socket(AF_UNIX, SOCK_DGRAM, 0);
    if (fd_ < 0) {
        std::perror("result_publisher: socket");
        return false;
    }
    return true;
}

void ResultPublisher::close()
{
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

void ResultPublisher::publish(const UavCResult &result)
{
    if (fd_ < 0) return;
    send_to(UAV_NPU_RESULT_GW_PATH,  result);
    send_to(UAV_NPU_RESULT_APP_PATH, result);
}

bool ResultPublisher::send_to(const char *path, const UavCResult &result)
{
    sockaddr_un addr{};
    addr.sun_family = AF_UNIX;
    std::strncpy(addr.sun_path, path, sizeof(addr.sun_path) - 1);

    ssize_t n = ::sendto(fd_,
                         &result, sizeof(result),
                         MSG_DONTWAIT,
                         reinterpret_cast<const sockaddr *>(&addr), sizeof(addr));
    if (n < 0 && errno != ENOENT && errno != ECONNREFUSED) {
        // Log unexpected errors but keep running; missing socket is normal.
        std::perror("result_publisher: sendto");
        return false;
    }
    return true;
}
