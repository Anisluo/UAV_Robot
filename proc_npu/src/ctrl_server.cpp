#include "ctrl_server.h"

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <utility>

#include <poll.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

CtrlServer::CtrlServer() : fd_(-1), path_(), handler_() {}

CtrlServer::~CtrlServer() {
    stop();
}

bool CtrlServer::start(const std::string &path, Handler handler) {
    stop();
    fd_ = socket(AF_UNIX, SOCK_DGRAM, 0);
    if (fd_ < 0) {
        std::fprintf(stderr, "proc_npu: ctrl socket failed: %s\n", std::strerror(errno));
        return false;
    }

    sockaddr_un addr{};
    addr.sun_family = AF_UNIX;
    std::snprintf(addr.sun_path, sizeof(addr.sun_path), "%s", path.c_str());
    unlink(path.c_str());
    if (bind(fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) != 0) {
        std::fprintf(stderr, "proc_npu: ctrl bind failed: %s\n", std::strerror(errno));
        stop();
        return false;
    }
    path_ = path;
    handler_ = std::move(handler);
    return true;
}

void CtrlServer::stop() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
    if (!path_.empty()) {
        unlink(path_.c_str());
        path_.clear();
    }
}

void CtrlServer::poll_once(int timeout_ms) {
    if (fd_ < 0) {
        return;
    }
    pollfd pfd{};
    pfd.fd = fd_;
    pfd.events = POLLIN;
    if (poll(&pfd, 1, timeout_ms) <= 0 || !(pfd.revents & POLLIN)) {
        return;
    }

    uint8_t buffer[sizeof(UavIpcHeader) + sizeof(UavCtrlPayload)]{};
    const ssize_t n = recv(fd_, buffer, sizeof(buffer), 0);
    if (n < static_cast<ssize_t>(sizeof(UavIpcHeader))) {
        return;
    }

    UavIpcHeader hdr{};
    std::memcpy(&hdr, buffer, sizeof(hdr));
    if (hdr.magic != UAV_IPC_MAGIC || hdr.type != UAV_IPC_MSG_CTRL || hdr.payload_size != sizeof(UavCtrlPayload)) {
        return;
    }
    if (n < static_cast<ssize_t>(sizeof(UavIpcHeader) + sizeof(UavCtrlPayload))) {
        return;
    }

    UavCtrlPayload payload{};
    std::memcpy(&payload, buffer + sizeof(hdr), sizeof(payload));
    if (handler_) {
        handler_(payload);
    }
}
